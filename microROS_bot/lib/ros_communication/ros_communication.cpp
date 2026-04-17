#include "ros_communication.h"
#include "motor_control.h"
#include "shared_state.h"
#include "wifi_config.h"
#include <stdio.h>

// ── micro-ROS entities ────────────────────────────────────────────────────────
rcl_subscription_t cmd_vel_sub;
rcl_subscription_t servo_sub;
rcl_subscription_t pid_gains_sub;
rcl_publisher_t    debug_pub;

geometry_msgs__msg__Twist            cmd_vel_msg;
std_msgs__msg__Float32MultiArray     servo_msg;
std_msgs__msg__Float32MultiArray     pid_gains_msg;
std_msgs__msg__Float32MultiArray     debug_msg;

rclc_executor_t executor;
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;

static float servo_data[2]       = {90.0f, 90.0f};
static float motor_debug_data[6] = {0};
static float gains_data[3]       = {2.0f, 0.5f, 1.5f};

unsigned long RosCommunication::lastCbPrint = 0;

// Prints error location and causes create_entities() to return false
#define RCCHECK(fn) { \
    rcl_ret_t _rc = (fn); \
    if (_rc != RCL_RET_OK) { \
        Serial.printf("RCCHECK failed line %d rc=%d\n", __LINE__, (int)_rc); \
        return false; \
    } \
}

// ── Constructor ───────────────────────────────────────────────────────────────
RosCommunication::RosCommunication() : state_(WAITING_AGENT) {}

// ── One-time WiFi + transport setup ──────────────────────────────────────────
void RosCommunication::initialize() {
    char wifi_ssid[]     = WIFI_SSID;
    char wifi_password[] = WIFI_PASSWORD;
    IPAddress agent_ip(AGENT_IP);

    set_microros_wifi_transports(wifi_ssid, wifi_password, agent_ip, AGENT_PORT);

    Serial.print("Connecting to WiFi");
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - t0 > 15000) {
            Serial.println("\nWiFi timeout — check credentials");
            return;
        }
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.print("WiFi connected, IP: ");
    Serial.println(WiFi.localIP());

    WiFi.setSleep(false);  // disable power save — eliminates DTIM beacon latency (~100ms)

    state_ = WAITING_AGENT;
}

// ── Create all micro-ROS entities (called on connect / reconnect) ─────────────
bool RosCommunication::create_entities() {
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "diff_drive_node", "", &support));

    RCCHECK(rclc_subscription_init_default(
        &cmd_vel_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"));

    servo_msg.data.data     = servo_data;
    servo_msg.data.capacity = 2;
    servo_msg.data.size     = 2;
    RCCHECK(rclc_subscription_init_default(
        &servo_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/servo/cmd"));

    pid_gains_msg.data.data     = gains_data;
    pid_gains_msg.data.capacity = 3;
    pid_gains_msg.data.size     = 3;
    RCCHECK(rclc_subscription_init_default(
        &pid_gains_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/pid_gains"));

    RCCHECK(rclc_publisher_init_default(
        &debug_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/motor_debug"));
    debug_msg.data.data     = motor_debug_data;
    debug_msg.data.capacity = 6;
    debug_msg.data.size     = 6;

    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &cmd_vel_sub, &cmd_vel_msg,
        &RosCommunication::cmd_vel_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &servo_sub, &servo_msg,
        &RosCommunication::servo_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &pid_gains_sub, &pid_gains_msg,
        &RosCommunication::pid_gains_callback, ON_NEW_DATA));

    return true;
}

// ── Destroy all micro-ROS entities (called on disconnect) ─────────────────────
void RosCommunication::destroy_entities() {
    rclc_executor_fini(&executor);
    rcl_publisher_fini(&debug_pub, &node);
    rcl_subscription_fini(&pid_gains_sub, &node);
    rcl_subscription_fini(&servo_sub, &node);
    rcl_subscription_fini(&cmd_vel_sub, &node);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

// ── Main loop — reconnection state machine ────────────────────────────────────
void RosCommunication::start_receiving_msgs() {
    static unsigned long lastAgentCheck = 0;
    static unsigned long lastPingCheck  = 0;
    static unsigned long lastPrint      = 0;
    static int           spinCount      = 0;

    switch (state_) {
        case WAITING_AGENT:
            if (millis() - lastAgentCheck >= 300) {
                lastAgentCheck = millis();
                if (rmw_uros_ping_agent(200, 1) == RMW_RET_OK) {
                    if (create_entities()) {
                        state_ = AGENT_CONNECTED;
                        if (xSemaphoreTake(serial_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                            Serial.println("micro-ROS agent connected");
                            xSemaphoreGive(serial_mutex);
                        }
                    }
                } else if (millis() - lastPrint >= 5000) {
                    lastPrint = millis();
                    if (xSemaphoreTake(serial_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                        Serial.println("Waiting for micro-ROS agent...");
                        xSemaphoreGive(serial_mutex);
                    }
                }
            }
            break;

        case AGENT_CONNECTED:
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0));
            publish_debug();
            spinCount++;

            if (millis() - lastPrint >= 5000) {
                lastPrint = millis();
                if (xSemaphoreTake(serial_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    Serial.print("Executor alive, spins: ");
                    Serial.println(spinCount);
                    xSemaphoreGive(serial_mutex);
                }
            }

            // Periodic ping to detect agent disconnect
            if (millis() - lastPingCheck >= 3000) {
                lastPingCheck = millis();
                if (rmw_uros_ping_agent(50, 1) != RMW_RET_OK) {
                    destroy_entities();
                    state_ = WAITING_AGENT;
                    if (xSemaphoreTake(serial_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                        Serial.println("micro-ROS agent lost, reconnecting...");
                        xSemaphoreGive(serial_mutex);
                    }
                }
            }
            break;
    }
}

// ── Callbacks ─────────────────────────────────────────────────────────────────
void RosCommunication::cmd_vel_callback(const void *msg_recv) {
    const geometry_msgs__msg__Twist *msg =
        (const geometry_msgs__msg__Twist *)msg_recv;

    float linear  = msg->linear.x;
    float angular = msg->angular.z;
    float leftVel  = linear - (angular * WHEEL_BASE / 2.0f);
    float rightVel = linear + (angular * WHEEL_BASE / 2.0f);

    targetRPM_L   = (leftVel  / (2.0f * PI * WHEEL_RADIUS)) * 60.0f;
    targetRPM_R   = (rightVel / (2.0f * PI * WHEEL_RADIUS)) * 60.0f;
    last_cmd_time = millis();

    if (millis() - lastCbPrint >= 1000) {
        lastCbPrint = millis();
        if (xSemaphoreTake(serial_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            Serial.print("cmd_vel RPM L:"); Serial.print(targetRPM_L);
            Serial.print(" R:");            Serial.println(targetRPM_R);
            xSemaphoreGive(serial_mutex);
        }
    }
}

void RosCommunication::servo_callback(const void *msg_recv) {
    const std_msgs__msg__Float32MultiArray *msg =
        (const std_msgs__msg__Float32MultiArray *)msg_recv;

    if (msg->data.size < 1) return;

    float angle1 = msg->data.data[0];
    float angle2 = (msg->data.size >= 2) ? msg->data.data[1] : 90.0f;

    set_servo_1(angle1);
    set_servo_2(angle2);

    if (xSemaphoreTake(serial_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        Serial.print("Servo S1:"); Serial.print(angle1);
        Serial.print(" S2:");      Serial.println(angle2);
        xSemaphoreGive(serial_mutex);
    }
}

// ── Debug publisher — /motor_debug Float32[6]: tgt_L, act_L, tgt_R, act_R, pid_L, pid_R ──
void RosCommunication::publish_debug() {
    static unsigned long lastDebugPub = 0;
    if (millis() - lastDebugPub < 100) return;  // 10 Hz
    lastDebugPub = millis();

    motor_debug_data[0] = (float)targetRPM_L;
    motor_debug_data[1] = (float)actual_rpm_L;
    motor_debug_data[2] = (float)targetRPM_R;
    motor_debug_data[3] = (float)actual_rpm_R;
    motor_debug_data[4] = (float)pid_out_L;
    motor_debug_data[5] = (float)pid_out_R;

    rcl_publish(&debug_pub, &debug_msg, NULL);
}

// ── PID gains callback — /pid_gains Float32[2]: [kp, ki] ─────────────────────
void RosCommunication::pid_gains_callback(const void *msg_recv) {
    const std_msgs__msg__Float32MultiArray *msg =
        (const std_msgs__msg__Float32MultiArray *)msg_recv;
    if (msg->data.size < 2) return;

    KP_live = msg->data.data[0];
    KI_live = msg->data.data[1];
    if (msg->data.size >= 3)
        KF_live = msg->data.data[2];

    if (xSemaphoreTake(serial_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        Serial.print("PID gains KP:"); Serial.print(KP_live);
        Serial.print(" KI:");          Serial.print(KI_live);
        Serial.print(" KF:");          Serial.println(KF_live);
        xSemaphoreGive(serial_mutex);
    }
}
