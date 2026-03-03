#include "ros_communication.h"
#include "motor_control.h"
#include "shared_state.h"
#include <esp_timer.h>
#include <stdio.h>

rcl_subscription_t cmd_vel_sub;
rcl_subscription_t servo_sub;
rcl_publisher_t    debug_pub;

geometry_msgs__msg__Twist            cmd_vel_msg;
std_msgs__msg__Float32MultiArray     servo_msg;
std_msgs__msg__String                debug_msg;

rclc_executor_t executor;
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;

static float  servo_data[2] = {90.0f, 90.0f};
static char   debug_buf[256];

unsigned long RosCommunication::lastCbPrint = 0;

RosCommunication::RosCommunication() {}

void RosCommunication::initialize() {
    Serial.begin(115200);
    Serial.println("ROS Communication node started");

    char wifi_ssid[]     = "FASTWEB-9FKRUN";
    char wifi_password[] = "2H9RYCT4NG";

    IPAddress agent_ip(192, 168, 1, 65);
    set_microros_wifi_transports(wifi_ssid, wifi_password, agent_ip, 8888);

    delay(2000);
    Serial.print("ESP32 IP: ");
    Serial.println(WiFi.localIP());
    Serial.println("Transport set, initializing micro-ROS...");

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "diff_drive_node", "", &support);
}

void RosCommunication::subscriber_define() {
    rclc_subscription_init_default(
        &cmd_vel_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel");

    servo_msg.data.data     = servo_data;
    servo_msg.data.capacity = 2;
    servo_msg.data.size     = 2;

    rclc_subscription_init_default(
        &servo_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/servo/cmd");
}

void RosCommunication::publisher_define() {
    rclc_publisher_init_default(
        &debug_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/robot/debug");

    debug_msg.data.data     = debug_buf;
    debug_msg.data.capacity = sizeof(debug_buf);
    debug_msg.data.size     = 0;
}

void RosCommunication::executors_start() {
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(
        &executor, &cmd_vel_sub, &cmd_vel_msg,
        &RosCommunication::cmd_vel_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(
        &executor, &servo_sub, &servo_msg,
        &RosCommunication::servo_callback, ON_NEW_DATA);
    Serial.println("Executors started");
}

void RosCommunication::cmd_vel_callback(const void *msg_recv) {
    const geometry_msgs__msg__Twist *msg =
        (const geometry_msgs__msg__Twist *)msg_recv;

    float linear  = msg->linear.x;
    float angular = msg->angular.z;
    float leftVel  = linear - (angular * WHEEL_BASE / 2.0f);
    float rightVel = linear + (angular * WHEEL_BASE / 2.0f);

    targetRPM_L   = (leftVel  / (2.0f * PI * WHEEL_RADIUS)) * 60.0f;
    targetRPM_R   = (rightVel / (2.0f * PI * WHEEL_RADIUS)) * 60.0f;
    last_cmd_time = esp_timer_get_time();

    if (millis() - lastCbPrint >= 1000) {
        lastCbPrint = millis();
        Serial.print("cmd_vel RPM L:"); Serial.print(targetRPM_L);
        Serial.print(" R:");            Serial.println(targetRPM_R);
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

    Serial.print("Servo S1:"); Serial.print(angle1);
    Serial.print(" S2:");      Serial.println(angle2);
}

void RosCommunication::publish_debug() {
    static unsigned long lastDebugPub = 0;
    unsigned long now = millis();
    if (now - lastDebugPub < 200) return;
    lastDebugPub = now;

    int64_t wd_gap_ms = (esp_timer_get_time() - last_cmd_time) / 1000;

    int len = snprintf(debug_buf, sizeof(debug_buf),
        "[ESP32] tgt_L:%.1f tgt_R:%.1f | ip:%s rssi:%d | wd_gap:%lldms | uptime:%lus",
        (float)targetRPM_L, (float)targetRPM_R,
        WiFi.localIP().toString().c_str(),
        WiFi.RSSI(), wd_gap_ms, now / 1000);

    debug_msg.data.size = len;
    rcl_ret_t ret = rcl_publish(&debug_pub, &debug_msg, NULL);
    if (ret != RCL_RET_OK) Serial.println("WARN: debug publish failed");
}

void RosCommunication::start_receiving_msgs() {
    static unsigned long lastPrint = 0;
    static int spinCount = 0;
    spinCount++;

    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    publish_debug();

    if (millis() - lastPrint >= 5000) {
        lastPrint = millis();
        Serial.print("Executor alive spins: ");
        Serial.println(spinCount);
    }
}