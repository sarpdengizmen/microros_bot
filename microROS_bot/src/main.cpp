#include "ros_communication.h"
#include "motor_control.h"
#include "shared_state.h"

RosCommunication ros_comm;

void ros_task(void *arg) {
    while (1) {
        ros_comm.start_receiving_msgs();
        vTaskDelay(1);
    }
}

void setup() {
    Serial.begin(115200);
    serial_mutex = xSemaphoreCreateMutex();

    motor_setup();
    servo_setup();
    ros_comm.initialize();

    // Core 0: ROS communication — co-located with WiFi stack
    xTaskCreatePinnedToCore(ros_task,           "ros",    8192, NULL,  5, NULL, 0);
    // Core 1: Motor control — dedicated real-time core
    xTaskCreatePinnedToCore(motor_control_task, "motors", 4096, NULL, 10, NULL, 1);
}

void loop() {
    vTaskDelay(portMAX_DELAY);  // loop() unused — both tasks run independently
}
