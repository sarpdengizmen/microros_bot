#include "ros_communication.h"
#include "motor_control.h"

RosCommunication ros_comm;

void setup() {
    motor_setup();
    servo_setup();
    ros_comm.initialize();
    ros_comm.subscriber_define();
    ros_comm.publisher_define();
    ros_comm.executors_start();

    xTaskCreatePinnedToCore(motor_control_task, "motors", 4096, NULL, 10, NULL, 1);
}

void loop() {
    ros_comm.start_receiving_msgs();
}