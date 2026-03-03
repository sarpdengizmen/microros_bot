#pragma once

#include <micro_ros_platformio.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float32_multi_array.h>

class RosCommunication {
    public:
        RosCommunication();
        void initialize();
        void subscriber_define();
        void publisher_define();
        static void cmd_vel_callback(const void *msg_recv);
        static void servo_callback(const void *msg_recv);
        void start_receiving_msgs();
        void executors_start();
        void publish_debug();

    private:
        static unsigned long lastCbPrint;
};