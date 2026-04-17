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
        void initialize();           // one-time WiFi + transport setup
        void start_receiving_msgs(); // call from loop() — handles reconnection

        static void cmd_vel_callback(const void *msg_recv);
        static void servo_callback(const void *msg_recv);
        static void pid_gains_callback(const void *msg_recv);
        void publish_debug();

    private:
        enum State { WAITING_AGENT, AGENT_CONNECTED };
        State state_;

        bool create_entities();
        void destroy_entities();

        static unsigned long lastCbPrint;
};
