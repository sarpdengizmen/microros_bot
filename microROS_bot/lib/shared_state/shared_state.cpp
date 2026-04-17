// shared_state.cpp
#include "shared_state.h"

volatile float    targetRPM_L   = 0.0f;
volatile float    targetRPM_R   = 0.0f;
volatile uint32_t last_cmd_time = 0;

volatile float    actual_rpm_L  = 0.0f;
volatile float    actual_rpm_R  = 0.0f;
volatile float    pid_out_L     = 0.0f;
volatile float    pid_out_R     = 0.0f;

volatile float    KP_live       = 3.0f;
volatile float    KI_live       = 1.0f;
volatile float    KF_live       = 0.9f;  // feed-forward: PWM/RPM — tune to your motor

SemaphoreHandle_t serial_mutex  = nullptr;
