#pragma once
#include <Arduino.h>

// ── Motor Driver Pins (GPIO → DRV8833) ───────────────────────────────────────
#define MOTOR_R_IN1   26
#define MOTOR_R_IN2   25
#define MOTOR_L_IN1   14
#define MOTOR_L_IN2   27
#define DRV_SLEEP      4    // nSLEEP — LOW=sleep, HIGH=active

// ── Encoder Pins ─────────────────────────────────────────────────────────────
#define ENC_R_A       18
#define ENC_R_B       19
#define ENC_L_A       23
#define ENC_L_B       22

// ── Servo Pins ───────────────────────────────────────────────────────────────
#define SERVO_1_PIN      32
#define SERVO_2_PIN      33
#define PWM_CH_SERVO_1    4   // LEDC channels 0-3 taken by motors
#define PWM_CH_SERVO_2    5

// ── Servo Limits & Defaults ───────────────────────────────────────────────────
#define SERVO_1_MIN      10.0f
#define SERVO_1_MAX     160.0f
#define SERVO_1_DEFAULT  90.0f   // centre on boot

#define SERVO_2_MIN      20.0f
#define SERVO_2_MAX     180.0f
#define SERVO_2_DEFAULT 180.0f   // fully open on boot

// Pulse width — standard servo
#define SERVO_PULSE_MIN_US   500
#define SERVO_PULSE_MAX_US  2500

// ── PI Tuning ────────────────────────────────────────────────────────────────
const float COUNTS_PER_REV = 420.0f;
const float KP             = 2.0f;
const float KI             = 0.5f;
const float ALPHA_RPM      = 0.1f;
const float RAMP_RATE      = 20.0f;
const int   MIN_PWM        = 40;
const int   PID_INTERVAL   = 20;

// ── Function Declarations ────────────────────────────────────────────────────
void motor_setup();
void servo_setup();
void set_servo_1(float angle_deg);   // enforces SERVO_1 limits
void set_servo_2(float angle_deg);   // enforces SERVO_2 limits
void motors_wake();
void motors_sleep();
void setMotor_L(float val);
void setMotor_R(float val);
void motor_control_task(void *arg);