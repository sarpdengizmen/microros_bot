// shared_state.h
#pragma once
#include <Arduino.h>
#include <freertos/semphr.h>

extern volatile float     targetRPM_L;
extern volatile float     targetRPM_R;
extern volatile uint32_t  last_cmd_time;  // millis() — 32-bit, atomic on Xtensa

// Written by motor task, read by ROS task for /motor_debug
extern volatile float     actual_rpm_L;
extern volatile float     actual_rpm_R;
extern volatile float     pid_out_L;
extern volatile float     pid_out_R;

// Live-tunable gains — updated via /pid_gains topic as Float32[3]: [kp, ki, kf]
// KF is the feed-forward gain (PWM per RPM): preloads motor drive for the target,
// so the PI only corrects residual error. Tune: at your robot's free-run max RPM,
// KF ≈ 255 / max_RPM. Start ~1.5 and increase until you see overshoot.
extern volatile float     KP_live;
extern volatile float     KI_live;
extern volatile float     KF_live;

extern SemaphoreHandle_t  serial_mutex;

// ── Diff Drive Geometry — measure your robot ─────────────────────────────────
const float WHEEL_RADIUS   = 0.033f;  // meters
const float WHEEL_BASE     = 0.16f;   // meters
