// shared_state.h
#pragma once
#include <Arduino.h>

extern volatile float targetRPM_L;
extern volatile float targetRPM_R;
extern volatile int64_t last_cmd_time;

// ── Diff Drive Geometry — measure your robot ─────────────────────────────────
const float WHEEL_RADIUS   = 0.033f;  // meters
const float WHEEL_BASE     = 0.16f;   // meters