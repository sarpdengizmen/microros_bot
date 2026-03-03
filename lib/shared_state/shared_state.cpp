// shared_state.cpp
#include "shared_state.h"

volatile float targetRPM_L = 0.0;
volatile float targetRPM_R = 0.0;
volatile int64_t last_cmd_time = 0;