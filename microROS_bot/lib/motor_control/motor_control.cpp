#include "motor_control.h"
#include "shared_state.h"

// ── Encoder counts — modified in ISR ─────────────────────────────────────────
volatile long pos_L = 0;
volatile long pos_R = 0;

void IRAM_ATTR readEncoder_L() {
    if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) pos_L++;
    else pos_L--;
}
void IRAM_ATTR readEncoder_R() {
    if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B)) pos_R++;
    else pos_R--;
}

// ── Slow decay — ch0=R_IN1, ch1=R_IN2, ch2=L_IN1, ch3=L_IN2 ─────────────────
void setMotor_L(float val) {
    int output = constrain((int)val, -255, 255);
    if (abs(output) < MIN_PWM) {
        ledcWrite(2, 0); ledcWrite(3, 0);  // coast — avoids abrupt braking oscillation
        return;
    }
    if (output >= 0) {
        ledcWrite(2, 255); ledcWrite(3, 255 - output);
    } else {
        ledcWrite(2, 255 - abs(output)); ledcWrite(3, 255);
    }
}

void setMotor_R(float val) {
    int output = constrain((int)val, -255, 255);
    if (abs(output) < MIN_PWM) {
        ledcWrite(0, 0); ledcWrite(1, 0);  // coast — avoids abrupt braking oscillation
        return;
    }
    if (output >= 0) {
        ledcWrite(0, 255); ledcWrite(1, 255 - output);
    } else {
        ledcWrite(0, 255 - abs(output)); ledcWrite(1, 255);
    }
}

void motors_wake()  { digitalWrite(DRV_SLEEP, HIGH); }
void motors_sleep() { digitalWrite(DRV_SLEEP, LOW);  }

// ── Servo helpers ─────────────────────────────────────────────────────────────
// 50Hz 16-bit: period=20000us, 65535 ticks
// 500us  → 1638 ticks (0 deg)
// 2500us → 8191 ticks (180 deg)
static uint32_t angleToTicks(float angle_deg) {
    float pulse_us = SERVO_PULSE_MIN_US +
                     (angle_deg / 180.0f) * (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US);
    return (uint32_t)((pulse_us / 20000.0f) * 65535);
}

// Servo 1 — limits 10-160deg, default 90deg
void set_servo_1(float angle_deg) {
    angle_deg = constrain(angle_deg, SERVO_1_MIN, SERVO_1_MAX);
    ledcWrite(PWM_CH_SERVO_1, angleToTicks(angle_deg));
}

// Servo 2 — limits 20-180deg, default 180deg
void set_servo_2(float angle_deg) {
    angle_deg = constrain(angle_deg, SERVO_2_MIN, SERVO_2_MAX);
    ledcWrite(PWM_CH_SERVO_2, angleToTicks(angle_deg));
}

void servo_setup() {
    ledcSetup(PWM_CH_SERVO_1, 50, 16);
    ledcSetup(PWM_CH_SERVO_2, 50, 16);
    ledcAttachPin(SERVO_1_PIN, PWM_CH_SERVO_1);
    ledcAttachPin(SERVO_2_PIN, PWM_CH_SERVO_2);

    set_servo_1(SERVO_1_DEFAULT);  // 90deg
    set_servo_2(SERVO_2_DEFAULT);  // 180deg

    Serial.print("Servo 1 → "); Serial.print(SERVO_1_DEFAULT);
    Serial.print("deg  limits("); Serial.print(SERVO_1_MIN);
    Serial.print("-"); Serial.print(SERVO_1_MAX); Serial.println("deg)");

    Serial.print("Servo 2 → "); Serial.print(SERVO_2_DEFAULT);
    Serial.print("deg  limits("); Serial.print(SERVO_2_MIN);
    Serial.print("-"); Serial.print(SERVO_2_MAX); Serial.println("deg)");
}

// ── Motor setup ───────────────────────────────────────────────────────────────
void motor_setup() {
    pinMode(DRV_SLEEP, OUTPUT);
    motors_wake();
    Serial.print("DRV_SLEEP pin state: ");
    Serial.println(digitalRead(DRV_SLEEP));

    ledcSetup(0, 10000, 8); ledcAttachPin(MOTOR_R_IN1, 0);
    ledcSetup(1, 10000, 8); ledcAttachPin(MOTOR_R_IN2, 1);
    ledcSetup(2, 10000, 8); ledcAttachPin(MOTOR_L_IN1, 2);
    ledcSetup(3, 10000, 8); ledcAttachPin(MOTOR_L_IN2, 3);

    ledcWrite(0, 255); ledcWrite(1, 255);
    ledcWrite(2, 255); ledcWrite(3, 255);

    pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
    pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_L_A), readEncoder_L, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_R_A), readEncoder_R, CHANGE);

    last_cmd_time = millis();

    Serial.println("Motor setup done");
}

// ── Motor control task ────────────────────────────────────────────────────────
void motor_control_task(void *arg) {
    float filteredRPM_L = 0, integral_L = 0;
    float filteredRPM_R = 0, integral_R = 0;
    float rampedTarget_L = 0, rampedTarget_R = 0;

    unsigned long lastPIDTime   = millis();
    unsigned long lastRampTime  = millis();
    unsigned long lastPrintTime = millis();

    while(1) {
        unsigned long now = millis();

        if (now - lastRampTime >= 20) {
            lastRampTime = now;
            float tL = targetRPM_L;
            float tR = targetRPM_R;

            if (rampedTarget_L < tL)
                rampedTarget_L = min(rampedTarget_L + RAMP_RATE, tL);
            else if (rampedTarget_L > tL)
                rampedTarget_L = max(rampedTarget_L - RAMP_RATE, tL);

            if (rampedTarget_R < tR)
                rampedTarget_R = min(rampedTarget_R + RAMP_RATE, tR);
            else if (rampedTarget_R > tR)
                rampedTarget_R = max(rampedTarget_R - RAMP_RATE, tR);
        }

        if (now - lastPIDTime >= PID_INTERVAL) {
            float dt = (now - lastPIDTime) / 1000.0f;
            lastPIDTime = now;

            uint32_t gap_ms = millis() - last_cmd_time;
            if (gap_ms > 2000) {
                targetRPM_L    = 0.0f; targetRPM_R    = 0.0f;
                rampedTarget_L = 0.0f; rampedTarget_R = 0.0f;
                integral_L     = 0.0f; integral_R     = 0.0f;
            }

            noInterrupts();
            long count_L = pos_L; pos_L = 0;
            long count_R = pos_R; pos_R = 0;
            interrupts();

            float rawRPM_L = (count_L / COUNTS_PER_REV) / dt * 60.0f;
            float rawRPM_R = (count_R / COUNTS_PER_REV) / dt * 60.0f;

            filteredRPM_L = (ALPHA_RPM * rawRPM_L) + ((1.0f - ALPHA_RPM) * filteredRPM_L);
            filteredRPM_R = (ALPHA_RPM * rawRPM_R) + ((1.0f - ALPHA_RPM) * filteredRPM_R);

            float error_L = rampedTarget_L - filteredRPM_L;
            float error_R = rampedTarget_R - filteredRPM_R;

            if (abs(error_L) > 5.0f)
                integral_L = constrain(integral_L + error_L * dt, -100.0f, 100.0f);
            else
                integral_L *= 0.95f;

            if (abs(error_R) > 5.0f)
                integral_R = constrain(integral_R + error_R * dt, -100.0f, 100.0f);
            else
                integral_R *= 0.95f;

            float pidOut_L = (KF_live * rampedTarget_L) + (KP_live * error_L) + (KI_live * integral_L);
            float pidOut_R = (KF_live * rampedTarget_R) + (KP_live * error_R) + (KI_live * integral_R);

            actual_rpm_L = filteredRPM_L;
            actual_rpm_R = filteredRPM_R;
            pid_out_L    = pidOut_L;
            pid_out_R    = pidOut_R;

            setMotor_L(pidOut_L);
            setMotor_R(pidOut_R);

            if (now - lastPrintTime >= 2000) {
                lastPrintTime = now;
                if (xSemaphoreTake(serial_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    Serial.println("── Motor ──");
                    Serial.print("  Target  L:"); Serial.print(targetRPM_L);
                    Serial.print(" R:");          Serial.println(targetRPM_R);
                    Serial.print("  Ramped  L:"); Serial.print(rampedTarget_L);
                    Serial.print(" R:");          Serial.println(rampedTarget_R);
                    Serial.print("  Filt    L:"); Serial.print(filteredRPM_L);
                    Serial.print(" R:");          Serial.println(filteredRPM_R);
                    Serial.print("  PIDOut  L:"); Serial.print(pidOut_L);
                    Serial.print(" R:");          Serial.println(pidOut_R);
                    Serial.print("  WD gap ms:"); Serial.println(gap_ms);
                    xSemaphoreGive(serial_mutex);
                }
            }
        }

        vTaskDelay(1);
    }
}