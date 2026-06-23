/*
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║            AisleBot ESP32 Motor Controller v2.0                 ║
 * ║                                                                  ║
 * ║  PID + Feedforward | Hardware PCNT | 3-Speed Gear System        ║
 * ║  Asymmetric Mecanum IK | Dual-Core FreeRTOS                     ║
 * ║                                                                  ║
 * ║  Aritra Das (25D0074) — IIT Bombay — Prof. Ambarish Kunwar      ║
 * ╚══════════════════════════════════════════════════════════════════╝
 *
 * ARCHITECTURE:
 *   Core 0: WiFi AP + WebSocket + HTTP server + Serial parser + Telemetry
 *   Core 1: PID loop at 50 Hz — dedicated, never interrupted
 *
 * COMMAND SOURCES (arbitrated on Core 0):
 *   1. WiFi Joystick (phone)  — body velocities (vx,vy,wz); ESP32 runs IK
 *   2. USB Serial (Pi / ROS2) — wheel velocities in rad/s via <V,...>
 *   WiFi overrides Serial while joystick is active (≤ 200 ms timeout).
 *   Release → Serial resumes within 200 ms.
 *
 * CORRECTIONS v2.0:
 *   ✓ E-STOP latches — no auto-clear. Resume with <E1>.
 *   ✓ WHEEL_RADIUS = 0.0762 m (DekuPro 6-inch: 152.4 mm OD / 2)
 *   ✓ Asymmetric IK: K_OUTER(FR,RL) ≠ K_INNER(FL,RR)
 *   ✓ delay() removed from FreeRTOS tasks
 *   ✓ Unused mutex removed
 *   ✓ Graphs removed from joystick page
 *   ✓ 3-speed gear selector: SLOW / NORMAL / FAST
 *
 * LIBRARIES (Arduino Library Manager):
 *   WebSockets  by Markus Sattler (Links2004)
 *   ArduinoJson by Benoit Blanchon
 *
 * BOARD SETTINGS:
 *   Board: ESP32 Dev Module | CPU: 240 MHz (WiFi/BT) | Flash: 4 MB
 *   Partition: Default 4 MB with spiffs | PSRAM: Disabled
 *   Upload Speed: 921600
 */

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include "driver/pcnt.h"

// ═══════════════════════════════════════════════════════════════════
// 1. PIN DEFINITIONS
//    RIGHT side  → Motor driver outputs  (3.3 V, no level shifting)
//    LEFT  side  → Encoder inputs        (5 V via TXS0108E)
// ═══════════════════════════════════════════════════════════════════

// Driver 1 — Front motors (FR = Ch1, FL = Ch2)
#define PWM1_PIN   4    // G4  → Driver 1 PWM1  (FR)
#define DIR1_PIN  16    // G16 → Driver 1 DIR1  (FR)
#define PWM2_PIN  17    // G17 → Driver 1 PWM2  (FL)
#define DIR2_PIN  18    // G18 → Driver 1 DIR2  (FL)

// Driver 2 — Rear motors (RR = Ch1, RL = Ch2)
#define PWM3_PIN  19    // G19 → Driver 2 PWM1  (RR)
#define DIR3_PIN  21    // G21 → Driver 2 DIR1  (RR)
#define PWM4_PIN  22    // G22 → Driver 2 PWM2  (RL)
#define DIR4_PIN  23    // G23 → Driver 2 DIR2  (RL)

// Encoders — via TXS0108E level shifter (5 V → 3.3 V)
#define ENC1_A_PIN  36  // SP  → FR Encoder A
#define ENC1_B_PIN  39  // SN  → FR Encoder B
#define ENC2_A_PIN  34  // G34 → FL Encoder A
#define ENC2_B_PIN  35  // G35 → FL Encoder B
#define ENC3_A_PIN  32  // G32 → RR Encoder A
#define ENC3_B_PIN  33  // G33 → RR Encoder B
#define ENC4_A_PIN  25  // G25 → RL Encoder A
#define ENC4_B_PIN  26  // G26 → RL Encoder B

// ═══════════════════════════════════════════════════════════════════
// 2. MOTOR & ROBOT CONSTANTS
// ═══════════════════════════════════════════════════════════════════

#define FR  0
#define FL  1
#define RR  2
#define RL  3
#define NUM_MOTORS 4

const char* MOTOR_NAMES[NUM_MOTORS] = {"FR", "FL", "RR", "RL"};

const uint8_t PWM_PINS[NUM_MOTORS]   = {PWM1_PIN, PWM2_PIN, PWM3_PIN, PWM4_PIN};
const uint8_t DIR_PINS[NUM_MOTORS]   = {DIR1_PIN, DIR2_PIN, DIR3_PIN, DIR4_PIN};
const uint8_t ENC_A_PINS[NUM_MOTORS] = {ENC1_A_PIN, ENC2_A_PIN, ENC3_A_PIN, ENC4_A_PIN};
const uint8_t ENC_B_PINS[NUM_MOTORS] = {ENC1_B_PIN, ENC2_B_PIN, ENC3_B_PIN, ENC4_B_PIN};

// Direction signs — right-side motors face opposite direction to left.
// MOTOR_DIR_SIGN and ENC_DIR_SIGN MUST be identical per motor.
// If they disagree → PID sees positive feedback → runaway.
const int8_t MOTOR_DIR_SIGN[NUM_MOTORS] = {-1, +1, -1, +1}; // FR,FL,RR,RL
const int8_t ENC_DIR_SIGN[NUM_MOTORS]   = {-1, +1, -1, +1}; // Must match above

const pcnt_unit_t PCNT_UNITS[NUM_MOTORS] = {
    PCNT_UNIT_0, PCNT_UNIT_1, PCNT_UNIT_2, PCNT_UNIT_3
};

// ── Encoder ─────────────────────────────────────────────────────────
// RMCS-2086: 500 lines × 4 (full quad PCNT) × 47 (gear ratio) = 93 132
const float ENCODER_CPR = 93132.0f;

// ── Wheel ────────────────────────────────────────────────────────────
// DekuPro 6-inch SR Mecanum: OD = 152.4 mm → radius = 76.2 mm = 0.0762 m
const float WHEEL_RADIUS = 0.0762f;

// ── Asymmetric robot geometry (from SolidWorks) ──────────────────────
// l1 = 0.403 m  outer pair  (FR, RL)
// l2 = 0.333 m  inner pair  (FL, RR)
// d  = 0.15769 m  half-track width
const float ROBOT_L1 = 0.403f;
const float ROBOT_L2 = 0.333f;
const float ROBOT_D  = 0.15769f;
const float K_OUTER  = ROBOT_L1 + ROBOT_D;  // 0.56069 m  (FR, RL)
const float K_INNER  = ROBOT_L2 + ROBOT_D;  // 0.49069 m  (FL, RR)

// ── Speed limits — hardware maximums ─────────────────────────────────
// Joystick gear multipliers (0.25 / 0.60 / 1.00) are applied client-side
// before these values, so full-stick never exceeds hardware limits.
const float MAX_WHEEL_SPEED   = 6.28f;   // rad/s  ≙ 60 RPM rated maximum
const float MAX_LINEAR_SPEED  = 0.48f;   // m/s    ≙ MAX_WHEEL_SPEED × WHEEL_RADIUS
const float MAX_ANGULAR_SPEED = 1.0f;    // rad/s  yaw rate at joystick ±1

// ── PWM ──────────────────────────────────────────────────────────────
const int PWM_MAX           = 255;
const int MIN_PWM_THRESHOLD = 15;   // Below this → motor hums but does not spin
const int PWM_FREQUENCY     = 5000; // 5 kHz — above audible range
const int PWM_RESOLUTION    = 8;    // 8-bit (0–255)

// ═══════════════════════════════════════════════════════════════════
// 3. PID CONFIGURATION
// ═══════════════════════════════════════════════════════════════════

const float PID_LOOP_HZ          = 50.0f;
const float PID_LOOP_INTERVAL_MS = 1000.0f / PID_LOOP_HZ;  // 20 ms
const float PID_DT               = 1.0f   / PID_LOOP_HZ;   // 0.02 s

// Gains — apply to all motors. Tune live with <G,Kp,Ki,Kd>.
float Kp = 50.0f;
float Ki = 30.0f;
float Kd =  3.0f;  // Viable on ESP32 thanks to hardware PCNT (no ISR noise)

// Per-motor feedforward: PWM per (rad/s) — calibrated in air.
// Recalibrate on ground after deadband measurement.
float Kff[NUM_MOTORS] = {42.1f, 40.2f, 43.7f, 47.9f};  // FR, FL, RR, RL

const float INTEGRAL_MAX   = 200.0f;  // Anti-windup clamp
const float D_FILTER_ALPHA = 0.3f;    // Derivative EMA (0 = heavy, 1 = none)
const float VEL_FILTER_ALPHA = 0.5f;  // Velocity measurement EMA

// ═══════════════════════════════════════════════════════════════════
// 4. TIMING & SAFETY
// ═══════════════════════════════════════════════════════════════════

const unsigned long WATCHDOG_TIMEOUT_MS  = 1000;  // No command → stop motors
const unsigned long WIFI_CMD_TIMEOUT_MS  = 200;   // WiFi stale after 200 ms
const unsigned long TELEMETRY_INTERVAL_MS = 100;  // 10 Hz serial CSV
const unsigned long SERIAL_BAUD = 921600;

// ═══════════════════════════════════════════════════════════════════
// 5. WiFi
// ═══════════════════════════════════════════════════════════════════

const char* WIFI_SSID = "AisleBot-Control";
const char* WIFI_PASS = "aislebot123";
const int   WEBSOCKET_PORT = 81;
const int   WEBSERVER_PORT = 80;

// ═══════════════════════════════════════════════════════════════════
// 6. DATA STRUCTURES
// ═══════════════════════════════════════════════════════════════════

struct MotorState {
    float target_velocity;     // rad/s
    float actual_velocity;     // rad/s (filtered)
    float raw_velocity;        // rad/s (unfiltered, debug)
    float error;
    float integral;
    float prev_error;
    float filtered_derivative;
    int   pwm_output;          // -255 … +255
    float ff_output;
    float pid_output;
};

MotorState motors[NUM_MOTORS];

enum CommandSource { SRC_NONE, SRC_SERIAL, SRC_WIFI };

volatile CommandSource  active_source      = SRC_NONE;
volatile unsigned long  last_serial_cmd_ms = 0;
volatile unsigned long  last_wifi_cmd_ms   = 0;
volatile unsigned long  last_any_cmd_ms    = 0;

volatile bool motors_enabled  = true;
volatile bool watchdog_enabled = true;
volatile bool serial_logging  = false;
volatile bool estop_active    = false;

// WiFi joystick body velocities (written Core 0, read Core 1)
volatile float wifi_vx = 0.0f;
volatile float wifi_vy = 0.0f;
volatile float wifi_wz = 0.0f;

WebServer        webServer(WEBSERVER_PORT);
WebSocketsServer wsServer(WEBSOCKET_PORT);

// ═══════════════════════════════════════════════════════════════════
// 7. PCNT ENCODER SETUP — full quadrature, both channels, all edges
// ═══════════════════════════════════════════════════════════════════

void setupPCNT(int idx) {
    pcnt_unit_t unit = PCNT_UNITS[idx];
    uint8_t     pinA = ENC_A_PINS[idx];
    uint8_t     pinB = ENC_B_PINS[idx];

    // Channel 0: count A edges, direction from B
    pcnt_config_t cfg0 = {};
    cfg0.pulse_gpio_num = pinA;
    cfg0.ctrl_gpio_num  = pinB;
    cfg0.channel        = PCNT_CHANNEL_0;
    cfg0.unit           = unit;
    cfg0.pos_mode       = PCNT_COUNT_INC;
    cfg0.neg_mode       = PCNT_COUNT_DEC;
    cfg0.lctrl_mode     = PCNT_MODE_REVERSE;
    cfg0.hctrl_mode     = PCNT_MODE_KEEP;
    cfg0.counter_h_lim  =  32767;
    cfg0.counter_l_lim  = -32768;
    pcnt_unit_config(&cfg0);

    // Channel 1: count B edges, direction from A
    pcnt_config_t cfg1 = {};
    cfg1.pulse_gpio_num = pinB;
    cfg1.ctrl_gpio_num  = pinA;
    cfg1.channel        = PCNT_CHANNEL_1;
    cfg1.unit           = unit;
    cfg1.pos_mode       = PCNT_COUNT_DEC;
    cfg1.neg_mode       = PCNT_COUNT_INC;
    cfg1.lctrl_mode     = PCNT_MODE_REVERSE;
    cfg1.hctrl_mode     = PCNT_MODE_KEEP;
    cfg1.counter_h_lim  =  32767;
    cfg1.counter_l_lim  = -32768;
    pcnt_unit_config(&cfg1);

    pcnt_set_filter_value(unit, 100);
    pcnt_filter_enable(unit);
    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);
    pcnt_counter_resume(unit);
}

// Returns encoder delta since last call, sign-corrected for motor direction.
int32_t readEncoderDelta(int idx) {
    int16_t count = 0;
    pcnt_get_counter_value(PCNT_UNITS[idx], &count);
    pcnt_counter_clear(PCNT_UNITS[idx]);
    return (int32_t)count * ENC_DIR_SIGN[idx];
}

// ═══════════════════════════════════════════════════════════════════
// 8. MOTOR OUTPUT
// ═══════════════════════════════════════════════════════════════════

void setupMotorPins() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        pinMode(DIR_PINS[i], OUTPUT);
        digitalWrite(DIR_PINS[i], LOW);
        ledcAttach(PWM_PINS[i], PWM_FREQUENCY, PWM_RESOLUTION);
        ledcWrite(PWM_PINS[i], 0);
    }
}

void setMotorOutput(int idx, int pwm_value) {
    int  adjusted = pwm_value * MOTOR_DIR_SIGN[idx];
    bool reverse  = (adjusted < 0);
    int  abs_pwm  = abs(adjusted);

    if (abs_pwm < MIN_PWM_THRESHOLD) abs_pwm = 0;
    if (abs_pwm > PWM_MAX)           abs_pwm = PWM_MAX;

    digitalWrite(DIR_PINS[idx], reverse ? HIGH : LOW);
    ledcWrite(PWM_PINS[idx], abs_pwm);
}

void stopAllMotors() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].target_velocity    = 0;
        motors[i].integral           = 0;
        motors[i].prev_error         = 0;
        motors[i].filtered_derivative = 0;
        motors[i].pwm_output         = 0;
        setMotorOutput(i, 0);
    }
}

// ═══════════════════════════════════════════════════════════════════
// 9. PID + FEEDFORWARD
// ═══════════════════════════════════════════════════════════════════

void computePID(int idx) {
    MotorState& m = motors[idx];

    // Feedforward — provides ~90 % of required PWM instantly
    m.ff_output = Kff[idx] * m.target_velocity;

    // Proportional
    m.error     = m.target_velocity - m.actual_velocity;
    float p_term = Kp * m.error;

    // Integral with anti-windup + zero-velocity clear
    m.integral += m.error * PID_DT;
    m.integral  = constrain(m.integral, -INTEGRAL_MAX, INTEGRAL_MAX);
    if (fabsf(m.target_velocity) < 0.01f && fabsf(m.actual_velocity) < 0.1f) {
        m.integral = 0.0f;
    }
    float i_term = Ki * m.integral;

    // Derivative with low-pass EMA
    float raw_d = (m.error - m.prev_error) / PID_DT;
    m.filtered_derivative = D_FILTER_ALPHA * raw_d +
                            (1.0f - D_FILTER_ALPHA) * m.filtered_derivative;
    float d_term = Kd * m.filtered_derivative;
    m.prev_error = m.error;

    // Total output
    m.pid_output = p_term + i_term + d_term;
    float total  = m.ff_output + m.pid_output;

    // Force zero if target is zero (prevents integral creep at standstill)
    if (fabsf(m.target_velocity) < 0.01f) total = 0.0f;

    m.pwm_output = (int)constrain(roundf(total), -PWM_MAX, PWM_MAX);
}

// ═══════════════════════════════════════════════════════════════════
// 10. ASYMMETRIC MECANUM INVERSE KINEMATICS
//
//     vx = forward  (+)    vy = right strafe (+)    wz = CCW (+)
//
//     ωFR = (vx + vy + wz·K_OUTER) / R    outer pair: FR, RL
//     ωFL = (vx − vy − wz·K_INNER) / R    inner pair: FL, RR
//     ωRR = (vx − vy + wz·K_INNER) / R
//     ωRL = (vx + vy − wz·K_OUTER) / R
//
//     K_OUTER = l1 + d = 0.403 + 0.15769 = 0.56069 m
//     K_INNER = l2 + d = 0.333 + 0.15769 = 0.49069 m
// ═══════════════════════════════════════════════════════════════════

void mecanumIK(float vx, float vy, float wz, float* ws) {
    ws[FR] = ( vx + vy + wz * K_OUTER) / WHEEL_RADIUS;
    ws[FL] = ( vx - vy - wz * K_INNER) / WHEEL_RADIUS;
    ws[RR] = ( vx - vy + wz * K_INNER) / WHEEL_RADIUS;
    ws[RL] = ( vx + vy - wz * K_OUTER) / WHEEL_RADIUS;

    // Proportional scaling — keeps direction if any wheel exceeds limit
    float max_spd = 0.0f;
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (fabsf(ws[i]) > max_spd) max_spd = fabsf(ws[i]);
    }
    if (max_spd > MAX_WHEEL_SPEED) {
        float scale = MAX_WHEEL_SPEED / max_spd;
        for (int i = 0; i < NUM_MOTORS; i++) ws[i] *= scale;
    }
}

// ═══════════════════════════════════════════════════════════════════
// 11. CORE 1 — PID TASK (50 Hz, hard real-time)
// ═══════════════════════════════════════════════════════════════════

void pidControlTask(void* /*param*/) {
    TickType_t       xLastWake = xTaskGetTickCount();
    const TickType_t xPeriod   = pdMS_TO_TICKS((int)PID_LOOP_INTERVAL_MS);

    while (true) {
        // ── 1. Read encoders & update velocity estimate ───────────────
        for (int i = 0; i < NUM_MOTORS; i++) {
            int32_t delta    = readEncoderDelta(i);
            float   raw_vel  = ((float)delta / ENCODER_CPR) * (2.0f * PI) / PID_DT;
            motors[i].raw_velocity = raw_vel;
            motors[i].actual_velocity =
                VEL_FILTER_ALPHA * raw_vel +
                (1.0f - VEL_FILTER_ALPHA) * motors[i].actual_velocity;
        }

        // ── 2. E-STOP / disabled guard ─────────────────────────────────
        if (estop_active || !motors_enabled) {
            for (int i = 0; i < NUM_MOTORS; i++) {
                motors[i].target_velocity    = 0;
                motors[i].integral           = 0;
                motors[i].pwm_output         = 0;
                setMotorOutput(i, 0);
            }
            vTaskDelayUntil(&xLastWake, xPeriod);
            continue;
        }

        // ── 3. Watchdog ────────────────────────────────────────────────
        if (watchdog_enabled &&
            (millis() - last_any_cmd_ms > WATCHDOG_TIMEOUT_MS)) {
            for (int i = 0; i < NUM_MOTORS; i++) {
                motors[i].target_velocity = 0;
                motors[i].integral        = 0;
            }
        }

        // ── 4. Apply WiFi joystick targets (if active) ─────────────────
        if (millis() - last_wifi_cmd_ms < WIFI_CMD_TIMEOUT_MS) {
            float ws[NUM_MOTORS];
            mecanumIK(wifi_vx, wifi_vy, wifi_wz, ws);
            for (int i = 0; i < NUM_MOTORS; i++) {
                motors[i].target_velocity = ws[i];
            }
        }

        // ── 5. PID + output ────────────────────────────────────────────
        for (int i = 0; i < NUM_MOTORS; i++) {
            computePID(i);
            setMotorOutput(i, motors[i].pwm_output);
        }

        vTaskDelayUntil(&xLastWake, xPeriod);
    }
}

// ═══════════════════════════════════════════════════════════════════
// 12. SERIAL COMMAND PARSER
//     All commands wrapped in < >
//     e.g.  <V,1.5,1.5,1.5,1.5>   or   <S>
// ═══════════════════════════════════════════════════════════════════

char serialBuffer[256];
int  serialBufIdx   = 0;
bool serialReceiving = false;

void processSerialCommand(const char* cmd) {
    last_serial_cmd_ms = millis();
    last_any_cmd_ms    = millis();

    switch (cmd[0]) {

        case 'V': { // <V,fr,fl,rr,rl>  set wheel velocities rad/s via PID
            float v[4];
            if (sscanf(cmd, "V,%f,%f,%f,%f", &v[0],&v[1],&v[2],&v[3]) == 4) {
                for (int i = 0; i < NUM_MOTORS; i++) {
                    motors[i].target_velocity =
                        constrain(v[i], -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);
                }
                Serial.printf("[OK,V,%.2f,%.2f,%.2f,%.2f]\n",
                              v[0],v[1],v[2],v[3]);
            } else Serial.println("[ERR,BAD_V]");
            break;
        }

        case 'M': { // <M,fr,fl,rr,rl>  direct PWM, bypasses PID
            int p[4];
            if (sscanf(cmd, "M,%d,%d,%d,%d", &p[0],&p[1],&p[2],&p[3]) == 4) {
                for (int i = 0; i < NUM_MOTORS; i++) {
                    p[i] = constrain(p[i], -PWM_MAX, PWM_MAX);
                    motors[i].target_velocity = 0;
                    motors[i].integral        = 0;
                    motors[i].pwm_output      = p[i];
                    setMotorOutput(i, p[i]);
                }
                Serial.printf("[OK,M,%d,%d,%d,%d]\n", p[0],p[1],p[2],p[3]);
            } else Serial.println("[ERR,BAD_M]");
            break;
        }

        case 'T': { // <T,idx,vel>  test single motor
            int idx; float vel;
            if (sscanf(cmd, "T,%d,%f", &idx, &vel) == 2 &&
                idx >= 0 && idx < NUM_MOTORS) {
                motors[idx].target_velocity =
                    constrain(vel, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);
                Serial.printf("[OK,T,%d,%.2f]\n", idx, vel);
            } else Serial.println("[ERR,BAD_T]");
            break;
        }

        case 'S': // <S>  emergency stop — LATCHES, clear with <E1>
            estop_active = true;
            stopAllMotors();
            Serial.println("[OK,ESTOP_LATCHED]");
            break;

        case 'P': // <P>  ping
            Serial.println("[PONG]");
            break;

        case 'I': // <I>  system info
            Serial.println("[INFO,AisleBot_ESP32_v2.0]");
            Serial.printf("[GAINS,Kp=%.1f,Ki=%.1f,Kd=%.1f]\n", Kp, Ki, Kd);
            Serial.printf("[FF,FR=%.1f,FL=%.1f,RR=%.1f,RL=%.1f]\n",
                          Kff[FR],Kff[FL],Kff[RR],Kff[RL]);
            Serial.printf("[GEOM,R=%.4f,K_OUT=%.5f,K_INN=%.5f]\n",
                          WHEEL_RADIUS, K_OUTER, K_INNER);
            Serial.printf("[LIMITS,V_max=%.2f,Lin=%.2f,Ang=%.2f]\n",
                          MAX_WHEEL_SPEED, MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED);
            break;

        case '?': // <?>  live state
            Serial.printf("[STATE,EN=%d,WD=%d,ESTOP=%d]\n",
                          motors_enabled, watchdog_enabled, estop_active);
            for (int i = 0; i < NUM_MOTORS; i++) {
                Serial.printf("[%s,tgt=%.2f,act=%.2f,pwm=%d,err=%.3f]\n",
                    MOTOR_NAMES[i],
                    motors[i].target_velocity,
                    motors[i].actual_velocity,
                    motors[i].pwm_output,
                    motors[i].error);
            }
            break;

        case 'G': { // <G,Kp,Ki,Kd>  set PID gains live
            float kp,ki,kd;
            if (sscanf(cmd, "G,%f,%f,%f", &kp,&ki,&kd) == 3) {
                Kp=kp; Ki=ki; Kd=kd;
                for (int i = 0; i < NUM_MOTORS; i++) motors[i].integral = 0;
                Serial.printf("[OK,GAINS,%.1f,%.1f,%.1f]\n", Kp,Ki,Kd);
            } else Serial.println("[ERR,BAD_G]");
            break;
        }

        case 'F': { // <F,fr,fl,rr,rl>  set feedforward gains live
            float f[4];
            if (sscanf(cmd, "F,%f,%f,%f,%f", &f[0],&f[1],&f[2],&f[3]) == 4) {
                for (int i = 0; i < NUM_MOTORS; i++) Kff[i] = f[i];
                Serial.printf("[OK,FF,%.1f,%.1f,%.1f,%.1f]\n",
                              f[0],f[1],f[2],f[3]);
            } else Serial.println("[ERR,BAD_F]");
            break;
        }

        case 'E': // <E1> enable / <E0> disable
            if (cmd[1] == '1') {
                motors_enabled = true;
                estop_active   = false;  // E1 also clears E-STOP latch
                Serial.println("[OK,ENABLED]");
            } else {
                motors_enabled = false;
                stopAllMotors();
                Serial.println("[OK,DISABLED]");
            }
            break;

        case 'W': // <W1> / <W0>  watchdog on/off
            watchdog_enabled = (cmd[1] == '1');
            Serial.printf("[OK,WDOG=%d]\n", watchdog_enabled);
            break;

        case 'L': // <L1> / <L0>  serial CSV telemetry
            serial_logging = (cmd[1] == '1');
            Serial.printf("[OK,LOG=%d]\n", serial_logging);
            break;

        case 'H': // <H>  help
            Serial.println("=== AisleBot v2.0 Commands ===");
            Serial.println("<V,fr,fl,rr,rl>  wheel vel rad/s (PID)");
            Serial.println("<M,fr,fl,rr,rl>  direct PWM -255..255");
            Serial.println("<T,idx,vel>      test single motor");
            Serial.println("<G,Kp,Ki,Kd>     set PID gains");
            Serial.println("<F,fr,fl,rr,rl>  set FF gains");
            Serial.println("<S>              E-STOP (latches)");
            Serial.println("<E1>/<E0>        enable/disable + clear ESTOP");
            Serial.println("<W1>/<W0>        watchdog on/off");
            Serial.println("<L1>/<L0>        serial telemetry on/off");
            Serial.println("<P>              ping");
            Serial.println("<I>              system info");
            Serial.println("<?>              live state");
            break;

        default:
            Serial.printf("[ERR,UNKNOWN:%c]\n", cmd[0]);
            break;
    }
}

void handleSerialInput() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '<') {
            serialReceiving = true;
            serialBufIdx    = 0;
        } else if (c == '>' && serialReceiving) {
            serialReceiving          = false;
            serialBuffer[serialBufIdx] = '\0';
            processSerialCommand(serialBuffer);
        } else if (serialReceiving) {
            if (serialBufIdx < (int)sizeof(serialBuffer) - 1) {
                serialBuffer[serialBufIdx++] = c;
            } else {
                serialReceiving = false;
                serialBufIdx    = 0;
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════
// 13. WEBSOCKET HANDLER
// ═══════════════════════════════════════════════════════════════════

void webSocketEvent(uint8_t /*num*/, WStype_t type,
                    uint8_t* payload, size_t length) {
    switch (type) {

        case WStype_DISCONNECTED:
            wifi_vx = 0; wifi_vy = 0; wifi_wz = 0;
            break;

        case WStype_CONNECTED:
            break;

        case WStype_TEXT: {
            StaticJsonDocument<256> doc;
            if (deserializeJson(doc, payload, length)) break;

            const char* msgType = doc["type"];
            if (!msgType) break;

            if (strcmp(msgType, "joy") == 0) {
                // Joystick sends normalized values already scaled by gear
                // multiplier on the client. ESP32 multiplies by hardware max.
                wifi_vx = doc["vx"].as<float>() * MAX_LINEAR_SPEED;
                wifi_vy = doc["vy"].as<float>() * MAX_LINEAR_SPEED;
                wifi_wz = doc["wz"].as<float>() * MAX_ANGULAR_SPEED;
                last_wifi_cmd_ms = millis();
                last_any_cmd_ms  = millis();
            }
            else if (strcmp(msgType, "stop") == 0) {
                // E-STOP from phone — latches, no auto-clear
                estop_active = true;
                stopAllMotors();
                wifi_vx = 0; wifi_vy = 0; wifi_wz = 0;
            }
            else if (strcmp(msgType, "gains") == 0) {
                Kp = doc["kp"].as<float>();
                Ki = doc["ki"].as<float>();
                Kd = doc["kd"].as<float>();
                for (int i = 0; i < NUM_MOTORS; i++) motors[i].integral = 0;
            }
            else if (strcmp(msgType, "resume") == 0) {
                // Resume from E-STOP via phone (E-STOP button second tap)
                estop_active   = false;
                motors_enabled = true;
            }
            break;
        }

        default: break;
    }
}

// ── Telemetry broadcast to WebSocket (for future debugging) ─────────
void broadcastTelemetry() {
    if (wsServer.connectedClients() == 0) return;
    StaticJsonDocument<320> doc;
    doc["type"] = "tel";
    doc["t"]    = millis();
    doc["estop"] = estop_active;
    doc["src"]  = (millis() - last_wifi_cmd_ms < WIFI_CMD_TIMEOUT_MS)
                  ? "WIFI" : "SERIAL";
    JsonArray arr = doc.createNestedArray("m");
    for (int i = 0; i < NUM_MOTORS; i++) {
        JsonObject m = arr.createNestedObject();
        m["n"]   = MOTOR_NAMES[i];
        m["tgt"] = roundf(motors[i].target_velocity * 100.0f) / 100.0f;
        m["act"] = roundf(motors[i].actual_velocity * 100.0f) / 100.0f;
        m["pwm"] = motors[i].pwm_output;
    }
    char buf[320];
    size_t len = serializeJson(doc, buf, sizeof(buf));
    wsServer.broadcastTXT(buf, len);
}

// ═══════════════════════════════════════════════════════════════════
// 14. JOYSTICK WEB PAGE
//
//  Layout (no PID graphs — removed):
//    ┌─────────────────────────────────┐
//    │  Header: title | gear | src | ● │  37 px
//    ├──────────────────────┬──────────┤
//    │                      │   ROT    │
//    │   Joystick (large)   │  slider  │  flex 1
//    │                      ├──────────┤
//    │                      │  E-STOP  │
//    ├──────────────────────┴──────────┤
//    │   SLOW  │  NORMAL  │   FAST    │  70 px
//    └─────────────────────────────────┘
//
//  Speed modes (gear multiplier applied client-side):
//    SLOW   0.25 × → vx_max 0.12 m/s  (safe for ground testing)
//    NORMAL 0.60 × → vx_max 0.29 m/s
//    FAST   1.00 × → vx_max 0.48 m/s  (hardware maximum)
// ═══════════════════════════════════════════════════════════════════

const char WEBPAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>AisleBot</title>
<style>
*{margin:0;padding:0;box-sizing:border-box;-webkit-user-select:none;user-select:none;touch-action:none}
html,body{width:100vw;height:100vh;overflow:hidden;background:#0a0e17;color:#e0e0e0;font-family:'Courier New',monospace}

/* ── Header ── */
.hdr{display:flex;justify-content:space-between;align-items:center;
     height:40px;padding:0 14px;background:#111827;
     border-bottom:1px solid #1e3a5f;flex-shrink:0}
.hdr-title{font-size:13px;font-weight:700;color:#38bdf8;letter-spacing:1.5px}
.hdr-right{display:flex;align-items:center;gap:8px}
.badge{padding:2px 7px;border-radius:4px;font-size:10px;font-weight:700;
       background:#1e3a5f;color:#7dd3fc;letter-spacing:.5px}
.badge.estop-badge{background:#7f1d1d;color:#fca5a5;display:none}
.badge.estop-badge.show{display:inline-block}
.gear-badge{padding:2px 7px;border-radius:4px;font-size:10px;font-weight:700;
            background:#052e16;color:#22c55e;letter-spacing:.5px;
            transition:background .2s,color .2s}
.dot{width:9px;height:9px;border-radius:50%;background:#ef4444;flex-shrink:0}
.dot.on{background:#22c55e;box-shadow:0 0 6px #22c55e}

/* ── Page body ── */
.page{display:flex;flex-direction:column;height:calc(100vh - 40px)}

/* ── Controls row ── */
.controls{display:flex;flex:1;min-height:0}

/* ── Joystick area ── */
.joy-area{flex:1;position:relative;display:flex;align-items:center;
          justify-content:center;background:#0f1729;overflow:hidden}
.joy-ring{border:2px solid #1e3a5f;border-radius:50%;position:relative;
          width:min(62vw,280px);height:min(62vw,280px)}
.joy-thumb{width:68px;height:68px;border-radius:50%;position:absolute;
           top:50%;left:50%;transform:translate(-50%,-50%);
           background:radial-gradient(circle at 35% 35%,#38bdf8,#0369a1);
           box-shadow:0 0 18px rgba(56,189,248,.35);transition:box-shadow .1s}
.joy-thumb.active{box-shadow:0 0 30px rgba(56,189,248,.7)}
.jlbl{position:absolute;font-size:9px;color:#334155;font-weight:700;letter-spacing:1px}
.jlbl.t{top:7%;left:50%;transform:translateX(-50%)}
.jlbl.b{bottom:7%;left:50%;transform:translateX(-50%)}
.jlbl.l{left:7%;top:50%;transform:translateY(-50%)}
.jlbl.r{right:7%;top:50%;transform:translateY(-50%)}

/* ── Right panel ── */
.right-panel{width:110px;display:flex;flex-direction:column;align-items:center;
             background:#0f1729;border-left:1px solid #1e3a5f;
             padding:14px 0 10px;gap:0}

/* Rotation slider */
.rot-wrap{flex:1;display:flex;flex-direction:column;align-items:center;
          justify-content:center;gap:6px;width:100%}
.rot-lbl{font-size:9px;color:#334155;letter-spacing:1px;font-weight:700}
.rot-track{width:26px;height:140px;background:#111827;border-radius:13px;
           border:1px solid #1e3a5f;position:relative;cursor:pointer}
.rot-thumb{width:42px;height:42px;border-radius:50%;position:absolute;
           left:50%;top:50%;transform:translate(-50%,-50%);
           background:radial-gradient(circle at 35% 35%,#f59e0b,#92400e);
           box-shadow:0 0 12px rgba(245,158,11,.3)}

/* E-STOP in right panel */
.estop-wrap{padding:10px 0 4px;display:flex;align-items:center;justify-content:center;
            border-top:1px solid #1e3a5f;width:100%}
.estop-btn{width:72px;height:72px;border-radius:50%;
           background:radial-gradient(circle at 40% 35%,#f87171,#7f1d1d);
           border:2.5px solid #fca5a5;color:#fff;font-weight:700;font-size:10px;
           letter-spacing:.5px;cursor:pointer;
           box-shadow:0 0 18px rgba(239,68,68,.25),inset 0 -3px 6px rgba(0,0,0,.3);
           display:flex;align-items:center;justify-content:center;
           line-height:1.2;text-align:center;flex-direction:column}
.estop-btn:active{transform:scale(.9);box-shadow:0 0 28px rgba(239,68,68,.6)}
.estop-btn.armed{background:radial-gradient(circle at 40% 35%,#4ade80,#14532d);
                 border-color:#86efac;box-shadow:0 0 18px rgba(74,222,128,.25)}

/* ── Speed bar ── */
.speed-bar{display:flex;height:70px;flex-shrink:0;
           background:#111827;border-top:1px solid #1e3a5f}
.spd-btn{flex:1;display:flex;flex-direction:column;align-items:center;
         justify-content:center;gap:3px;background:transparent;
         border:none;border-right:1px solid #1e3a5f;cursor:pointer;
         position:relative;transition:background .15s}
.spd-btn:last-child{border-right:none}
.spd-btn::after{content:'';position:absolute;bottom:0;left:0;right:0;
                height:3px;background:var(--c,#334155);opacity:.3;
                transition:opacity .2s}
.spd-btn.active{background:rgba(255,255,255,.03)}
.spd-btn.active::after{opacity:1}
.spd-btn.active .spd-name{color:var(--c,#fff)}
.spd-name{font-size:12px;font-weight:700;letter-spacing:1px;
          color:#4b5563;transition:color .2s}
.spd-val{font-size:10px;color:#374151;font-family:'Courier New',monospace}

/* ── PID panel (fixed overlay) ── */
.pid-panel{position:fixed;top:40px;right:0;width:210px;
           background:#111827ee;border-left:1px solid #1e3a5f;
           border-bottom:1px solid #1e3a5f;border-bottom-left-radius:8px;
           padding:12px;display:none;z-index:99}
.pid-panel.open{display:block}
.pid-row{display:flex;justify-content:space-between;align-items:center;margin:5px 0}
.pid-row label{font-size:11px;color:#94a3b8;width:28px}
.pid-row input{width:88px;background:#0a0e17;border:1px solid #1e3a5f;
               color:#e0e0e0;padding:4px 6px;font-size:12px;
               font-family:'Courier New',monospace;border-radius:3px}
.pid-apply{width:100%;margin-top:8px;padding:6px;background:#1e3a5f;
           border:none;color:#7dd3fc;font-size:11px;cursor:pointer;
           border-radius:3px;font-family:'Courier New',monospace;
           letter-spacing:.5px}
.pid-apply:active{background:#2563eb}
.pid-toggle{font-size:10px;color:#7dd3fc;cursor:pointer;padding:2px 6px;
            border:1px solid #1e3a5f;border-radius:3px}

/* ── ESTOP overlay flash ── */
.flash{position:fixed;inset:0;background:rgba(239,68,68,.18);
       pointer-events:none;opacity:0;transition:opacity .3s;z-index:200}
.flash.show{opacity:1}
</style>
</head>
<body>

<!-- ═══ HEADER ═══════════════════════════════════════════════════ -->
<div class="hdr">
  <span class="hdr-title">AISLEBOT v2.0</span>
  <div class="hdr-right">
    <span class="pid-toggle" onclick="togglePID()">PID</span>
    <span class="badge estop-badge" id="estopBadge">STOPPED</span>
    <span class="badge" id="srcBadge">---</span>
    <span class="gear-badge" id="gearBadge">SLOW</span>
    <span class="dot" id="wsDot"></span>
  </div>
</div>

<!-- ═══ PAGE BODY ════════════════════════════════════════════════ -->
<div class="page">

  <!-- Controls row: joystick + right panel -->
  <div class="controls">

    <!-- Joystick -->
    <div class="joy-area" id="joyArea">
      <div class="jlbl t">FWD</div>
      <div class="jlbl b">REV</div>
      <div class="jlbl l">LEFT</div>
      <div class="jlbl r">RIGHT</div>
      <div class="joy-ring" id="joyRing">
        <div class="joy-thumb" id="joyThumb"></div>
      </div>
    </div>

    <!-- Right panel: rotation + E-STOP -->
    <div class="right-panel">
      <div class="rot-wrap">
        <span class="rot-lbl">CCW</span>
        <div class="rot-track" id="rotTrack">
          <div class="rot-thumb" id="rotThumb"></div>
        </div>
        <span class="rot-lbl">CW</span>
      </div>
      <div class="estop-wrap">
        <button class="estop-btn" id="estopBtn">E<br>STOP</button>
      </div>
    </div>

  </div><!-- /controls -->

  <!-- Speed bar -->
  <div class="speed-bar">
    <button class="spd-btn active" id="spd0" style="--c:#22c55e"
            onclick="setSpeed(0)">
      <span class="spd-name">SLOW</span>
      <span class="spd-val">0.12 m/s</span>
    </button>
    <button class="spd-btn" id="spd1" style="--c:#f59e0b"
            onclick="setSpeed(1)">
      <span class="spd-name">NORMAL</span>
      <span class="spd-val">0.29 m/s</span>
    </button>
    <button class="spd-btn" id="spd2" style="--c:#ef4444"
            onclick="setSpeed(2)">
      <span class="spd-name">FAST</span>
      <span class="spd-val">0.48 m/s</span>
    </button>
  </div>

</div><!-- /page -->

<!-- ═══ PID PANEL ════════════════════════════════════════════════ -->
<div class="pid-panel" id="pidPanel">
  <div class="pid-row"><label>Kp</label><input type="number" id="inKp" value="50" step="5" min="0"></div>
  <div class="pid-row"><label>Ki</label><input type="number" id="inKi" value="30" step="5" min="0"></div>
  <div class="pid-row"><label>Kd</label><input type="number" id="inKd" value="3"  step="0.5" min="0"></div>
  <button class="pid-apply" onclick="sendGains()">APPLY GAINS</button>
</div>

<!-- E-STOP flash overlay -->
<div class="flash" id="flashDiv"></div>

<script>
// ── WebSocket ─────────────────────────────────────────────────────
let ws, wsOk = false;
const WS_URL = 'ws://' + location.hostname + ':81/';

function wsConnect() {
  ws = new WebSocket(WS_URL);
  ws.onopen  = () => { wsOk = true;  document.getElementById('wsDot').classList.add('on'); };
  ws.onclose = () => { wsOk = false; document.getElementById('wsDot').classList.remove('on'); setTimeout(wsConnect, 1000); };
  ws.onerror = () => ws.close();
  ws.onmessage = e => { try { handleTel(JSON.parse(e.data)); } catch(_) {} };
}
wsConnect();

function wsSend(o) { if (wsOk) ws.send(JSON.stringify(o)); }

// ── Telemetry handler ─────────────────────────────────────────────
function handleTel(d) {
  if (d.type !== 'tel') return;
  document.getElementById('srcBadge').textContent = d.src || '---';
  const eb = document.getElementById('estopBadge');
  eb.classList.toggle('show', !!d.estop);
}

// ── Speed modes ───────────────────────────────────────────────────
// Multiplier applied to normalized joystick (-1..1) before sending.
// ESP32 multiplies vx/vy by MAX_LINEAR_SPEED (0.48) and wz by MAX_ANGULAR_SPEED (1.0).
const SPEEDS = [
  { name:'SLOW',   mult:0.25, gColor:'#22c55e', gBg:'#052e16' },
  { name:'NORMAL', mult:0.60, gColor:'#f59e0b', gBg:'#431407' },
  { name:'FAST',   mult:1.00, gColor:'#ef4444', gBg:'#450a0a' },
];
let speedIdx = 0;

function setSpeed(idx) {
  speedIdx = idx;
  for (let i = 0; i < 3; i++) {
    document.getElementById('spd'+i).classList.toggle('active', i === idx);
  }
  const s  = SPEEDS[idx];
  const gb = document.getElementById('gearBadge');
  gb.textContent       = s.name;
  gb.style.color       = s.gColor;
  gb.style.background  = s.gBg;
}

// ── Joystick ──────────────────────────────────────────────────────
const joyRing  = document.getElementById('joyRing');
const joyThumb = document.getElementById('joyThumb');
let joyActive = false, joyId = null, joyX = 0, joyY = 0;

function getJoyPos(touch) {
  const r   = joyRing.getBoundingClientRect();
  const cx  = r.left + r.width / 2;
  const cy  = r.top  + r.height / 2;
  const maxR = r.width / 2 - 34;
  let dx = touch.clientX - cx;
  let dy = touch.clientY - cy;
  const dist = Math.sqrt(dx*dx + dy*dy);
  if (dist > maxR) { dx = dx / dist * maxR; dy = dy / dist * maxR; }
  // nx = right (+1), ny = up (+1, because screen Y inverts)
  return { dx, dy, nx: dx / maxR, ny: -dy / maxR };
}

document.getElementById('joyArea').addEventListener('touchstart', e => {
  e.preventDefault();
  if (joyActive) return;
  const t = e.changedTouches[0];
  joyActive = true; joyId = t.identifier;
  joyThumb.classList.add('active');
  const p = getJoyPos(t);
  joyX = p.nx; joyY = p.ny;
  joyThumb.style.transform = `translate(calc(-50% + ${p.dx}px),calc(-50% + ${p.dy}px))`;
}, { passive: false });

document.getElementById('joyArea').addEventListener('touchmove', e => {
  e.preventDefault();
  for (const t of e.changedTouches) {
    if (t.identifier !== joyId) continue;
    const p = getJoyPos(t);
    joyX = p.nx; joyY = p.ny;
    joyThumb.style.transform = `translate(calc(-50% + ${p.dx}px),calc(-50% + ${p.dy}px))`;
  }
}, { passive: false });

document.getElementById('joyArea').addEventListener('touchend', e => {
  for (const t of e.changedTouches) {
    if (t.identifier !== joyId) continue;
    joyActive = false; joyId = null; joyX = 0; joyY = 0;
    joyThumb.classList.remove('active');
    joyThumb.style.transform = 'translate(-50%,-50%)';
    sendJoy();
  }
}, { passive: false });

// ── Rotation slider ───────────────────────────────────────────────
const rotTrack = document.getElementById('rotTrack');
const rotThumb = document.getElementById('rotThumb');
let rotActive = false, rotId = null, rotVal = 0;

function clamp01(x) { return Math.max(0, Math.min(1, x)); }

rotTrack.addEventListener('touchstart', e => {
  e.preventDefault();
  if (rotActive) return;
  const t = e.changedTouches[0];
  rotActive = true; rotId = t.identifier;
  updateRot(t);
}, { passive: false });

rotTrack.addEventListener('touchmove', e => {
  e.preventDefault();
  for (const t of e.changedTouches) {
    if (t.identifier === rotId) updateRot(t);
  }
}, { passive: false });

rotTrack.addEventListener('touchend', e => {
  for (const t of e.changedTouches) {
    if (t.identifier !== rotId) continue;
    rotActive = false; rotId = null; rotVal = 0;
    rotThumb.style.top = '50%';
    sendJoy();
  }
}, { passive: false });

function updateRot(touch) {
  const r   = rotTrack.getBoundingClientRect();
  const pct = clamp01((touch.clientY - r.top) / r.height);
  rotThumb.style.top = (pct * 100) + '%';
  // pct=0 (top) → CCW (+1), pct=0.5 (centre) → 0, pct=1 (bottom) → CW (-1)
  rotVal = -(pct - 0.5) * 2;
}

// ── Send joystick command ─────────────────────────────────────────
const DEADZONE = 0.08;
function applyDead(v) { return Math.abs(v) < DEADZONE ? 0 : v; }

function sendJoy() {
  const m  = SPEEDS[speedIdx].mult;
  const vx = applyDead(joyY)  * m;   // forward/backward
  const vy = applyDead(-joyX)  * m;   // right strafe (+) / left (-)
  const wz = applyDead(rotVal) * m;  // CCW rotation (+)
  wsSend({ type:'joy',
           vx: +vx.toFixed(3),
           vy: +vy.toFixed(3),
           wz: +wz.toFixed(3) });
}

// 20 Hz send loop while any input is active
setInterval(() => { if (joyActive || rotActive) sendJoy(); }, 50);

// ── E-STOP ────────────────────────────────────────────────────────
let estopped = false;
const estopBtn  = document.getElementById('estopBtn');
const flashDiv  = document.getElementById('flashDiv');

function flashRed() {
  flashDiv.classList.add('show');
  setTimeout(() => flashDiv.classList.remove('show'), 400);
}

estopBtn.addEventListener('touchstart', e => {
  e.preventDefault();
  e.stopPropagation();
  if (!estopped) {
    // First tap → E-STOP
    estopped = true;
    wsSend({ type: 'stop' });
    joyX = 0; joyY = 0; rotVal = 0; joyActive = false; rotActive = false;
    joyThumb.style.transform = 'translate(-50%,-50%)';
    rotThumb.style.top = '50%';
    estopBtn.classList.add('armed');
    estopBtn.innerHTML = 'RESUME';
    flashRed();
  } else {
    // Second tap → Resume
    estopped = false;
    wsSend({ type: 'resume' });
    estopBtn.classList.remove('armed');
    estopBtn.innerHTML = 'E<br>STOP';
  }
}, { passive: false });

// ── PID panel ─────────────────────────────────────────────────────
function togglePID() {
  document.getElementById('pidPanel').classList.toggle('open');
}
function sendGains() {
  wsSend({
    type: 'gains',
    kp: parseFloat(document.getElementById('inKp').value),
    ki: parseFloat(document.getElementById('inKi').value),
    kd: parseFloat(document.getElementById('inKd').value),
  });
}
</script>
</body>
</html>
)rawliteral";

// ═══════════════════════════════════════════════════════════════════
// 15. WEB SERVER
// ═══════════════════════════════════════════════════════════════════

void handleRoot() {
    webServer.send_P(200, "text/html", WEBPAGE);
}

void setupWiFi() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_SSID, WIFI_PASS);

    IPAddress ip = WiFi.softAPIP();
    Serial.println("=================================");
    Serial.printf("  WiFi SSID : %s\n", WIFI_SSID);
    Serial.printf("  Password  : %s\n", WIFI_PASS);
    Serial.printf("  IP        : %s\n", ip.toString().c_str());
    Serial.printf("  Joystick  : http://%s/\n", ip.toString().c_str());
    Serial.printf("  WebSocket : ws://%s:%d/\n", ip.toString().c_str(), WEBSOCKET_PORT);
    Serial.println("=================================");

    webServer.on("/", handleRoot);
    webServer.begin();

    wsServer.begin();
    wsServer.onEvent(webSocketEvent);
}

// ═══════════════════════════════════════════════════════════════════
// 16. SERIAL TELEMETRY — CSV output at 10 Hz when <L1> active
//     Format: timestamp_ms,FR_tgt,FR_act,FR_pwm,FL_tgt,...,RL_pwm
//     Parsed by aislebot_ground_logger.py on the Pi
// ═══════════════════════════════════════════════════════════════════

void outputSerialTelemetry() {
    if (!serial_logging) return;
    Serial.printf("%lu", millis());
    for (int i = 0; i < NUM_MOTORS; i++) {
        Serial.printf(",%.2f,%.2f,%d",
                      motors[i].target_velocity,
                      motors[i].actual_velocity,
                      motors[i].pwm_output);
    }
    Serial.println();
}

// ═══════════════════════════════════════════════════════════════════
// 17. CORE 0 — COMMUNICATION TASK
// ═══════════════════════════════════════════════════════════════════

void communicationTask(void* /*param*/) {
    unsigned long lastTelMs = 0;
    while (true) {
        wsServer.loop();
        webServer.handleClient();
        handleSerialInput();

        if (millis() - lastTelMs >= TELEMETRY_INTERVAL_MS) {
            lastTelMs = millis();
            broadcastTelemetry();
            outputSerialTelemetry();
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ═══════════════════════════════════════════════════════════════════
// 18. SETUP & MAIN LOOP
// ═══════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(300);

    Serial.println();
    Serial.println("╔════════════════════════════════════════════╗");
    Serial.println("║  AisleBot ESP32 v2.0  —  Starting...       ║");
    Serial.println("╚════════════════════════════════════════════╝");

    memset(motors, 0, sizeof(motors));

    Serial.print("  Motor pins    ... ");
    setupMotorPins();
    Serial.println("OK");

    Serial.print("  PCNT encoders ... ");
    for (int i = 0; i < NUM_MOTORS; i++) setupPCNT(i);
    Serial.println("OK  (full quad, 93132 CPR)");

    Serial.print("  WiFi AP       ... ");
    setupWiFi();
    Serial.println("OK");

    Serial.print("  Core 0 task   ... ");
    xTaskCreatePinnedToCore(communicationTask, "Comms", 8192, NULL, 1, NULL, 0);
    Serial.println("OK  (WiFi + Serial + Telemetry)");

    Serial.print("  Core 1 task   ... ");
    xTaskCreatePinnedToCore(pidControlTask, "PID", 4096, NULL, 2, NULL, 1);
    Serial.println("OK  (PID @ 50 Hz)");

    last_any_cmd_ms = millis();  // Prevent immediate watchdog trigger

    Serial.println();
    Serial.printf("  PID     : Kp=%.0f  Ki=%.0f  Kd=%.0f  @ %.0f Hz\n",
                  Kp, Ki, Kd, PID_LOOP_HZ);
    Serial.printf("  Kff     : FR=%.1f  FL=%.1f  RR=%.1f  RL=%.1f\n",
                  Kff[FR], Kff[FL], Kff[RR], Kff[RL]);
    Serial.printf("  Geometry: K_OUTER=%.5f  K_INNER=%.5f  R=%.4f\n",
                  K_OUTER, K_INNER, WHEEL_RADIUS);
    Serial.printf("  Speed   : v_max=%.2f m/s  w_max=%.2f rad/s\n",
                  MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED);
    Serial.println();
    Serial.println("  E-STOP : <S> latches. Clear with <E1>.");
    Serial.println("  Logging: send <L1> to start Pi telemetry.");
    Serial.println();
    Serial.println("[READY]");
}

void loop() {
    // All work is done in pinned FreeRTOS tasks.
    // loop() runs on Core 1 (Arduino default) at lowest priority.
    vTaskDelay(pdMS_TO_TICKS(1000));
}
