// ============================================================================
//  AisleBot Arm + UV Lighting — Serial-Controlled v8
//
//  Receives commands from Raspberry Pi over USB serial at 115200 baud.
//  Optional analog joystick remains wired for standalone bench testing.
//
//  Author: Aritra Das (Roll 25D0074) | IIT Bombay BSBE | June 2026
//  Compatible with the unified ROS2 stack (arm_bridge.py)
//
//  NEW IN v8 — UV tube staged switching:
//    Three relay-channel pins drive the UV tubes through two 4-channel
//    active-LOW relay boards whose IN1/IN2/IN3 are ganged (both inverter
//    sides fire together). On <U1> the tubes light in a staircase:
//        tube 1  at t = 0
//        tube 2  at t = 5 s
//        tube 3  at t = 10 s
//    <U0> drops all three immediately. ESTOP also kills the tubes.
//    Sequencing is non-blocking (millis), so arm motion is never paused.
//
//  WIRING (arm — unchanged from v6/v7):
//    TB6600 #1 (Right arm): +5V→5V, CLK-→Pin2, CW-→Pin3, EN-=NC
//    TB6600 #2 (Left  arm): +5V→5V, CLK-→Pin4, CW-→Pin5, EN-=NC
//    BH-MSD (Lift):  PUL+→Pin6, DIR+→Pin7, PUL-/DIR-/EN-→GND, ENA+=NC
//    Joystick (optional): GND→GND, 5V→5V, VRx→A0, VRy→A1, SW→A2
//
//  WIRING (UV relays — new in v8, active-LOW, both boards ganged):
//    Pin 53 → IN1 on board 1 AND board 2   (tube 1, both sides)
//    Pin 51 → IN2 on board 1 AND board 2   (tube 2, both sides)
//    Pin 49 → IN3 on board 1 AND board 2   (tube 3, both sides)
//    Board VCC ← 5V buck (NOT the Mega).  Board GND ← common ground.
//    Relay contacts: COM = that inverter's Line, NO = choke red, NC empty.
//    Neutral stays common per inverter and is never switched.
//
//  SERIAL PROTOCOL (Pi → Mega):
//    <P>                 ping → [PONG]
//    <I>                 info dump
//    <?>                 status → [STATUS,arm_pos,lift_pos,enabled,estop,homing]
//    <E1> / <E0>         enable / disable arm motion
//    <S>                 ESTOP — latched, requires <C> to clear (also UV off)
//    <C>                 clear ESTOP
//    <H>                 begin HOME sequence (close arms, then lower lift)
//    <A,arm,lift>        velocity command, two floats in -1.0..+1.0
//                          arm  > 0  →  open both arms
//                          lift > 0  →  platform up
//    <J1> / <J0>         enable / disable analog joystick fallback
//    <U1>                UV cycle start (staged tubes 1→2→3)
//    <U0>                UV all off
//    <U?>                UV status → [UV,active,stage]
//
//  WATCHDOG: if no <A,..> command for 500 ms, motion stops automatically.
//            (UV is NOT watchdogged — tubes stay on until <U0> or ESTOP.)
//
//  All position-limit and motor-direction conventions match v6 exactly.
// ============================================================================

#include <AccelStepper.h>

// ── Pins ─────────────────────────────────────────────────────────
#define STEP_M1   2    // Right NEMA23 — CLK- on TB6600 #1
#define DIR_M1    3    // Right NEMA23 — CW-  on TB6600 #1
#define STEP_M2   4    // Left  NEMA23 — CLK- on TB6600 #2
#define DIR_M2    5    // Left  NEMA23 — CW-  on TB6600 #2
#define STEP_M3   6    // NEMA34 lift  — PUL+ on BH-MSD
#define DIR_M3    7    // NEMA34 lift  — DIR+ on BH-MSD
#define JOY_X     A0
#define JOY_Y     A1
#define JOY_SW    A2
#define LED_PIN   13

// ── UV tube relays (active-LOW; both 4-ch boards ganged) ─────────
#define UV_T1     53   // tube 1, both sides
#define UV_T2     51   // tube 2, both sides
#define UV_T3     49   // tube 3, both sides

// ── Motion parameters ────────────────────────────────────────────
const float ARM_SPEED  = 500.0;   // steps/sec (full-scale)
const float LIFT_SPEED = 400.0;   // steps/sec (full-scale)
const float ARM_ACC    = 600.0;
const float LIFT_ACC   = 500.0;

// ── Soft limits ──────────────────────────────────────────────────
const long ARM_MAX  = 500000L;    // tune after physical measurement
const long LIFT_MAX = 32000L;     // ~20 revs up from boot
const long LIFT_MIN = -32000L;    // ~20 revs below boot

// Set this nonzero (e.g. 1600) AFTER assembly to enforce safety interlock:
// arms will refuse to OPEN unless lift position >= MIN_LIFT_FOR_ARM.
const long MIN_LIFT_FOR_ARM = 0;

// ── Joystick deadzone ───────────────────────────────────────────
const int DEADZONE = 80;

// ── Watchdog ─────────────────────────────────────────────────────
const unsigned long WATCHDOG_MS = 500;

// ── UV staged lighting ───────────────────────────────────────────
const unsigned long UV_STAGE_MS = 5000;   // gap between successive tubes

// ── Motor objects ────────────────────────────────────────────────
AccelStepper mRight(AccelStepper::DRIVER, STEP_M1, DIR_M1);
AccelStepper mLeft (AccelStepper::DRIVER, STEP_M2, DIR_M2);
AccelStepper mLift (AccelStepper::DRIVER, STEP_M3, DIR_M3);

// ── State ────────────────────────────────────────────────────────
bool  enabled       = false;      // motion only runs when true
bool  estop         = false;      // latched safety stop
bool  homing        = false;
byte  homePhase     = 0;          // 0 = close arms, 1 = lower lift
bool  joystickMode  = true;       // bench testing default; Pi will <J0> on connect

float cmdArmSpd     = 0.0f;       // -1..+1, +ve = OPEN
float cmdLiftSpd    = 0.0f;       // -1..+1, +ve = UP
unsigned long lastCmdMs = 0;

// ── UV lighting state ────────────────────────────────────────────
bool          uvActive  = false;  // is a lighting cycle running?
byte          uvStage   = 0;      // 0 = off, 1 = t1, 2 = t1+t2, 3 = all
unsigned long uvStartMs = 0;      // millis() when the cycle began

// ── Joystick button (long-press ESTOP, short-press HOME) ─────────
bool          btnHeld   = false;
unsigned long btnStart  = 0;
const unsigned long ESTOP_HOLD_MS = 3000;

// ── Serial parser ────────────────────────────────────────────────
String  serialBuf  = "";
bool    inMessage  = false;

// ── Function Prototypes ──────────────────────────────────────────
void publishStatus(bool force = false);
void uvAllOff();
void uvStart();
void uvUpdate();

// ─────────────────────────────────────────────────────────────────
void setup() {
  // UV relays OFF before anything else. Boards are active-LOW, so HIGH
  // holds the contacts open (tubes dark) the instant the pins are driven.
  pinMode(UV_T1, OUTPUT); digitalWrite(UV_T1, HIGH);
  pinMode(UV_T2, OUTPUT); digitalWrite(UV_T2, HIGH);
  pinMode(UV_T3, OUTPUT); digitalWrite(UV_T3, HIGH);

  Serial.begin(115200);
  pinMode(JOY_SW, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  mRight.setMaxSpeed(ARM_SPEED);   mRight.setAcceleration(ARM_ACC);
  mLeft.setMaxSpeed(ARM_SPEED);    mLeft.setAcceleration(ARM_ACC);
  mLift.setMaxSpeed(LIFT_SPEED);   mLift.setAcceleration(LIFT_ACC);
  mRight.setCurrentPosition(0);
  mLeft.setCurrentPosition(0);
  mLift.setCurrentPosition(0);

  Serial.println(F("[BOOT] AisleBot Arm v8 ready (arm + UV lighting)"));
  Serial.println(F("[BOOT] Serial protocol active. <I> for help."));
}

// ─────────────────────────────────────────────────────────────────
void loop() {
  pollSerial();
  pollJoystickButton();
  applyWatchdog();

  // Advance the UV staircase every loop, independent of arm state.
  // It is a no-op whenever uvActive is false, so estop/homing/disabled
  // do not affect it beyond the explicit uvAllOff() on estop.
  uvUpdate();

  if (estop) { ledBlink(250); return; }

  if (homing) {
    ledBlink(100);
    runHomingStep();
    return;
  }

  if (joystickMode) readJoystickAxes();    // overwrites cmdArmSpd/cmdLiftSpd

  if (!enabled) { digitalWrite(LED_PIN, LOW); return; }

  driveMotors();
  publishStatus();
}

// ───────────────────────── SERIAL ───────────────────────────────
void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if      (c == '<') { serialBuf = ""; inMessage = true; }
    else if (c == '>' && inMessage) { handleCommand(serialBuf); inMessage = false; }
    else if (inMessage && serialBuf.length() < 64) serialBuf += c;
  }
}

void handleCommand(const String& cmd) {
  if (cmd.length() == 0) return;

  if (cmd == "P") { Serial.println(F("[PONG]")); return; }
  if (cmd == "I") { printInfo();        return; }
  if (cmd == "?") { publishStatus(true); return; }

  if (cmd == "E1") { enabled = true;  estop = false;
                     Serial.println(F("[OK] enabled")); return; }
  if (cmd == "E0") { enabled = false; cmdArmSpd = cmdLiftSpd = 0;
                     Serial.println(F("[OK] disabled")); return; }
  if (cmd == "S")  { estop = true; homing = false; cmdArmSpd = cmdLiftSpd = 0;
                     uvAllOff();                       // kill UV on ESTOP
                     Serial.println(F("[OK] ESTOP")); return; }
  if (cmd == "C")  { estop = false; Serial.println(F("[OK] clear")); return; }
  if (cmd == "H")  { if (estop) { Serial.println(F("[ERR] estop")); return; }
                     homing = true;
                     homePhase = 0;
                     Serial.println(F("[OK] homing")); return; }
  if (cmd == "J1") { joystickMode = true;
                     Serial.println(F("[OK] joystick on")); return; }
  if (cmd == "J0") { joystickMode = false;
                     Serial.println(F("[OK] joystick off")); return; }

  // ── UV lighting ──────────────────────────────────────────────
  if (cmd == "U1") { if (estop) { Serial.println(F("[ERR] estop")); return; }
                     uvStart();  Serial.println(F("[OK] UV cycle")); return; }
  if (cmd == "U0") { uvAllOff(); Serial.println(F("[OK] UV off"));  return; }
  if (cmd == "U?") { Serial.print(F("[UV,"));
                     Serial.print(uvActive ? 1 : 0); Serial.print(',');
                     Serial.print(uvStage);
                     Serial.println(']'); return; }

  if (cmd.startsWith("A,")) {
    int comma = cmd.indexOf(',', 2);
    if (comma < 0) { Serial.println(F("[ERR] A,arm,lift")); return; }
    float a = cmd.substring(2, comma).toFloat();
    float l = cmd.substring(comma + 1).toFloat();
    cmdArmSpd  = constrain(a, -1.0f, 1.0f);
    cmdLiftSpd = constrain(l, -1.0f, 1.0f);
    lastCmdMs  = millis();
    return;
  }

  Serial.print(F("[ERR] unknown: ")); Serial.println(cmd);
}

void printInfo() {
  Serial.println(F("─────────── AisleBot Arm v8 ───────────"));
  Serial.println(F("Commands: <P> <I> <?> <E1>/<E0> <S> <C> <H> <J1>/<J0>"));
  Serial.println(F("          <A,arm,lift>  arm/lift in -1.0..+1.0"));
  Serial.println(F("          <U1> UV cycle  <U0> UV off  <U?> UV status"));
  Serial.print  (F("ARM_SPEED  = ")); Serial.println(ARM_SPEED);
  Serial.print  (F("LIFT_SPEED = ")); Serial.println(LIFT_SPEED);
  Serial.print  (F("ARM_MAX    = ")); Serial.println(ARM_MAX);
  Serial.print  (F("LIFT_MAX   = ")); Serial.println(LIFT_MAX);
  Serial.print  (F("LIFT_MIN   = ")); Serial.println(LIFT_MIN);
  Serial.print  (F("MIN_LIFT_FOR_ARM = ")); Serial.println(MIN_LIFT_FOR_ARM);
  Serial.print  (F("UV_STAGE   = ")); Serial.print(UV_STAGE_MS); Serial.println(F(" ms"));
  Serial.print  (F("WATCHDOG   = ")); Serial.print(WATCHDOG_MS); Serial.println(F(" ms"));
}

void publishStatus(bool force) {
  static unsigned long lastT = 0;
  if (!force && millis() - lastT < 200) return;
  lastT = millis();
  Serial.print(F("[STATUS,"));
  Serial.print(mRight.currentPosition()); Serial.print(',');
  Serial.print(mLift.currentPosition());  Serial.print(',');
  Serial.print(enabled ? 1 : 0);          Serial.print(',');
  Serial.print(estop   ? 1 : 0);          Serial.print(',');
  Serial.print(homing  ? 1 : 0);
  Serial.println(']');
}

// ───────────────────────── INPUT ────────────────────────────────
void pollJoystickButton() {
  bool pressed = (digitalRead(JOY_SW) == LOW);
  if (pressed && !btnHeld) { btnStart = millis(); btnHeld = true; }
  if (pressed && btnHeld && !estop &&
      (millis() - btnStart) >= ESTOP_HOLD_MS) {
    estop = true;
    homing = false; cmdArmSpd = cmdLiftSpd = 0;
    uvAllOff();                                 // kill UV on button ESTOP
    Serial.println(F("[ESTOP] joystick button"));
  }
  if (!pressed && btnHeld) {
    btnHeld = false;
    unsigned long held = millis() - btnStart;
    if (!estop && held >= 50 && held < ESTOP_HOLD_MS) {
      homing = true;
      homePhase = 0;
      Serial.println(F("[HOME] joystick button"));
    }
  }
}

void readJoystickAxes() {
  int rawX = analogRead(JOY_X);
  int rawY = analogRead(JOY_Y);
  int jx = rawX - 512;
  int jy = rawY - 512;
  if (abs(jx) < DEADZONE) jx = 0;
  if (abs(jy) < DEADZONE) jy = 0;

  // Match v6 conventions:
  //   Joy RIGHT (jx>0) → cmdArmSpd>0 → arms OPEN
  //   Joy UP    (jy<0) → cmdLiftSpd>0 (we flip sign) → platform UP
  cmdArmSpd  =  jx / 512.0f;
  cmdLiftSpd = -jy / 512.0f;        // flip so +ve = UP at the protocol level
  if (jx != 0 || jy != 0) lastCmdMs = millis();
}

// ───────────────────────── SAFETY ───────────────────────────────
void applyWatchdog() {
  if (millis() - lastCmdMs > WATCHDOG_MS) {
    cmdArmSpd  = 0;
    cmdLiftSpd = 0;
  }
}

// ───────────────────────── UV LIGHTING ──────────────────────────
// Relay boards are active-LOW: driving a pin LOW closes its relay and
// lights that tube; HIGH opens it. Because IN1/IN2/IN3 of both boards
// share each pin, one write controls that tube on BOTH inverter sides.

void uvAllOff() {
  digitalWrite(UV_T1, HIGH);
  digitalWrite(UV_T2, HIGH);
  digitalWrite(UV_T3, HIGH);
  uvActive = false;
  uvStage  = 0;
}

void uvStart() {
  uvStartMs = millis();
  uvActive  = true;
  uvStage   = 1;
  digitalWrite(UV_T1, LOW);     // tube 1 strikes immediately
  digitalWrite(UV_T2, HIGH);
  digitalWrite(UV_T3, HIGH);
}

void uvUpdate() {
  if (!uvActive) return;
  unsigned long dt = millis() - uvStartMs;
  if (uvStage < 2 && dt >= UV_STAGE_MS)        { digitalWrite(UV_T2, LOW); uvStage = 2; }
  if (uvStage < 3 && dt >= 2UL * UV_STAGE_MS)  { digitalWrite(UV_T3, LOW); uvStage = 3; }
}

// ───────────────────── MOTION CORE ──────────────────────────────
void driveMotors() {
  // ── Arms (mirrored: right=+arm, left=-arm) ──
  float armRate = cmdArmSpd * ARM_SPEED;
  long  armPos  = mRight.currentPosition();

  if (armRate > 0 && armPos >=  ARM_MAX) armRate = 0;
  if (armRate < 0 && armPos <= -ARM_MAX) armRate = 0;

  // Safety interlock: refuse to OPEN when lift below threshold
  if (MIN_LIFT_FOR_ARM > 0 && armRate > 0 &&
      mLift.currentPosition() < MIN_LIFT_FOR_ARM) {
    armRate = 0;
  }

  mRight.setSpeed(+armRate);
  mLeft.setSpeed (-armRate);
  mRight.runSpeed();
  mLeft.runSpeed();

  // ── Lift (protocol +ve = UP; v6 uses -ve speed for UP, so flip) ──
  float liftRate = -cmdLiftSpd * LIFT_SPEED;
  long  liftPos  = mLift.currentPosition();

  if (liftRate > 0 && liftPos >= LIFT_MAX) liftRate = 0;
  if (liftRate < 0 && liftPos <= LIFT_MIN) liftRate = 0;

  mLift.setSpeed(liftRate);
  mLift.runSpeed();
}

void runHomingStep() {
  if (homePhase == 0) {
    mRight.moveTo(0);
    mLeft.moveTo(0);
    mRight.run();
    mLeft.run();
    if (mRight.distanceToGo() == 0 && mLeft.distanceToGo() == 0) {
      mRight.setCurrentPosition(0);
      mLeft.setCurrentPosition(0);
      homePhase = 1;
      Serial.println(F("[HOME] arms closed, lowering"));
    }
  } else {
    mLift.moveTo(0);
    mLift.run();
    if (mLift.distanceToGo() == 0) {
      mLift.setCurrentPosition(0);
      homing = false;
      digitalWrite(LED_PIN, LOW);
      Serial.println(F("[HOME] complete"));
    }
  }
}

// ──────────────────────── UTIL ──────────────────────────────────
void ledBlink(int periodHalfMs) {
  digitalWrite(LED_PIN, (millis() / periodHalfMs) % 2);
}
