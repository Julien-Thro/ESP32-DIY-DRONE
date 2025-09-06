#include <Arduino.h>
#include <DShotRMT.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <Wire.h>
#include <MPU6050_light.h>
#include <math.h>

// ===================== SERIAL =====================
static constexpr auto &USB_SERIAL = Serial0;
static constexpr uint32_t USB_SERIAL_BAUD = 115200;

// ===================== RADIO (NRF24) =====================
RF24 radio(4, 5);  // CE, CSN
const uint64_t adresse = 0x1111111111ULL;

struct __attribute__((packed)) RadioData {
  uint16_t throttle, pitch, roll, yaw; // Échelle DShot (~48..2047), centre ~1047
};
RadioData rxData = {48, 1047, 1047, 1047};

// ===================== DSHOTRMT =====================
static constexpr dshot_mode_t DSHOT_MODE = DSHOT1200;
static constexpr bool IS_BIDIRECTIONAL = false;

// Pins (Quad X) : FL-CCW / RL-CW / RR-CCW / FR-CW
static constexpr int MOTOR01_PIN = 16; // FL (CCW)
static constexpr int MOTOR02_PIN = 17; // RL (CW)
static constexpr int MOTOR03_PIN = 32; // RR (CCW)
static constexpr int MOTOR04_PIN = 33; // FR (CW)

DShotRMT motor01(MOTOR01_PIN, DSHOT_MODE, IS_BIDIRECTIONAL);
DShotRMT motor02(MOTOR02_PIN, DSHOT_MODE, IS_BIDIRECTIONAL);
DShotRMT motor03(MOTOR03_PIN, DSHOT_MODE, IS_BIDIRECTIONAL);
DShotRMT motor04(MOTOR04_PIN, DSHOT_MODE, IS_BIDIRECTIONAL);

// ===================== ARM / FAILSAFE =====================
bool armed = false, justArmed = false;
unsigned long armStartMs = 0, lastRadioTime = 0;
bool failsafeActive = false, failsafeStopSent = false;

static constexpr uint16_t ARMING_THRESHOLD = 50;     // ARM si throttle < 50
static constexpr uint32_t FAILSAFE_TIMEOUT = 1000;   // ms sans radio => STOP
static constexpr uint16_t IDLE_VALUE = 110;          // idle DShot (~5%)

// ===================== LISSAGE / DEADZONE =====================
constexpr int AXIS_CENTER   = 1047;
constexpr int AXIS_DEADZONE = 20;

int16_t applyDeadzone(int16_t v, int16_t center = AXIS_CENTER, int16_t dz = AXIS_DEADZONE) {
  if (abs(v - center) < dz) return center;
  return v;
}
int smoothStep(int target, int current, int step) {
  if (step <= 0) return target;
  int d = target - current;
  if (abs(d) <= step) return target;
  return current + (d > 0 ? step : -step);
}
static constexpr int THROTTLE_SMOOTH_STEP = 40;
static constexpr int AXIS_SMOOTH_STEP     = 30;

// ===================== BATTERIE (6S, 68k/10k => 7.8) =====================
#define BATTERY_PIN GPIO_NUM_34
static constexpr float ADC_REF_V    = 3.3f;
static constexpr float VBAT_RATIO   = 7.8f;
static constexpr float VBAT_LOW_ON  = 18.0f;
static constexpr float VBAT_LOW_OFF = 18.5f;
static constexpr float VBAT_ALPHA   = 0.20f;
float vbatFilt = 0.0f; bool vbatLow = false; unsigned long lastBeepMs = 0;

float readBatteryVoltageRaw() {
  int raw = analogRead(BATTERY_PIN);           // 0..4095
  float vAdc = (raw / 4095.0f) * ADC_REF_V;    // V au pin
  return vAdc * VBAT_RATIO;                    // V batterie
}
float readBatteryVoltage() {
  float v = readBatteryVoltageRaw();
  vbatFilt = (1.0f - VBAT_ALPHA) * vbatFilt + VBAT_ALPHA * v;
  return vbatFilt;
}
void updateVbatAlarm(float v) {
  if (!vbatLow && v <= VBAT_LOW_ON)  vbatLow = true;
  if ( vbatLow && v >= VBAT_LOW_OFF) vbatLow = false;
}

// ===================== BUZZER =====================
#define BUZZER_PIN 13
#define BUZZER_FREQ 4000
void beep(int ms = 120) { tone(BUZZER_PIN, BUZZER_FREQ); delay(ms); noTone(BUZZER_PIN); }

// ===================== MPU6050 (RATE PID + angles debug) =====================
MPU6050 mpu(Wire);

// Signes gyro (adapter si orientation capteur ≠ châssis)
static constexpr int ROLL_SIGN  = +1; // getGyroX()
static constexpr int PITCH_SIGN = +1; // getGyroY()
static constexpr int YAW_SIGN   = +1; // getGyroZ()

// Signes sticks — roll inversé (constaté en vol)
static constexpr int ROLL_STICK_SIGN  = -1;
static constexpr int PITCH_STICK_SIGN = +1;
static constexpr int YAW_STICK_SIGN   = +1;

// Cibles & mesures (°/s)
static constexpr float MAX_RATE_DPS      = 380.0f;  // R/P
static constexpr float YAW_MAX_RATE_DPS  = 300.0f;  // Yaw
static constexpr float STICK_FULL_SCALE  = 500.0f;  // ~100% amplitude
static constexpr float GYRO_ALPHA        = 0.35f;   // LPF code côté soft
float rollRateFilt = 0.0f, pitchRateFilt = 0.0f, yawRateFilt = 0.0f;

inline float lowpass(float prev, float in, float a) { return (1.0f - a)*prev + a*in; }
inline float stickToRateTrimmed(int axisVal, int trimCounts, float maxRateDps) {
  float norm = (float)(axisVal - (AXIS_CENTER + trimCounts)) / STICK_FULL_SCALE;
  norm = constrain(norm, -1.0f, 1.0f);
  return norm * maxRateDps;
}

// ---- Config registre MPU (selon FireDIY) ----
static inline void mpuWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(0x68); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}
void mpuConfigureRegisters() {
  // PWR_MGMT_1: horloge interne, capteur réveillé
  mpuWriteReg(0x6B, 0x00);        // PWR_MGMT_1
  // GYRO_CONFIG: FS_SEL=±500 dps
  mpuWriteReg(0x1B, 0x08);        // ±500°/s
  // ACCEL_CONFIG: AFS_SEL=±8g
  mpuWriteReg(0x1C, 0x10);        // ±8g
  // CONFIG: DLPF_CFG=3 (~43 Hz)
  mpuWriteReg(0x1A, 0x03);        // DLPF ≈ 43 Hz
  // SMPLRT_DIV: 3 → 1kHz/(1+3)=250 Hz (DLPF actif)
  mpuWriteReg(0x19, 0x03);
}

// ---- Angles (debug) via filtre complémentaire ----
float rollAngleDeg = 0.0f, pitchAngleDeg = 0.0f;
bool imuAnglesInit = false;
static constexpr float COMP_TAU_S = 10.0f; // ~10 s → ≈0.04% acc @ 250 Hz

// PID rate
struct RatePID {
  float kp, ki, kd, iTerm, prevErr;
  float step(float err, float dt, bool i_enable) {
    if (i_enable) {
      iTerm += ki * err * dt;
      if (iTerm >  300.0f) iTerm =  300.0f;
      if (iTerm < -300.0f) iTerm = -300.0f;
    } else {
      iTerm *= 0.98f;
    }
    float dErr = (err - prevErr) / dt;
    prevErr = err;
    return kp*err + iTerm + kd*dErr;
  }
  void reset() { iTerm = 0.0f; prevErr = 0.0f; }
};
RatePID pidRoll  { 0.10f, 0.015f, 0.0012f, 0.0f, 0.0f };
RatePID pidPitch { 0.10f, 0.015f, 0.0012f, 0.0f, 0.0f };
RatePID pidYaw   { 0.12f, 0.010f, 0.0000f, 0.0f, 0.0f };

// Conversion PID (°/s) -> “unités DShot”
static constexpr float PID_TO_CMD = 1.2f;

// ===================== RC TRIM AUTO (200 ms après ARM) =====================
bool rcTrimCapturing = false, rcTrimReady = false;
unsigned long rcTrimStartMs = 0;
long sumRoll=0, sumPitch=0, sumYaw=0; int cntTrim=0;
int rcTrimRoll=0, rcTrimPitch=0, rcTrimYaw=0; // en “counts” vs AXIS_CENTER

// ===================== Airmode OFF près du ralenti =====================
static constexpr int THR_STAB_START = IDLE_VALUE + 10;
static constexpr int THR_STAB_FULL  = IDLE_VALUE + 120;
inline float smooth01(float x) { x = constrain(x, 0.0f, 1.0f); return x*x*(3.0f - 2.0f*x); }

// ===================== MIXAGE =====================
void mixFromTerms(int base, float pTermCmd, float rTermCmd, float yTermCmd,
                  uint16_t &m1, uint16_t &m2, uint16_t &m3, uint16_t &m4) {
  // Quad X:
  // M1 FL(CCW) = +P -R +Y
  // M2 RL(CW)  = -P -R -Y
  // M3 RR(CCW) = -P +R +Y
  // M4 FR(CW)  = +P +R -Y
  float m1f = base + pTermCmd - rTermCmd + yTermCmd;
  float m2f = base - pTermCmd - rTermCmd - yTermCmd;
  float m3f = base - pTermCmd + rTermCmd + yTermCmd;
  float m4f = base + pTermCmd + rTermCmd - yTermCmd;
  m1 = constrain((int)m1f, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX);
  m2 = constrain((int)m2f, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX);
  m3 = constrain((int)m3f, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX);
  m4 = constrain((int)m4f, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX);
}

// ===================== SETUP =====================
void setup() {
  USB_SERIAL.begin(USB_SERIAL_BAUD);

  pinMode(BUZZER_PIN, OUTPUT);
  beep(100); delay(100); beep(100); delay(100); beep(100);

  analogReadResolution(12);
  analogSetPinAttenuation(BATTERY_PIN, ADC_11db);
  vbatFilt = readBatteryVoltageRaw();

  // Radio
  radio.begin();
  radio.setDataRate(RF24_1MBPS);
  radio.setRetries(0, 1);
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(110);
  radio.openReadingPipe(1, adresse);
  radio.startListening();

  // DSHOT init
  motor01.begin(); motor02.begin(); motor03.begin(); motor04.begin();
  for (int i = 0; i < 6; ++i) {
    motor01.sendCommand(DSHOT_CMD_MOTOR_STOP);
    motor02.sendCommand(DSHOT_CMD_MOTOR_STOP);
    motor03.sendCommand(DSHOT_CMD_MOTOR_STOP);
    motor04.sendCommand(DSHOT_CMD_MOTOR_STOP);
    delay(20);
  }

  // MPU6050
  Wire.begin();
  Wire.setClock(400000);
  if (mpu.begin() != 0) { USB_SERIAL.println("Erreur MPU6050 ❌"); beep(2000); while (1) { delay(10); } }

  // >>> Config registres (FireDIY) avant offsets
  mpuConfigureRegisters();

  USB_SERIAL.println("Calibration MPU6050...");
  mpu.calcOffsets(true, true);
  USB_SERIAL.println("MPU OK ✅");
  beep(50); delay(100); beep(50);

  USB_SERIAL.println("Boot OK - PID R/P + Yaw (rate) - AirmodeOff bas gaz - MPU cfg(DLPF±FS±SRD) - Angles debug");
}

// ===================== LOOP =====================
void loop() {
  static int current_throttle = DSHOT_THROTTLE_MIN;
  static int current_pitch = AXIS_CENTER, current_roll = AXIS_CENTER, current_yaw = AXIS_CENTER;

  int mapped_throttle = current_throttle;
  int mapped_pitch = current_pitch;
  int mapped_roll = current_roll;
  int mapped_yaw = current_yaw;

  // ---- RADIO ----
  if (radio.available()) {
    radio.read(&rxData, sizeof(rxData));
    lastRadioTime = millis();
    if (failsafeActive) { failsafeActive = false; failsafeStopSent = false; }

    mapped_throttle = constrain((int)rxData.throttle, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX);
    mapped_pitch    = constrain((int)rxData.pitch,    DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX);
    mapped_roll     = constrain((int)rxData.roll,     DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX);
    mapped_yaw      = constrain((int)rxData.yaw,      DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX);

    if (!armed && rxData.throttle < ARMING_THRESHOLD) {
      armed = true; justArmed = true; armStartMs = millis();
      current_throttle = DSHOT_THROTTLE_MIN;
      current_pitch = AXIS_CENTER; current_roll = AXIS_CENTER; current_yaw = AXIS_CENTER;
      pidRoll.reset(); pidPitch.reset(); pidYaw.reset();
      rollRateFilt = pitchRateFilt = yawRateFilt = 0.0f;
      // RC auto-trim (200 ms)
      rcTrimCapturing = true; rcTrimReady = false; rcTrimStartMs = armStartMs;
      sumRoll = sumPitch = sumYaw = 0; cntTrim = 0;
      // Angles debug init
      imuAnglesInit = false;
      beep(90);
    }
  }

  // ---- FAILSAFE ----
  if (millis() - lastRadioTime > FAILSAFE_TIMEOUT) {
    failsafeActive = true; armed = false;
    mapped_throttle = 0;
    mapped_pitch = mapped_roll = mapped_yaw = AXIS_CENTER;
    if (!failsafeStopSent) {
      motor01.sendCommand(DSHOT_CMD_MOTOR_STOP);
      motor02.sendCommand(DSHOT_CMD_MOTOR_STOP);
      motor03.sendCommand(DSHOT_CMD_MOTOR_STOP);
      motor04.sendCommand(DSHOT_CMD_MOTOR_STOP);
      failsafeStopSent = true;
    }
  }

  // ---- LISSAGE ENTRÉES ----
  current_throttle = smoothStep(mapped_throttle, current_throttle, THROTTLE_SMOOTH_STEP);
  current_pitch    = smoothStep(applyDeadzone(mapped_pitch), current_pitch, AXIS_SMOOTH_STEP);
  current_roll     = smoothStep(applyDeadzone(mapped_roll),  current_roll,  AXIS_SMOOTH_STEP);
  current_yaw      = smoothStep(applyDeadzone(mapped_yaw),   current_yaw,   AXIS_SMOOTH_STEP);

  // ---- dt ----
  static unsigned long lastTs = micros();
  unsigned long nowTs = micros();
  float dt = (nowTs - lastTs) / 1e6f;
  if (dt < 0.0002f) dt = 0.0002f;
  if (dt > 0.02f)   dt = 0.02f;
  lastTs = nowTs;

  // ---- MPU update & filt ----
  mpu.update();
  float rollRateMeas  = ROLL_SIGN  * mpu.getGyroX(); // °/s
  float pitchRateMeas = PITCH_SIGN * mpu.getGyroY(); // °/s
  float yawRateMeas   = YAW_SIGN   * mpu.getGyroZ(); // °/s
  rollRateFilt  = lowpass(rollRateFilt,  rollRateMeas,  GYRO_ALPHA);
  pitchRateFilt = lowpass(pitchRateFilt, pitchRateMeas, GYRO_ALPHA);
  yawRateFilt   = lowpass(yawRateFilt,   yawRateMeas,   GYRO_ALPHA);

  // ---- Angles debug (complémentaire) ----
  // Acc angles (formules classiques)
  float ax = mpu.getAccX(); // en g
  float ay = mpu.getAccY();
  float az = mpu.getAccZ();
  float rollAccDeg  = atan2f(ay, az) * 180.0f / PI; // dépend orientation capteur -> ajuster si besoin
  float pitchAccDeg = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / PI;

  float alpha = COMP_TAU_S / (COMP_TAU_S + dt);
  if (!imuAnglesInit) {
    // init à l'acc (démarrage sur plan incliné)
    rollAngleDeg  = rollAccDeg;
    pitchAngleDeg = pitchAccDeg;
    imuAnglesInit = true;
  } else {
    // intégrer gyro + corriger avec acc
    rollAngleDeg  = alpha * (rollAngleDeg  + rollRateMeas  * dt) + (1.0f - alpha) * rollAccDeg;
    pitchAngleDeg = alpha * (pitchAngleDeg + pitchRateMeas * dt) + (1.0f - alpha) * pitchAccDeg;
  }

  // ---- RC auto-trim (200 ms après ARM) ----
  if (armed && rcTrimCapturing) {
    if ((millis() - rcTrimStartMs) <= 200) {
      sumRoll += rxData.roll; sumPitch += rxData.pitch; sumYaw += rxData.yaw; cntTrim++;
    } else {
      if (cntTrim > 0) {
        int avgR = (int)(sumRoll  / cntTrim);
        int avgP = (int)(sumPitch / cntTrim);
        int avgY = (int)(sumYaw   / cntTrim);
        rcTrimRoll  = constrain(avgR - AXIS_CENTER, -120, 120);
        rcTrimPitch = constrain(avgP - AXIS_CENTER, -120, 120);
        rcTrimYaw   = constrain(avgY - AXIS_CENTER, -120, 120);
      } else { rcTrimRoll = rcTrimPitch = rcTrimYaw = 0; }
      rcTrimCapturing = false; rcTrimReady = true;
    }
  }

  // ---- Cibles (sticks -> °/s) avec TRIM & signes ----
  int trR = rcTrimReady ? rcTrimRoll  : 0;
  int trP = rcTrimReady ? rcTrimPitch : 0;
  int trY = rcTrimReady ? rcTrimYaw   : 0;

  float rollRateTarget  = ROLL_STICK_SIGN  * stickToRateTrimmed(current_roll,  trR, MAX_RATE_DPS);
  float pitchRateTarget = PITCH_STICK_SIGN * stickToRateTrimmed(current_pitch, trP, MAX_RATE_DPS);
  float yawRateTarget   = YAW_STICK_SIGN   * stickToRateTrimmed(current_yaw,   trY, YAW_MAX_RATE_DPS);

  // ---- Airmode OFF / gating corrections ----
  int baseThrottle = current_throttle;
  if (armed && baseThrottle < IDLE_VALUE) baseThrottle = IDLE_VALUE;

  float stabScale = 0.0f; // 0..1
  if (armed) {
    float x = (float)(baseThrottle - THR_STAB_START) / (float)(THR_STAB_FULL - THR_STAB_START);
    stabScale = smooth01(x);
  }

  // ---- PID rate (R/P/Y) avec freeze I très bas gaz ----
  bool i_enable = (stabScale > 0.2f);
  float rollErr   = rollRateTarget  - rollRateFilt;
  float pitchErr  = pitchRateTarget - pitchRateFilt;
  float yawErr    = yawRateTarget   - yawRateFilt;

  float rollCorrDps  = pidRoll.step (rollErr,  dt, i_enable);
  float pitchCorrDps = pidPitch.step(pitchErr, dt, i_enable);
  float yawCorrDps   = pidYaw.step  (yawErr,   dt, i_enable);

  // ---- Corrections finales (arm ramp * stabScale) ----
  float armRamp = 1.0f;
  if (justArmed) {
    unsigned long el = millis() - armStartMs;
    if (el < 300) armRamp = (float)el / 300.0f;
    else { armRamp = 1.0f; justArmed = false; }
  }
  float pidScale = armRamp * stabScale;

  float rollCmd  = (rollCorrDps  * PID_TO_CMD) * pidScale;
  float pitchCmd = (pitchCorrDps * PID_TO_CMD) * pidScale;
  float yawCmd   = (yawCorrDps   * PID_TO_CMD) * pidScale;

  // ---- MIXAGE & ENVOI ----
  if (armed && !failsafeActive) {
    if ((millis() - armStartMs) < 120) { rollCmd = 0.0f; pitchCmd = 0.0f; yawCmd = 0.0f; }

    uint16_t m1, m2, m3, m4;
    mixFromTerms(baseThrottle, pitchCmd, rollCmd, yawCmd, m1, m2, m3, m4);

    motor01.sendThrottle(m1); motor02.sendThrottle(m2);
    motor03.sendThrottle(m3); motor04.sendThrottle(m4);

    float vbat = readBatteryVoltage(); updateVbatAlarm(vbat);
    if (vbat > 10.0f && vbatLow && (millis() - lastBeepMs >= 1000)) { lastBeepMs = millis(); beep(120); }

    static unsigned long lastDbg = 0;
    if (millis() - lastDbg > 500) {
      lastDbg = millis();
      USB_SERIAL.printf(
        "Armed=%d FS=%d | Vbat: %.2fV | ThrCur:%d Base:%d | ArmRamp:%.2f Stab:%.2f | Gyro R=%.1f P=%.1f Y=%.1f | PIDout R=%.1f P=%.1f Y=%.1f | Ang R=%.1f P=%.1f | Trim R:%d P:%d Y:%d | M1:%u M2:%u M3:%u M4:%u | RXraw T:%u P:%u R:%u Y:%u\n",
        armed, failsafeActive, vbat, current_throttle, baseThrottle,
        armRamp, stabScale, rollRateFilt, pitchRateFilt, yawRateFilt,
        rollCmd, pitchCmd, yawCmd,
        rollAngleDeg, pitchAngleDeg,
        rcTrimRoll, rcTrimPitch, rcTrimYaw,
        m1, m2, m3, m4,
        rxData.throttle, rxData.pitch, rxData.roll, rxData.yaw
      );
    }
  } else {
    motor01.sendThrottle(0); motor02.sendThrottle(0);
    motor03.sendThrottle(0); motor04.sendThrottle(0);

    float vbat = readBatteryVoltage(); updateVbatAlarm(vbat);
    if (vbat > 10.0f && vbatLow && (millis() - lastBeepMs >= 1000)) { lastBeepMs = millis(); beep(120); }

    static unsigned long lastDbg2 = 0;
    if (millis() - lastDbg2 > 500) {
      lastDbg2 = millis();
      USB_SERIAL.printf("[DISARM/FS] Vbat: %.2fV | Thr:%d P:%d R:%d Y:%d | Gyro R=%.1f P=%.1f Y=%.1f | Ang R=%.1f P=%.1f\n",
                        vbat, current_throttle, current_pitch, current_roll, current_yaw,
                        rollRateFilt, pitchRateFilt, yawRateFilt,
                        rollAngleDeg, pitchAngleDeg);
    }
  }
}
