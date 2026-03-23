#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>

/*
  ServoCalibrationTool.ino

  Purpose:
  - Standalone bench-test and calibration sketch for the 6 hobby servos used by the
    Stewart platform project.
  - Targets Arduino Uno R4 WiFi with a PCA9685-compatible HW-170 servo driver.
  - Uses Serial commands only so each servo can be centered, swept, trimmed, and
    verified before integrating with the full Stewart platform inverse-kinematics sketch.

  Safety notes:
  - Power servos from an external 5V to 6V supply sized for the total stall current.
  - Tie servo power ground to PCA9685 ground and Arduino ground.
  - Keep linkages detached during first calibration so unexpected motion cannot damage
    the platform.
  - The sketch clamps every command to configured angle and pulse limits.
*/

// -----------------------------------------------------------------------------
// CONFIG SECTION
// -----------------------------------------------------------------------------

static const uint8_t SERVO_COUNT = 6;
static const unsigned long SERIAL_BAUD = 115200;
static const uint8_t PCA9685_I2C_ADDRESS = 0x40;
static const float PCA9685_PWM_FREQUENCY_HZ = 50.0f;
static const uint32_t PCA9685_OSCILLATOR_HZ = 27000000UL;
static const unsigned long MOTION_UPDATE_INTERVAL_MS = 20;
static const float DEFAULT_STEP_DEG = 1.0f;
static const float DEFAULT_SLEW_DEG_PER_UPDATE = 1.5f;
static const float DEFAULT_SWEEP_STEP_DEG = 2.0f;
static const unsigned long DEFAULT_SWEEP_DWELL_MS = 120;

struct ServoCalibration {
  uint8_t channel;
  float zeroOffsetDeg;
  bool invert;
  float minAngleDeg;
  float maxAngleDeg;
  uint16_t minPulseUs;
  uint16_t maxPulseUs;
  float homeAngleDeg;
};

static ServoCalibration servoConfig[SERVO_COUNT] = {
  {0, 90.0f, false, 10.0f, 170.0f, 500, 2500, 90.0f},
  {1, 90.0f, true,  10.0f, 170.0f, 500, 2500, 90.0f},
  {2, 90.0f, false, 10.0f, 170.0f, 500, 2500, 90.0f},
  {3, 90.0f, true,  10.0f, 170.0f, 500, 2500, 90.0f},
  {4, 90.0f, false, 10.0f, 170.0f, 500, 2500, 90.0f},
  {5, 90.0f, true,  10.0f, 170.0f, 500, 2500, 90.0f}
};

// -----------------------------------------------------------------------------
// GLOBAL STATE
// -----------------------------------------------------------------------------

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_I2C_ADDRESS);

float currentAngleDeg[SERVO_COUNT] = {90, 90, 90, 90, 90, 90};
float targetAngleDeg[SERVO_COUNT] = {90, 90, 90, 90, 90, 90};
float slewRateDegPerUpdate = DEFAULT_SLEW_DEG_PER_UPDATE;
float manualStepDeg = DEFAULT_STEP_DEG;

bool outputEnabled = true;
bool sweepActive = false;
uint8_t sweepServoIndex = 0;
float sweepMinDeg = 70.0f;
float sweepMaxDeg = 110.0f;
float sweepStepDeg = DEFAULT_SWEEP_STEP_DEG;
bool sweepAscending = true;
unsigned long sweepDwellMs = DEFAULT_SWEEP_DWELL_MS;
unsigned long lastSweepUpdateMs = 0;
unsigned long lastMotionUpdateMs = 0;
String serialBuffer;

// -----------------------------------------------------------------------------
// DECLARATIONS
// -----------------------------------------------------------------------------

void initializeHardware();
void initializeDriver();
void printHelp();
void printStatus();
void printServoConfig(uint8_t index);
void parseSerialCommands();
void handleCommand(String line);
void updateServoMotion();
void updateSweep();
void stopSweep();
void homeAllServos();
void centerAllServos();
void moveServoToAngle(uint8_t index, float angleDeg, bool immediate = false);
void offsetServoAngle(uint8_t index, float deltaDeg);
uint16_t servoAngleToPulseUs(uint8_t index, float logicalAngleDeg);
uint16_t pulseUsToTicks(uint16_t pulseUs);
void writeServoPulseUs(uint8_t index, uint16_t pulseUs);
float clampFloat(float value, float minValue, float maxValue);
float parseFloatArg(char *token, bool &ok);
long parseLongArg(char *token, bool &ok);
int parseServoIndex(char *token, bool &ok);
void disableOutputs();
void enableOutputs();

// -----------------------------------------------------------------------------
// SETUP / LOOP
// -----------------------------------------------------------------------------

void setup() {
  initializeHardware();
  initializeDriver();
  centerAllServos();
  printHelp();
  printStatus();
}

void loop() {
  parseSerialCommands();
  updateSweep();
  updateServoMotion();
}

// -----------------------------------------------------------------------------
// INITIALIZATION
// -----------------------------------------------------------------------------

void initializeHardware() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial && millis() < 3000) {
    delay(10);
  }

  Wire.begin();
  Wire.setClock(400000);

  Serial.println();
  Serial.println(F("Servo Calibration Tool starting..."));
}

void initializeDriver() {
  pwm.begin();
  pwm.setOscillatorFrequency(PCA9685_OSCILLATOR_HZ);
  pwm.setPWMFreq(PCA9685_PWM_FREQUENCY_HZ);
  delay(10);
  Serial.print(F("PCA9685 initialized at 0x"));
  Serial.println(PCA9685_I2C_ADDRESS, HEX);
}

// -----------------------------------------------------------------------------
// COMMAND INTERFACE
// -----------------------------------------------------------------------------

void parseSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      String line = serialBuffer;
      serialBuffer = "";
      line.trim();
      if (line.length() > 0) {
        handleCommand(line);
      }
    } else {
      if (serialBuffer.length() < 200) {
        serialBuffer += c;
      }
    }
  }
}

void handleCommand(String line) {
  char buffer[201];
  line.toCharArray(buffer, sizeof(buffer));
  char *token = strtok(buffer, " ");
  if (token == NULL) {
    return;
  }

  if (strcasecmp(token, "HELP") == 0) {
    printHelp();
    return;
  }

  if (strcasecmp(token, "STATUS") == 0) {
    printStatus();
    return;
  }

  if (strcasecmp(token, "HOME") == 0) {
    stopSweep();
    homeAllServos();
    Serial.println(F("Moving all servos to configured home angles."));
    return;
  }

  if (strcasecmp(token, "CENTER") == 0) {
    stopSweep();
    centerAllServos();
    Serial.println(F("Moving all servos to their zero-offset centers."));
    return;
  }

  if (strcasecmp(token, "ENABLE") == 0) {
    enableOutputs();
    return;
  }

  if (strcasecmp(token, "DISABLE") == 0) {
    disableOutputs();
    return;
  }

  if (strcasecmp(token, "STOP") == 0) {
    stopSweep();
    for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
      targetAngleDeg[i] = currentAngleDeg[i];
    }
    Serial.println(F("Motion stopped. Targets held at current positions."));
    return;
  }

  if (strcasecmp(token, "SERVO") == 0) {
    bool ok = true;
    int index = parseServoIndex(strtok(NULL, " "), ok);
    float angle = parseFloatArg(strtok(NULL, " "), ok);
    if (!ok) {
      Serial.println(F("Usage: SERVO <0-5> <angleDeg>"));
      return;
    }
    stopSweep();
    moveServoToAngle((uint8_t)index, angle);
    printServoConfig((uint8_t)index);
    return;
  }

  if (strcasecmp(token, "NUDGE") == 0) {
    bool ok = true;
    int index = parseServoIndex(strtok(NULL, " "), ok);
    float delta = parseFloatArg(strtok(NULL, " "), ok);
    if (!ok) {
      Serial.println(F("Usage: NUDGE <0-5> <deltaDeg>"));
      return;
    }
    stopSweep();
    offsetServoAngle((uint8_t)index, delta);
    printServoConfig((uint8_t)index);
    return;
  }

  if (strcasecmp(token, "ZERO") == 0) {
    bool ok = true;
    int index = parseServoIndex(strtok(NULL, " "), ok);
    float zero = parseFloatArg(strtok(NULL, " "), ok);
    if (!ok) {
      Serial.println(F("Usage: ZERO <0-5> <zeroOffsetDeg>"));
      return;
    }
    servoConfig[index].zeroOffsetDeg = zero;
    targetAngleDeg[index] = zero;
    Serial.print(F("Updated zero offset for servo "));
    Serial.print(index);
    Serial.print(F(" to "));
    Serial.println(zero, 2);
    return;
  }

  if (strcasecmp(token, "LIMITS") == 0) {
    bool ok = true;
    int index = parseServoIndex(strtok(NULL, " "), ok);
    float minDeg = parseFloatArg(strtok(NULL, " "), ok);
    float maxDeg = parseFloatArg(strtok(NULL, " "), ok);
    if (!ok || maxDeg < minDeg) {
      Serial.println(F("Usage: LIMITS <0-5> <minDeg> <maxDeg>"));
      return;
    }
    servoConfig[index].minAngleDeg = minDeg;
    servoConfig[index].maxAngleDeg = maxDeg;
    targetAngleDeg[index] = clampFloat(targetAngleDeg[index], minDeg, maxDeg);
    currentAngleDeg[index] = clampFloat(currentAngleDeg[index], minDeg, maxDeg);
    Serial.print(F("Updated limits for servo "));
    Serial.print(index);
    Serial.print(F(": "));
    Serial.print(minDeg, 2);
    Serial.print(F(" to "));
    Serial.println(maxDeg, 2);
    return;
  }

  if (strcasecmp(token, "PULSE") == 0) {
    bool ok = true;
    int index = parseServoIndex(strtok(NULL, " "), ok);
    long minUs = parseLongArg(strtok(NULL, " "), ok);
    long maxUs = parseLongArg(strtok(NULL, " "), ok);
    if (!ok || minUs <= 0 || maxUs <= minUs) {
      Serial.println(F("Usage: PULSE <0-5> <minUs> <maxUs>"));
      return;
    }
    servoConfig[index].minPulseUs = (uint16_t)minUs;
    servoConfig[index].maxPulseUs = (uint16_t)maxUs;
    Serial.print(F("Updated pulse window for servo "));
    Serial.print(index);
    Serial.print(F(": "));
    Serial.print(minUs);
    Serial.print(F(" to "));
    Serial.println(maxUs);
    return;
  }

  if (strcasecmp(token, "INVERT") == 0) {
    bool ok = true;
    int index = parseServoIndex(strtok(NULL, " "), ok);
    long value = parseLongArg(strtok(NULL, " "), ok);
    if (!ok || (value != 0 && value != 1)) {
      Serial.println(F("Usage: INVERT <0-5> <0|1>"));
      return;
    }
    servoConfig[index].invert = (value == 1);
    Serial.print(F("Updated invert flag for servo "));
    Serial.print(index);
    Serial.print(F(" to "));
    Serial.println(servoConfig[index].invert ? F("true") : F("false"));
    return;
  }

  if (strcasecmp(token, "SLEW") == 0) {
    bool ok = true;
    float slew = parseFloatArg(strtok(NULL, " "), ok);
    if (!ok || slew <= 0.0f) {
      Serial.println(F("Usage: SLEW <degPerUpdate>"));
      return;
    }
    slewRateDegPerUpdate = slew;
    Serial.print(F("Updated slew rate to "));
    Serial.println(slewRateDegPerUpdate, 2);
    return;
  }

  if (strcasecmp(token, "STEP") == 0) {
    bool ok = true;
    float step = parseFloatArg(strtok(NULL, " "), ok);
    if (!ok || step <= 0.0f) {
      Serial.println(F("Usage: STEP <deg>"));
      return;
    }
    manualStepDeg = step;
    Serial.print(F("Updated manual nudge step to "));
    Serial.println(manualStepDeg, 2);
    return;
  }

  if (strcasecmp(token, "SWEEP") == 0) {
    bool ok = true;
    int index = parseServoIndex(strtok(NULL, " "), ok);
    float minDeg = parseFloatArg(strtok(NULL, " "), ok);
    float maxDeg = parseFloatArg(strtok(NULL, " "), ok);
    float stepDeg = parseFloatArg(strtok(NULL, " "), ok);
    long dwell = parseLongArg(strtok(NULL, " "), ok);
    if (!ok || maxDeg < minDeg || stepDeg <= 0.0f || dwell < 10) {
      Serial.println(F("Usage: SWEEP <0-5> <minDeg> <maxDeg> <stepDeg> <dwellMs>"));
      return;
    }
    sweepServoIndex = (uint8_t)index;
    sweepMinDeg = clampFloat(minDeg, servoConfig[index].minAngleDeg, servoConfig[index].maxAngleDeg);
    sweepMaxDeg = clampFloat(maxDeg, servoConfig[index].minAngleDeg, servoConfig[index].maxAngleDeg);
    sweepStepDeg = stepDeg;
    sweepDwellMs = (unsigned long)dwell;
    sweepAscending = true;
    sweepActive = true;
    lastSweepUpdateMs = 0;
    moveServoToAngle(sweepServoIndex, sweepMinDeg);
    Serial.print(F("Sweep started for servo "));
    Serial.println(index);
    return;
  }

  if (strcasecmp(token, "REPORT") == 0) {
    bool ok = true;
    int index = parseServoIndex(strtok(NULL, " "), ok);
    if (!ok) {
      Serial.println(F("Usage: REPORT <0-5>"));
      return;
    }
    printServoConfig((uint8_t)index);
    return;
  }

  Serial.print(F("Unknown command: "));
  Serial.println(line);
  Serial.println(F("Type HELP for command list."));
}

void printHelp() {
  Serial.println(F(""));
  Serial.println(F("=== Servo Calibration Commands ==="));
  Serial.println(F("HELP                              -> show this help"));
  Serial.println(F("STATUS                            -> print all servo states"));
  Serial.println(F("HOME                              -> move all servos to configured home angles"));
  Serial.println(F("CENTER                            -> move all servos to zero-offset center positions"));
  Serial.println(F("SERVO <i> <angle>                 -> move one servo to a logical angle"));
  Serial.println(F("NUDGE <i> <delta>                 -> offset one servo by delta degrees"));
  Serial.println(F("STEP <deg>                        -> set the preferred manual step size"));
  Serial.println(F("ZERO <i> <zeroOffset>             -> set logical center for one servo"));
  Serial.println(F("LIMITS <i> <min> <max>            -> update safe angle range"));
  Serial.println(F("PULSE <i> <minUs> <maxUs>         -> update pulse width range"));
  Serial.println(F("INVERT <i> <0|1>                  -> set inversion flag"));
  Serial.println(F("SLEW <degPerUpdate>               -> set smoothing rate"));
  Serial.println(F("SWEEP <i> <min> <max> <step> <ms> -> sweep a servo within safe limits"));
  Serial.println(F("STOP                              -> stop sweep and hold position"));
  Serial.println(F("ENABLE                            -> enable PWM output updates"));
  Serial.println(F("DISABLE                           -> de-energize all configured channels"));
  Serial.println(F("REPORT <i>                        -> print calibration info for one servo"));
  Serial.println(F(""));
  Serial.println(F("Calibration workflow suggestion:"));
  Serial.println(F("1) Keep rods disconnected."));
  Serial.println(F("2) Use CENTER, then ZERO to match horn geometry."));
  Serial.println(F("3) Use SERVO / NUDGE / SWEEP to find safe travel."));
  Serial.println(F("4) Copy the final zero, limit, pulse, and invert values into the Stewart sketch."));
}

// -----------------------------------------------------------------------------
// MOTION CONTROL
// -----------------------------------------------------------------------------

void updateServoMotion() {
  if (!outputEnabled) {
    return;
  }
  if (millis() - lastMotionUpdateMs < MOTION_UPDATE_INTERVAL_MS) {
    return;
  }
  lastMotionUpdateMs = millis();

  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    float current = currentAngleDeg[i];
    float target = targetAngleDeg[i];
    float delta = target - current;
    if (fabs(delta) <= slewRateDegPerUpdate) {
      current = target;
    } else {
      current += (delta > 0.0f) ? slewRateDegPerUpdate : -slewRateDegPerUpdate;
    }

    current = clampFloat(current, servoConfig[i].minAngleDeg, servoConfig[i].maxAngleDeg);
    currentAngleDeg[i] = current;
    writeServoPulseUs(i, servoAngleToPulseUs(i, current));
  }
}

void updateSweep() {
  if (!sweepActive) {
    return;
  }
  if (millis() - lastSweepUpdateMs < sweepDwellMs) {
    return;
  }
  lastSweepUpdateMs = millis();

  float nextTarget = targetAngleDeg[sweepServoIndex] + (sweepAscending ? sweepStepDeg : -sweepStepDeg);
  if (nextTarget >= sweepMaxDeg) {
    nextTarget = sweepMaxDeg;
    sweepAscending = false;
  } else if (nextTarget <= sweepMinDeg) {
    nextTarget = sweepMinDeg;
    sweepAscending = true;
  }
  moveServoToAngle(sweepServoIndex, nextTarget);
}

void stopSweep() {
  sweepActive = false;
}

void homeAllServos() {
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    targetAngleDeg[i] = clampFloat(servoConfig[i].homeAngleDeg, servoConfig[i].minAngleDeg, servoConfig[i].maxAngleDeg);
  }
}

void centerAllServos() {
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    moveServoToAngle(i, servoConfig[i].zeroOffsetDeg, true);
  }
}

void moveServoToAngle(uint8_t index, float angleDeg, bool immediate) {
  if (index >= SERVO_COUNT) {
    return;
  }
  float clamped = clampFloat(angleDeg, servoConfig[index].minAngleDeg, servoConfig[index].maxAngleDeg);
  targetAngleDeg[index] = clamped;
  if (immediate) {
    currentAngleDeg[index] = clamped;
    if (outputEnabled) {
      writeServoPulseUs(index, servoAngleToPulseUs(index, clamped));
    }
  }
}

void offsetServoAngle(uint8_t index, float deltaDeg) {
  if (fabs(deltaDeg) < 0.0001f) {
    deltaDeg = manualStepDeg;
  }
  moveServoToAngle(index, targetAngleDeg[index] + deltaDeg);
}

// -----------------------------------------------------------------------------
// STATUS / REPORTING
// -----------------------------------------------------------------------------

void printStatus() {
  Serial.println(F(""));
  Serial.println(F("=== Servo Calibration Status ==="));
  Serial.print(F("Outputs: "));
  Serial.println(outputEnabled ? F("enabled") : F("disabled"));
  Serial.print(F("Slew rate (deg/update): "));
  Serial.println(slewRateDegPerUpdate, 2);
  Serial.print(F("Default nudge step (deg): "));
  Serial.println(manualStepDeg, 2);
  Serial.print(F("Sweep active: "));
  Serial.println(sweepActive ? F("yes") : F("no"));

  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    printServoConfig(i);
  }
}

void printServoConfig(uint8_t index) {
  if (index >= SERVO_COUNT) {
    return;
  }

  uint16_t pulseUs = servoAngleToPulseUs(index, currentAngleDeg[index]);
  Serial.print(F("Servo "));
  Serial.print(index);
  Serial.print(F(" | ch="));
  Serial.print(servoConfig[index].channel);
  Serial.print(F(" | current="));
  Serial.print(currentAngleDeg[index], 2);
  Serial.print(F(" | target="));
  Serial.print(targetAngleDeg[index], 2);
  Serial.print(F(" | zero="));
  Serial.print(servoConfig[index].zeroOffsetDeg, 2);
  Serial.print(F(" | limits="));
  Serial.print(servoConfig[index].minAngleDeg, 1);
  Serial.print(F(".."));
  Serial.print(servoConfig[index].maxAngleDeg, 1);
  Serial.print(F(" | pulse="));
  Serial.print(servoConfig[index].minPulseUs);
  Serial.print(F(".."));
  Serial.print(servoConfig[index].maxPulseUs);
  Serial.print(F(" | invert="));
  Serial.print(servoConfig[index].invert ? F("1") : F("0"));
  Serial.print(F(" | outUs="));
  Serial.println(pulseUs);
}

// -----------------------------------------------------------------------------
// PCA9685 OUTPUT MAPPING
// -----------------------------------------------------------------------------

uint16_t servoAngleToPulseUs(uint8_t index, float logicalAngleDeg) {
  const ServoCalibration &cfg = servoConfig[index];
  float clamped = clampFloat(logicalAngleDeg, cfg.minAngleDeg, cfg.maxAngleDeg);

  float ratio = 0.0f;
  float span = cfg.maxAngleDeg - cfg.minAngleDeg;
  if (span > 0.0001f) {
    ratio = (clamped - cfg.minAngleDeg) / span;
  }
  if (cfg.invert) {
    ratio = 1.0f - ratio;
  }

  float pulse = (float)cfg.minPulseUs + ratio * (float)(cfg.maxPulseUs - cfg.minPulseUs);
  pulse = clampFloat(pulse, (float)cfg.minPulseUs, (float)cfg.maxPulseUs);
  return (uint16_t)(pulse + 0.5f);
}

uint16_t pulseUsToTicks(uint16_t pulseUs) {
  float periodUs = 1000000.0f / PCA9685_PWM_FREQUENCY_HZ;
  float ticks = (4096.0f * pulseUs) / periodUs;
  if (ticks < 0.0f) {
    ticks = 0.0f;
  }
  if (ticks > 4095.0f) {
    ticks = 4095.0f;
  }
  return (uint16_t)(ticks + 0.5f);
}

void writeServoPulseUs(uint8_t index, uint16_t pulseUs) {
  if (index >= SERVO_COUNT) {
    return;
  }
  pwm.setPWM(servoConfig[index].channel, 0, pulseUsToTicks(pulseUs));
}

void disableOutputs() {
  outputEnabled = false;
  stopSweep();
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    pwm.setPWM(servoConfig[i].channel, 0, 0);
  }
  Serial.println(F("Outputs disabled. Servos are no longer actively driven."));
}

void enableOutputs() {
  outputEnabled = true;
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    writeServoPulseUs(i, servoAngleToPulseUs(i, currentAngleDeg[i]));
  }
  Serial.println(F("Outputs enabled."));
}

// -----------------------------------------------------------------------------
// UTILITY HELPERS
// -----------------------------------------------------------------------------

float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}

float parseFloatArg(char *token, bool &ok) {
  if (token == NULL) {
    ok = false;
    return 0.0f;
  }
  char *endPtr = NULL;
  float value = strtof(token, &endPtr);
  if (endPtr == token || *endPtr != '\0') {
    ok = false;
  }
  return value;
}

long parseLongArg(char *token, bool &ok) {
  if (token == NULL) {
    ok = false;
    return 0;
  }
  char *endPtr = NULL;
  long value = strtol(token, &endPtr, 10);
  if (endPtr == token || *endPtr != '\0') {
    ok = false;
  }
  return value;
}

int parseServoIndex(char *token, bool &ok) {
  long value = parseLongArg(token, ok);
  if (!ok || value < 0 || value >= SERVO_COUNT) {
    ok = false;
    return 0;
  }
  return (int)value;
}
