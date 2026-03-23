#include <WiFiS3.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

/*
  StewartPlatformController.ino

  Coordinate system:
  - X axis: positive to the platform's right when viewed from the front.
  - Y axis: positive toward the front of the platform.
  - Z axis: positive upward away from the base.
  - Roll: rotation about X axis in degrees.
  - Pitch: rotation about Y axis in degrees.
  - Yaw: rotation about Z axis in degrees.

  Rotary-servo Stewart platforms require calibration of linkage geometry. This first-pass
  sketch provides a complete control framework for an Arduino Uno R4 WiFi using the
  WiFiS3 library and a PCA9685-compatible HW-170 servo driver. The inverse kinematics
  pipeline is intentionally organized for future refinement; placeholder geometry values
  and a conservative servo-angle approximation are clearly marked in the CONFIG section.
*/

// -----------------------------------------------------------------------------
// CONFIG SECTION
// -----------------------------------------------------------------------------

static const uint8_t SERVO_COUNT = 6;
static const char WIFI_SSID[] = "Abnormal Net";
static const char WIFI_PASSWORD[] = "unevenmango994";
static const bool USE_STATIC_IP = false;
static IPAddress STATIC_IP(192, 168, 1, 70);
static IPAddress STATIC_GATEWAY(192, 168, 1, 1);
static IPAddress STATIC_SUBNET(255, 255, 255, 0);
static IPAddress STATIC_DNS(8, 8, 8, 8);

static const uint8_t PCA9685_DRIVER_ADDRESS = 0x40;
static const float PCA9685_DRIVER_PWM_FREQUENCY = 50.0f;
static const uint32_t PCA9685_DRIVER_OSCILLATOR_HZ = 27000000UL;
static const uint16_t HTTP_PORT = 80;

// Placeholder geometry values in millimeters for initial development.
// TODO: Replace with measured coordinates from the real base and moving platform.
struct Vec3 {
  float x;
  float y;
  float z;
};

struct Pose {
  float x;
  float y;
  float z;
  float roll;
  float pitch;
  float yaw;
};

struct ServoCalibration {
  uint8_t channel;
  float zeroOffsetDeg;
  bool invert;
  float minAngleDeg;
  float maxAngleDeg;
  uint16_t minPulseUs;
  uint16_t maxPulseUs;
};

struct SystemState {
  bool wifiConnected;
  bool webServerStarted;
  bool stopped;
  bool motionEnabled;
  bool hasActiveError;
  String lastError;
  String lastInfo;
};

static const Vec3 BASE_JOINTS[SERVO_COUNT] = {
  { 95.0f,   0.0f, 0.0f},
  { 47.5f,  82.3f, 0.0f},
  {-47.5f,  82.3f, 0.0f},
  {-95.0f,   0.0f, 0.0f},
  {-47.5f, -82.3f, 0.0f},
  { 47.5f, -82.3f, 0.0f}
};

static const Vec3 PLATFORM_JOINTS[SERVO_COUNT] = {
  { 70.0f,  10.0f, 0.0f},
  { 26.3f,  65.8f, 0.0f},
  {-26.3f,  65.8f, 0.0f},
  {-70.0f,  10.0f, 0.0f},
  {-43.7f, -55.8f, 0.0f},
  { 43.7f, -55.8f, 0.0f}
};

static const float SERVO_HORN_LENGTH_MM = 20.0f;
static const float CONNECTING_ROD_LENGTH_MM = 120.0f;

static const ServoCalibration SERVO_CAL[SERVO_COUNT] = {
  {0, 90.0f, false, 10.0f, 170.0f, 500, 2500},
  {1, 90.0f, true,  10.0f, 170.0f, 500, 2500},
  {2, 90.0f, false, 10.0f, 170.0f, 500, 2500},
  {3, 90.0f, true,  10.0f, 170.0f, 500, 2500},
  {4, 90.0f, false, 10.0f, 170.0f, 500, 2500},
  {5, 90.0f, true,  10.0f, 170.0f, 500, 2500}
};

static const Pose HOME_POSE = {0.0f, 0.0f, 105.0f, 0.0f, 0.0f, 0.0f};
static const float SMOOTHING_TRANSLATION_STEP_MM = 1.5f;
static const float SMOOTHING_ROTATION_STEP_DEG = 1.2f;
static const unsigned long MOTION_UPDATE_INTERVAL_MS = 20;
static const unsigned long WIFI_RETRY_INTERVAL_MS = 10000;
static const unsigned long STATUS_PRINT_INTERVAL_MS = 5000;
static const unsigned long SERIAL_BAUD = 115200;

static const float WORKSPACE_X_MIN = -25.0f;
static const float WORKSPACE_X_MAX =  25.0f;
static const float WORKSPACE_Y_MIN = -25.0f;
static const float WORKSPACE_Y_MAX =  25.0f;
static const float WORKSPACE_Z_MIN =  85.0f;
static const float WORKSPACE_Z_MAX = 130.0f;
static const float WORKSPACE_ROLL_MIN  = -15.0f;
static const float WORKSPACE_ROLL_MAX  =  15.0f;
static const float WORKSPACE_PITCH_MIN = -15.0f;
static const float WORKSPACE_PITCH_MAX =  15.0f;
static const float WORKSPACE_YAW_MIN   = -20.0f;
static const float WORKSPACE_YAW_MAX   =  20.0f;

// -----------------------------------------------------------------------------
// GLOBAL STATE
// -----------------------------------------------------------------------------

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_DRIVER_ADDRESS);
WiFiServer server(HTTP_PORT);
SystemState systemState = {false, false, false, true, false, "", "Booting"};

Pose currentPose = HOME_POSE;
Pose targetPose = HOME_POSE;
Pose lastSolvedPose = HOME_POSE;
float currentServoAngles[SERVO_COUNT] = {90, 90, 90, 90, 90, 90};
float targetServoAngles[SERVO_COUNT] = {90, 90, 90, 90, 90, 90};
float lastLegLengths[SERVO_COUNT] = {0, 0, 0, 0, 0, 0};

unsigned long lastMotionUpdateMs = 0;
unsigned long lastWifiAttemptMs = 0;
unsigned long lastStatusPrintMs = 0;
String serialLineBuffer;

static const char CONTROL_PAGE[] = R"HTML(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Stewart Platform</title>
  <style>
    body { font-family: Arial, sans-serif; margin: 1rem; background: #f4f7fb; color: #1d2733; }
    .card { max-width: 640px; margin: 0 auto; padding: 1rem; background: #fff; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.08); }
    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 0.75rem; }
    label { display: block; font-size: 0.9rem; margin-bottom: 0.25rem; }
    input { width: 100%; padding: 0.5rem; box-sizing: border-box; }
    button { padding: 0.7rem 1rem; margin: 0.25rem 0.25rem 0.25rem 0; }
    pre { background: #0e1621; color: #dce6f2; padding: 0.75rem; border-radius: 8px; min-height: 7rem; white-space: pre-wrap; }
  </style>
</head>
<body>
  <div class="card">
    <h1>Stewart Platform Control</h1>
    <div class="grid">
      <div><label for="x">X (mm)</label><input id="x" type="number" step="0.1" value="0"></div>
      <div><label for="y">Y (mm)</label><input id="y" type="number" step="0.1" value="0"></div>
      <div><label for="z">Z (mm)</label><input id="z" type="number" step="0.1" value="105"></div>
      <div><label for="roll">Roll (deg)</label><input id="roll" type="number" step="0.1" value="0"></div>
      <div><label for="pitch">Pitch (deg)</label><input id="pitch" type="number" step="0.1" value="0"></div>
      <div><label for="yaw">Yaw (deg)</label><input id="yaw" type="number" step="0.1" value="0"></div>
    </div>
    <p>
      <button onclick="applyPose()">Apply</button>
      <button onclick="homePose()">Home</button>
      <button onclick="stopMotion()">Stop</button>
      <button onclick="refreshStatus()">Status refresh</button>
    </p>
    <h2>Status</h2>
    <pre id="status">Waiting for status...</pre>
  </div>
<script>
async function request(path) {
  const response = await fetch(path);
  const text = await response.text();
  document.getElementById('status').textContent = text;
}
function values() {
  const ids = ['x', 'y', 'z', 'roll', 'pitch', 'yaw'];
  return ids.map(id => id + '=' + encodeURIComponent(document.getElementById(id).value)).join('&');
}
function applyPose() { request('/pose?' + values()); }
function homePose() { request('/home'); }
function stopMotion() { request('/stop'); }
function refreshStatus() { request('/status'); }
refreshStatus();
</script>
</body>
</html>
)HTML";

// -----------------------------------------------------------------------------
// FUNCTION DECLARATIONS
// -----------------------------------------------------------------------------

void initializeHardware();
void initializeWiFi();
void initializeWebServer();
void initializeDriver();
void computeRotationMatrix(float rollDeg, float pitchDeg, float yawDeg, float matrix[3][3]);
void transformPlatformPoints(const Pose &pose, Vec3 transformedPoints[SERVO_COUNT]);
bool solveInverseKinematics(const Pose &pose, float servoAnglesOut[SERVO_COUNT], float legLengthsOut[SERVO_COUNT]);
float legLengthToServoAngle(uint8_t index, float legLengthMm);
uint16_t servoAngleToPulse(uint8_t index, float angleDeg);
void writeServoPulse(uint8_t index, uint16_t pulseUs);
void moveToPoseSmooth();
void handleWebClient();
void handleHttpRequest(WiFiClient &client, const String &requestLine, const String &requestHeaders);
void parseSerialCommand();
void printDebugInfo();
void goHome();
void stopMotion();
bool isPoseWithinWorkspace(const Pose &pose);
bool parsePoseFromQuery(const String &query, Pose &poseOut);
String buildStatusJson();
String urlDecode(const String &input);
float readQueryFloat(const String &query, const char *key, float fallbackValue, bool &foundAllRequired);
float clampFloat(float value, float minValue, float maxValue);
float shortestStep(float currentValue, float targetValue, float maxStep);
void setError(const String &message);
void clearError(const String &infoMessage);
void applyCurrentServoTargets();

// -----------------------------------------------------------------------------
// SETUP AND LOOP
// -----------------------------------------------------------------------------

void setup() {
  initializeHardware();
  initializeDriver();
  initializeWiFi();
  initializeWebServer();
  goHome();
}

void loop() {
  handleWebClient();
  parseSerialCommand();
  moveToPoseSmooth();

  if (!systemState.wifiConnected && (millis() - lastWifiAttemptMs >= WIFI_RETRY_INTERVAL_MS)) {
    initializeWiFi();
  }

  if (millis() - lastStatusPrintMs >= STATUS_PRINT_INTERVAL_MS) {
    printDebugInfo();
    lastStatusPrintMs = millis();
  }
}

// -----------------------------------------------------------------------------
// INITIALIZATION
// -----------------------------------------------------------------------------

void initializeHardware() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial && millis() < 3000) {
    // Allow serial port time to become available without blocking forever.
  }

  Serial.println();
  Serial.println(F("[BOOT] Stewart Platform Controller starting"));
  Serial.println(F("[BOOT] Initializing I2C"));
  Wire.begin();
  Wire.setClock(400000);
  serialLineBuffer.reserve(128);
}

void initializeDriver() {
  Serial.println(F("[PCA9685] Initializing servo driver"));
  pwm.begin();
  pwm.setOscillatorFrequency(PCA9685_DRIVER_OSCILLATOR_HZ);
  pwm.setPWMFreq(PCA9685_DRIVER_PWM_FREQUENCY);
  delay(10);
  clearError("Servo driver initialized");
}

void initializeWiFi() {
  lastWifiAttemptMs = millis();
  Serial.println(F("[WIFI] Initializing Wi-Fi"));

  if (WiFi.status() == WL_NO_MODULE) {
    setError("Wi-Fi module not detected; serial control only");
    systemState.wifiConnected = false;
    return;
  }

  if (USE_STATIC_IP) {
    WiFi.config(STATIC_IP, STATIC_DNS, STATIC_GATEWAY, STATIC_SUBNET);
  }

  uint8_t attempts = 0;
  int status = WL_IDLE_STATUS;
  while (attempts < 3 && status != WL_CONNECTED) {
    Serial.print(F("[WIFI] Connecting attempt "));
    Serial.println(attempts + 1);
    status = WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    unsigned long startMs = millis();
    while (millis() - startMs < 5000 && status != WL_CONNECTED) {
      delay(200);
      status = WiFi.status();
    }
    attempts++;
  }

  if (status == WL_CONNECTED) {
    systemState.wifiConnected = true;
    clearError("Wi-Fi connected");
    Serial.print(F("[WIFI] Connected. IP: "));
    Serial.println(WiFi.localIP());
  } else {
    systemState.wifiConnected = false;
    setError("Wi-Fi unavailable; continuing with serial control");
  }
}

void initializeWebServer() {
  server.begin();
  systemState.webServerStarted = true;
  Serial.print(F("[HTTP] Server started on port "));
  Serial.println(HTTP_PORT);
}

// -----------------------------------------------------------------------------
// KINEMATICS
// -----------------------------------------------------------------------------

void computeRotationMatrix(float rollDeg, float pitchDeg, float yawDeg, float matrix[3][3]) {
  const float roll = radians(rollDeg);
  const float pitch = radians(pitchDeg);
  const float yaw = radians(yawDeg);

  const float cr = cosf(roll);
  const float sr = sinf(roll);
  const float cp = cosf(pitch);
  const float sp = sinf(pitch);
  const float cy = cosf(yaw);
  const float sy = sinf(yaw);

  // ZYX rotation: world = Rz(yaw) * Ry(pitch) * Rx(roll) * local
  matrix[0][0] = cy * cp;
  matrix[0][1] = cy * sp * sr - sy * cr;
  matrix[0][2] = cy * sp * cr + sy * sr;
  matrix[1][0] = sy * cp;
  matrix[1][1] = sy * sp * sr + cy * cr;
  matrix[1][2] = sy * sp * cr - cy * sr;
  matrix[2][0] = -sp;
  matrix[2][1] = cp * sr;
  matrix[2][2] = cp * cr;
}

void transformPlatformPoints(const Pose &pose, Vec3 transformedPoints[SERVO_COUNT]) {
  float rotation[3][3];
  computeRotationMatrix(pose.roll, pose.pitch, pose.yaw, rotation);

  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    const Vec3 &local = PLATFORM_JOINTS[i];
    transformedPoints[i].x = rotation[0][0] * local.x + rotation[0][1] * local.y + rotation[0][2] * local.z + pose.x;
    transformedPoints[i].y = rotation[1][0] * local.x + rotation[1][1] * local.y + rotation[1][2] * local.z + pose.y;
    transformedPoints[i].z = rotation[2][0] * local.x + rotation[2][1] * local.y + rotation[2][2] * local.z + pose.z;
  }
}

bool solveInverseKinematics(const Pose &pose, float servoAnglesOut[SERVO_COUNT], float legLengthsOut[SERVO_COUNT]) {
  if (!isPoseWithinWorkspace(pose)) {
    setError("Requested pose exceeds workspace limits");
    return false;
  }

  Vec3 transformedPlatform[SERVO_COUNT];
  transformPlatformPoints(pose, transformedPlatform);

  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    const float dx = transformedPlatform[i].x - BASE_JOINTS[i].x;
    const float dy = transformedPlatform[i].y - BASE_JOINTS[i].y;
    const float dz = transformedPlatform[i].z - BASE_JOINTS[i].z;
    const float legLength = sqrtf(dx * dx + dy * dy + dz * dz);
    legLengthsOut[i] = legLength;

    // Basic validity checks. For a real rotary-servo Stewart platform this function should
    // be replaced with exact horn-angle geometry using measured pivot offsets and rod-end
    // attachment points. This placeholder maps leg-length delta to a servo angle around
    // each servo's calibrated neutral offset while still rejecting obviously impossible poses.
    const float minReach = fabsf(CONNECTING_ROD_LENGTH_MM - SERVO_HORN_LENGTH_MM) + 5.0f;
    const float maxReach = CONNECTING_ROD_LENGTH_MM + SERVO_HORN_LENGTH_MM - 5.0f;
    if (legLength < minReach || legLength > maxReach) {
      setError("Inverse kinematics rejected unreachable leg length");
      return false;
    }

    const float angle = legLengthToServoAngle(i, legLength);
    if (angle < SERVO_CAL[i].minAngleDeg || angle > SERVO_CAL[i].maxAngleDeg) {
      setError("Inverse kinematics produced servo angle outside safe bounds");
      return false;
    }
    servoAnglesOut[i] = angle;
  }

  clearError("IK solve ok");
  lastSolvedPose = pose;
  return true;
}

float legLengthToServoAngle(uint8_t index, float legLengthMm) {
  const float neutralLength = sqrtf(HOME_POSE.x * HOME_POSE.x + HOME_POSE.y * HOME_POSE.y + HOME_POSE.z * HOME_POSE.z);
  const float deltaLength = legLengthMm - neutralLength;
  const float gainDegPerMm = 1.25f; // TODO: Replace with calibrated rotary-servo linkage math.
  float angle = SERVO_CAL[index].zeroOffsetDeg + (SERVO_CAL[index].invert ? -deltaLength : deltaLength) * gainDegPerMm;
  angle = clampFloat(angle, SERVO_CAL[index].minAngleDeg, SERVO_CAL[index].maxAngleDeg);
  return angle;
}

uint16_t servoAngleToPulse(uint8_t index, float angleDeg) {
  const ServoCalibration &cal = SERVO_CAL[index];
  const float clampedAngle = clampFloat(angleDeg, cal.minAngleDeg, cal.maxAngleDeg);
  const float angleSpan = cal.maxAngleDeg - cal.minAngleDeg;
  float normalized = 0.0f;
  if (angleSpan > 0.001f) {
    normalized = (clampedAngle - cal.minAngleDeg) / angleSpan;
  }
  float pulseUs = cal.minPulseUs + normalized * (cal.maxPulseUs - cal.minPulseUs);
  pulseUs = clampFloat(pulseUs, cal.minPulseUs, cal.maxPulseUs);
  return static_cast<uint16_t>(pulseUs + 0.5f);
}

void writeServoPulse(uint8_t index, uint16_t pulseUs) {
  if (index >= SERVO_COUNT) {
    return;
  }
  if (systemState.stopped || !systemState.motionEnabled) {
    return;
  }

  const ServoCalibration &cal = SERVO_CAL[index];
  const uint16_t clampedPulse = constrain(pulseUs, cal.minPulseUs, cal.maxPulseUs);
  const float microsecondsPerPeriod = 1000000.0f / PCA9685_DRIVER_PWM_FREQUENCY;
  const float ticksPerMicrosecond = 4096.0f / microsecondsPerPeriod;
  const uint16_t ticks = static_cast<uint16_t>(clampedPulse * ticksPerMicrosecond);
  pwm.setPWM(cal.channel, 0, ticks);
}

// -----------------------------------------------------------------------------
// MOTION CONTROL
// -----------------------------------------------------------------------------

void moveToPoseSmooth() {
  if (millis() - lastMotionUpdateMs < MOTION_UPDATE_INTERVAL_MS) {
    return;
  }
  lastMotionUpdateMs = millis();

  if (systemState.stopped || !systemState.motionEnabled) {
    return;
  }

  Pose nextPose = currentPose;
  nextPose.x = shortestStep(currentPose.x, targetPose.x, SMOOTHING_TRANSLATION_STEP_MM);
  nextPose.y = shortestStep(currentPose.y, targetPose.y, SMOOTHING_TRANSLATION_STEP_MM);
  nextPose.z = shortestStep(currentPose.z, targetPose.z, SMOOTHING_TRANSLATION_STEP_MM);
  nextPose.roll = shortestStep(currentPose.roll, targetPose.roll, SMOOTHING_ROTATION_STEP_DEG);
  nextPose.pitch = shortestStep(currentPose.pitch, targetPose.pitch, SMOOTHING_ROTATION_STEP_DEG);
  nextPose.yaw = shortestStep(currentPose.yaw, targetPose.yaw, SMOOTHING_ROTATION_STEP_DEG);

  float solvedAngles[SERVO_COUNT];
  float solvedLegs[SERVO_COUNT];
  if (!solveInverseKinematics(nextPose, solvedAngles, solvedLegs)) {
    Serial.println(F("[MOTION] IK solve failed, holding current pose"));
    return;
  }

  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    targetServoAngles[i] = solvedAngles[i];
    lastLegLengths[i] = solvedLegs[i];
  }

  currentPose = nextPose;
  applyCurrentServoTargets();
}

void applyCurrentServoTargets() {
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    currentServoAngles[i] = targetServoAngles[i];
    const uint16_t pulseUs = servoAngleToPulse(i, currentServoAngles[i]);
    writeServoPulse(i, pulseUs);
  }
}

void goHome() {
  systemState.stopped = false;
  systemState.motionEnabled = true;
  targetPose = HOME_POSE;
  Serial.println(F("[CMD] HOME requested"));

  float startupAngles[SERVO_COUNT];
  float startupLegs[SERVO_COUNT];
  if (solveInverseKinematics(HOME_POSE, startupAngles, startupLegs)) {
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
      currentServoAngles[i] = startupAngles[i];
      targetServoAngles[i] = startupAngles[i];
      lastLegLengths[i] = startupLegs[i];
    }
    currentPose = HOME_POSE;
    applyCurrentServoTargets();
  } else {
    setError("Home pose invalid; outputs not updated");
  }
}

void stopMotion() {
  systemState.stopped = true;
  systemState.motionEnabled = false;
  systemState.lastInfo = "Motion stopped";
  Serial.println(F("[CMD] STOP engaged; outputs frozen"));
}

// -----------------------------------------------------------------------------
// WEB SERVER
// -----------------------------------------------------------------------------

void handleWebClient() {
  WiFiClient client = server.available();
  if (!client) {
    return;
  }

  String requestLine;
  String headers;
  unsigned long startMs = millis();
  while (client.connected() && millis() - startMs < 1000) {
    while (client.available()) {
      char c = static_cast<char>(client.read());
      headers += c;
      if (c == '\n' && requestLine.length() == 0) {
        int lineEnd = headers.indexOf('\r');
        if (lineEnd >= 0) {
          requestLine = headers.substring(0, lineEnd);
        }
      }
      if (headers.endsWith("\r\n\r\n")) {
        handleHttpRequest(client, requestLine, headers);
        delay(1);
        client.stop();
        return;
      }
    }
  }
  client.stop();
}

void handleHttpRequest(WiFiClient &client, const String &requestLine, const String &requestHeaders) {
  (void)requestHeaders;
  Serial.print(F("[HTTP] "));
  Serial.println(requestLine);

  int firstSpace = requestLine.indexOf(' ');
  int secondSpace = requestLine.indexOf(' ', firstSpace + 1);
  if (firstSpace < 0 || secondSpace < 0) {
    client.println(F("HTTP/1.1 400 Bad Request\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nBad request"));
    return;
  }

  const String method = requestLine.substring(0, firstSpace);
  const String fullPath = requestLine.substring(firstSpace + 1, secondSpace);
  String path = fullPath;
  String query;
  int queryIndex = fullPath.indexOf('?');
  if (queryIndex >= 0) {
    path = fullPath.substring(0, queryIndex);
    query = fullPath.substring(queryIndex + 1);
  }

  if (method != "GET" && method != "POST") {
    client.println(F("HTTP/1.1 405 Method Not Allowed\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nMethod not allowed"));
    return;
  }

  if (path == "/") {
    client.println(F("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n"));
    client.print(CONTROL_PAGE);
    return;
  }

  if (path == "/status") {
    client.println(F("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n"));
    client.print(buildStatusJson());
    return;
  }

  if (path == "/home") {
    goHome();
    client.println(F("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n"));
    client.print(buildStatusJson());
    return;
  }

  if (path == "/stop") {
    stopMotion();
    client.println(F("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n"));
    client.print(buildStatusJson());
    return;
  }

  if (path == "/pose") {
    Pose requestedPose = currentPose;
    if (!parsePoseFromQuery(query, requestedPose)) {
      client.println(F("HTTP/1.1 400 Bad Request\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nInvalid pose parameters"));
      return;
    }

    if (systemState.stopped) {
      client.println(F("HTTP/1.1 423 Locked\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nSystem is stopped; send /home to re-enable"));
      return;
    }

    float solvedAngles[SERVO_COUNT];
    float solvedLegs[SERVO_COUNT];
    if (!solveInverseKinematics(requestedPose, solvedAngles, solvedLegs)) {
      client.println(F("HTTP/1.1 422 Unprocessable Content\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nPose rejected by safety checks"));
      return;
    }

    targetPose = requestedPose;
    client.println(F("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n"));
    client.print(buildStatusJson());
    return;
  }

  client.println(F("HTTP/1.1 404 Not Found\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nNot found"));
}

// -----------------------------------------------------------------------------
// SERIAL COMMANDS
// -----------------------------------------------------------------------------

void parseSerialCommand() {
  while (Serial.available()) {
    char c = static_cast<char>(Serial.read());
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      String line = serialLineBuffer;
      serialLineBuffer = "";
      line.trim();
      if (line.length() == 0) {
        return;
      }

      Serial.print(F("[SERIAL] Received: "));
      Serial.println(line);

      if (line.equalsIgnoreCase("HOME")) {
        goHome();
      } else if (line.equalsIgnoreCase("STOP")) {
        stopMotion();
      } else if (line.equalsIgnoreCase("STATUS")) {
        Serial.println(buildStatusJson());
      } else if (line.equalsIgnoreCase("HELP")) {
        Serial.println(F("Commands: HOME, STOP, STATUS, HELP, POSE x y z roll pitch yaw"));
      } else if (line.startsWith("POSE ")) {
        char buffer[96];
        line.toCharArray(buffer, sizeof(buffer));
        Pose pose;
        int parsed = sscanf(buffer, "POSE %f %f %f %f %f %f", &pose.x, &pose.y, &pose.z, &pose.roll, &pose.pitch, &pose.yaw);
        if (parsed == 6 && isPoseWithinWorkspace(pose)) {
          if (systemState.stopped) {
            Serial.println(F("[SERIAL] Ignored POSE because system is stopped. Send HOME to re-enable."));
          } else {
            float solvedAngles[SERVO_COUNT];
            float solvedLegs[SERVO_COUNT];
            if (solveInverseKinematics(pose, solvedAngles, solvedLegs)) {
              targetPose = pose;
              Serial.println(F("[SERIAL] Target pose accepted"));
            } else {
              Serial.println(F("[SERIAL] POSE rejected by IK/safety checks"));
            }
          }
        } else {
          Serial.println(F("[SERIAL] Invalid POSE format or values"));
        }
      } else {
        Serial.println(F("[SERIAL] Unknown command. Send HELP."));
      }
      return;
    }
    if (serialLineBuffer.length() < 120) {
      serialLineBuffer += c;
    }
  }
}

// -----------------------------------------------------------------------------
// DIAGNOSTICS AND HELPERS
// -----------------------------------------------------------------------------

void printDebugInfo() {
  Serial.print(F("[STATUS] pose="));
  Serial.print(currentPose.x, 2); Serial.print(',');
  Serial.print(currentPose.y, 2); Serial.print(',');
  Serial.print(currentPose.z, 2); Serial.print(',');
  Serial.print(currentPose.roll, 2); Serial.print(',');
  Serial.print(currentPose.pitch, 2); Serial.print(',');
  Serial.print(currentPose.yaw, 2);
  Serial.print(F(" targetZ="));
  Serial.print(targetPose.z, 2);
  Serial.print(F(" stopped="));
  Serial.print(systemState.stopped ? F("true") : F("false"));
  Serial.print(F(" wifi="));
  Serial.println(systemState.wifiConnected ? F("connected") : F("offline"));
}

bool isPoseWithinWorkspace(const Pose &pose) {
  return pose.x >= WORKSPACE_X_MIN && pose.x <= WORKSPACE_X_MAX &&
         pose.y >= WORKSPACE_Y_MIN && pose.y <= WORKSPACE_Y_MAX &&
         pose.z >= WORKSPACE_Z_MIN && pose.z <= WORKSPACE_Z_MAX &&
         pose.roll >= WORKSPACE_ROLL_MIN && pose.roll <= WORKSPACE_ROLL_MAX &&
         pose.pitch >= WORKSPACE_PITCH_MIN && pose.pitch <= WORKSPACE_PITCH_MAX &&
         pose.yaw >= WORKSPACE_YAW_MIN && pose.yaw <= WORKSPACE_YAW_MAX;
}

bool parsePoseFromQuery(const String &query, Pose &poseOut) {
  bool foundAllRequired = true;
  poseOut.x = readQueryFloat(query, "x", poseOut.x, foundAllRequired);
  poseOut.y = readQueryFloat(query, "y", poseOut.y, foundAllRequired);
  poseOut.z = readQueryFloat(query, "z", poseOut.z, foundAllRequired);
  poseOut.roll = readQueryFloat(query, "roll", poseOut.roll, foundAllRequired);
  poseOut.pitch = readQueryFloat(query, "pitch", poseOut.pitch, foundAllRequired);
  poseOut.yaw = readQueryFloat(query, "yaw", poseOut.yaw, foundAllRequired);
  if (!foundAllRequired) {
    setError("HTTP pose command missing one or more parameters");
    return false;
  }
  if (!isPoseWithinWorkspace(poseOut)) {
    setError("HTTP pose command outside configured workspace");
    return false;
  }
  return true;
}

String buildStatusJson() {
  String json = "{";
  json += "\"wifiConnected\":" + String(systemState.wifiConnected ? "true" : "false");
  json += ",\"webServerStarted\":" + String(systemState.webServerStarted ? "true" : "false");
  json += ",\"stopped\":" + String(systemState.stopped ? "true" : "false");
  json += ",\"hasError\":" + String(systemState.hasActiveError ? "true" : "false");
  json += ",\"lastError\":\"" + systemState.lastError + "\"";
  json += ",\"lastInfo\":\"" + systemState.lastInfo + "\"";
  json += ",\"currentPose\":{";
  json += "\"x\":" + String(currentPose.x, 2);
  json += ",\"y\":" + String(currentPose.y, 2);
  json += ",\"z\":" + String(currentPose.z, 2);
  json += ",\"roll\":" + String(currentPose.roll, 2);
  json += ",\"pitch\":" + String(currentPose.pitch, 2);
  json += ",\"yaw\":" + String(currentPose.yaw, 2) + "}";
  json += ",\"targetPose\":{";
  json += "\"x\":" + String(targetPose.x, 2);
  json += ",\"y\":" + String(targetPose.y, 2);
  json += ",\"z\":" + String(targetPose.z, 2);
  json += ",\"roll\":" + String(targetPose.roll, 2);
  json += ",\"pitch\":" + String(targetPose.pitch, 2);
  json += ",\"yaw\":" + String(targetPose.yaw, 2) + "}";
  json += "}";
  return json;
}

String urlDecode(const String &input) {
  String decoded;
  decoded.reserve(input.length());
  for (unsigned int i = 0; i < input.length(); i++) {
    char c = input[i];
    if (c == '+') {
      decoded += ' ';
    } else if (c == '%' && i + 2 < input.length()) {
      char hex[3] = {input[i + 1], input[i + 2], '\0'};
      decoded += static_cast<char>(strtol(hex, NULL, 16));
      i += 2;
    } else {
      decoded += c;
    }
  }
  return decoded;
}

float readQueryFloat(const String &query, const char *key, float fallbackValue, bool &foundAllRequired) {
  String token = String(key) + "=";
  int start = query.indexOf(token);
  if (start < 0) {
    foundAllRequired = false;
    return fallbackValue;
  }
  start += token.length();
  int end = query.indexOf('&', start);
  if (end < 0) {
    end = query.length();
  }
  String value = urlDecode(query.substring(start, end));
  return value.toFloat();
}

float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}

float shortestStep(float currentValue, float targetValue, float maxStep) {
  const float delta = targetValue - currentValue;
  if (fabsf(delta) <= maxStep) {
    return targetValue;
  }
  return currentValue + (delta > 0.0f ? maxStep : -maxStep);
}

void setError(const String &message) {
  systemState.hasActiveError = true;
  systemState.lastError = message;
  systemState.lastInfo = "";
  Serial.print(F("[ERROR] "));
  Serial.println(message);
}

void clearError(const String &infoMessage) {
  systemState.hasActiveError = false;
  systemState.lastError = "";
  systemState.lastInfo = infoMessage;
}

/*
Documentation
1. Wiring summary for Uno R4 WiFi to HW-170 / PCA9685
   - Uno R4 WiFi 5V -> PCA9685 VCC only if your board expects 5V logic reference.
   - Uno R4 WiFi GND -> PCA9685 GND.
   - Uno R4 WiFi SDA -> PCA9685 SDA.
   - Uno R4 WiFi SCL -> PCA9685 SCL.
   - Servos connect to PCA9685 channels 0-5.

2. Servo power wiring notes
   - Power servos from a dedicated external supply sized for stall current.
   - Do not power six hobby servos directly from the Uno 5V pin.
   - Always connect external servo supply ground to Arduino/PCA9685 ground.

3. Wi-Fi setup notes
   - Replace WIFI_SSID and WIFI_PASSWORD in the CONFIG section.
   - Optional static IP configuration can be enabled with USE_STATIC_IP.

4. Opening the control page
   - Open the IP address printed to Serial in a browser on the same network.
   - If Wi-Fi is unavailable, serial control remains active.

5. Serial command examples
   - HOME
   - STOP
   - STATUS
   - HELP
   - POSE 0 0 105 0 0 0

6. Calibration procedure for servo zeroing
   - Start with linkages disconnected.
   - Adjust zeroOffsetDeg and invert for each servo until neutral horn orientation is correct.
   - Reconnect linkages and reduce workspace limits until motion is safe.

7. Workspace and pulse tuning notes
   - Tune WORKSPACE_* limits conservatively first.
   - Tune min/max pulse widths to match the actual servos and horn travel.
   - Replace placeholder BASE_JOINTS and PLATFORM_JOINTS with measured coordinates.

8. Example home pose and test poses
   - Home: POSE 0 0 105 0 0 0
   - Example 1: POSE 5 0 108 2 0 0
   - Example 2: POSE -5 3 110 0 -3 4
   - Example 3: POSE 0 -4 102 -2 2 -5

9. Modifying the web page layout
   - Edit the CONTROL_PAGE raw HTML string.
   - Keep the page simple to stay within available memory.
*/
