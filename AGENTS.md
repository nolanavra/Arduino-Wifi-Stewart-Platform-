Write a complete Arduino program for a 6-DOF Stewart platform driven by 6 hobby servos using:

- Arduino Uno R4 WiFi as the main controller
- HW-170 16-channel servo controller board (PCA9685-based, I2C)
- A simple built-in web interface hosted by the Arduino Uno R4 WiFi for controlling the platform from a browser

Hardware assumptions:
- Main board: Arduino Uno R4 WiFi
- Networking: use the Uno R4 WiFi’s built-in Wi-Fi with the WiFiS3 library
- Servo controller: HW-170 / PCA9685 over I2C
- 6 servos connected to PCA9685 channels 0–5
- Stewart platform is a 6-servo rotary-servo design
- Browser clients on the same network should be able to open a simple control page served by the Arduino

Library and platform requirements:
- Use WiFiS3 for Wi-Fi networking and web server functionality, appropriate for Arduino Uno R4 WiFi
- Use a PCA9685-compatible Arduino library such as Adafruit PWM Servo Driver
- Do NOT use Servo.h for servo output
- Program must be real, compilable Arduino C++ for Uno R4 WiFi
- Prefer robust code organization and clarity over compactness

Core functionality:
- Implement inverse kinematics for a 6-DOF Stewart platform
- Accept target pose values:
  - X, Y, Z
  - roll, pitch, yaw
- Solve the required leg geometry
- Convert solved servo angles into PCA9685 pulse outputs
- Command the 6 servos safely through the HW-170 board

Web interface requirements:
- Create a simple embedded web server on the Uno R4 WiFi
- Serve a basic HTML control page that works on desktop and mobile browsers
- The web page must include:
  - sliders or numeric inputs for X, Y, Z
  - sliders or numeric inputs for roll, pitch, yaw
  - a HOME button
  - a STOP button
  - a STATUS section showing current pose and system state
  - a SEND / APPLY button
- Keep the HTML simple enough to store directly in the sketch as a raw string or PROGMEM content
- The page should submit commands to Arduino over HTTP using simple GET or POST endpoints
- Also provide a minimal JSON or plain-text status endpoint for debugging
- Avoid heavyweight frontend frameworks; use plain HTML/CSS/JavaScript only
- Keep memory use reasonable for Uno R4 WiFi

Behavior requirements:
- On startup:
  - initialize serial output
  - initialize I2C
  - initialize PCA9685
  - connect to Wi-Fi using configurable SSID and password
  - start a web server
  - move platform to a neutral home pose
- Support both browser-based control and serial control
- Serial command format should still support:
  - POSE x y z roll pitch yaw
  - HOME
  - STATUS
  - HELP
  - STOP
- Motion must be smoothed/interpolated instead of jumping instantly
- Reject unreachable poses
- Prevent invalid leg geometry
- Clamp servo outputs to safe limits
- Include a safe stop behavior that halts motion and ignores further movement until reset or re-enabled

Configuration section:
Create a clearly labeled CONFIG section at the top of the sketch with editable constants for:
- Wi-Fi SSID
- Wi-Fi password
- optional static IP settings or DHCP choice
- PCA9685 I2C address
- PCA9685 PWM frequency
- base joint coordinates
- platform joint coordinates
- servo horn length
- connecting rod length
- servo zero offsets
- servo direction inversion
- per-servo min/max angle
- per-servo min/max pulse width in microseconds
- channel mapping for each servo
- neutral/home pose
- smoothing speed / interpolation step size
- workspace limits for X, Y, Z, roll, pitch, yaw

Code organization:
Structure the sketch into clean functions such as:
- initializeHardware()
- initializeWiFi()
- initializeWebServer()
- initializeDriver()
- computeRotationMatrix()
- transformPlatformPoints()
- solveInverseKinematics()
- legLengthToServoAngle()
- servoAngleToPulse()
- writeServoPulse()
- moveToPoseSmooth()
- handleWebClient()
- handleHttpRequest()
- parseSerialCommand()
- printDebugInfo()
- goHome()
- stopMotion()

Math and implementation notes:
- Thoroughly comment the inverse kinematics math
- Explain the coordinate system at the top of the file
- Explain how platform coordinates are transformed by roll, pitch, yaw
- Explain how servo angle is mapped to PCA9685 pulse width
- Use structs for vectors, poses, and servo calibration data
- Use arrays for the 6 joints and 6 servos
- Avoid pseudocode: provide real, compilable code

Web API requirements:
Provide simple endpoints such as:
- GET /              -> serve control page
- GET /status        -> return current pose and state
- GET /home          -> command home pose
- GET /stop          -> stop motion
- GET or POST /pose  -> accept x, y, z, roll, pitch, yaw parameters
Use straightforward parsing and validate all incoming values before moving the platform

Safety requirements:
- Never send invalid pulses to servos
- Detect impossible IK solutions
- Reject commands outside configured workspace
- Print error messages to Serial and expose basic error state in /status
- Ensure startup behavior is safe even if Wi-Fi is unavailable
- If Wi-Fi connection fails, continue allowing serial control and keep retry logic simple and non-blocking if possible

Documentation to include after the code:
1. Wiring summary for Uno R4 WiFi to HW-170 / PCA9685
2. Servo power wiring notes and warnings about external power
3. Wi-Fi setup notes
4. How to open the control page in a browser
5. Serial command examples
6. Calibration procedure for servo zeroing
7. Notes on tuning workspace and pulse limits
8. Example home pose and 3 example pose commands
9. Brief explanation of how to modify the web page layout

Important assumptions:
- Assume the Arduino Uno R4 WiFi uses the WiFiS3 library because its onboard ESP32-S3 module provides Wi-Fi networking
- Assume the HW-170 behaves as a standard PCA9685 servo driver over I2C
- Keep the web UI intentionally simple and reliable rather than fancy
- Prefer code that is easy to modify for a real prototype build
