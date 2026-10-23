/*
 * Task 9 - LMS Implementation Challenge (Way 1: Simple & Direct ESP32 Line Follower)
 * 
 * Hardware Reference: Strictly follows Appendix A (Tasks.pdf)
 * - Platform     : ESP32-WROOM-32 (Arduino Framework, C++)
 * - IR Array     : 5 Analog Sensors mapped to weights [-2, -1, 0, 1, 2]
 *                  Higher voltage / ADC value = Darker surface (line detected)
 * - Motor Driver : L298N Dual H-Bridge powered by 3S LiPo (11.1 V)
 * - Serial Debug : 115200 baud streaming sensors, error, and motor speeds
 */

#include <Arduino.h>

// ============================================================================
// 1. PIN DEFINITIONS (Strictly as specified in Appendix A - DO NOT CHANGE)
// ============================================================================
const int NUM_SENSORS = 5;

// IR Sensor Array GPIO Pins (Leftmost S1 to Rightmost S5)
const int IR_PINS[NUM_SENSORS] = {34, 35, 32, 33, 25};

// Sensor positional weights from Appendix A: [-2, -1, 0, 1, 2]
const float WEIGHTS[NUM_SENSORS] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};

// L298N Motor Driver GPIO Pins (from Appendix A)
const int PIN_ENA = 14;  // Left motor PWM speed (0-255 via analogWrite)
const int PIN_IN1 = 27;  // Left motor direction A
const int PIN_IN2 = 26;  // Left motor direction B

const int PIN_ENB = 13;  // Right motor PWM speed (0-255 via analogWrite)
const int PIN_IN3 = 12;  // Right motor direction A
const int PIN_IN4 = 11;  // Right motor direction B (exactly as listed in Appendix A)

// ============================================================================
// 2. CALIBRATION & CONTROL VARIABLES
// ============================================================================
// Arrays to store the minimum (white floor) and maximum (dark line) raw ADC values
int sensorMin[NUM_SENSORS] = {4095, 4095, 4095, 4095, 4095};
int sensorMax[NUM_SENSORS] = {0, 0, 0, 0, 0};

// Array to hold normalized sensor readings (scaled 0.0 to 1000.0)
float normVals[NUM_SENSORS] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

// PID Gains (Tuned for smooth differential steering)
float Kp = 60.0f;        // Proportional gain: steers proportionally to current line offset
float Ki = 0.0f;         // Integral gain: eliminates steady-state drift (0 for PD control)
float Kd = 15.0f;        // Derivative gain: damps oscillation and overshoot on turns

// PID State Variables
float prevError = 0.0f;  // Error from the previous loop iteration (for Derivative)
float integral  = 0.0f;  // Accumulated error over time (for Integral)
float lastValidError = 0.0f; // Remembers which side (-left or +right) the line was last seen

// Motor Speed Settings (PWM range: 0 to 255)
const int BASE_SPEED  = 140; // Default forward cruising speed when error == 0
const int MAX_SPEED   = 255; // Maximum allowed PWM magnitude
const int PIVOT_SPEED = 110; // Speed used to spin in place when recovering a lost line

// Lost-Line & Timing Constants
const float LINE_THRESHOLD = 200.0f; // Minimum sum of normalized readings to confirm line is present
const unsigned long LOST_TIMEOUT_MS = 500; // Stop motors safely if line is lost for > 500 ms
unsigned long lineLostStartMs = 0;   // Timestamp when the line was first lost
bool isLineLost = false;             // True when all sensors read ~0 (white background)

// Set to true to validate code with hard-coded sample arrays (Task 9 validation requirement),
// or false to read live physical sensors via analogRead().
const bool USE_SAMPLE_INPUTS = false;

// ============================================================================
// 3. HELPER FUNCTIONS
// ============================================================================

/*
 * Function: clampMotor
 * Purpose : Limits a signed motor command to [-maxMag, +maxMag] while preserving its sign.
 */
int clampMotor(int speed, int maxMag) {
  if (speed > maxMag)  return maxMag;
  if (speed < -maxMag) return -maxMag;
  return speed;
}

/*
 * Function: setMotorSpeeds
 * Purpose : Drives the L298N H-bridge using signed left and right motor commands.
 *           Positive speed = Forward, Negative speed = Reverse, 0 = Stop.
 */
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  // 1. Clamp both commands to [-MAX_SPEED, +MAX_SPEED]
  leftSpeed  = clampMotor(leftSpeed, MAX_SPEED);
  rightSpeed = clampMotor(rightSpeed, MAX_SPEED);

  // 2. Left Motor Direction & PWM (ENA, IN1, IN2)
  if (leftSpeed > 0) {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_ENA, leftSpeed);
  } else if (leftSpeed < 0) {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
    analogWrite(PIN_ENA, -leftSpeed);
  } else {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_ENA, 0);
  }

  // 3. Right Motor Direction & PWM (ENB, IN3, IN4)
  if (rightSpeed > 0) {
    digitalWrite(PIN_IN3, HIGH);
    digitalWrite(PIN_IN4, LOW);
    analogWrite(PIN_ENB, rightSpeed);
  } else if (rightSpeed < 0) {
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, HIGH);
    analogWrite(PIN_ENB, -rightSpeed);
  } else {
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, LOW);
    analogWrite(PIN_ENB, 0);
  }
}

/*
 * Function: calibrateSensors
 * Purpose : Performs min-max calibration at startup by reading all 5 sensors
 *           repeatedly while the robot/sensor array is swept across the line.
 */
void calibrateSensors() {
  Serial.println("Starting Min-Max Sensor Calibration (3 seconds)...");
  unsigned long startMs = millis();

  while (millis() - startMs < 3000) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      int raw = analogRead(IR_PINS[i]);  // 12-bit ADC: 0 to 4095
      if (raw < sensorMin[i]) sensorMin[i] = raw;
      if (raw > sensorMax[i]) sensorMax[i] = raw;
    }
    delay(5);
  }

  // Fallback defaults if a sensor did not see enough contrast during startup
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorMax[i] - sensorMin[i] < 200) {
      sensorMin[i] = 300;
      sensorMax[i] = 3800;
    }
  }
  Serial.println("Calibration Complete.");
}

/*
 * Function: readAndNormalizeSensors
 * Purpose : Reads raw 12-bit ADC values (0-4095) and maps each sensor to 0.0 - 1000.0
 *           using the startup min/max bounds. Higher value = darker surface (line).
 */
void readAndNormalizeSensors(const int rawInput[NUM_SENSORS], float outNorm[NUM_SENSORS]) {
  for (int i = 0; i < NUM_SENSORS; i++) {
    int span = sensorMax[i] - sensorMin[i];
    if (span <= 0) {
      outNorm[i] = 0.0f;
    } else {
      float scaled = (float)(rawInput[i] - sensorMin[i]) * 1000.0f / (float)span;
      if (scaled < 0.0f)    scaled = 0.0f;
      if (scaled > 1000.0f) scaled = 1000.0f;
      outNorm[i] = scaled;
    }
  }
}

/*
 * Function: computeWeightedCentroid
 * Purpose : Computes the weighted average line position in [-2.0, +2.0]:
 *           position = sum(WEIGHTS[i] * v[i]) / sum(v[i])
 *           Sets lineDetected = false if the sum of readings is below LINE_THRESHOLD
 *           (explicitly handling the all-zero / lost-line case without dividing by zero).
 */
float computeWeightedCentroid(const float v[NUM_SENSORS], bool &lineDetected) {
  float weightedSum = 0.0f;
  float totalSum    = 0.0f;

  for (int i = 0; i < NUM_SENSORS; i++) {
    weightedSum += WEIGHTS[i] * v[i];
    totalSum    += v[i];
  }

  // Handle the all-zero / lost-line case explicitly
  if (totalSum < LINE_THRESHOLD || totalSum <= 0.0f) {
    lineDetected = false;
    return 0.0f;
  }

  lineDetected = true;
  return weightedSum / totalSum;
}

/*
 * Function: computePID
 * Purpose : Calculates the differential steering correction from the current error.
 */
float computePID(float error) {
  // Accumulate integral (with simple anti-windup clamp)
  integral += error;
  if (integral > 50.0f)  integral = 50.0f;
  if (integral < -50.0f) integral = -50.0f;

  // Compute rate of change of error
  float derivative = error - prevError;
  prevError = error;

  // Standard PID output equation
  return (Kp * error) + (Ki * integral) + (Kd * derivative);
}

/*
 * Function: printTelemetry
 * Purpose : Streams sensor values, computed error, and motor commands over Serial (Appendix A).
 */
void printTelemetry(const float v[NUM_SENSORS], bool lineSeen, float error, int leftCmd, int rightCmd) {
  Serial.print("S:[");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print((int)v[i]);
    if (i < NUM_SENSORS - 1) Serial.print(",");
  }
  Serial.print("] | Seen:");
  Serial.print(lineSeen ? 1 : 0);
  Serial.print(" | Err:");
  Serial.print(error, 2);
  Serial.print(" | L:");
  Serial.print(leftCmd);
  Serial.print(" R:");
  Serial.println(rightCmd);
}

/*
 * Function: runSampleValidation
 * Purpose : Validates the pipeline against hard-coded sample sensor arrays (Task 9 prompt).
 */
void runSampleValidation() {
  Serial.println("\n--- Running Task 9 Sample Input Validation ---");
  // Set known calibration bounds for deterministic test verification
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorMin[i] = 300;
    sensorMax[i] = 3300;
  }

  // 6 sample 12-bit ADC frames: Centred -> Slight Right -> Far Right -> Slight Left -> Lost Line (Recovery) -> Centred
  const int sampleFrames[6][NUM_SENSORS] = {
    {300,  300, 3300,  300,  300},  // 1. Line dead-centre under S3 (pos = 0.0)
    {300,  300, 1800, 1800,  300},  // 2. Line between S3 and S4 (pos = +0.5, steer right)
    {300,  300,  300,  900, 2700},  // 3. Line far right under S4/S5 (pos = +1.75, hard right)
    {300, 2700,  900,  300,  300},  // 4. Line to the left under S2/S3 (pos = -0.75, steer left)
    {300,  300,  300,  300,  300},  // 5. All-zero / Lost line (triggers recovery pivot)
    {300,  300, 3300,  300,  300}   // 6. Line re-acquired at centre
  };

  for (int f = 0; f < 6; f++) {
    readAndNormalizeSensors(sampleFrames[f], normVals);
    bool lineSeen = false;
    float pos = computeWeightedCentroid(normVals, lineSeen);

    int leftMotor = 0, rightMotor = 0;
    float error = 0.0f;

    if (lineSeen) {
      error = pos;  // Setpoint is 0.0 (centre sensor S3)
      lastValidError = error;
      float correction = computePID(error);
      leftMotor  = clampMotor((int)(BASE_SPEED + correction), MAX_SPEED);
      rightMotor = clampMotor((int)(BASE_SPEED - correction), MAX_SPEED);
    } else {
      // Simple recovery: pivot toward the side where the line was last seen
      error = lastValidError;
      if (lastValidError >= 0.0f) {
        leftMotor  =  PIVOT_SPEED;
        rightMotor = -PIVOT_SPEED;
      } else {
        leftMotor  = -PIVOT_SPEED;
        rightMotor =  PIVOT_SPEED;
      }
    }
    printTelemetry(normVals, lineSeen, error, leftMotor, rightMotor);
  }
  Serial.println("--- End of Sample Validation ---\n");
}

// ============================================================================
// 4. ARDUINO SETUP & MAIN LOOP
// ============================================================================
void setup() {
  // Initialise Serial debug at 115200 baud (Appendix A requirement)
  Serial.begin(115200);

  // Configure ESP32 ADC to 12-bit resolution (0 - 4095)
  analogReadResolution(12);

  // Configure IR sensor pins as inputs
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(IR_PINS[i], INPUT);
  }

  // Configure L298N motor driver pins as outputs
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);

  // Ensure motors are stopped during startup calibration
  setMotorSpeeds(0, 0);

  // Run one-time validation print on sample arrays, then calibrate live sensors
  runSampleValidation();
  if (!USE_SAMPLE_INPUTS) {
    calibrateSensors();
  }
}

void loop() {
  // 1. Read raw 12-bit ADC values from the 5 IR sensors
  int rawReadings[NUM_SENSORS];
  for (int i = 0; i < NUM_SENSORS; i++) {
    rawReadings[i] = analogRead(IR_PINS[i]);
  }

  // 2. Normalize readings to 0.0 .. 1000.0 using calibrated min/max
  readAndNormalizeSensors(rawReadings, normVals);

  // 3. Estimate line position using weighted centroid [-2, -1, 0, 1, 2]
  bool lineDetected = false;
  float position = computeWeightedCentroid(normVals, lineDetected);

  int leftCmd  = 0;
  int rightCmd = 0;
  float error  = 0.0f;

  // 4. Compute PID correction or Lost-Line recovery
  if (lineDetected) {
    isLineLost = false;
    error = position;                  // Target setpoint is 0.0 (centre sensor S3)
    lastValidError = error;            // Remember last valid side in case we lose the line

    float pidCorrection = computePID(error);

    // Differential steering:
    // If line is to the right (error > 0), left wheel speeds up and right wheel slows down
    leftCmd  = clampMotor((int)(BASE_SPEED + pidCorrection), MAX_SPEED);
    rightCmd = clampMotor((int)(BASE_SPEED - pidCorrection), MAX_SPEED);
  } else {
    // Lost-line condition: all sensors read white background
    if (!isLineLost) {
      isLineLost = true;
      lineLostStartMs = millis();
    }

    // If lost for longer than LOST_TIMEOUT_MS (500 ms), perform a safe stop
    if (millis() - lineLostStartMs > LOST_TIMEOUT_MS) {
      leftCmd  = 0;
      rightCmd = 0;
    } else {
      // Otherwise, pivot in place toward the last known side of the line
      if (lastValidError >= 0.0f) {
        leftCmd  =  PIVOT_SPEED;       // Spin clockwise (right)
        rightCmd = -PIVOT_SPEED;
      } else {
        leftCmd  = -PIVOT_SPEED;       // Spin counter-clockwise (left)
        rightCmd =  PIVOT_SPEED;
      }
    }
    // Reset previous error so there is no derivative spike when the line is found again
    prevError = lastValidError;
    integral  = 0.0f;
  }

  // 5. Apply commands to the L298N motor driver
  setMotorSpeeds(leftCmd, rightCmd);

  // 6. Stream telemetry over Serial at 115200 baud
  printTelemetry(normVals, lineDetected, error, leftCmd, rightCmd);

  // Fixed 10 ms loop interval (~100 Hz control rate)
  delay(10);
}
