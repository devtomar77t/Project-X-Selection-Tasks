// Task 9 - simple PID line follower, ESP32 + L298N (pins from Appendix A)
//   1) weighted centroid -> line position     2) PID error -> differential motor speeds     3) lost-line handling
// SIMULATE 1 = run hard-coded sample sensor frames (no robot needed).  Set it to 0 to drive the real robot.
#define SIMULATE 0

const int IR[5] = {34, 35, 32, 33, 25};     // S1 (left) .. S5 (right)
const int ENA = 14, IN1 = 27, IN2 = 26;  // left motor
const int ENB = 13, IN3 = 12, IN4 = 4; // right motor (Appendix A says IN4 = GPIO11: that is the flash pin, so GPIO4 is used)

float Kp = 60, Ki = 0, Kd = 2;  // PID gains
int   baseSpeed = 140;  // PWM 0..255 on a straight line
float lastError = 0, integral = 0;
unsigned long lostSince = 0;  // 0 = line is visible
int mn[5], mx[5]; // calibration min / max per sensor

#if SIMULATE
int frames[][5] = {  // already scaled 0..1000
  {0, 0, 1000, 0, 0},      // centred
  {0, 0, 200, 800, 0},     // line drifting right
  {0, 0, 0, 300, 900},     // line far right
  {0, 0, 0, 0, 0},         // LOST
  {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0},   // still lost -> safe stop
  {0, 800, 900, 0, 0},     // line found again -> normal driving resumes
  {0, 0, 1000, 0, 0}};
const int NFRAMES = sizeof(frames) / sizeof(frames[0]);
int frameNo = 0;
#endif

// ---- hardware helpers ----------------------------------------------------
void setMotor(int en, int a, int b, int speed) { // speed -255..255, sign = direction
  speed = constrain(speed, -255, 255);
  digitalWrite(a, speed > 0);
  digitalWrite(b, speed < 0);
  analogWrite(en, abs(speed));
}

void calibrate() {  // slide the robot by hand over line AND floor for 3 s
  for (int i = 0; i < 5; i++) { mn[i] = 4095; mx[i] = 0; }
  unsigned long t = millis();
  while (millis() - t < 3000) {
    for (int i = 0; i < 5; i++) {
      int r = analogRead(IR[i]);
      if (r < mn[i]) mn[i] = r;
      if (r > mx[i]) mx[i] = r;
    }
    delay(5);
  }
}

void readSensors(int v[5]) {    // fills v[] with 0..1000 (line = high)
#if SIMULATE
  for (int i = 0; i < 5; i++) v[i] = frames[frameNo][i];
#else
  for (int i = 0; i < 5; i++) {
    int span = mx[i] - mn[i];
    v[i] = (span > 100) ? constrain((analogRead(IR[i]) - mn[i]) * 1000L / span, 0, 1000) : 0;
  }
#endif
}

// ---- 1) weighted centroid ------------------------------------------------
bool getPosition(const int v[5], float &pos) {    // pos: -2 (far left) .. +2 (far right)
  const int w[5] = {-2, -1, 0, 1, 2};
  long num = 0, sum = 0;
  for (int i = 0; i < 5; i++) { num += (long)w[i] * v[i]; sum += v[i]; }
  if (sum < 200) return false;       // (almost) all zero: no line, avoid dividing by zero
  pos = (float)num / sum;
  return true;
}

// ---- 2) PID -> differential speed ----------------------------------------
void followLine(float error) {
  integral += error;
  float correction = Kp * error + Ki * integral + Kd * (error - lastError);
  lastError = error;
  setMotor(ENA, IN1, IN2, baseSpeed + (int)correction);   // line on the right -> left wheel faster
  setMotor(ENB, IN3, IN4, baseSpeed - (int)correction);
}

// ---- 3) lost line: pivot toward the last side for 300 ms, then stop ------
void handleLostLine() {
  if (lostSince == 0) lostSince = millis();
  if (millis() - lostSince < 300) {
    int s = (lastError >= 0) ? 110 : -110;    // line was last on the right -> turn right
    setMotor(ENA, IN1, IN2, s);
    setMotor(ENB, IN3, IN4, -s);
  } else {
    setMotor(ENA, IN1, IN2, 0);    // safe stop
    setMotor(ENB, IN3, IN4, 0);
  }
}

// ---- main ------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  const int out[] = {ENA, IN1, IN2, ENB, IN3, IN4};
  for (int p : out) pinMode(p, OUTPUT);
#if !SIMULATE
  calibrate();
#endif
}

void loop() {
#if SIMULATE
  if (frameNo >= NFRAMES) { setMotor(ENA, IN1, IN2, 0); setMotor(ENB, IN3, IN4, 0); return; }
#endif
  int v[5];
  float pos = 0;
  readSensors(v);

  if (getPosition(v, pos)) {    // line visible
    if (lostSince != 0) { integral = 0; lostSince = 0; }  // just found it again: forget old integral
    followLine(pos);      // setpoint is 0 (centre), so error = position
  } else {
    handleLostLine();
  }

  Serial.printf("s: %4d %4d %4d %4d %4d | pos %+.2f | lost %d\n", v[0], v[1], v[2], v[3], v[4], pos, lostSince != 0);
#if SIMULATE
  frameNo++;
#endif
  delay(10);      // fixed 10 ms loop = 100 Hz
}
