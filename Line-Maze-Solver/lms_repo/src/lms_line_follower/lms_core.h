// lms_core.h  --  hardware-independent core of the LMS / Meshmerize line follower.
// No Arduino calls in here, so the SAME code runs on the ESP32 and in the host tests
// (src/tests). Covers: Task 1 (centroid + clamp), Task 3 (PID), Task 9 (pipeline).
//
// Hardware Driver Note:
// - Tasks 1-8 use the TB6612FNG dual MOSFET motor driver with a 2S LiPo (7.4 V nominal).
// - Task 9 uses the Appendix A specification (L298N motor driver with a 3S 11.1 V LiPo).
// Both drivers accept signed 8-bit PWM commands in [-255, +255] produced by this core.
#pragma once
#include <stdint.h>
#include <math.h>

namespace lms {

static const int   kNumSensors = 5;
static const float kWeights[kNumSensors] = {-2.f, -1.f, 0.f, 1.f, 2.f};  // S1..S5

// ---------------------------------------------------------------- Task 1a
struct Centroid {
  bool  valid;      // false when every sensor reads ~0 (line not under the array)
  float position;   // weighted centroid in sensor units, -2 (far left) .. +2 (far right)
};

// position = sum(w_i * v_i) / sum(v_i).  v_i are calibrated, non-negative readings.
// If the denominator is below minTotal (or NaN) the result is flagged invalid instead of
// dividing by (almost) zero -- the caller decides what "lost line" means.
inline Centroid weightedCentroid(const float v[kNumSensors], float minTotal = 1.0f) {
  float num = 0.f, den = 0.f;
  for (int i = 0; i < kNumSensors; i++) {
    float x = (isnan(v[i]) || isinf(v[i]) || v[i] < 0.f) ? 0.f : v[i];  // defensive clip
    num += kWeights[i] * x;
    den += x;
  }
  Centroid c;
  if (!(den >= minTotal && den > 0.f)) {
    c.valid = false;
    c.position = 0.f;
  } else {
    c.valid = true;
    c.position = num / den;
  }
  return c;
}

// ---------------------------------------------------------------- Task 1b
// Limit |cmd| to maxMag; the sign (direction) is preserved.
inline int clampMotor(int cmd, int maxMag) {
  int m = maxMag < 0 ? -maxMag : maxMag;
  if (cmd >  m) return  m;
  if (cmd < -m) return -m;
  return cmd;
}
inline void clampMotors(int &left, int &right, int maxMag) {
  left  = clampMotor(left,  maxMag);
  right = clampMotor(right, maxMag);
}

// ------------------------------------------------------------ calibration
// Min-max calibration: raw 12-bit ADC (0..4095) -> 0..1000 per sensor.
struct Calibration {
  int minRaw[kNumSensors];
  int maxRaw[kNumSensors];
  void reset() { for (int i = 0; i < kNumSensors; i++) { minRaw[i] = 4095; maxRaw[i] = 0; } }
  void update(const int raw[kNumSensors]) {
    for (int i = 0; i < kNumSensors; i++) {
      if (raw[i] < minRaw[i]) minRaw[i] = raw[i];
      if (raw[i] > maxRaw[i]) maxRaw[i] = raw[i];
    }
  }
  // returns 0..1000, larger = more "line". lineIsHigh=false inverts (dark line on light floor).
  float normalize(int i, int raw, bool lineIsHigh) const {
    if (i < 0 || i >= kNumSensors) return 0.f;
    int span = maxRaw[i] - minRaw[i];
    if (span < 200) return 0.f;             // sensor never saw contrast: treat as "no line"
    float x = (raw - minRaw[i]) * 1000.0f / span;
    if (x < 0.f) x = 0.f; else if (x > 1000.f) x = 1000.f;
    return lineIsHigh ? x : 1000.f - x;
  }
};

// -------------------------------------------------------------------- PID
// Derivative on error with a first-order low-pass; integral clamped and only
// accumulated while the output is not saturated (anti-windup).
struct Pid {
  float kp, ki, kd;
  float iLimit;      // max |integral contribution| in output units
  float dAlpha;      // derivative filter 0..1 (1 = unfiltered)
  float integ, prevErr, dFilt;
  bool  first;
  Pid() : kp(0), ki(0), kd(0), iLimit(30.f), dAlpha(0.5f), integ(0), prevErr(0), dFilt(0), first(true) {}
  void reset() { integ = 0; prevErr = 0; dFilt = 0; first = true; }
  float update(float err, float dt, float outLimit) {
    float lim = fabsf(outLimit);
    float iMax = fabsf(iLimit);
    if (first) {
      first = false;
      prevErr = err;
      dFilt = 0.f;
      if (dt > 0.f) {
        float iNew = integ + ki * err * dt;
        if (iNew >  iMax) iNew =  iMax;
        if (iNew < -iMax) iNew = -iMax;
        float out = kp * err + iNew;
        float sat = out > lim ? lim : (out < -lim ? -lim : out);
        if (sat == out || ((out > 0.f) != (err > 0.f))) integ = iNew;
        return sat;
      }
      float out = kp * err + integ;
      return out > lim ? lim : (out < -lim ? -lim : out);
    }

    if (dt <= 0.f) {
      // Do not corrupt prevErr, dFilt, or integ if called with a zero/negative time step.
      float out = kp * err + integ + kd * dFilt;
      return out > lim ? lim : (out < -lim ? -lim : out);
    }

    float d = (err - prevErr) / dt;
    dFilt += dAlpha * (d - dFilt);
    prevErr = err;

    float iNew = integ + ki * err * dt;
    if (iNew >  iMax) iNew =  iMax;
    if (iNew < -iMax) iNew = -iMax;

    float out = kp * err + iNew + kd * dFilt;
    float sat = out > lim ? lim : (out < -lim ? -lim : out);
    if (sat == out || ((out > 0.f) != (err > 0.f)))         // not saturated, or error is unwinding it
      integ = iNew;
    return sat;
  }
};

// ------------------------------------------------- line follower (Task 9)
enum State : uint8_t { FOLLOW = 0, GRACE_HOLD, RECOVER_LEFT, RECOVER_RIGHT, STOPPED };

struct Params {
  float    baseSpeed;        // PWM counts (0..255) when error = 0
  float    kp, ki, kd;       // PWM counts per sensor-unit (kd: per sensor-unit/second)
  float    outLimit;         // |PID correction| cap, PWM counts
  int      maxPwm;           // hard motor clamp
  float    minTotal;         // sum of 5 normalised readings below this => line lost
  uint32_t graceMs;          // keep last command this long on a short dropout (bumps)
  uint32_t recoverTimeoutMs; // give up and stop after this long without a line
  int      pivotSpeed;       // PWM counts for the in-place search turn
  bool     lineIsHigh;       // true: line gives HIGH reading after normalisation
  Params() : baseSpeed(140), kp(60), ki(0), kd(2.0f), outLimit(110), maxPwm(255), minTotal(200),
             graceMs(30), recoverTimeoutMs(500), pivotSpeed(110), lineIsHigh(true) {}
};

struct Output {
  int   left, right;   // signed PWM commands, -255..255
  float position;      // centroid (valid only if lineSeen)
  float error;         // PID input (= position, setpoint 0)
  bool  lineSeen;
  State state;
};

class LineFollower {
 public:
  Calibration cal;
  Params      p;
  Pid         pid;
  State       state;

  LineFollower() : state(FOLLOW), lastPos_(0), lostSince_(0), lastL_(0), lastR_(0) { cal.reset(); applyGains(); }
  void applyGains() { pid.kp = p.kp; pid.ki = p.ki; pid.kd = p.kd; }
  void reset() { state = FOLLOW; pid.reset(); lastPos_ = 0; lostSince_ = 0; lastL_ = lastR_ = 0; }

  // raw: 12-bit ADC readings S1..S5; nowMs: millis(); dt: loop period in seconds
  Output step(const int raw[kNumSensors], uint32_t nowMs, float dt) {
    applyGains();  // Ensure pid always uses the latest gains in p

    float v[kNumSensors];
    for (int i = 0; i < kNumSensors; i++) v[i] = cal.normalize(i, raw[i], p.lineIsHigh);
    Centroid c = weightedCentroid(v, p.minTotal);

    Output o; o.left = o.right = 0; o.position = c.position; o.error = 0; o.lineSeen = c.valid;

    if (state == STOPPED) { o.state = state; return o; }          // latched safe stop

    if (c.valid) {
      if (state != FOLLOW) pid.reset();                           // coming back from a dropout
      state = FOLLOW;
      lastPos_ = c.position;
      o.error = c.position;                                       // setpoint = 0 (centre sensor)
      float corr = pid.update(o.error, dt, p.outLimit);
      o.left  = (int)(p.baseSpeed + corr);                        // line on right (+) -> left faster
      o.right = (int)(p.baseSpeed - corr);
      clampMotors(o.left, o.right, p.maxPwm);
      lastL_ = o.left; lastR_ = o.right;
    } else {
      if (state == FOLLOW) { lostSince_ = nowMs; state = GRACE_HOLD; }
      uint32_t lost = (nowMs >= lostSince_) ? (nowMs - lostSince_) : 0;
      if (state == GRACE_HOLD) {
        if (lost <= p.graceMs) { o.left = lastL_; o.right = lastR_; }   // ride over a bump
        else state = (lastPos_ >= 0.f) ? RECOVER_RIGHT : RECOVER_LEFT;  // search toward last side
      }
      if (state == RECOVER_LEFT || state == RECOVER_RIGHT) {
        if (lost > p.recoverTimeoutMs) { state = STOPPED; o.left = o.right = 0; }
        else {
          int s = (state == RECOVER_RIGHT) ? p.pivotSpeed : -p.pivotSpeed;  // pivot in place
          o.left = s; o.right = -s;
        }
      }
      clampMotors(o.left, o.right, p.maxPwm);
    }
    o.state = state;
    return o;
  }

 private:
  float    lastPos_;
  uint32_t lostSince_;
  int      lastL_, lastR_;
};

}  // namespace lms
