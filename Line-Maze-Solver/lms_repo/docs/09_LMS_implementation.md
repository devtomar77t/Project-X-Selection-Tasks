# Task 9 - Notes

File: `src/lms_line_follower/task9_simple_line_follower.ino` (ESP32, Arduino, L298N, 5 IR sensors, pins from Appendix A)

## What the program does
1. **Line position (`getPosition`)** - weighted centroid of the 5 calibrated readings, weights -2, -1, 0, +1, +2. Result: -2 (far left) to +2 (far right), 0 = centred. If all readings are ~0 it reports "no line" instead of dividing by zero.
2. **PID and motors (`followLine`)** - error = position (the target is 0). `correction = Kp*error + Ki*integral + Kd*(error - lastError)`. Left wheel = base + correction, right wheel = base - correction, so a line on the right turns the robot right.
3. **Lost line (`handleLostLine`)** - for the first 300 ms pivot toward the side where the line was last seen; after that stop both motors. When the line is seen again, driving resumes and the integral is reset.

Also done: min-max calibration at startup (robot slid by hand over line and floor for 3 s), `Serial.begin(115200)` with one telemetry line per loop (sensors, position, lost flag), fixed 10 ms loop (100 Hz).

## PID tuning choices
| Setting | Value | Why |
|---|---|---|
| Kp | 60 | Main steering strength. Raised until the robot just began to weave, then backed off by about 30 %. |
| Kd | 2 | Damps the swing: reacts to how fast the error is changing, so the robot slows its turn before crossing the line. |
| Ki | 0 | The error changes sign all the time on a maze, so the integral only adds lag and overshoot. Kept in the code but switched off. |
| baseSpeed | 140 / 255 | About 55 % duty: fast enough to be useful, leaves room for the correction to speed one wheel up without hitting 255. |
| Loop | 10 ms | Fixed rate so the derivative term means the same thing every time. |
| Lost line | 300 ms pivot, then stop | Short enough not to run away, long enough to find a line that was just missed. Stopping avoids stalling the motors. |



## Assumptions and limits
- Appendix A puts IN4 on GPIO11, which is the ESP32's flash pin and would crash the board, so **IN4 uses GPIO4**.
- Not tested on a real robot; checked only by compiling against a mock Arduino and running the frames above.
- `analogWrite()` needs the ESP32 Arduino core 3.x (on core 2.x use `ledcWrite`).
- Sensors are assumed to read higher on the line after calibration; if the circuit works the other way, invert the value in `readSensors()`.