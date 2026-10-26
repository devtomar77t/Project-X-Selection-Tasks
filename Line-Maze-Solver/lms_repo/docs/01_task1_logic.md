# Task 1 - Logic Explanation

Task 1 has two small functions: **line position** (weighted centroid) and **motor limiting** (clamp).

## 1. Line position (weighted centroid)

The five sensors sit at positions **-2, -1, 0, +1, +2**, where 0 is the middle of the robot. Each reading says how strongly that sensor sees the line, so the line position is the centre of mass of the readings:

```
position = (sum of weight x reading) / (sum of readings)
```

**Example:** readings `[0, 0, 200, 800, 0]`

```
(0 + 0 + 0 + 800 + 0) / 1000 = +0.8
```

The line is mostly under sensor 4, so it is slightly to the right.

**Why this way**
- It gives a smooth value *between* sensors, not just "which sensor is on". That smooth value is what lets the PID steer smoothly.
- Dividing by the sum means the result does not depend on how bright the readings are overall.

### The all-zero case

If every reading is 0, the line is not under the robot and the formula would divide by zero.

- The function must **not** return 0, because 0 means "perfectly centred" and the robot would drive straight off the track.
- Instead it returns a "no line" result (`None` in Python, `valid = false` in C++) and lets the lost-line code decide what to do.

| Readings | Position |
|---|---|
| `[0, 0, 1000, 0, 0]` | 0.0 (centred) |
| `[0, 1000, 1000, 0, 0]` | -0.5 |
| `[0, 0, 200, 800, 0]` | +0.8 |
| `[1000, 0, 0, 0, 0]` | -2.0 (far left) |
| `[0, 0, 0, 0, 0]` | no line |

## 2. Motor limiting (clamp)

Each motor command is a signed number: **positive = forward, negative = reverse**. The function limits only the *size* to the maximum and keeps the *sign*:

- above +max becomes +max
- below -max becomes -max
- in between stays unchanged

**Example (max = 255)**

| Input (left, right) | Output |
|---|---|
| `(300, -300)` | `(255, -255)` |
| `(100, -50)` | `(100, -50)` |
| `(-1000, 255)` | `(-255, 255)` |

**Why the sign must be kept:** the sign sets the motor direction. If the code clamped the absolute value and lost the sign, a wheel that should reverse would drive forward.

## One-line summary

> Centroid = weighted average of sensor positions (with an explicit "no line" result for all zeros); clamp = limit the size of each motor command but keep its direction.