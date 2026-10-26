# LMS Line Follower & Maze Solving Robot

A comprehensive Robotics and Embedded Systems project built for autonomous navigation, line tracking, precise motor control, and maze-solving logic using PID controllers and web-based Webots 3D simulation interfaces.

---

## Layout
| Task | Where |
|---|---|
| 1 Basic coding | `src/task1_basics.py` (Python) and `docs/01_task1_logic.md` |
| 2 Datasheet literacy | `docs/02_datasheet_literacy.md` |
| 3 PID control | `docs/03_pid_control_and_tuning.md` |
| 4 BOM | `BOM/BOM.md`, `BOM/BOM.csv`, `docs/img/wiring.jpg` |
| 5 Power budget & battery | `BOM/power_budget.md` |
| 6 Wiring & troubleshooting | `docs/06_electronics_and_troubleshooting.md`, `docs/img/wiring.jpg` |
| 7 Wireless diagnostics | `docs/07_wireless_diagnostics.md` |
| 8 Maze algorithms | `docs/08_maze_algorithms.md`, `src/maze/maze_demo.py` |
| 9 Implementation | `src/lms_line_follower/task9_simple_line_follower.ino`, `docs/09_LMS_implementation.md` |
| Extras | An extra video of simulation how the program works in webots and some material that can be used in future. `Extras/Webots/bot_image.png`, `Extras/Webots/webots_lms.webm` |

---

## Project Overview

This project provides end-to-end software and documentation for an autonomous mobile robot capable of dual operational modes:
1. **Precision Line Tracking:** Using infrared/line sensor arrays with a tuned **Proportional-Integral-Derivative (PID)** control loop for fast, stable line adherence.
2. **Maze Exploration & Navigation:** Implementing path-finding algorithms (DFS, Flood Fill, and Dijkstra) to solve complex grid-based mazes efficiently both in hardware and a Simulated environment via Webots.

---

## Key Features

* **PID Control Engine:** Smooth motor output calculation preventing overshoot, oscillation, or line departure.
* **Autonomous Maze Navigation:** Built-in Left-Hand/Right-Hand rule heuristics, Depth-First Search (DFS), and Dijkstra pathfinding for short-path calculations.
* **Hardware Interfacing & Schematics:** Documented schematics, power budgets, and wiring layouts for standard microcontroller and driver setups.
* **3D Simulation Video:** Video of simulation done on the logic implemented.
* **Telemetry & Diagnostics:** Real-time data logging and wireless communication options for remote sensor monitoring and threshold calibration.

---

## Hardware & Components

Refer to `BOM/BOM.md` and `BOM/power_budget.md` for full component specifications.

* **Microcontroller:** Arduino UNO / ESP32 / ATmega328P
* **Sensors:** 5 to 8-channel IR Line Sensor Array, Ultrasonic / Distance Sensors (for maze walls)
* **Motors & Actuators:** Dual N20 / TT Gear Motors with Quadrature Encoders
* **Motor Driver:** L298N / DRV8833 Dual H-Bridge Driver
* **Power System:** 2S/3S LiPo Battery with LM2596 Buck Converter for stable 5V logic rails.

---

## Webots Simulation Integration

1. A video of simulation done on the logic implemented (`Extras/Webots/bot_image.png`, `Extras/Webots/webots_lms.webm`).
2. Track prototypes for future work
 (`MeshmerizeTrack1.proto` to `MeshmerizeTrack5.proto`).


## Assumptions and limitations
* Prices marked "est." in the BOM and motor currents are estimates; no hardware measurements were made.
* Simulation uses a simplified plant; gains need re-tuning on the robot. Junction/END detection and the maze run are designed (docs/08) but not implemented in firmware.
* Appendix A fixes applied: IN4 moved GPIO11→GPIO4; GPIO25 is ADC2 (Wi-Fi must stay off); BLE chosen for diagnostics; wireless must be disabled in competition builds (Meshmerize rules).
* The supplied course README's turn-reduction table has an error (L+B+R is B, not S) — corrected in code and docs.

