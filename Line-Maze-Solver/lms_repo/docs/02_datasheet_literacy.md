# Task 2 — Datasheet Literacy: Toshiba TB6612FNG Dual DC Motor Driver IC

**Source References:** Toshiba TB6612FNG Dual DC Motor Driver IC Datasheet (Absolute Maximum Ratings, Operating Range, and Electrical Characteristics tables) and Robu.in / SparkFun breakout module documentation. *(Note: Tasks 1–8 use the TB6612FNG with a 2S 7.4 V LiPo battery; Task 9 uses the Appendix A L298N + 3S configuration.)*

---

## 1. Three Datasheet Parameters That Constrain the Circuit Design

| Parameter (Datasheet Symbol) | Datasheet Rating | Design & BOM Decision Driven by This Parameter |
| :--- | :--- | :--- |
| **Output Current per Channel ($I_{\text{OUT}}$)** | **1.2 A continuous (average)**; **2.0 A peak (20 ms)**; **3.2 A single pulse (10 ms)** | We selected **6 V N20 Micro Metal Gearmotors** with a running current of **~0.15–0.25 A** and a stall current of **~0.8 A–1.0 A** per motor. This keeps even a worst-case continuous wheel stall safely inside the **1.2 A DC limit** per channel (whereas bulky plastic/metal gearmotors stalling at 2.0–2.5 A would burn out the channel). Firmware also limits recovery time and latches a `STOPPED` state after **500 ms** without a line. |
| **Supply Voltage Ranges ($V_M$ and $V_{\text{CC}}$)** | **Motor supply ($V_M$):** 2.5 V to 13.5 V operating (15 V absolute max)<br>**Logic supply ($V_{\text{CC}}$):** 2.7 V to 5.5 V (6.0 V absolute max)<br>**Logic Input High ($V_{\text{IN(H)}}$):** $\ge 0.7 \times V_{\text{CC}}$ | **Battery & Logic Matching:** A **2S LiPo (7.4 V nominal, 8.4 V full charge)** sits comfortably in the middle of the 2.5–13.5 V $V_M$ operating window with 6.6 V of headroom below the 15 V breakdown limit. Crucially, we power $V_{\text{CC}}$ directly from the **ESP32 3.3 V rail** (not 5 V). At $V_{\text{CC}} = 3.3\text{ V}$, $V_{\text{IN(H)}} = 0.7 \times 3.3\text{ V} = 2.31\text{ V}$, which the ESP32's 3.3 V GPIOs easily satisfy. *(If $V_{\text{CC}}$ were tied to 5.0 V, $V_{\text{IN(H)}}$ would be $0.7 \times 5.0\text{ V} = 3.5\text{ V}$, which a 3.3 V ESP32 pin cannot legally guarantee!)* |
| **Output MOSFET On-Resistance ($R_{\text{DS(on)}}$ / $V_{\text{sat}}$)** | **$R_{\text{DS(on)}} = 0.5\,\Omega$ typical** ($0.7\,\Omega$ max at $I_{\text{OUT}} = 1\text{ A}$) $\rightarrow$ **$V_{\text{drop}} \approx 0.5\text{ V}$ at 1 A** (and $\approx 0.1\text{ V}$ at 0.2 A running) | Because the low-resistance MOSFET H-bridge drops only **0.1–0.5 V** (compared to 1.8–3.2 V wasted in an L298N Darlington stage), the motors receive **~7.3 V** from a 7.4 V 2S LiPo. This allows us to use a lighter **2S battery** with **6 V N20 motors** instead of a heavy 3S pack, eliminating a bulky heatsink. Additionally, the datasheet specifies a **Standby (`STBY`) pin** that puts the IC in high-impedance sleep when `LOW`, so our wiring pulls `STBY` `HIGH` via an ESP32 GPIO (or to 3.3 V through a $10\text{ k}\Omega$ resistor) to enable operation. |

---

## 2. Ratings That Would Damage the Component & How Our Design Avoids Them

### A. Exceeding Continuous Channel Current ($I_{\text{OUT}} > 1.2\text{ A}$) or Package Power Dissipation ($P_D > 1.36\text{ W}$ on PCB)
Unlike the bulky Multiwatt L298N, the TB6612FNG uses a compact surface-mount **SSOP24 package** with a thermal resistance of **$R_{\theta\text{JA}} \approx 91.5\text{ }\text{°C/W}$** when mounted on a standard PCB ($P_D \le 1.36\text{ W}$ at **25 °C**, absolute maximum junction temperature $T_{j(\text{max})} = 150\text{ °C}$). Using $P_{\text{loss}} = I_{\text{load}}^2 \times R_{\text{DS(on)}}$ per channel:

| Operating Condition (per channel) | $R_{\text{DS(on)}}$ (typ–max) | $V_{\text{drop}} = I \times R_{\text{DS(on)}}$ | Total IC Dissipation (2 channels) | Estimated $T_j = T_{\text{amb}}(25\text{ °C}) + P_{\text{loss}} \cdot R_{\theta\text{JA}}$ |
| :--- | :---: | :---: | :---: | :---: |
| **0.20 A normal running** | $0.5\,\Omega$ | 0.10 V | 0.04 W | **~28.7 °C** (runs completely cool) |
| **0.50 A brisk acceleration** | $0.5\,\Omega$ | 0.25 V | 0.25 W | **~47.9 °C** (well within safe limits) |
| **1.00 A stall (6 V N20)** | $0.5\text{–}0.7\,\Omega$ | 0.50–0.70 V | 1.00–1.40 W | **~116–153 °C** (approaches thermal shutdown at **175 °C** if held indefinitely) |
| **2.00 A illegal stall (e.g., high-draw motor)** | $0.7\,\Omega$ | 1.40 V | **5.60 W** | **exceeds 1.2 A DC rating & destroys bond wires/MOSFETs** |

### B. Over-Voltage / Reverse-Polarity on $V_M$ ($>15\text{ V}$) or $V_{\text{CC}}$ ($>6.0\text{ V}$)
Accidentally connecting the **2S LiPo (7.4–8.4 V)** to the logic supply pin **$V_{\text{CC}}$** (rated 6.0 V abs max), or inductive back-EMF spikes pushing $V_M$ above 15 V, or reverse battery insertion will instantly destroy the CMOS logic gates and internal MOSFET body diodes.

### How Our Design Prevents Damage:
1. **Motor Stall Current Capped Below 1.2 A by Hardware Selection:** Choosing N20 motors with a $\le 1.0\text{ A}$ stall current guarantees a single stalled wheel never exceeds the 1.2 A continuous rating.
2. **Firmware Stall & Timeout Protection:** In `lms_core.h`, motor PWM is clamped (`clampMotors`) and the lost-line state machine (`GRACE_HOLD` $\rightarrow$ `RECOVER_LEFT/RIGHT` $\rightarrow$ `STOPPED`) cuts motor power to `0` after **`recoverTimeoutMs = 500 ms`**, preventing sustained stall heating against maze walls or obstacles.
3. **Strict Rail Isolation ($V_M$ vs. $V_{\text{CC}}$) & Polarity Keying:** The 2S LiPo connects via a keyed **XT30 / JST connector** with an inline **2.5 A–3 A polyfuse** to $V_M$ (and the 5 V buck input), while $V_{\text{CC}}$ is wired exclusively to the regulated **3.3 V ESP32 rail**.
4. **Decoupling Against Voltage Transients:** A **100–470 µF electrolytic bulk capacitor** and a **0.1 µF ceramic capacitor** are placed across $V_M$ and $\text{GND}$ right at the TB6612FNG pins to absorb inductive back-EMF spikes during rapid PWM direction reversals.