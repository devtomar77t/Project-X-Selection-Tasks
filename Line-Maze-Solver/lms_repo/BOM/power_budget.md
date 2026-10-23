# Task 5 — Power Budget & Battery Selection

All figures below are computed deterministically by `BOM/bom_calc.py` (re-run `python3 bom_calc.py` to update any assumption). **Hardware Configuration (Tasks 1–8):** Toshiba TB6612FNG dual MOSFET motor driver, ESP32 DevKit V1 (WROOM-32), 5× TCRT5000 IR reflectance array, MP1584EN 5.0 V buck converter, and 2× N20 6 V 600 RPM micro metal gearmotors powered by an **Orange 2S 7.4 V 1000 mAh 40C LiPo pack (55 g)**. *(Task 9 retains the Appendix A L298N + 3S configuration.)*

---

## 1. Current Draw Estimation (Per Rail & Reflected to 2S Battery)

**Power Rails:**
* **7.4 V Battery Rail ($6.6\text{–}8.4\text{ V}$):** Feeds the TB6612FNG motor supply pin (`VM`) and the MP1584EN buck converter input (`IN+`).
* **5.0 V Buck Rail:** Feeds `ESP32 Vin`.
* **3.3 V LDO Rail (onboard ESP32 AMS1117-3.3):** Feeds the ESP32 core, the 5× TCRT5000 IR array, the TB6612FNG logic supply (`VCC = 3.3 V`) and `STBY` pull-up, plus the END/status LEDs.

Because the ESP32's 3.3 V regulator is a linear LDO fed from `Vin` (5.0 V), every milliamp drawn on the 3.3 V rail is drawn 1:1 from the 5.0 V buck rail ($I_{5\text{V}} = I_{3.3\text{V}}$). The switching buck converter ($\eta \approx 0.88$ at $7.4\text{ V} \to 5.0\text{ V}$) steps voltage down while stepping current down on the battery side according to power conservation:

$$I_{\text{bat,logic}} = \frac{V_{\text{buck}} \times I_{5\text{V}}}{\eta_{\text{buck}} \times V_{\text{bat}}} = \frac{5.0\text{ V} \times I_{5\text{V}}}{0.88 \times 7.4\text{ V}} \approx 0.768 \times I_{5\text{V}}$$

| Component | Rail | Idle (mA) | Typical Running (mA) | Peak / Stall (mA) | Basis / Datasheet Reference |
|---|---|--:|--:|--:|---|
| **ESP32 DevKit V1 (240 MHz, radio off $\to$ BLE/RF burst)** | 5.0 V / 3.3 V | 80 | 80 | 240 | README: ~80 mA radio off; up to 240 mA during RF bursts |
| **IR Array, 5 × TCRT5000 (LED $3.3\text{ V}, 220\,\Omega$)** | 3.3 V | 47 | 47 | 47 | $5 \times (3.3\text{ V} - 1.25\text{ V}) / 220\,\Omega \approx 46.6\text{ mA}$ |
| **TB6612FNG Logic Supply ($I_{\text{CC}}$ at $V_{\text{CC}} = 3.3\text{ V}$)** | 3.3 V | 2 | 2 | 5 | Toshiba TB6612FNG datasheet: $I_{\text{CC}} = 2.2\text{ mA}$ typ, $5\text{ mA}$ max |
| **Red END LED + Status LED** | 3.3 V | 5 | 5 | 10 | ~5 mA per LED through current-limiting resistor |
| **5.0 V / 3.3 V Logic Rail Subtotal ($I_{5\text{V}}$)** | **5.0 V** | **134 mA** | **134 mA** | **302 mA** | Sum of all logic & sensor loads |
| **Logic Current Reflected to 2S Battery ($I_{\text{bat,logic}}$)** | **7.4 V** | **103 mA** | **103 mA** | **232 mA** | $5.0\text{ V} \times I_{5\text{V}} \div (0.88 \times 7.4\text{ V})$ |
| **2 × N20 6 V 600 RPM Motors (on `VM`)** | 7.4 V | 0 | 300 ($2\times 150$) | **2000 ($2\times 1000$)** | Running: ~150 mA each; Stall: ~1.0 A each (README: 0.8–1.5 A) |
| **Total System Current at 2S Battery ($7.4\text{ V}$)** | **7.4 V** | **103 mA** | **403 mA** | **2232 mA (2.23 A)** | **Worst-case total = Peak Logic (232 mA) + Dual Stall (2000 mA)** |

*(Note: Comparing the TB6612FNG's $I_{\text{CC}} = 2\text{ mA}$ against the L298N's $I_{\text{SS}} \approx 30\text{–}36\text{ mA}$ saves ~28 mA of continuous logic current on top of eliminating ~2.0 V of motor saturation drop.)*

---

## 2. Runtime Calculation (2S 7.4 V, 1000 mAh LiPo, Derating $\eta = 0.80$)

Using the standard derated capacity formula:

$$\text{Runtime (hours)} = \frac{\text{Capacity (mAh)} \times \text{Derating Factor }(0.80)}{I_{\text{load}}\text{ (mA)}}$$

| Operating Scenario | Battery Current ($I_{\text{bat}}$) | Usable Capacity ($0.80 \times 1000\text{ mAh}$) | Estimated Runtime (Hours) | Estimated Runtime (Minutes) |
|---|--:|--:|--:|--:|
| **Idle (MCU & sensors active, motors stopped)** | 103 mA | 800 mAh | **7.77 h** | **~466 min** |
| **Typical Line Following (logic + 2 motors @ 150 mA)** | **403 mA** | 800 mAh | **1.99 h** | **~119 min** |
| **Worst-Case Continuous Stall (RF peak + 2 motors @ 1 A)** | 2232 mA | 800 mAh | **0.36 h** | **~22 min** *(firmware stops after 0.5 s)* |

### Stated Assumptions & Margin Analysis
1. **20% Derating Factor ($0.80$):** Accounts for stopping discharge at **$3.3\text{ V/cell}$ ($6.6\text{ V}$ pack voltage)** via the low-voltage alarm to prevent LiPo damage, plus internal resistance losses and cell ageing.
2. **Match Energy Consumption:** A complete Meshmerize match (3 min Dry Run + 2 min 30 s Actual Run = **5.5 minutes** maximum) consumes only:
   $$E_{\text{match}} = 403\text{ mA} \times \frac{5.5\text{ min}}{60\text{ min/h}} \approx \mathbf{36.9\text{ mAh}} \quad (\mathbf{3.7\%}\text{ of the 1000 mAh pack})$$
   Even if aggressive cornering and braking triple the average motor current to $450\text{ mA}$ per motor ($1003\text{ mA}$ total), runtime is still $\approx 48\text{ minutes}$ ($>8\times$ the full match duration).
3. **C-Rating Verification:**
   $$C_{\text{required}} = \frac{I_{\text{peak}}}{\text{Capacity}} = \frac{2.232\text{ A}}{1.0\text{ Ah}} = \mathbf{2.23\text{C}}$$
   Our **40C** 2S LiPo is rated for **$40\text{ A}$ continuous discharge** ($>17\times$ the worst-case stall current), guaranteeing negligible voltage sag when reversing motor polarity during dead-end pivots.

---

## 3. Cell Chemistry Justification: 2S LiPo vs. 2S Li-ion (18650)

We compared a **2S LiPo pouch pack (7.4 V, 1000 mAh, 40C)** against a **2S cylindrical Li-ion pack (2 × 18650 cells, 7.4 V, 2500 mAh)**:

| Criterion | **2S LiPo Pouch — 7.4 V 1000 mAh 40C (Chosen)** | **2S Li-ion Cylindrical — 2 × 18650 2500 mAh** | Engineering Impact on Meshmerize Robot |
|---|---|---|---|
| **Nominal Voltage & Energy** | $7.4\text{ V}$, **$7.4\text{ Wh}$** | $7.4\text{ V}$, **$18.5\text{ Wh}$** | Robot uses only $36.9\text{ mAh}$ ($0.27\text{ Wh}$) per match; extra Wh in 18650 is dead weight. |
| **Total Pack Mass (incl. holder/leads)** | **$\approx 55\text{ g}$** ($17\times 34\times 55\text{ mm}$) | **$\approx 110\text{ g}$** ($2\times 45\text{ g}$ cells $+ 20\text{ g}$ holder) | **LiPo saves $55\text{ g}$ ($50\%$ lighter)**, lowering rotational inertia for faster $90^\circ$ turns. |
| **Specific Energy (Gravimetric)** | $\approx 135\text{ Wh/kg}$ | $\approx 168\text{ Wh/kg}$ | Li-ion has higher cell energy density, but forces oversized minimum capacity ($\ge 2000\text{ mAh}$). |
| **C-Rating & Max Discharge Current** | **40C ($40\text{ A}$ max)**; ultra-low ESR | **1C–3C ($2.5\text{–}7.5\text{ A}$)** standard; up to 8C–10C ($20\text{ A}$) high-drain | LiPo eliminates voltage sag on the 7.4 V rail during dual-motor inrush spikes. |
| **Mechanical Form Factor & CoG** | Flat, low-profile pouch; mounts flush to acrylic plate | Tall $18\text{ mm}$ cylinders + spring holder; raises Centre of Gravity (CoG) | Low CoG prevents wheel lift and sensor bounce over arena bumps. |
| **Safety & Handling** | Soft pouch requires balance charger, LiPo strap, and LVC alarm | Rigid steel can; spring holders can lose contact on bumps | Both mitigated: BOM includes iMAX B6 balance charger, LVC alarm, and strap. |
| **Approximate Cost (INR)** | **~₹750** (plug-and-play XT30 + JST-XH) | **~₹700–₹850** (2 high-drain cells + 2S holder + BMS) | Nearly identical cost; soldered LiPo pack avoids spring-holder contact bounce. |

### Justification Summary
We selected the **2S 7.4 V 1000 mAh 40C LiPo** because a line-maze solver is **power-spike and inertia constrained, not energy constrained**. Since a 5.5-minute heat uses less than $4\%$ of a 1000 mAh pack, carrying a $110\text{ g}$ 2S 18650 pack adds $55\text{ g}$ of unnecessary mass that slows down cornering and increases stopping distance at dead ends. Moreover, spring-loaded 18650 holders are notorious for microsecond contact bounce over arena bumps (Task 7), whereas a soldered 2S LiPo pouch with an XT30 connector provides rock-solid power delivery.
