"""Computes BOM totals (BOM.csv) and the power budget numbers quoted in the docs.

Hardware Context:
- Tasks 1-8: Toshiba TB6612FNG dual MOSFET motor driver + 2S 7.4 V 1000 mAh 40C LiPo (55 g) + 2x N20 6 V 600 RPM motors.
- Task 9: Appendix A specification (L298N + 3S 11.1 V LiPo).
"""
import csv

R = "https://robu.in"
def S(q: str) -> str:
    return f"{R}/?s={q}&post_type=product"

# (ref, item, qty, unit_inr, price_basis, supplier, link)
items = [
    ("Controller", "ESP32 DevKit V1, 30-pin (ESP32-WROOM-32)", 1, 450, "est. (seen Rs289-559)", "Robu / Flyrobo", S("ESP32+DevKit+V1")),
    ("Motor driver", "Toshiba TB6612FNG dual MOSFET H-bridge module", 1, 180, "est. (seen Rs149-220)", "Robu", S("TB6612FNG")),
    ("Motors", "N20 6 V 600 RPM micro metal gear motor", 2, 340, "est.", "Robu", S("N20+6V+600+RPM")),
    ("Motor mounts", "Mounting bracket for N20 motors (pack of 2)", 1, 35, "robu page", "Robu", S("N20+mounting+bracket")),
    ("Wheels", "44 mm 'miniQ' rubber tyre wheel for N20", 2, 67, "robu page (Rs57-67)", "Robu", S("miniQ+wheel+44mm+N20")),
    ("Caster", "Metal ball caster (front support)", 1, 60, "est.", "Robu", S("ball+caster")),
    ("Battery", "Orange 7.4 V 1000 mAh 40C 2S LiPo (55 g)", 1, 750, "est.", "Robu", S("7.4V+1000mAh+2S+LiPo")),
    ("Charger", "iMAX B6-class balance charger (2S-6S)", 1, 1800, "est. (SkyRC B6AC V2 = Rs4017 on Robu)", "Robu", S("imax+b6+charger")),
    ("Battery safety", "LiPo low-voltage alarm / cell checker (2S-8S)", 1, 100, "est.", "Robu", S("lipo+voltage+alarm")),
    ("Battery safety", "LiPo battery strap, 30 cm", 1, 53, "robu page", "Robu", S("lipo+battery+strap")),
    ("5 V supply", "MP1584EN mini buck converter module (set to 5.0 V)", 1, 80, "est.", "Robu / ElectronicsComp", S("MP1584EN")),
    ("Sensors", "TCRT5000 reflective IR sensor (bare), 5 + 2 spare", 7, 18, "est.", "Robu", S("TCRT5000")),
    ("Sensor board", "Perfboard 3x7 cm + male header strip", 1, 60, "est.", "ElectronicsComp", "https://www.electronicscomp.com/"),
    ("Passives", "Resistor assortment (220 ohm, 10 k, etc.)", 1, 100, "est.", "ElectronicsComp", "https://www.electronicscomp.com/"),
    ("Passives", "Electrolytic capacitor 470 uF / 25 V", 2, 12, "est.", "ElectronicsComp", "https://www.electronicscomp.com/"),
    ("Passives", "Ceramic capacitor 0.1 uF (104), pack of 10", 1, 20, "est.", "ElectronicsComp", "https://www.electronicscomp.com/"),
    ("Passives", "1N5819 Schottky diodes (protection), pack of 8", 1, 40, "est.", "ElectronicsComp", "https://www.electronicscomp.com/"),
    ("UI", "Red 5 mm LED (END indicator) + status LED, pack of 5", 1, 20, "est.", "ElectronicsComp", "https://www.electronicscomp.com/"),
    ("UI", "Tactile push-buttons (calibration / run select), pack of 5", 1, 25, "est.", "ElectronicsComp", "https://www.electronicscomp.com/"),
    ("Power", "Slide / rocker power switch (single start switch)", 2, 20, "est.", "ElectronicsComp", "https://www.electronicscomp.com/"),
    ("Power", "3 A fuse + inline holder", 1, 40, "est.", "ElectronicsComp", "https://www.electronicscomp.com/"),
    ("Wiring", "Silicone wire 22 AWG red + black, 2 m each", 1, 120, "est.", "Robu", S("silicone+wire+22awg")),
    ("Wiring", "Dupont jumper set (M-F / F-F)", 1, 130, "est.", "Robu", S("dupont+jumper+wire")),
    ("Connectors", "XT30 pair (robot side) + JST-XH 2S balance lead", 1, 80, "est.", "Robu", S("XT30+connector")),
    ("Connectors", "Female header strips (ESP32 & TB6612FNG sockets)", 1, 30, "est.", "ElectronicsComp", "https://www.electronicscomp.com/"),
    ("Chassis", "Custom chassis: 3 mm acrylic laser-cut or PLA 3D print", 1, 450, "est.", "Local fab / Robu", S("acrylic+sheet+3mm")),
    ("Hardware", "M3 screws, nuts, standoffs kit", 1, 150, "est.", "Robu", S("M3+standoff+kit")),
    ("Hardware", "Heat-shrink, double-sided tape, cable ties", 1, 50, "est.", "Robu", S("heat+shrink+tube")),
]

rows, sub = [], 0
for ref, item, q, u, basis, sup, link in items:
    t = q * u
    sub += t
    rows.append((ref, item, q, u, t, basis, sup, link))

ship = 250
cont = round(0.10 * (sub + ship))
total = sub + ship + cont

with open("BOM.csv", "w", newline="", encoding="utf-8") as f:
    w = csv.writer(f)
    w.writerow(["#", "Category", "Item / exact model", "Qty", "Unit INR", "Line INR", "Price basis", "Supplier", "Link"])
    for i, r in enumerate(rows, 1):
        w.writerow([i, *r])
    w.writerow([])
    w.writerow(["", "", "Subtotal", "", "", sub, "", "", ""])
    w.writerow(["", "", "Shipping (2 suppliers + LiPo handling)", "", "", ship, "", "", ""])
    w.writerow(["", "", "Contingency 10% of (subtotal+shipping)", "", "", cont, "", "", ""])
    w.writerow(["", "", "GRAND TOTAL", "", "", total, "", "", ""])

print("items", len(rows), "subtotal", sub, "shipping", ship, "contingency", cont, "TOTAL", total, "headroom", 10000 - total)

md = [
    "| # | Category | Item / exact model | Qty | Unit ₹ | Line ₹ | Price basis | Supplier | Link |",
    "|--:|---|---|--:|--:|--:|---|---|---|"
]
for i, (ref, item, q, u, t, basis, sup, link) in enumerate(rows, 1):
    lab = "product page" if "?p=" in link else ("search (confirm listing)" if "?s=" in link else "store home")
    md.append(f"| {i} | {ref} | {item} | {q} | {u:,} | {t:,} | {basis} | {sup} | [{lab}]({link}) |")
md += [
    f"| | | **Subtotal** | | | **{sub:,}** | | | |",
    f"| | | Shipping (2 suppliers + LiPo handling) | | | {ship:,} | | | |",
    f"| | | Contingency (10 % of subtotal + shipping) | | | {cont:,} | | | |",
    f"| | | **GRAND TOTAL** | | | **{total:,}** | | | |"
]
with open("bom_table.md", "w", encoding="utf-8") as f:
    f.write("\n".join(md) + "\n")

# ---------------------------------------------------------------- power budget
print("\n--- power budget (TB6612FNG + 2S 7.4V LiPo) ---")
V_BAT, V_BUCK, EFF = 7.4, 5.0, 0.88
loads5 = {  # name: (idle mA, peak mA) on the 5 V rail (3.3 V LDO current is drawn from 5 V too)
    "ESP32 DevKit (WROOM-32, radio off / RF burst)": (80, 240),
    "IR array 5x TCRT5000 (LED 3.3V-220R ~9.3 mA each)": (47, 47),
    "TB6612FNG logic supply (Icc @ 3.3V, typ 2 mA, max 5 mA)": (2, 5),
    "END + status LED (~5 mA each @ 3.3V)": (5, 10),
}
idle5 = sum(a for a, b in loads5.values())
peak5 = sum(b for a, b in loads5.values())
print("5V/3.3V logic subtotal: idle", idle5, "mA  peak", peak5, "mA")

bat_logic_idle = (V_BUCK * (idle5 / 1000.0)) / (EFF * V_BAT) * 1000.0
bat_logic_peak = (V_BUCK * (peak5 / 1000.0)) / (EFF * V_BAT) * 1000.0
print("battery-side logic current (at 7.4V via buck eff %.2f): idle %.1f mA, peak %.1f mA" % (EFF, bat_logic_idle, bat_logic_peak))

MOT_RUN, MOT_STALL = 150, 1000
tot_idle = round(bat_logic_idle)
tot_typ = round(bat_logic_idle) + 2 * MOT_RUN
tot_peak = round(bat_logic_peak) + 2 * MOT_STALL
print("system total at 7.4V battery: idle %d mA  typical %d mA  worst-case(stall) %d mA" % (tot_idle, tot_typ, tot_peak))

CAP, DER = 1000.0, 0.80
for lab, i in (("idle", tot_idle), ("typical", tot_typ), ("worst/stall", tot_peak)):
    hrs = (CAP * DER) / i
    print("runtime %-12s %.2f h = %.0f min" % (lab, hrs, hrs * 60.0))

match_mah = tot_typ * (5.5 / 60.0)
print("energy in 5.5 min match at typical (%d mA): %.1f mAh (%.1f%% of 1000 mAh pack)" % (tot_typ, match_mah, match_mah / CAP * 100.0))
print("C needed for worst case: %.2f C ; 40C pack gives %.0f A (>17x headroom)" % (tot_peak / CAP, 40 * CAP / 1000.0))
print("2S LiPo pack energy: 7.4V x 1.0 Ah = %.1f Wh; specific energy at 55 g = %.0f Wh/kg" % (7.4, 7.4 / 0.055))
print("2S Li-ion 2x18650 (2500 mAh, ~45 g each + 20 g holder): %.1f Wh, %.0f g -> %.0f Wh/kg" % (2 * 3.7 * 2.5, 2 * 45 + 20, (2 * 3.7 * 2.5) / ((2 * 45 + 20) / 1000.0)))

# TB6612FNG MOSFET dissipation (R_DS(on) = 0.5 ohm typical, R_thetaJA = 91.5 C/W on PCB)
R_DS_ON = 0.5
R_THETA_JA = 91.5
for lab, i in (("running 150 mA/ch", 0.15), ("brisk 0.5 A/ch", 0.50), ("stall 1.0 A/ch", 1.00)):
    vdrop = i * R_DS_ON
    p = 2 * vdrop * i
    tj = 25.0 + R_THETA_JA * p
    print("TB6612FNG %-18s Vdrop = %.3f V, P_total = 2 x %.3f W = %.3f W -> Tj(91.5 C/W, 25 C amb) = %.1f C" % (lab, vdrop, vdrop * i, p, tj))

v_mot_nom = V_BAT - 0.15 * R_DS_ON
v_mot_full = 8.4 - 0.15 * R_DS_ON
print("motor voltage at full pack (8.4 V) minus ~0.08 V MOSFET drop = %.2f V ; at nominal 7.4 V minus 0.08 V = %.2f V" % (v_mot_full, v_mot_nom))
rpm_at_140_pwm = 600.0 * (v_mot_nom / 6.0) * (140.0 / 255.0)
speed_ms = (rpm_at_140_pwm / 60.0) * 3.14159 * 0.044
print("wheel speed @ baseSpeed=140 PWM (effective %.2f V on 6V N20): %.0f rpm -> %.2f m/s (44 mm wheel)" % (v_mot_nom * 140.0 / 255.0, rpm_at_140_pwm, speed_ms))
