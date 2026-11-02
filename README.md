# WP1: CDO & Arrival Sequencing Assessment at Barcelona-El Prat Airport (LEBL)

**Course:** Infraestructures del Transport Aeri (ITA)

**Institution:** Escola d'Enginyeria de Telecomunicació i Aeroespacial de Castelldefels (EETAC) — UPC

**Deliverable:** WP1 Code Package (`ITA_2026-QP_WP1_GA1_code.zip`)

## 📌 Project Overview

This repository contains the complete Python implementation for simulating Continuous Descent Operations (CDO) trajectories, evaluating arrival sequencing strategies, and checking altitude constraint compliance for arrivals at Barcelona-El Prat Airport (LEBL) Runway 24L across six published Standard Terminal Arrival Routes (STARs):

* **ALBER1Z**, **CASPE2W**, **LOBAR2W**, **MARTA3Z**, **MATEX3Z**, and **PUMAL1Z**.

The codebase computes 2D backward trajectories from the Initial Approach Fix (IAF) at $h = 6,000\text{ ft}$ up to cruise level ($FL400$) using aircraft aerodynamic and engine performance data from EUROCONTROL's Base of Aircraft Data (BADA v3.10) under International Standard Atmosphere (ISA) conditions.

## 🚀 Key Features

1. **BADA 3.10 Performance Integration:** Models thrust, drag polar, flap configuration transitions at $6,000\text{ ft}$, and true airspeed dependent fuel consumption for five distinct aircraft models:

   * **B767-300ER**, **B777-300**, **B737**, **A320-212**, and **A319-131**.

2. **Analytical Minimum Rate of Descent (RoD) Solver:** Computes the optimal speed profile $V_{\text{min RoD}}$ analytically at each numerical step to maximize flight endurance and model idle-thrust continuous descents.

3. **Backward Numerical Integration:** Simulates time-step integrated ($\Delta t = 1.0\text{ s}$) trajectories from the IAF back to the Top of Descent (ToD) to account for target mass configurations ($80\%$ and $100\%$ MLW at IAF).

4. **Sequencing Analysis Engine:**

   * **Scenario 1:** Evaluates simultaneous arrival at STAR entry waypoints at `11:45:00 AM` and determines time of arrival (ToA) at the IAF.

   * **Scenario 2:** Solves for strict ATC fixed time-separation ($120\text{ s}$ / 2 minutes) at the IAF while maintaining original sequence order, calculating required delays and STAR entry times.

5. **STAR Altitude Compliance Checker:** Automatically verifies simulated altitude profiles against official ENAIRE AIP altitude constraints for all waypoints along the six STARs and generates standalone analytical figures.

## 📁 Repository & Zip Structure

```
ITA_2026-QP_WP1_GA1_code/
├── main.py                    # Main execution script (Generates figures 1-8 and CSVs)
├── simulator.py               # Core CDO simulation engine & ISA/BADA physical models
├── README.md                  # Project documentation & execution guide
├── scenario_1.csv             # Output: Scenario 1 sequence & arrival times at IAF
├── scenario_1.xlsx            # Output: Scenario 1 formatted spreadsheet
├── scenario_2.csv             # Output: Scenario 2 fixed 2-min separation sequencing
├── scenario_2.xlsx            # Output: Scenario 2 formatted spreadsheet
├── waypoints_altitude.csv     # Output: Aircraft altitudes at all STAR waypoints
├── waypoints_altitude.xlsx    # Output: Waypoints altitude formatted spreadsheet
├── Figure_1.png               # Plot: Global Spatial Flight Profile (h vs x)
├── Figure_2.png               # Plot: Global Temporal Flight Profile (h vs t)
├── Figure_3.png               # Plot: CDO Compliance - ALBER1Z STAR
├── Figure_4.png               # Plot: CDO Compliance - CASPE2W STAR
├── Figure_5.png               # Plot: CDO Compliance - LOBAR2W STAR
├── Figure_6.png               # Plot: CDO Compliance - MARTA3Z STAR
├── Figure_7.png               # Plot: CDO Compliance - MATEX3Z STAR
└── Figure_8.png               # Plot: CDO Compliance - PUMAL1Z STAR

```

## 🛠️ Requirements & Installation

The project is implemented in pure **Python 3.8+** with minimal dependencies.

### Dependencies

* `matplotlib` (for 2D trajectory and STAR constraint visualization)

To install the required library:

```
pip install matplotlib

```

## 💻 How to Run the Code

Execute the primary script from your terminal or Python IDE:

```
python main.py

```

### Execution Workflow:

1. **Figure 1 (Spatial Profile):** Displays pressure altitude $h\text{ (m)}$ vs. horizontal distance $x\text{ (m)}$ relative to the IAF for all 10 aircraft/weight combinations.

2. **Figure 2 (Temporal Profile):** Displays pressure altitude $h\text{ (m)}$ vs. descent time $t\text{ (s)}$ relative to the IAF.

3. **Figures 3–8 (STAR Compliance):** Displays route-by-route altitude windows alongside official ENAIRE AIP maximum (crimson dotted) and minimum (blue dotted) altitude bounds.

   *(Note: Close each interactive plot window to proceed to the next STAR analysis).*

4. **CSV Export:** Upon closing the graphical windows, the script automatically exports `waypoints_altitude.csv`, `scenario_1.csv`, and `scenario_2.csv` to the working directory.

## 📊 Summary of Physics & Mathematical Models

### 1. Atmosphere Model (ISA)

* **Troposphere (**$h < 11,000\text{ m}$**):** $T = T_0 - 0.0065 \cdot h$, $P = P_0 \left(\frac{T}{T_0}\right)^{5.2561}$

* **Lower Stratosphere (**$h \ge 11,000\text{ m}$**):** $T = 216.65\text{ K}$, $P = P_{11} \cdot \exp\left(-\frac{g_0 (h - 11000)}{R T}\right)$

* **Density:** $\rho = \frac{P}{R T}$

### 2. Analytical Minimum Rate of Descent Speed

$$
V_{\text{min RoD}} = \sqrt{\frac{\frac{T_{\text{desc}}}{3 C_{D0}} + \sqrt{\left(\frac{T_{\text{desc}}}{3 C_{D0}}\right)^2 + \frac{4 C_{D2} (m g)^2}{3 C_{D0}}}}{\rho \cdot S}}
$$

### 3. Flight Path Angle & State Vector Update

$$
\sin\gamma = \frac{T_{\text{desc}} - D}{m g_0}
$$

$$
h_{k+1} = h_k - V \sin\gamma \cdot \Delta t, \quad x_{k+1} = x_k - V \cos\gamma \cdot \Delta t
$$

$$
m_{k+1} = m_k + \text{FF} \cdot \Delta t
$$

## 👥 Authors & Group Information

* **Course:** Infraestructures del Transport Aeri (ITA) — 2026/2027

* **Deliverable:** WP1 - CDO & Arrival Sequencing Assessment

* **File Naming Convention:** `ITA_2026-QP_WP1_GA1_code.zip`