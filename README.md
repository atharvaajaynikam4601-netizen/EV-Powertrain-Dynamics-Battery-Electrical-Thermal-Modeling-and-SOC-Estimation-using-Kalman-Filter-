# EV Powertrain Dynamics using UDDS Drive Cycle, Battery Electrical & Thermal Analysis using Kalman Filter

## Abstract

This project presents a physics-based simulation of a Battery Electric Vehicle (BEV) powertrain implemented entirely in **MATLAB**, using the **EPA Urban Dynamometer Driving Schedule (UDDS)** as the driving input.

The simulation integrates multiple subsystems of an electric vehicle including:

- Vehicle longitudinal dynamics
- Motor torque–speed constraints
- Drivetrain efficiency losses
- Regenerative braking
- Battery electrical behavior
- Battery thermal dynamics
- State of Charge (SOC) estimation using a Kalman Filter

Unlike simplified EV energy models, this implementation captures **realistic hardware constraints** of electric vehicles, including traction motor limits, internal battery losses, accessory loads, and regenerative braking effectiveness.

The model provides system-level insight into **energy flow from the wheels to the battery**, allowing evaluation of **vehicle efficiency, thermal behavior, power demand, and range estimation** under a real driving cycle.

---

# Contents

1. [Introduction](1-Introduction)
2. [System Modeling Architecture](2-System-Modeling-Architecture)
3. [Mathematical Modeling](3-Mathematical-Modeling)
4. [MATLAB Implementation Methodology](4-MATLAB-Implementation-Methodology)
5. [Simulation Results and Performance Metrics](5-Simulation-Results-and-Performance-Metrics)
6. [Debugging of Code](6-Debugging-of-Code)
7. [Learning Outcomes](7-Learning-Outcomes)

---

# 1 Introduction

## 1.1 Objective

The objective of this project is to simulate the **dynamic energy behaviour of a Battery Electric Vehicle (BEV)** under a real driving cycle and evaluate:

- Vehicle longitudinal power demand
- Battery energy consumption
- Regenerative braking effectiveness
- Battery electrical characteristics
- Battery thermal behaviour
- State of Charge (SOC) evolution
- SOC estimation accuracy using Kalman Filtering
- Estimated vehicle driving range

The project bridges the gap between **vehicle mechanics, electrical powertrain modeling, and battery system simulation**.

---

## 1.2 Tools and Libraries Used

**MATLAB**

The entire simulation is implemented in MATLAB using numerical methods and built-in functions.

Key MATLAB functions used:

- `readtable()` → Import drive cycle dataset  
- `diff()` → Numerical differentiation to compute acceleration  
- `trapz()` → Numerical integration for energy and distance calculations  
- `interp1()` → Lookup table interpolation for SOC-OCV modeling  
- `scatter()` → Efficiency map visualization  
- `fprintf()` → Structured result reporting  

---

## 1.3 Input Dataset

The simulation uses the **EPA Urban Dynamometer Driving Schedule (UDDS)** which represents **urban driving conditions with frequent stop-and-go behavior**.

| Parameter | Value |
|--------|------|
Duration | 1369 s |
Distance | ~12 km |
Driving Pattern | Urban |
Average Speed | ~31 km/h |

The dataset contains **time and vehicle speed values**, which are converted into SI units before simulation.

---

# 2 System Modeling Architecture

The simulation follows a **backward-facing vehicle modeling approach** where the required wheel power determines the electrical power demand from the battery.


Drive Cycle
↓
Vehicle Longitudinal Dynamics
↓
Wheel Mechanical Power
↓
Motor Torque-Speed Constraints
↓
Drivetrain Efficiency
↓
Battery Power Flow
↓
Battery Electrical Model
↓
Battery Thermal Model
↓
SOC Calculation & Estimation


The system consists of the following major modules:

1. Vehicle Longitudinal Dynamics  
2. Wheel Mechanical Power Calculation  
3. Motor Torque-Speed Limits  
4. Drivetrain Efficiency Modeling  
5. Battery Power Flow  
6. Battery Electrical Model  
7. Battery Thermal Model  
8. SOC Estimation using Kalman Filter  
9. Energy Flow Analysis  
10. Range Estimation  

---

# 3 Mathematical Modeling

## 3.1 Vehicle Longitudinal Dynamics

The required traction force is calculated as:


F_total = F_rr + F_aero + F_acc + F_rg


Where:

Rolling Resistance


F_rr = m · g · C_rr


Aerodynamic Drag


F_aero = 0.5 · ρ · C_d · A · v²


Acceleration Force


F_acc = m_eq · a


Equivalent vehicle mass including drivetrain rotational inertia:


m_eq = m (1 + δ)


This accounts for the **rotational inertia of wheels, gears and shafts**.

---

## 3.2 Wheel Mechanical Power

The mechanical power required at the wheel is calculated as:


P_wheel = F_total × v


Tire losses are incorporated using a tire efficiency factor.

---

## 3.3 Motor Torque-Speed Constraint

Electric traction motors operate in two distinct regions.

### Constant Torque Region


T = T_max


### Constant Power Region (Field Weakening)


T = P_max / ω


The base speed separating these regions is


ω_base = P_max / T_max


This logic is implemented using conditional control logic in MATLAB.

---

## 3.4 Motor Efficiency Model

Motor efficiency varies with operating point. Normalized parameters are defined as:


Tn = T / T_max
ωn = ω / ω_max


Efficiency is approximated using a quadratic loss model.

Typical operating ranges:

Drive efficiency


0.75 ≤ η_drive ≤ 0.92


Regenerative braking efficiency


0.55 ≤ η_regen ≤ 0.80


---

## 3.5 Regenerative Braking

During deceleration phases the electric motor operates as a generator.

Recovered power is limited by:

- Maximum regenerative torque
- Battery charging power
- Minimum vehicle speed

Regenerative braking is disabled below


v < 1.5 m/s


to represent **back-EMF limitations of traction motors**.

---

## 3.6 Battery Electrical Model

Battery current is computed as:


I_batt = P_batt / V_nom


Terminal voltage is modeled using internal resistance:


V_batt = V_oc − I_batt · R_internal


The open-circuit voltage is obtained from an **SOC-OCV lookup table** using interpolation.

---

## 3.7 Battery Thermal Model

Battery thermal behavior is modeled using a lumped thermal network.

Heat generation


Q = I² · R_internal


Cooling heat transfer


Q_cooling = (T_batt − T_amb) / R_th


Temperature dynamics


dT/dt = (Q − Q_cooling) / C_th


This allows monitoring of **battery thermal safety limits** during operation.

---

## 3.8 State of Charge Model

SOC is updated based on energy consumption.


SOC(k) = SOC(k−1) − E_step / Battery Capacity


SOC is constrained within physical limits:


0 ≤ SOC ≤ 1


---

## 3.9 SOC Estimation using Kalman Filter

A simplified Kalman Filter is implemented to estimate SOC using noisy voltage measurements.

Prediction step:


SOC_pred = SOC_prev − (I × dt) / Capacity


Update step:


SOC_est = SOC_pred + K (V_meas − V_pred)


This approximates **real Battery Management System (BMS) SOC estimation algorithms** used in electric vehicles.

---

# 4 MATLAB Implementation Methodology

The project is structured into modular code sections.

### Part 1 — Vehicle Parameters

Defines:

- Vehicle mass
- Aerodynamic coefficients
- Drivetrain parameters
- Battery capacity

---

### Part 2 — Drive Cycle Import

Drive cycle data is imported using


readtable()


Vehicle speed is converted from mph to m/s.

Acceleration is calculated using


diff()


Acceleration limits are applied to prevent unrealistic spikes.

---

### Part 3 — Force Calculation

Rolling resistance, aerodynamic drag, and acceleration forces are computed.

---

### Part 4 — Wheel Power Calculation

Wheel mechanical power is calculated using


P = F × v


---

### Part 5 — Motor Constraints

Motor torque-speed limits and regenerative braking limits are implemented using conditional logic.

---

### Part 6 — Energy Calculation

Energy and travel distance are computed using numerical integration:


trapz()


---

### Part 7 — Efficiency Modeling

Motor efficiency is dynamically computed based on normalized speed and torque.

---

### Part 8 — Battery Power Flow

Battery power includes:

- propulsion power
- regenerative braking energy
- accessory load

Accessory load used in simulation:


900 W


---

### Part 9 — Battery Electrical and Thermal Modeling

Battery voltage, current, SOC, and temperature are simulated simultaneously.

---

### Part 10 — Visualization

Simulation outputs include:

- UDDS speed profile
- Battery power flow
- Battery temperature
- SOC evolution
- SOC estimation comparison
- Motor torque-speed curve
- Motor efficiency map

---

# 5 Simulation Results and Performance Metrics

The MATLAB simulation evaluates the performance of the electric vehicle under the **UDDS urban driving cycle**. The following results summarize the energy consumption, power demand, battery behaviour, and system efficiency of the vehicle model.

---

## 5.1 Vehicle and Drive Cycle Characteristics

| Parameter | Value |
|-----------|------|
Vehicle Mass | 1800 kg |
Drive Cycle Duration | 1369 s |
Total Distance Travelled | 11.990 km |
Average Speed | 31.51 km/h |

The UDDS cycle represents **urban stop-and-go driving conditions**, which significantly influence regenerative braking opportunities and energy consumption.

---

# 5.2 Energy Consumption Analysis

| Metric | Value |
|------|------|
Wheel Mechanical Energy | 815.62 Wh |
Battery Energy Drawn | 2222.15 Wh |
Battery Energy Regenerated | 644.54 Wh |
Net Battery Energy Used | 1577.61 Wh |
Specific Energy Consumption | **131.57 Wh/km** |

Although only **815.62 Wh of mechanical work** was required at the wheels, the battery supplied **2222.15 Wh**, demonstrating the impact of:

- drivetrain losses
- motor efficiency
- auxiliary loads
- power electronics losses

Regenerative braking recovered **644.54 Wh**, significantly reducing the net battery energy demand.

---

# 5.3 Power Performance

| Metric | Value |
|------|------|
Maximum Wheel Power | 43.32 kW |
Maximum Battery Drive Power | 50.08 kW |
Maximum Regenerative Power | 26.51 kW |

The difference between **wheel power and battery power** reflects losses in:

- drivetrain
- motor efficiency
- inverter conversion

---

# 5.4 Regenerative Braking Performance

| Metric | Value |
|------|------|
Mechanical Braking Energy | 994.54 Wh |
Recovered Regenerative Energy | 644.54 Wh |
Regen Energy Recovery | **64.81 %** |

This indicates that nearly **65% of braking energy** was recovered through regenerative braking under the UDDS cycle.

Urban driving cycles tend to produce **high regenerative recovery potential** due to frequent deceleration events.

---

# 5.5 Battery State of Charge (SOC)

| Metric | Value |
|------|------|
Initial SOC | 90.00 % |
Final SOC | 86.16 % |
SOC Change | 3.840 % |

The SOC reduction corresponds to the **net battery energy consumed during the drive cycle**.

---

# 5.6 Energy Flow Analysis

| Energy Flow Component | Value |
|------|------|
Drive Energy From Battery | 2222.15 Wh |
Recovered Regen Energy | 644.54 Wh |
Accessory Consumption | 342.25 Wh |
Net Battery Consumption | **1577.61 Wh** |

Accessory loads contribute significantly to overall energy consumption, especially in urban driving cycles.

The simulation includes a **900 W auxiliary load** representing HVAC and low-voltage systems.

---

# 5.7 Estimated EV Range

Vehicle range is estimated using the usable battery SOC window.


Usable SOC = 80% − 20%


| Metric | Value |
|------|------|
Estimated Range (UDDS Based) | **228.0 km** |

The range is computed using:


Range = Usable Battery Energy / Energy per km


This provides a **cycle-based range estimate under urban driving conditions**.

---

# 5.8 Battery Electrical Performance

| Metric | Value |
|------|------|
Average Battery Voltage | 387.05 V |
Maximum Battery Voltage | 393.27 V |
Minimum Battery Voltage | 382.11 V |
Maximum Battery Current | 145.67 A |

Battery voltage variation reflects **internal resistance losses under load conditions**.

Peak current occurs during **high acceleration demand**.

---

# 5.9 SOC Estimation Performance (Kalman Filter)

| Metric | Value |
|------|------|
Kalman Filter RMSE | 1.291 % |
Maximum SOC Estimation Error | 5.060 % |

The Kalman Filter successfully tracks SOC with **low estimation error**, demonstrating the feasibility of **voltage-based SOC estimation methods used in Battery Management Systems (BMS)**.

---

# 5.10 Battery Thermal Performance

| Metric | Value |
|------|------|
Maximum Battery Temperature | 26.59 °C |
Minimum Battery Temperature | 25.00 °C |
Temperature Rise | 1.59 °C |

The small temperature rise indicates that the **thermal load on the battery during the UDDS cycle is relatively low**, primarily due to moderate current levels.

---

# Key Insight from Simulation

The simulation highlights the strong influence of:

- drivetrain losses
- accessory loads
- regenerative braking efficiency
- battery internal resistance

on overall EV energy consumption.
---

# 6 Debugging of Code

During development several coding and modeling errors occurred.

---

### Error / Bug

MATLAB interpolation error


Error using interp1
X and V must be of the same length


Debugging Method

The SOC-OCV lookup table arrays were incorrectly defined with unequal lengths.  
Both arrays were corrected to have identical dimensions.

Correct implementation:


SOC_curve = [0 0.1 0.2 0.4 0.6 0.8 1.0]
OCV_curve = [300 320 335 350 365 380 400]


---

### Error / Bug

Battery terminal voltage remained **0 V during simulation**.

Debugging Method

Battery voltage was not being updated dynamically within the simulation loop.

Solution implemented:


V_batt = V_oc − I_batt · R_internal


This equation was moved inside the time-step simulation loop.

---

### Error / Bug

Minimum battery voltage reported as **0 V in results**.

Debugging Method

The battery voltage vector was not initialized at the first timestep.

Initialization added:


V_oc_vec(1) = interp1(...)
V_batt(1) = V_oc_vec(1) − I_batt(1) * R_internal


---

### Error / Bug

SOC estimation diverged significantly from true SOC.

Debugging Method

Kalman Filter parameters were poorly tuned.

The following parameters were adjusted:


Process Noise (Q) = 1e−6
Measurement Noise (R) = 0.5


---

### Error / Bug

Regenerative braking occurred at extremely low vehicle speeds.

Debugging Method

A minimum speed threshold for regen activation was implemented.


v_min_regen = 1.5 m/s


---

# 7 Learning Outcomes

This project enabled development of practical skills in **vehicle system simulation and EV battery modeling**.

Key engineering insights gained:

- Vehicle longitudinal dynamics modeling
- Powertrain energy flow analysis
- Motor torque-speed envelope implementation
- Efficiency map modeling
- Battery electrical modeling using internal resistance
- Battery thermal behavior simulation
- SOC estimation using Kalman Filtering
- Numerical integration for energy analysis
- System-level EV simulation using MATLAB

The project demonstrates how **mechanical, electrical, and thermal subsystems interact within an electric vehicle powertrain**.

Even though the **mechanical energy requirement was only 815 Wh**, the battery delivered **over 2.2 kWh**, demonstrating the importance of **system-level efficiency modeling in EV simulations**.
