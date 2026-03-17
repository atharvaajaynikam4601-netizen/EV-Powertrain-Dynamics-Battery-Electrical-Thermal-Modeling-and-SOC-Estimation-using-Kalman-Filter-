# High-Fidelity Longitudinal Dynamics Modeling and Energy Management of Electric Vehicles

## 1. Project Overview
This project presents a deterministic simulation framework for evaluating Electric Vehicle (EV) energy consumption and range estimation. The study bridges the gap between **idealized analytical models** and **high-fidelity simulations** by integrating non-linear motor efficiency maps, regenerative braking constraints, and standardized drive cycle validation (UDDS).

The simulation allows for the comparison of theoretical energy requirements against real-world powertrain limitations, providing critical insights into the efficiency losses inherent in electric propulsion systems.

## 2. Technical Methodology & Plant Modeling

### 2.1 Vehicle Longitudinal Dynamics
The model solves the fundamental Equation of Motion at each discrete time step ($dt$):

$$F_{total} = F_{acc} + F_{aero} + F_{rr} + F_{rg}$$

Where:
* **Inertial Force ($F_{acc}$):** Implemented as $m_{eq} \cdot a$, where the equivalent mass ($m_{eq}$) incorporates a **Rotational Mass Factor** ($\delta = 0.08$) to account for the inertia of the rotor, gears, and wheels.
* **Aerodynamic Drag ($F_{aero}$):** Modeled using the squared velocity relationship: $\frac{1}{2} \rho C_d A v^2$.
* **Rolling Resistance ($F_{rr}$):** $m \cdot g \cdot C_{rr}$, simulating tire-road interface losses.
* **Grade Resistance ($F_{rg}$):** Evaluated at a constant 2° incline in the UDDS model to test gravity-induced torque demand.

### 2.2 Propulsion System (UDDS High-Fidelity Model)
The model respects the physical hardware limits of a PMSM (Permanent Magnet Synchronous Motor) through a dual-region torque-speed envelope:
* **Constant Torque Region:** Torque is limited by maximum inverter current ($T = T_{max}$) for speeds below $\omega_{base}$.
* **Constant Power (Field Weakening) Region:** Torque is limited by the DC link voltage, governed by the hyperbolic relationship $T = P_{max} / \omega$.



### 2.3 Non-Linear Efficiency Mapping
Rather than assuming a static efficiency, this model implements a **Quadratic Efficiency Approximation** for the combined motor and inverter system:

$$\eta_{drive} = 0.90 - 0.15(1-\omega_{norm})^2 - 0.20(T_{norm}-0.6)^2$$

This mathematical model ensures that efficiency is maximized at a "sweet spot" (60% torque, high speed) and penalized during high-current low-speed acceleration and low-load cruising.

## 3. Energy Management & SOC Tracking

### 3.1 Advanced Regenerative Braking Logic
The regeneration strategy is governed by three primary constraints:
1. **Mechanical Limit:** Regen torque is capped at 120 Nm to ensure vehicle stability and battery safety.
2. **Efficiency Loss:** $P_{regen}$ is scaled by $\eta_{regen}$ to account for energy conversion heat losses.
3. **Low-Speed Cut-off:** Regeneration is strictly disabled below $1.5$ m/s ($5.4$ km/h) due to insufficient Back-EMF to overcome the battery terminal voltage.

### 3.2 Battery State of Charge (SOC) Estimation
SOC is estimated via a "Coulomb Counting" approach in the energy domain, integrated using the trapezoidal rule:

$$SOC_{k} = SOC_{k-1} - \frac{(P_{drive} - P_{regen} + P_{accessory}) \cdot \Delta t}{E_{capacity}}$$

A constant **Accessory Load** of 900W is applied to simulate the parasitic drain of HVAC and low-voltage electronics.



## 4. Model Comparison: Analytical vs. High-Fidelity

| Parameter | Ideal Energy Model | UDDS High-Fidelity Model |
| :--- | :--- | :--- |
| **Input Speed Trace** | Synthetic Linear (0-60 km/h) | Standard EPA UDDS (Time-Series) |
| **Mass Factor** | Pure Translational | Translational + Rotational ($\delta=0.08$) |
| **Motor Constraints** | Unconstrained | Torque-Speed Envelope Limited |
| **Efficiency** | Static Baseline | Dynamic Quadratic Mapping $\eta(T, \omega)$ |
| **Regen Logic** | Idealized 60% | $T_{limit}$ + $v_{min}$ Cut-off |
| **Range Estimation** | Theoretical Maximum | Validated (80%-20% Usable Window) |

## 5. Learning Outcomes
* **System Integration:** Successful coupling of mechanical load models with electrical powertrain constraints.
* **Numerical Analysis:** Practical application of numerical differentiation (`diff`) and integration (`trapz`, `cumtrapz`) for real-time signal processing.
* **Normalization Techniques:** Use of normalized parameters to create scalable efficiency maps.
* **Sensitivity Studies:** Quantifying how parametric changes (e.g., vehicle mass) impact specific energy consumption ($Wh/km$).

## 6. How to Run
1. Ensure `uddsdc.csv` is located in the root directory.
2. Execute `UDDS_Model.m` in MATLAB to generate the performance plots and range estimation.
3. Use `Ideal_Model.m` to perform parametric sensitivity analysis on vehicle mass.

---
**References**
* [1] EPA Urban Dynamometer Driving Schedule (UDDS) Standards.
* [2] Guzzella, L., "Vehicle Propulsion Systems: Introduction to Modeling and Optimization."
* [3] IEEE Technical Report Writing Guidelines.
