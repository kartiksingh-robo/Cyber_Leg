# Cyber Leg Challenge: Control Philosophy and Implementation Report

## Control Philosophy
The control architecture uses a **Cascaded Proportional-Integral-Derivative (PID)** strategy. A cascaded loop provides several advantages:
1. **Separation of Concerns**: The outer loop solely focuses on tracking the height trajectory in Cartesian space, while the inner loop strictly ensures the joint angles track the references quickly.
2. **Disturbance Rejection**: The inner loop runs fast enough to react to non-linearities (like changing inertia and gravity terms as the leg extends), leaving the outer loop to just deal with generalized height errors.
3. **Safety**: We can limit the output of the outer loop to ensure the requested height is always within the physical monotonic workspace.

### System Flow
```
Height Reference (Z_ref) -> [ Outer PID ] -> Target Height (Z_target) 
-> [ Inverse Kinematics ] -> Target Joint Angles (th1_ref, th2_ref) 
-> [ Inner PIDs ] -> Target Voltages (V_hip, V_knee) -> [ Saturation ]
```

## Inverse Kinematics & Workspace Constraint
Because the leg has two joints (2 degrees of freedom) but we only need to track one Cartesian coordinate (Height Z), the system is redundant. To resolve this, we enforce a constraint defined as the **Symmetric Knee Convention**:
```math
\theta_1 + \theta_2 = \frac{\pi}{2}
```
Given the forward kinematics:
```math
Z = L_1 \sin(\theta_1) + L_2 \sin(\theta_1 + \theta_2)
```
Substituting the constraint yields:
```math
Z = L_1 \sin(\theta_1) + L_2 \sin(\frac{\pi}{2}) = L_1 \sin(\theta_1) + L_2
```
Rearranging to solve for the reference hip angle $\theta_1$:
```math
\theta_1 = \pi - \arcsin\left(\frac{Z - L_2}{L_1}\right)
```
*Note: The $\pi -$ is necessary because the defined monotonic workspace for $\theta_1$ is in the second quadrant (144° to 180°).*

## PID Equations and Low-Pass Filter
The discrete-time implementation for the PID controllers evaluates the following components every step `dt = 0.005s`:

**Proportional**:
$P[k] = K_p \cdot e[k]$

**Integral (Forward Euler)**:
$I_{acc}[k] = I_{acc}[k-1] + e[k] \cdot dt$
$I[k] = K_i \cdot I_{acc}[k]$

**Derivative (on Measurement)**:
To avoid derivative kick when the reference abruptly changes (which occurs in the step-like target trajectory), the derivative is taken on the measurement instead of the error.
$d_{raw}[k] = - \frac{y[k] - y[k-1]}{dt}$

### Low-pass Filter for Derivative
The height measurements and encoder readings are subject to Gaussian noise ($\sigma_Z = 0.004$, $\sigma_\theta = 0.008$). Differentiating this noise produces large spikes, destroying control signals. To mitigate this, an Exponentially Weighted Moving Average (EWMA) low-pass filter is applied to the derivative term:
```math
d_{filtered}[k] = \alpha \cdot d_{raw}[k] + (1 - \alpha) \cdot d_{filtered}[k-1]
```
Where $\alpha$ determines the cutoff frequency. We chose $\alpha = 0.15$ to provide significant smoothing while still allowing the derivative term to react quickly enough to dampen oscillations.
$D[k] = K_d \cdot d_{filtered}[k]$

## Anti-windup Method
When the requested voltage exceeds the motor limit of ±12V, the actuator saturates. If the controller continues to integrate the error during this period (integral windup), it will take a very long time to "unwind" once the error reverses sign, leading to massive overshoot.

We address this using **Conditional Integration (Clamping)**:
```matlab
if (V_raw > V_MAX && e > 0) || (V_raw < -V_MAX && e < 0)
    % Stop Integration
else
    % Accumulate Integration
end
```
This ensures the integral term stops growing if the actuator is already pushing as hard as it can in the direction of the error.

## Tuning Procedure
The tuning procedure was executed sequentially, starting from the inner loop:
1. **Inner Loop Tuning (Hip & Knee PIDs)**:
   - Evaluated step responses for arbitrary joint angles.
   - Initialized with $K_p$, increased until the joints reacted fast but became slightly oscillatory. ($K_p = 180$)
   - Added $K_d$ to increase damping and eliminate oscillations. Due to noise, large $K_d$ required a relatively low filter coefficient $\alpha$. ($K_d = 6$)
   - Added $K_i$ to remove steady-state error caused by gravity dropping the leg. ($K_i = 60$)
2. **Outer Loop Tuning (Height PID)**:
   - With the inner loops successfully tracking joint angles, the outer loop was tuned to track the height trajectory.
   - $K_p$ was set to react to height tracking errors. ($K_p = 0.8$)
   - $K_i$ was crucial here. When the 1.5kg payload drops, the leg sags drastically. A strong integral term in the outer loop guarantees the leg will recover and return to the reference height. ($K_i = 0.5$)

## Future Improvements
1. **Feedforward Gravity Compensation**: Currently, the PID integral terms carry the entire burden of counteracting gravity. By implementing a feedforward term that calculates the precise gravitational torque using the known masses $m_1, m_2$ and link lengths $L_1, L_2$, the controller's responsiveness would dramatically improve, reducing the reliance on the integral term and lowering overshoot.
2. **Dynamic Filter Cutoff**: Implementing a dynamic $\alpha$ or a Kalman filter that adapts to the noise variance rather than a static EWMA.
