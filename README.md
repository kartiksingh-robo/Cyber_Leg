# Cyber Leg Challenge

## Project Description
This repository contains the simulation environment and a robust cascaded PID control solution for the "SOI: Cyber Leg" competition. The objective of the competition is to stabilize and control a robotic two-link leg, tracking a target height trajectory in the presence of gravity, actuator limits (voltage/current), noisy sensor measurements, and random payload impact disturbances.

The provided controller utilizes a cascaded architecture:
1. **Outer Loop (Height Controller)**: Tracks the target `Z` height and produces a virtual `Z_target`.
2. **Inverse Kinematics**: Translates the `Z_target` into joint reference angles (`theta_1` and `theta_2`) using a symmetric knee convention.
3. **Inner Loop (Joint Controller)**: Tracks the target joint angles by outputting motor voltages.

The controller includes enhancements such as derivative-on-measurement, low-pass filtering on the derivative terms to handle noise, and anti-windup clamping to deal with voltage saturation.

## Folder Structure

```
robotic_leg_challenge/
├── src/
│   ├── challenge_environment.m              # Physics & hardware simulator (with impacts)
│   ├── challenge_environment_no_impact.m    # No-disturbance backend for tuning
│   ├── evaluation_script.m                  # Official grading engine
│   └── participant_template.m               # Complete cascaded PID controller
├── docs/
│   └── report.md                            # Detailed report on the control philosophy
└── README.md                                # This file
```

## Installation
Ensure you have **GNU Octave** installed on your system.
For Linux (Debian/Ubuntu):
```bash
sudo apt-get install octave
```

No external toolboxes are required. The project relies entirely on base Octave capabilities.

## Execution
To evaluate the controller and view the generated plots, navigate to the `src/` directory and run the evaluation script:

```bash
cd src
octave evaluation_script.m
```

This will run the leg simulation, trigger the random impact disturbance, and print the resulting performance metrics (Pre-impact RMS error, Max overshoot, Impact sag, Wasted energy proxy) to the terminal, followed by 4 plots (Height Tracking, Motor Voltages, Motor Currents, Joint Angles).

## How to Tune PID Parameters
The PID parameters are left configurable inside `src/participant_template.m`.
If you wish to re-tune the controller, open `participant_template.m` and modify the gains in the **Parameters & Gains** section.

**Tuning Workflow:**
1. **Tune the Inner Loop First**: Start by setting `Kp_Z`, `Ki_Z`, and `Kd_Z` in the outer loop to low values (or skip the outer loop temporarily) and focus on getting the joints to reach desired angles quickly without oscillating. Increase `Kp_th` until the leg oscillates, add `Kd_th` to dampen, and add a small `Ki_th` to eliminate steady-state error.
2. **Tune the Outer Loop**: Once the inner loop is fast and stable, start tuning the outer loop `Kp_Z` to track the height trajectory. 
3. **Handle Noise**: The environment adds noise to all measurements. If the motor voltages look erratic, adjust `alpha_d` (the low-pass filter coefficient). Lower values increase smoothing but introduce lag.
4. **Test in No-Impact Environment**: If you want to isolate tuning from random drops, you can temporarily switch `evaluation_script.m` to use `challenge_environment_no_impact()` instead of `challenge_environment()`, although the evaluation script already does this natively for baseline tests.
