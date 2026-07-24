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
