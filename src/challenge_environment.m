% =========================================================================
% challenge_environment.m  –  Cyber Leg Challenge: Physics & Hardware Simulator
% GNU Octave  |  Cyber Leg Competition
%
% HOW IT WORKS (overview)
% ──────────────────────────────────────────────────────────────────────────
%
% This file builds a closure-based simulation object that models a 2-DOF
% robotic leg. Every time env.step() is called it advances physics by one
% 5 ms time-step and returns noisy sensor readings – exactly what a real
% embedded controller would see.
%
% The simulation has five layers, executed in order each step:
%
%  1. ACTUATOR INPUT LAYER
%     Clamp the requested voltages to ±12 V (hardware rail limit).
%
%  2. ELECTRICAL LAYER  (DC motor circuit)
%     Each motor is modelled as a series R-L circuit with back-EMF:
%
%       L · di/dt = V_applied − R·i − Ke·ω
%
%     Integrated with Forward Euler over dt = 0.005 s.
%     Current is then clamped to ±4 A (thermal / driver limit).
%     Motor torque:  τ_motor = Kt · i
%
%  3. MECHANICAL LAYER  (rigid-body dynamics)
%     Each joint is treated as a lumped rotating inertia J:
%
%       J · α = τ_motor − τ_gravity − b·ω  [+ τ_impact if it happens]
%
%     Gravity torque for hip:   τ_g1 = (m1·L1/2 + m2·L1) · g · cos(θ1)
%     Gravity torque for knee:  τ_g2 =  m2·L2/2          · g · cos(θ1+θ2)
%
%     These come from differentiating the potential energy of each link
%     with respect to its joint angle – the standard Lagrangian approach.
%     Joint velocities and angles are integrated with Forward Euler.
%     Mechanical angle stops are enforced by clamping.
%
%  4. IMPACT LAYER
%     At a randomly chosen time (3–8 s, set at reset), a payload mass is
%     dropped. This is modelled as a one-step impulsive torque spike on
%     both joints, proportional to m·g·L, simulating a sudden load.
%
%  5. SENSOR LAYER
%     Forward kinematics converts joint angles to foot height:
%
%       Z = L1·sin(θ1) + L2·sin(θ1+θ2)
%
%     Independent Gaussian noise is added to Z, θ1, θ2 before returning
%     them – simulating real encoder and IMU measurement errors.
%
% PUBLIC INTERFACE
% ──────────────────────────────────────────────────────────────────────────
%   env = challenge_environment()
%   obs = env.reset()                     – reset to standing pose
%   [obs, done] = env.step(V_hip, V_knee) – advance 5 ms
%
% obs fields:
%   .Z_measured      noisy foot height  [m]
%   .Z_true          true foot height   [m]  (for logging only)
%   .theta_1         noisy hip encoder  [rad]
%   .theta_2         noisy knee encoder [rad]
%   .t               simulation time    [s]
%   .impact_occurred logical: true once payload has dropped
%   .V_hip_sat       actual voltage applied after saturation [V]
%   .V_knee_sat      actual voltage applied after saturation [V]
%   .i1              hip motor current  [A]
%   .i2              knee motor current [A]
% =========================================================================

function env = challenge_environment()

  % ── Physical constants ──────────────────────────────────────────────────
  L1 = 0.25;       % upper-leg link length [m]
  L2 = 0.25;       % lower-leg link length [m]
  m1 = 1.2;        % upper-leg mass [kg]
  m2 = 0.8;        % lower-leg mass [kg]
  g  = 9.81;       % gravity [m/s²]

  % ── DC Motor parameters (same for both joints) ──────────────────────────
  R_mot = 2.0;     % armature resistance  [Ω]
  L_mot = 0.005;   % armature inductance  [H]
  Ke    = 0.08;    % back-EMF constant    [V·s/rad]
  Kt    = 0.08;    % torque constant      [N·m/A]
  b_mot = 0.002;   % viscous friction     [N·m·s/rad]

  % ── Lumped joint inertias ───────────────────────────────────────────────
  J1 = m1*(L1/2)^2 + m2*L1^2 + 0.010;  % hip  [kg·m²]
  J2 = m2*(L2/2)^2             + 0.005;  % knee [kg·m²]

  % ── Actuator hard limits ────────────────────────────────────────────────
  V_MAX = 12.0;    % [V]
  I_MAX =  4.0;    % [A]

  % ── Sensor noise (1-sigma) ──────────────────────────────────────────────
  sigma_Z     = 0.004;   % height  [m]
  sigma_theta = 0.008;   % encoder [rad]

  % ── Simulation time-step ────────────────────────────────────────────────
  dt = 0.005;     % 5 ms → 200 Hz

  % ── Impact parameters ───────────────────────────────────────────────────
  impact_mass = 1.5;   % dropped payload [kg]

  % ── State  [θ1, ω1, i1, θ2, ω2, i2] ───────────────────────────────────
  state = zeros(6,1);

  % ── Runtime bookkeeping ─────────────────────────────────────────────────
  t_now           = 0;
  t_impact        = 0;
  impact_applied  = false;
  impact_occurred = false;

  % ── Expose read-only params ─────────────────────────────────────────────
  env.params.L1    = L1;
  env.params.L2    = L2;
  env.params.dt    = dt;
  env.params.V_MAX = V_MAX;
  env.params.I_MAX = I_MAX;
  env.params.t_end = 15.0;

  env.reset = @do_reset;
  env.step  = @do_step;

  % =========================================================================
  function obs = do_reset()
    % Standing pose: hip=103 deg, knee=-77 deg → foot height ≈ 0.353 m
    % This puts the leg in a natural upright stance within the usable
    % kinematic workspace for the target height range (0.28–0.43 m).
    state = [1.798; 0; 0; -1.344; 0; 0];
    t_now           = 0;
    impact_applied  = false;
    impact_occurred = false;
    t_impact        = 3.0 + 5.0 * rand();   % randomised [3, 8] s
    obs = build_obs(0, 0);
  end

  % =========================================================================
  function [obs, done] = do_step(V_hip, V_knee)

    % 1. ACTUATOR INPUT LAYER ─────────────────────────────────────────────
    V_hip  = hard_clamp(V_hip,  -V_MAX, V_MAX);
    V_knee = hard_clamp(V_knee, -V_MAX, V_MAX);

    % 2. ELECTRICAL LAYER ──────────────────────────────────────────────────
    th1 = state(1); w1 = state(2); i1 = state(3);
    th2 = state(4); w2 = state(5); i2 = state(6);

    di1 = (V_hip  - Ke*w1 - R_mot*i1) / L_mot;
    di2 = (V_knee - Ke*w2 - R_mot*i2) / L_mot;
    i1_new = hard_clamp(i1 + di1*dt, -I_MAX, I_MAX);
    i2_new = hard_clamp(i2 + di2*dt, -I_MAX, I_MAX);

    tau_m1 = Kt * i1_new;
    tau_m2 = Kt * i2_new;

    % 3. MECHANICAL LAYER ──────────────────────────────────────────────────
    tau_g1 = (m1*L1/2 + m2*L1) * g * cos(th1);
    tau_g2 =  m2*L2/2           * g * cos(th1 + th2);

    % 4. IMPACT LAYER ──────────────────────────────────────────────────────
    tau_imp1 = 0; tau_imp2 = 0;
    if ~impact_applied && t_now >= t_impact
      impact_applied  = true;
      impact_occurred = true;
      tau_imp1 = -impact_mass * g * L1 * 4;
      tau_imp2 = -impact_mass * g * L2 * 4;
    end

    alpha1 = (tau_m1 - tau_g1 - b_mot*w1 + tau_imp1) / J1;
    alpha2 = (tau_m2 - tau_g2 - b_mot*w2 + tau_imp2) / J2;

    w1_new  = w1 + alpha1 * dt;
    th1_new = th1 + w1 * dt;
    w2_new  = w2 + alpha2 * dt;
    th2_new = th2 + w2 * dt;

    % Mechanical angle stops — covers the full usable workspace
    th1_new = hard_clamp(th1_new,  1.40,  2.50);   % hip:  ~80° to ~143°
    th2_new = hard_clamp(th2_new, -1.57, -0.35);   % knee: ~-90° to ~-20°

    state = [th1_new; w1_new; i1_new; th2_new; w2_new; i2_new];
    t_now = t_now + dt;

    obs  = build_obs(V_hip, V_knee);
    done = (t_now >= env.params.t_end);
  end

  % =========================================================================
  % 5. SENSOR LAYER
  % =========================================================================
  function obs = build_obs(V_h, V_k)
    th1 = state(1);
    th2 = state(4);
    Z_true = L1*sin(th1) + L2*sin(th1 + th2);

    obs.Z_true          = Z_true;
    obs.Z_measured      = Z_true       + sigma_Z     * randn();
    obs.theta_1         = th1          + sigma_theta * randn();
    obs.theta_2         = th2          + sigma_theta * randn();
    obs.t               = t_now;
    obs.impact_occurred = impact_occurred;
    obs.V_hip_sat       = V_h;
    obs.V_knee_sat      = V_k;
    obs.i1              = state(3);
    obs.i2              = state(6);
  end

  % =========================================================================
  function y = hard_clamp(x, lo, hi)
    y = max(lo, min(hi, x));
  end

end
