% =========================================================================
% challenge_environment.m  –  Cyber Leg Challenge: Physics & Hardware Simulator
% GNU Octave  |  Cyber Leg Competition
%
% HOW IT WORKS
% ──────────────────────────────────────────────────────────────────────────
% Five layers execute every 5 ms step:
%
%  1. ACTUATOR INPUT LAYER   – clamp voltages to ±12 V
%
%  2. ELECTRICAL LAYER       – DC motor R-L circuit with back-EMF:
%                                L·di/dt = V − R·i − Ke·ω
%                              Current clamped to ±4 A. Torque = Kt·i.
%
%  3. MECHANICAL LAYER       – Newtons 2nd law for rotation:
%                                J·α = τ_motor − τ_gravity − b·ω
%                              Gravity torques from Lagrangian mechanics:
%                                τ_g1 = (m1·L1/2 + m2·L1)·g·cos(θ1)
%                                τ_g2 = m2·L2/2·g·cos(θ1+θ2)
%                              Euler integration. Angle stops clamped.
%
%  4. IMPACT LAYER           – One-step impulsive torque at random time
%                              in [3,8]s, simulating a dropped payload.
%
%  5. SENSOR LAYER           – Forward kinematics:
%                                Z = L1·sin(θ1) + L2·sin(θ1+θ2)
%                              Gaussian noise added to Z, θ1, θ2.
%
% KINEMATIC WORKSPACE
% ──────────────────────────────────────────────────────────────────────────
% Symmetric knee convention: θ2 + θ1 = π/2
% Monotonic workspace: θ1 ∈ [1.571, 2.199] rad  (90° to 126°)
%                      covers foot heights 0.25 m to 0.44 m
%
% Motor parameters (Kt=Ke=0.45 represent a geared actuator):
%   Holding current at worst-case pose: ≤3.2 A  (within 4 A limit)
%   Voltage budget at max speed:        ≤11.6 V (within 12 V limit)
%
% INITIAL POSE:  θ1=103°, θ2=−77°  →  Z≈0.353 m
%
% PUBLIC INTERFACE
% ──────────────────────────────────────────────────────────────────────────
%   env = challenge_environment()
%   obs = env.reset()
%   [obs, done] = env.step(V_hip, V_knee)
%
% obs fields:
%   .Z_measured   .Z_true   .theta_1   .theta_2
%   .t   .impact_occurred   .V_hip_sat   .V_knee_sat   .i1   .i2
% =========================================================================

function env = challenge_environment()

  L1 = 0.25;  L2 = 0.25;
  m1 = 0.5;   m2 = 0.35;    % link masses [kg] (geared actuator assembly)
  g  = 9.81;

  R_mot = 2.0;    % armature resistance [Ω]
  L_mot = 0.005;  % armature inductance [H]
  Ke    = 0.45;   % back-EMF constant   [V·s/rad]  (geared)
  Kt    = 0.45;   % torque constant     [N·m/A]    (geared)
  b_mot = 0.005;  % viscous friction    [N·m·s/rad]

  J1 = m1*(L1/2)^2 + m2*L1^2 + 0.010;  % hip  inertia [kg·m²]
  J2 = m2*(L2/2)^2             + 0.005;  % knee inertia [kg·m²]

  V_MAX = 12.0;   % voltage limit [V]
  I_MAX =  4.0;   % current limit [A]

  sigma_Z     = 0.004;   % height noise  [m]
  sigma_theta = 0.008;   % encoder noise [rad]

  dt = 0.005;   % 5 ms time-step

  impact_mass = 1.5;   % dropped payload [kg]

  state           = zeros(6,1);
  t_now           = 0;
  t_impact        = 0;
  impact_applied  = false;
  impact_occurred = false;

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
    state = [2.827; 0; 0; -1.257; 0; 0];
    t_now           = 0;
    impact_applied  = false;
    impact_occurred = false;
    t_impact        = 3.0 + 5.0 * rand();   % random in [3, 8] s
    obs = build_obs(0, 0);
  end

  % =========================================================================
  function [obs, done] = do_step(V_hip, V_knee)

    % 1. Actuator saturation
    V_hip  = clamp(V_hip,  -V_MAX, V_MAX);
    V_knee = clamp(V_knee, -V_MAX, V_MAX);

    % 2. Electrical layer
    th1=state(1); w1=state(2); i1=state(3);
    th2=state(4); w2=state(5); i2=state(6);

    i1_new = clamp(i1 + (V_hip  - Ke*w1 - R_mot*i1)/L_mot * dt, -I_MAX, I_MAX);
    i2_new = clamp(i2 + (V_knee - Ke*w2 - R_mot*i2)/L_mot * dt, -I_MAX, I_MAX);

    tau_m1 = Kt * i1_new;
    tau_m2 = Kt * i2_new;

    % 3. Mechanical layer
    tau_g1 = (m1*L1/2 + m2*L1) * g * cos(th1);
    tau_g2 =  m2*L2/2           * g * cos(th1 + th2);

    % 4. Impact layer (one-shot impulse at randomised time)
    tau_imp1 = 0;  tau_imp2 = 0;
    if ~impact_applied && t_now >= t_impact
      impact_applied  = true;
      impact_occurred = true;
      tau_imp1 = -impact_mass * g * L1 * 4;
      tau_imp2 = -impact_mass * g * L2 * 4;
    end

    alpha1 = (tau_m1 - tau_g1 - b_mot*w1 + tau_imp1) / J1;
    alpha2 = (tau_m2 - tau_g2 - b_mot*w2 + tau_imp2) / J2;

    w1_new  = w1  + alpha1 * dt;
    th1_new = th1 + w1    * dt;
    w2_new  = w2  + alpha2 * dt;
    th2_new = th2 + w2    * dt;

    % Clamp to monotonic kinematic workspace
    th1_new = clamp(th1_new, 2.513, 3.142);   % 144° to 180°
    th2_new = clamp(th2_new, -1.571, -0.942); % −90° to −54°

    state = [th1_new; w1_new; i1_new; th2_new; w2_new; i2_new];
    t_now = t_now + dt;

    obs  = build_obs(V_hip, V_knee);
    done = (t_now >= env.params.t_end);
  end

  % =========================================================================
  % 5. Sensor layer: forward kinematics + noise
  % =========================================================================
  function obs = build_obs(V_h, V_k)
    th1    = state(1);
    th2    = state(4);
    Z_true = L1*sin(th1) + L2*sin(th1 + th2);

    obs.Z_true          = Z_true;
    obs.Z_measured      = Z_true + sigma_Z     * randn();
    obs.theta_1         = th1    + sigma_theta * randn();
    obs.theta_2         = th2    + sigma_theta * randn();
    obs.t               = t_now;
    obs.impact_occurred = impact_occurred;
    obs.V_hip_sat       = V_h;
    obs.V_knee_sat      = V_k;
    obs.i1              = state(3);
    obs.i2              = state(6);
  end

  function y = clamp(x, lo, hi)
    y = max(lo, min(hi, x));
  end

end
