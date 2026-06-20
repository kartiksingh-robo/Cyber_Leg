% =========================================================================
% challenge_environment_no_impact.m  –  No-Disturbance Tuning Backend
% GNU Octave  |  Cyber Leg Competition
% =========================================================================

function env = challenge_environment_no_impact()

  L1 = 0.25;  L2 = 0.25;
  m1 = 0.5;   m2 = 0.35;    % link masses [kg]
  g  = 9.81;

  R_mot = 2.0;    % armature resistance [Ω]
  L_mot = 0.005;  % armature inductance [H]
  Ke    = 0.45;   % back-EMF constant   [V·s/rad]
  Kt    = 0.45;   % torque constant     [N·m/A]
  b_mot = 0.005;  % viscous friction    [N·m·s/rad]

  J1 = m1*(L1/2)^2 + m2*L1^2 + 0.010;  % hip  inertia [kg·m²]
  J2 = m2*(L2/2)^2             + 0.005;  % knee inertia [kg·m²]

  V_MAX = 12.0;   % voltage limit [V]
  I_MAX =  4.0;   % current limit [A]

  sigma_Z     = 0.004;   % height noise  [m]
  sigma_theta = 0.008;   % encoder noise [rad]

  dt = 0.005;   % 5 ms time-step

  state           = zeros(6,1);
  t_now           = 0;

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
    t_now = 0;
    obs   = build_obs(0, 0);
  end

  % =========================================================================
  function [obs, done] = do_step(V_hip, V_knee)

    % 1. Actuator saturation
    V_hip  = max(-V_MAX, min(V_MAX, V_hip));
    V_knee = max(-V_MAX, min(V_MAX, V_knee));

    % 2. Electrical layer
    th1=state(1); w1=state(2); i1=state(3);
    th2=state(4); w2=state(5); i2=state(6);

    i1_new = max(-I_MAX, min(I_MAX, i1 + (V_hip  - Ke*w1 - R_mot*i1)/L_mot * dt));
    i2_new = max(-I_MAX, min(I_MAX, i2 + (V_knee - Ke*w2 - R_mot*i2)/L_mot * dt));

    tau_m1 = Kt * i1_new;
    tau_m2 = Kt * i2_new;

    % 3. Mechanical layer
    tau_g1 = (m1*L1/2 + m2*L1) * g * cos(th1);
    tau_g2 =  m2*L2/2           * g * cos(th1 + th2);

    % Note: Impact layer logic has been removed entirely for clean tuning.
    alpha1 = (tau_m1 - tau_g1 - b_mot*w1) / J1;
    alpha2 = (tau_m2 - tau_g2 - b_mot*w2) / J2;

    w1_new  = w1  + alpha1 * dt;
    th1_new = th1 + w1    * dt;
    w2_new  = w2  + alpha2 * dt;
    th2_new = th2 + w2    * dt;

    % Clamp to workspace
    th1_new = max(2.513, min(3.142, th1_new));
    th2_new = max(-1.571, min(-0.942, th2_new));

    state = [th1_new; w1_new; i1_new; th2_new; w2_new; i2_new];
    t_now = t_now + dt;

    obs  = build_obs(V_hip, V_knee);
    done = (t_now >= env.params.t_end);
  end

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
    obs.impact_occurred = false;
    obs.V_hip_sat       = V_h;
    obs.V_knee_sat      = V_k;
    obs.i1              = state(3);
    obs.i2              = state(6);
  end

end
