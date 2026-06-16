function [V_hip, V_knee] = participant_template(Z_ref, Z_meas, theta_1, theta_2, dt, reset_flag)
% =========================================================================
% participant_template.m  –  Cyber Leg Challenge | Tuned Reference Baseline
% GNU Octave
% =========================================================================

  % ── PERSISTENT VARIABLE DECLARATIONS ───────────────────────────────────
  % These variables maintain their values across successive time-steps
  persistent Kp_h Ki_h Kd_h alpha_lpf_h
  persistent Kp_j Ki_j Kd_j alpha_lpf_j
  persistent int_h  prev_err_h  lpf_derr_h
  persistent int_j1 prev_err_j1 lpf_derr_j1
  persistent int_j2 prev_err_j2 lpf_derr_j2
  persistent Z_filt th1_filt th2_filt
  persistent int_lim_h L1 L2 V_MAX

  % ── INITIALIZATION / RESET LAYER ───────────────────────────────────────
  if reset_flag || isempty(Kp_h)
    % Physical Constants
    L1    = 0.25;   
    L2    = 0.25;   
    V_MAX = 12.0;                    

    % Outer Height Loop Controller Gains
    Kp_h        = 4.5;       % Moderate authority to command kinematics
    Ki_h        = 1.8;       % Stronger integration to erase gravitational droop
    Kd_h        = 0.02;      
    alpha_lpf_h = 0.10;      % Smoothing factor for height derivative

    % Inner Joint Loop Controller Gains
    Kp_j        = 12.0;      % Kept low to avoid amplifying high-frequency noise
    Ki_j        = 2.5;       % Drives steady state joint tracking error to zero
    Kd_j        = 0.0;       % Left at zero due to raw encoder noise constraints
    alpha_lpf_j = 0.05;      

    % Integrator Saturation Limits
    int_lim_h   = 0.15;   

    % Reset States
    int_h       = 0;  prev_err_h  = 0;  lpf_derr_h  = 0;
    int_j1      = 0;  prev_err_j1 = 0;  lpf_derr_j1 = 0;
    int_j2      = 0;  prev_err_j2 = 0;  lpf_derr_j2 = 0;

    % Seed the sensor filters with the very first measurement sample
    Z_filt      = Z_meas; 
    th1_filt    = theta_1; 
    th2_filt    = theta_2;
  end

  % ── SENSOR SIGNAL FILTER LAYER (LOW-PASS) ──────────────────────────────
  % Tames high-frequency white noise before it enters the controller loops.
  % 85% emphasis on historical state tracker, 15% on new noisy sample.
  Z_filt   = 0.85 * Z_filt   + 0.15 * Z_meas;
  th1_filt = 0.85 * th1_filt + 0.15 * theta_1;
  th2_filt = 0.85 * th2_filt + 0.15 * theta_2;

  % ── OUTER LOOP: HEIGHT TRACKING (PID + FEEDFORWARD) ────────────────────
  % Approximate Kinematic Jacobian Projection
  % Z = L1*sin(th1) + L2*sin(th1 + th2)
  Z_cur        = L1*sin(th1_filt) + L2*sin(2*th1_filt - pi);
  dZdt1        = L1*cos(th1_filt) + 2*L2*cos(2*th1_filt - pi);
  dZdt1        = sign(dZdt1) * max(abs(dZdt1), 0.01); % Prevent divide-by-zero
  Theta_ref_ff = th1_filt + (Z_ref - Z_cur) / dZdt1;

  % Height Error Calculations
  err_h        = Z_ref - Z_filt;
  int_h        = max(-int_lim_h, min(int_lim_h, int_h + err_h * dt));
  raw_derr_h   = (err_h - prev_err_h) / dt;
  lpf_derr_h   = alpha_lpf_h * raw_derr_h + (1 - alpha_lpf_h) * lpf_derr_h;
  prev_err_h   = err_h;

  % Output Joint Command Generation
  Theta_ref    = Theta_ref_ff + Kp_h*err_h + Ki_h*int_h + Kd_h*lpf_derr_h;
  Theta_ref    = max(1.571, min(2.199, Theta_ref)); % Workspace clamp

  % Symmetric Link Constraint Implementation
  Theta2_ref   = -(pi - Theta_ref);
  Theta2_ref   = max(-1.571, min(-0.942, Theta2_ref));

  % ── INNER LOOP 1: HIP TRACKING (PI + ANTI-WINDUP) ──────────────────────
  err_j1       = Theta_ref - th1_filt;
  raw_d1       = (err_j1 - prev_err_j1) / dt;
  lpf_derr_j1  = alpha_lpf_j * raw_d1 + (1 - alpha_lpf_j) * lpf_derr_j1;
  prev_err_j1  = err_j1;

  V_hip_raw    = Kp_j*err_j1 + Ki_j*int_j1 + Kd_j*lpf_derr_j1;
  V_hip        = max(-V_MAX, min(V_MAX, V_hip_raw));

  % Conditional Integration Anti-Windup
  saturated_hip = (V_hip_raw > V_MAX && err_j1 > 0) || (V_hip_raw < -V_MAX && err_j1 < 0);
  if ~saturated_hip
    int_j1 = int_j1 + err_j1 * dt;
  end

  % ── INNER LOOP 2: KNEE TRACKING (PI + ANTI-WINDUP) ─────────────────────
  err_j2       = Theta2_ref - th2_filt;
  raw_d2       = (err_j2 - prev_err_j2) / dt;
  lpf_derr_j2  = alpha_lpf_j * raw_d2 + (1 - alpha_lpf_j) * lpf_derr_j2;
  prev_err_j2  = err_j2;

  V_knee_raw   = Kp_j*err_j2 + Ki_j*int_j2 + Kd_j*lpf_derr_j2;
  V_knee       = max(-V_MAX, min(V_MAX, V_knee_raw));

  % Conditional Integration Anti-Windup
  saturated_knee = (V_knee_raw > V_MAX && err_j2 > 0) || (V_knee_raw < -V_MAX && err_j2 < 0);
  if ~saturated_knee
    int_j2 = int_j2 + err_j2 * dt;
  end

end