% =========================================================================
% participant_template.m  –  Cyber Leg Challenge | Participant Controller
% GNU Octave
%
% RULES
%   • Only edit code between WORKSPACE START and WORKSPACE END.
%   • Do NOT rename this function, do NOT change its signature.
%   • Do NOT attempt to read or reverse-engineer challenge_environment.p
%
% HOW YOUR CONTROLLER IS CALLED
%   The evaluation engine calls:
%       [V_hip, V_knee] = participant_template(Z_ref, Z_meas, theta_1, theta_2, dt, reset_flag)
%
%   reset_flag = true  on the very first call → re-initialise all state.
%   reset_flag = false on every subsequent call → run normal control step.
%
%   Inputs each step:
%     Z_ref      – target foot height        [m]
%     Z_meas     – noisy measured height     [m]
%     theta_1    – noisy hip encoder         [rad]
%     theta_2    – noisy knee encoder        [rad]
%     dt         – fixed time-step = 0.005 s
%     reset_flag – true on first call only
%
%   Outputs each step:
%     V_hip      – hip  motor voltage command  [V]
%     V_knee     – knee motor voltage command  [V]
%
% RECOMMENDED ARCHITECTURE
%   Outer loop : PID on height error  →  Theta_ref  (desired hip angle)
%   Inner loop1: PID on hip error     →  V_hip
%   Inner loop2: PID on knee error    →  V_knee
%
%   Symmetric knee reference (starting point):
%       Theta2_ref = -(pi - Theta_ref)
% =========================================================================

function [V_hip, V_knee] = participant_template(Z_ref, Z_meas, theta_1, theta_2, dt, reset_flag)

% =========================================================================
% =====================  WORKSPACE START  =================================
% =========================================================================

% persistent keeps these variables alive between calls
persistent Kp_h Ki_h Kd_h
persistent Kp_j Ki_j Kd_j
persistent alpha_lpf
persistent int_lim_h int_lim_j
persistent int_h  prev_err_h  lpf_derr_h
persistent int_j1 prev_err_j1 lpf_derr_j1
persistent int_j2 prev_err_j2 lpf_derr_j2

% ── Initialise on first call ──────────────────────────────────────────────
if reset_flag || isempty(Kp_h)

  % Outer loop gains (height → angle)
  Kp_h = 300;
  Ki_h = 46;
  Kd_h = 17;

  % Inner loop gains (angle → voltage)
  Kp_j = 18.0;
  Ki_j =  1.2;
  Kd_j =  1.5;

  % Low-pass filter on D-term  (0 = max filtering, 1 = no filtering)
  alpha_lpf = 0.15;

  % Anti-windup integrator clamp limits
  int_lim_h = 0.40;   % [rad]
  int_lim_j = 6.00;   % [V]

  % Integrator and derivative memory
  int_h  = 0;  prev_err_h  = 0;  lpf_derr_h  = 0;
  int_j1 = 0;  prev_err_j1 = 0;  lpf_derr_j1 = 0;
  int_j2 = 0;  prev_err_j2 = 0;  lpf_derr_j2 = 0;

end

% ── OUTER LOOP: height error → desired hip angle ──────────────────────────
err_h      = Z_ref - Z_meas;
int_h      = max(-int_lim_h, min(int_lim_h, int_h + err_h * dt));
raw_derr_h = (err_h - prev_err_h) / dt;
lpf_derr_h = alpha_lpf * raw_derr_h + (1 - alpha_lpf) * lpf_derr_h;
prev_err_h = err_h;

Theta_ref  = Kp_h * err_h + Ki_h * int_h + Kd_h * lpf_derr_h;
Theta_ref  = max(0.15, min(1.40, Theta_ref));

% Symmetric knee target
Theta2_ref = -(pi - Theta_ref);
Theta2_ref = max(-2.40, min(-0.15, Theta2_ref));

% ── INNER LOOP 1: hip ─────────────────────────────────────────────────────
err_j1      = Theta_ref  - theta_1;
int_j1      = max(-int_lim_j, min(int_lim_j, int_j1 + err_j1 * dt));
raw_d1      = (err_j1 - prev_err_j1) / dt;
lpf_derr_j1 = alpha_lpf * raw_d1 + (1 - alpha_lpf) * lpf_derr_j1;
prev_err_j1 = err_j1;

V_hip  = Kp_j * err_j1 + Ki_j * int_j1 + Kd_j * lpf_derr_j1;

% ── INNER LOOP 2: knee ────────────────────────────────────────────────────
err_j2      = Theta2_ref - theta_2;
int_j2      = max(-int_lim_j, min(int_lim_j, int_j2 + err_j2 * dt));
raw_d2      = (err_j2 - prev_err_j2) / dt;
lpf_derr_j2 = alpha_lpf * raw_d2 + (1 - alpha_lpf) * lpf_derr_j2;
prev_err_j2 = err_j2;

V_knee = Kp_j * err_j2 + Ki_j * int_j2 + Kd_j * lpf_derr_j2;

% =========================================================================
% ======================  WORKSPACE END  ==================================
% =========================================================================

end
