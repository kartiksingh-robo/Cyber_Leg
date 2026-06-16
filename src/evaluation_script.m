% =========================================================================
% evaluation_script.m  –  Cyber Leg Challenge | Official Grading Engine
% GNU Octave
%
% HOW TO RUN (from robotic_leg_challenge/src/):
%   octave evaluation_script.m
%
% Participants must NOT modify this file.
% =========================================================================

clear all; close all; clc;
addpath(pwd);

fprintf('=== Cyber Leg Challenge - Official Evaluation Engine ===\n\n');

% ── Load environment ───────────────────────────────────────────────────────
env = challenge_environment_no_impact();
obs = env.reset();
dt  = env.params.dt;

% ── Reference height trajectory ───────────────────────────────────────────
t_end = env.params.t_end;
N     = round(t_end / dt);
t_vec = (0:N-1) * dt;

Z_ref_traj = zeros(1, N);
for k = 1:N
  t = t_vec(k);
  if    t <  1.5,  Z_ref_traj(k) = 0.35;
  elseif t <  3.5, Z_ref_traj(k) = 0.43;
  elseif t <  6.0, Z_ref_traj(k) = 0.28;
  elseif t <  9.5, Z_ref_traj(k) = 0.40;
  elseif t < 12.0, Z_ref_traj(k) = 0.33;
  else,             Z_ref_traj(k) = 0.38;
  end
end

% ── Logging ───────────────────────────────────────────────────────────────
log_t        = nan(1, N);
log_Z_ref    = nan(1, N);
log_Z_meas   = nan(1, N);
log_Z_true   = nan(1, N);
log_V_hip    = nan(1, N);
log_V_knee   = nan(1, N);
log_theta1   = nan(1, N);
log_theta2   = nan(1, N);
log_i1       = nan(1, N);
log_i2       = nan(1, N);
log_impact_t = NaN;

% ── First call: reset participant controller persistent state ──────────────
[V_hip, V_knee] = participant_template( ...
  Z_ref_traj(1), obs.Z_measured, obs.theta_1, obs.theta_2, dt, true);

% =========================================================================
% MAIN SIMULATION LOOP
% =========================================================================
for k = 1:N

  Z_ref   = Z_ref_traj(k);
  Z_meas  = obs.Z_measured;
  theta_1 = obs.theta_1;
  theta_2 = obs.theta_2;

  % Call participant controller (persistent state survives between calls)
  [V_hip, V_knee] = participant_template(Z_ref, Z_meas, theta_1, theta_2, dt, false);

  % Clamp outputs
  V_hip  = max(-env.params.V_MAX, min(env.params.V_MAX, V_hip));
  V_knee = max(-env.params.V_MAX, min(env.params.V_MAX, V_knee));

  % Step physics
  [obs, done] = env.step(V_hip, V_knee);

  % Log
  log_t(k)      = obs.t;
  log_Z_ref(k)  = Z_ref;
  log_Z_meas(k) = obs.Z_measured;
  log_Z_true(k) = obs.Z_true;
  log_V_hip(k)  = V_hip;
  log_V_knee(k) = V_knee;
  log_theta1(k) = obs.theta_1;
  log_theta2(k) = obs.theta_2;
  log_i1(k)     = obs.i1;
  log_i2(k)     = obs.i2;

  if obs.impact_occurred && isnan(log_impact_t)
    log_impact_t = obs.t;
    fprintf('[t = %.3f s]  Payload impact!\n', log_impact_t);
  end

  if done, break; end
end

% =========================================================================
% SCORING
% =========================================================================
valid = ~isnan(log_t);

if ~isnan(log_impact_t)
  pre  = valid & (log_t <  log_impact_t);
  post = valid & (log_t >= log_impact_t) & (log_t < log_impact_t + 1.5);
else
  pre  = valid;
  post = false(size(valid));
end

rms_error     = sqrt(mean((log_Z_ref(pre) - log_Z_true(pre)).^2));
overshoot     = max(max(log_Z_true(valid) - log_Z_ref(valid)), 0);
impact_sag    = 0;
if any(post)
  impact_sag  = max(max(log_Z_ref(post) - log_Z_true(post)), 0);
end
dV            = diff(log_V_hip(valid)).^2 + diff(log_V_knee(valid)).^2;
wasted_energy = sum(dV);

fprintf('\n========================================\n');
fprintf('  EVALUATION RESULTS\n');
fprintf('========================================\n');
fprintf('  Pre-impact RMS error   : %8.4f m\n',  rms_error);
fprintf('  Max overshoot          : %8.4f m\n',  overshoot);
fprintf('  Impact sag             : %8.4f m\n',  impact_sag);
fprintf('  Wasted energy proxy    : %8.2f\n',    wasted_energy);
fprintf('========================================\n\n');

% =========================================================================
% PLOTS
% =========================================================================

figure(1);
plot(log_t(valid), log_Z_ref(valid),  'r--', 'linewidth', 2.0); hold on;
plot(log_t(valid), log_Z_meas(valid), 'b',   'linewidth', 0.8);
plot(log_t(valid), log_Z_true(valid), 'k',   'linewidth', 1.5);
if ~isnan(log_impact_t)
  plot([log_impact_t log_impact_t], [min(log_Z_true(valid))-0.05, max(log_Z_true(valid))+0.05], ...
       'm--', 'linewidth', 1.5);
  text(log_impact_t + 0.1, max(log_Z_ref(valid)) * 0.97, 'Impact', 'color', 'm', 'fontsize', 9);
end
legend('Z\_ref', 'Z\_meas (noisy)', 'Z\_true', 'location', 'northeast');
xlabel('Time [s]');  ylabel('Height [m]');
title('Height Tracking');
grid on;

figure(2);
plot(log_t(valid), log_V_hip(valid),               'b',   'linewidth', 1.2); hold on;
plot(log_t(valid), log_V_knee(valid),              'r',   'linewidth', 1.2);
plot(log_t(valid),  12*ones(sum(valid),1),         'k--', 'linewidth', 0.8);
plot(log_t(valid), -12*ones(sum(valid),1),         'k--', 'linewidth', 0.8);
legend('V\_hip', 'V\_knee', '\pm12 V limit', 'location', 'northeast');
xlabel('Time [s]');  ylabel('Voltage [V]');
title('Motor Voltages');
ylim([-14 14]);
grid on;

figure(3);
plot(log_t(valid), log_i1(valid),          'b',   'linewidth', 1.2); hold on;
plot(log_t(valid), log_i2(valid),          'r',   'linewidth', 1.2);
plot(log_t(valid),  4*ones(sum(valid),1),  'k--', 'linewidth', 0.8);
plot(log_t(valid), -4*ones(sum(valid),1),  'k--', 'linewidth', 0.8);
legend('i\_hip', 'i\_knee', '\pm4 A limit', 'location', 'northeast');
xlabel('Time [s]');  ylabel('Current [A]');
title('Motor Currents');
ylim([-5 5]);
grid on;

figure(4);
plot(log_t(valid), rad2deg(log_theta1(valid)), 'b', 'linewidth', 1.2); hold on;
plot(log_t(valid), rad2deg(log_theta2(valid)), 'r', 'linewidth', 1.2);
legend('\theta_1 hip', '\theta_2 knee', 'location', 'northeast');
xlabel('Time [s]');  ylabel('Angle [deg]');
title('Joint Angles (noisy encoder readings)');
grid on;

fprintf('Plots ready (figures 1-4). Press Enter to exit.\n');
pause;
