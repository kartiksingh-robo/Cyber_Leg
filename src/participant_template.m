function [V_hip, V_knee] = participant_template(Z_ref, Z_meas, theta_1, theta_2, dt, is_reset)
% PARTICIPANT_TEMPLATE  Cascaded PID controller for the Cyber Leg Challenge
%
%   [V_hip, V_knee] = participant_template(Z_ref, Z_meas, theta_1, theta_2, dt, is_reset)
%
%   Inputs:
%       Z_ref    : Target height reference [m]
%       Z_meas   : Current measured height (noisy) [m]
%       theta_1  : Measured hip joint angle (noisy) [rad]
%       theta_2  : Measured knee joint angle (noisy) [rad]
%       dt       : Controller timestep [s]
%       is_reset : Boolean flag indicating if internal state should be reset
%
%   Outputs:
%       V_hip    : Motor voltage for the hip joint [-12V to 12V]
%       V_knee   : Motor voltage for the knee joint [-12V to 12V]

    % =====================================================================
    % 1. Initialization and Persistent Variables
    % =====================================================================
    % Retain internal states between simulation steps for integrators & filters
    persistent int_Z prev_Z_meas lpf_dZ
    persistent int_th1 prev_th1 lpf_dth1
    persistent int_th2 prev_th2 lpf_dth2

    if isempty(int_Z) || is_reset
        % Initialize outer loop states
        int_Z = 0.0;
        prev_Z_meas = Z_meas;
        lpf_dZ = 0.0;
        
        % Initialize inner loop states (hip)
        int_th1 = 0.0;
        prev_th1 = theta_1;
        lpf_dth1 = 0.0;
        
        % Initialize inner loop states (knee)
        int_th2 = 0.0;
        prev_th2 = theta_2;
        lpf_dth2 = 0.0;
    end

    % =====================================================================
    % 2. Parameters & Gains
    % =====================================================================
    % System Constants
    L1 = 0.25;      % Hip link length [m]
    L2 = 0.25;      % Knee link length [m]
    V_MAX = 12.0;   % Maximum motor voltage [V]

    % Low-pass Filter Parameter for Derivative Terms
    % Used exponentially weighted moving average (EWMA).
    alpha_d = 0.15;  % Filter coefficient: lower = smoother, higher = faster

    % Outer Loop PID Gains (Height Z)
    % Outputs a target Z offset (or direct Z_target) to feed to IK
    Kp_Z = 1.0;
    Ki_Z = 0.2;
    Kd_Z = 0.02;

    % Inner Loop PID Gains (Hip Joint theta_1)
    Kp_th1 = 200.0;
    Ki_th1 = 150.0;
    Kd_th1 = 5.0;

    % Inner Loop PID Gains (Knee Joint theta_2)
    Kp_th2 = 200.0;
    Ki_th2 = 150.0;
    Kd_th2 = 5.0;

    % =====================================================================
    % 3. Outer Height PID Controller
    % =====================================================================
    % Compute height tracking error
    e_Z = Z_ref - Z_meas;
    
    % Proportional Term
    P_Z = Kp_Z * e_Z;
    
    % Integral Term (with clamping anti-windup to prevent excessive buildup)
    int_Z_new = int_Z + e_Z * dt;
    int_Z = max(-0.1, min(0.1, int_Z_new)); % Clamp integrated value
    I_Z = Ki_Z * int_Z;
    
    % Derivative Term (on measurement to avoid derivative kick from ref changes)
    raw_dZ = -(Z_meas - prev_Z_meas) / dt;
    lpf_dZ = alpha_d * raw_dZ + (1 - alpha_d) * lpf_dZ;
    D_Z = Kd_Z * lpf_dZ;
    
    % Update previous values
    prev_Z_meas = Z_meas;
    
    % The outer loop adjusts the requested reference height
    Z_target = Z_ref + P_Z + I_Z + D_Z;
    
    % Physically reachable bounds based on workspace 
    Z_target = max(0.25, min(0.48, Z_target));

    % =====================================================================
    % 4. Inverse Kinematics
    % =====================================================================
    % Forward kinematics: Z = L1*sin(th1) + L2*sin(th1+th2)
    % Competition constraints state: symmetric knee convention (th1 + th2 = pi/2)
    % Substituting this: Z = L1*sin(th1) + L2*sin(pi/2) = L1*sin(th1) + L2
    % Rearranging for th1: sin(th1) = (Z - L2) / L1
    
    sin_th1 = (Z_target - L2) / L1;
    sin_th1 = max(-1.0, min(1.0, sin_th1)); % Ensure domain validity for asin
    
    % The workspace for th1 is in the second quadrant: [144 deg, 180 deg]
    th1_ref = pi - asin(sin_th1);
    
    % Using the symmetric knee constraint to find knee angle reference
    th2_ref = pi/2 - th1_ref;

    % =====================================================================
    % 5. Inner Joint PID Controller (Hip / theta_1)
    % =====================================================================
    e_th1 = th1_ref - theta_1;
    
    P_th1 = Kp_th1 * e_th1;
    
    % Derivative on measurement (avoid ref kick)
    raw_dth1 = -(theta_1 - prev_th1) / dt;
    lpf_dth1 = alpha_d * raw_dth1 + (1 - alpha_d) * lpf_dth1;
    D_th1 = Kd_th1 * lpf_dth1;
    
    % Tentative Integral and Output
    int_th1_new = int_th1 + e_th1 * dt;
    V_hip_raw = P_th1 + Ki_th1 * int_th1_new + D_th1;
    
    % 6. Anti-windup (Conditional Integration / Clamping)
    if (V_hip_raw > V_MAX && e_th1 > 0) || (V_hip_raw < -V_MAX && e_th1 < 0)
        % Actuator saturated & integration making it worse: STOP integration
        V_hip_raw = P_th1 + Ki_th1 * int_th1 + D_th1; 
    else
        % Otherwise, keep the new integration value
        int_th1 = int_th1_new;
    end
    
    prev_th1 = theta_1;

    % =====================================================================
    % 7. Inner Joint PID Controller (Knee / theta_2)
    % =====================================================================
    e_th2 = th2_ref - theta_2;
    
    P_th2 = Kp_th2 * e_th2;
    
    % Derivative on measurement
    raw_dth2 = -(theta_2 - prev_th2) / dt;
    lpf_dth2 = alpha_d * raw_dth2 + (1 - alpha_d) * lpf_dth2;
    D_th2 = Kd_th2 * lpf_dth2;
    
    % Tentative Integral and Output
    int_th2_new = int_th2 + e_th2 * dt;
    V_knee_raw = P_th2 + Ki_th2 * int_th2_new + D_th2;
    
    % 6. Anti-windup (Conditional Integration / Clamping)
    if (V_knee_raw > V_MAX && e_th2 > 0) || (V_knee_raw < -V_MAX && e_th2 < 0)
        % Actuator saturated & integration making it worse: STOP integration
        V_knee_raw = P_th2 + Ki_th2 * int_th2 + D_th2;
    else
        int_th2 = int_th2_new;
    end
    
    prev_th2 = theta_2;

    % =====================================================================
    % 8. Output Saturation
    % =====================================================================
    V_hip  = max(-V_MAX, min(V_MAX, V_hip_raw));
    V_knee = max(-V_MAX, min(V_MAX, V_knee_raw));

end
