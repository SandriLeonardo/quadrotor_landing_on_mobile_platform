function [tau_phi,tau_theta,tau_psi] = attitude_control(ref_ang_pos,ref_ang_vel,ref_ang_accel,state,params)

%------------------------------------------------------------------------
%                        STATE PARAMETERS IMPORT
%------------------------------------------------------------------------

phi = state(4);
theta = state(5);
psi = state(6);
phi_dot = state(10);
theta_dot = state(11);
psi_dot = state(12);


%------------------------------------------------------------------------
%                         OTHER PARAMETERS IMPORT
%------------------------------------------------------------------------

Ix = params(3);
Iy = params(4);
Iz = params(5);
Kp_ang = params(15:17);
Kd_ang = params(18:20);
Ki_ang = params(21:23);
dt = params(24);


%------------------------------------------------------------------------
%                           RPY CONTROL INPUTS
%------------------------------------------------------------------------


% ---------- PHI ANGLE ERRORS (ALONG THE X AXIS) 
e_phi = ref_ang_pos(1) - phi;                         %PROPORTIONAL ERROR
e_phi_dot = ref_ang_vel(1) - phi_dot;                 %DERIVATIVE ERROR


persistent integral_error_phi;                        %INTEGRAL ERROR INITIAL.
if isempty(integral_error_phi)
    integral_error_phi = 0;
end

% ---------- PID CONTROL PHI (ROLL, ALONG X-AXIS)
tau_phi_pid = Kp_ang(1) * e_phi + Kd_ang(1) * e_phi_dot + Ki_ang(1) * integral_error_phi;

% ---------- INVERSE DYNAMICS FEEDFORWARD (Correction 1)
% Gyroscopic coupling terms
tau_phi_ff = Ix * ref_ang_accel(1) + (Iz - Iy) * theta_dot * psi_dot;

% Total control with feedforward
tau_phi_total = tau_phi_pid + tau_phi_ff;

% ---------- ANTI-WINDUP (Correction 2)
% Saturation limits for torque
tau_max_phi = 1.0;  % [N·m] - Adjust based on your actuator limits
tau_phi = saturate(tau_phi_total, -tau_max_phi, tau_max_phi);

% Only integrate if not saturated OR if error helps unsaturate
if (abs(tau_phi_total) <= tau_max_phi) || (sign(e_phi) ~= sign(tau_phi))
    integral_error_phi = integral_error_phi + e_phi * dt;
end
%-------------------------------------------------------------------------


% ---------- THETA ANGLE ERRORS (ALONG THE Y AXIS) 
e_theta = ref_ang_pos(2) - theta;                  %PROPORTIONAL ERROR
e_theta_dot = ref_ang_vel(2) - theta_dot;          %DERIVATIVE ERROR


persistent integral_error_theta;                   %INTEGRAL ERROR INITIAL.
if isempty(integral_error_theta)
    integral_error_theta = 0;
end

% ---------- PID CONTROL THETA (PITCH, ALONG Y-AXIS)
tau_theta_pid = Kp_ang(2) * e_theta + Kd_ang(2) * e_theta_dot + Ki_ang(2) * integral_error_theta;

% ---------- INVERSE DYNAMICS FEEDFORWARD (Correction 1)
% Gyroscopic coupling terms
tau_theta_ff = Iy * ref_ang_accel(2) + (Ix - Iz) * phi_dot * psi_dot;

% Total control with feedforward
tau_theta_total = tau_theta_pid + tau_theta_ff;

% ---------- ANTI-WINDUP (Correction 2)
tau_max_theta = 1.0;  % [N·m] - Adjust based on your actuator limits
tau_theta = saturate(tau_theta_total, -tau_max_theta, tau_max_theta);

% Only integrate if not saturated OR if error helps unsaturate
if (abs(tau_theta_total) <= tau_max_theta) || (sign(e_theta) ~= sign(tau_theta))
    integral_error_theta = integral_error_theta + e_theta * dt;
end
%-------------------------------------------------------------------------


% ---------- PSI ANGLE ERRORS (ALONG THE Z AXIS) 
e_psi = ref_ang_pos(3) - psi;                       %PROPORTIONAL ERROR
e_psi_dot = ref_ang_vel(3) - psi_dot;               %DERIVATIVE ERROR


persistent integral_error_psi;                      %INTEGRAL ERROR INITIAL.
if isempty(integral_error_psi)
    integral_error_psi = 0;
end

% ---------- PID CONTROL PSI (YAW, ALONG Z-AXIS)
tau_psi_pid = Kp_ang(3) * e_psi + Kd_ang(3) * e_psi_dot + Ki_ang(3) * integral_error_psi;

% ---------- INVERSE DYNAMICS FEEDFORWARD (Correction 1)
% Gyroscopic coupling terms
tau_psi_ff = Iz * ref_ang_accel(3) + (Iy - Ix) * phi_dot * theta_dot;

% Total control with feedforward
tau_psi_total = tau_psi_pid + tau_psi_ff;

% ---------- ANTI-WINDUP (Correction 2)
tau_max_psi = 0.5;  % [N·m] - Adjust based on your actuator limits
tau_psi = saturate(tau_psi_total, -tau_max_psi, tau_max_psi);

% Only integrate if not saturated OR if error helps unsaturate
if (abs(tau_psi_total) <= tau_max_psi) || (sign(e_psi) ~= sign(tau_psi))
    integral_error_psi = integral_error_psi + e_psi * dt;
end
%-------------------------------------------------------------------------

end

%UTILITY FUNCTION TO SATURATE SOME VARIABLES
function val_sat = saturate(val, min_val, max_val)
    val_sat = max(min(val, max_val), min_val);
end