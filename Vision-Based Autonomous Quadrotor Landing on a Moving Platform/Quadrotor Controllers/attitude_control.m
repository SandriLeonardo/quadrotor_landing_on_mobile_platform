function [tau_phi,tau_theta,tau_psi] = attitude_control(ref_ang_pos,ref_ang_vel,ref_ang_accel,state,params)

% State and Parameter Import
phi = state(4); theta = state(5); psi = state(6);
phi_dot = state(10); theta_dot = state(11); psi_dot = state(12);

Ix = params(3); Iy = params(4); Iz = params(5);
Kp_ang = params(15:17); Kd_ang = params(18:20); Ki_ang = params(21:23);
dt = params(24);

% ------------------------------------------------------------------------
% 1. ROLL CONTROL (Phi)
% ------------------------------------------------------------------------
e_phi = ref_ang_pos(1) - phi;
e_phi_dot = ref_ang_vel(1) - phi_dot;

persistent integral_error_phi;
if isempty(integral_error_phi), integral_error_phi = 0; end

% PID
tau_phi_pid = Kp_ang(1) * e_phi + Kd_ang(1) * e_phi_dot + Ki_ang(1) * integral_error_phi;

% SIMPLIFIED FEEDFORWARD 
tau_phi_ff = Ix * ref_ang_accel(1); 

% Total & Saturation
tau_phi_total = tau_phi_pid + tau_phi_ff;
tau_max_phi = 1.0; 
tau_phi = saturate(tau_phi_total, -tau_max_phi, tau_max_phi);

% Anti-Windup
if (abs(tau_phi_total) <= tau_max_phi) || (sign(e_phi) ~= sign(tau_phi))
    integral_error_phi = integral_error_phi + e_phi * dt;
end

% ------------------------------------------------------------------------
% 2. PITCH CONTROL (Theta)
% ------------------------------------------------------------------------
e_theta = ref_ang_pos(2) - theta;
e_theta_dot = ref_ang_vel(2) - theta_dot;

persistent integral_error_theta;
if isempty(integral_error_theta), integral_error_theta = 0; end

% PID
tau_theta_pid = Kp_ang(2) * e_theta + Kd_ang(2) * e_theta_dot + Ki_ang(2) * integral_error_theta;

% FEEDFORWARD
tau_theta_ff = Iy * ref_ang_accel(2);

% Total & Saturation
tau_theta_total = tau_theta_pid + tau_theta_ff;
tau_max_theta = 1.0;
tau_theta = saturate(tau_theta_total, -tau_max_theta, tau_max_theta);

% Anti-Windup
if (abs(tau_theta_total) <= tau_max_theta) || (sign(e_theta) ~= sign(tau_theta))
    integral_error_theta = integral_error_theta + e_theta * dt;
end

% ------------------------------------------------------------------------
% 3. YAW CONTROL (Psi)
% ------------------------------------------------------------------------
e_psi = ref_ang_pos(3) - psi;
% Angle Wrapping
while e_psi > pi, e_psi = e_psi - 2*pi; end
while e_psi < -pi, e_psi = e_psi + 2*pi; end

e_psi_dot = ref_ang_vel(3) - psi_dot;

persistent integral_error_psi;
if isempty(integral_error_psi), integral_error_psi = 0; end

% PID
tau_psi_pid = Kp_ang(3) * e_psi + Kd_ang(3) * e_psi_dot + Ki_ang(3) * integral_error_psi;

% SIMPLIFIED FEEDFORWARD
tau_psi_ff = Iz * ref_ang_accel(3);

% Total & Saturation
tau_psi_total = tau_psi_pid + tau_psi_ff;
tau_max_psi = 0.5;
tau_psi = saturate(tau_psi_total, -tau_max_psi, tau_max_psi);

% Anti-Windup
if (abs(tau_psi_total) <= tau_max_psi) || (sign(e_psi) ~= sign(tau_psi))
    integral_error_psi = integral_error_psi + e_psi * dt;
end

end

function val_sat = saturate(val, min_val, max_val)
    val_sat = max(min(val, max_val), min_val);
end