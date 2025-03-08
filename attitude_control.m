function [tau_phi,tau_theta,tau_psi] = attitude_control(ref_ang_pos,ref_ang_vel,state,params)

phi = state(7);
theta = state(8);
psi = state(9);
phi_dot = state(10);
theta_dot = state(11);
psi_dot = state(12);

Kp_ang = params(15:17);
Kd_ang = params(18:20);
Ki_ang = params(21:23);
dt = params(24);

% Calcolo dell'errore e della sua derivata
e_phi = ref_ang_pos(1) - phi;
e_phi_dot = ref_ang_vel(1) - phi_dot;

% Aggiornamento dell'integrale (supponendo dt definito)
persistent integral_error_phi;
if isempty(integral_error_phi)
    integral_error_phi = 0;
end
integral_error_phi = integral_error_phi + e_phi * dt;
integral_max = 10.0;  % Valore arbitrario
integral_error_phi = max(min(integral_error_phi, integral_max), -integral_max);

% Calcolo del termine di controllo lungo z (PID)
tau_phi = Kp_ang(1) * e_phi + Kd_ang(1) * e_phi_dot + Ki_ang(1) * integral_error_phi;


% Calcolo dell'errore e della sua derivata
e_theta = ref_ang_pos(2) - theta;
e_theta_dot = ref_ang_vel(2) - theta_dot;

% Aggiornamento dell'integrale (supponendo dt definito)
persistent integral_error_theta;
if isempty(integral_error_theta)
    integral_error_theta = 0;
end
integral_error_theta = integral_error_theta + e_theta * dt;
integral_max = 10.0;  % Valore arbitrario
integral_error_theta = max(min(integral_error_theta, integral_max), -integral_max);

% Calcolo del termine di controllo lungo z (PID)
tau_theta = Kp_ang(2) * e_theta + Kd_ang(2) * e_theta_dot + Ki_ang(2) * integral_error_theta;

% Calcolo dell'errore e della sua derivata
e_psi = ref_ang_pos(3) - psi;
e_psi_dot = ref_ang_vel(3) - psi_dot;

% Aggiornamento dell'integrale (supponendo dt definito)
persistent integral_error_psi;
if isempty(integral_error_psi)
    integral_error_psi = 0;
end
integral_error_psi = integral_error_psi + e_psi * dt;
integral_max = 10.0;  % Valore arbitrario
integral_error_psi = max(min(integral_error_psi, integral_max), -integral_max);

% Calcolo del termine di controllo lungo z (PID)
tau_psi = Kp_ang(3) * e_psi + Kd_ang(3) * e_psi_dot + Ki_ang(3) * integral_error_psi;
