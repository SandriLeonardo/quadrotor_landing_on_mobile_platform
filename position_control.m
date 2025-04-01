function [Uvec,Tvec,ref_ang_pos,ref_ang_vel] = position_control(desired_traj,state,params)
%run('C:\Users\loren\Desktop\Mag\Elective 1\quadrotor_landing_on_mobile_platform\setup\model_n_control_param.m')

%This epsilon is added to expression that could be zero (leading to
%singulairities)
epsilon_for_numerical_stability=1e-6;

x = state(1);
y = state(2);
z = state(3);
x_dot = state(4);
y_dot = state(5);
z_dot = state(6);
phi = state(7);
theta = state(8);
psi = state(9);


x_d = desired_traj(1);
y_d = desired_traj(2);
z_d = desired_traj(3);
x_d_dot = desired_traj(4);
y_d_dot = desired_traj(5);
z_d_dot = desired_traj(6);
x_d_ddot = desired_traj(7);
y_d_ddot = desired_traj(8);
z_d_ddot = desired_traj(9);

m = params(1);
g = params(2);
Kp_pos = params(6:8);
Kd_pos = params(9:11);
Ki_pos = params(12:14);
dt = params(24);

% Parametri (m, g, guadagni, dt, ecc.) devono essere definiti o caricati
% z_d, z, z_d_dot, z_dot, z_d_ddot sono rispettivamente la posizione, la velocità e
% l'accelerazione desiderate e attuali lungo l'asse z.

% Calcolo dell'errore e della sua derivata (lungo z)
ez = z_d - z;
ez_dot = z_d_dot - z_dot;

% Aggiornamento dell'errore integrale lungo z (supponendo dt definito)
persistent integral_error_z;
if isempty(integral_error_z)
    integral_error_z = 0;
end

integral_error_z = integral_error_z + ez * dt;
% integral_max = 10.0;  % Valore arbitrario
% integral_error_z = max(min(integral_error_z, integral_max), -integral_max);

% Calcolo del termine di controllo lungo z (PID)--------------------------|
T_z = -(Kp_pos(3) * ez + Kd_pos(3) * ez_dot + Ki_pos(3) * integral_error_z + z_d_ddot) + g;

% LYAPUNOV VERSION
% (Slide of Backstepping Control + Claude w/reasoningadding the integral term)
%T_z = z_d_ddot + (1+Kp_pos(3)*Kd_pos(3))*ez + (Kp_pos(3)+Kd_pos(3))*ez_dot + Ki_pos(3)*integral_error_z;

% Calcolo della spinta totale, considerando la proiezione
%T = m * T_z / ((cos(phi) * cos(theta)) + epsilon_for_numerical_stability);
T = (m / (cos(phi) * cos(theta))) * (g + T_z);

% Calcolo dell'errore e della sua derivata
ex = x_d - x;
ex_dot = x_d_dot - x_dot;

% Aggiornamento dell'integrale (supponendo dt definito)
persistent integral_error_x;
if isempty(integral_error_x)
    integral_error_x = 0;
end
integral_error_x = integral_error_x + ex * dt;
% integral_max = 10.0;  % Valore arbitrario
% integral_error_x = max(min(integral_error_x, integral_max), -integral_max);

% Calcolo del termine di controllo lungo x (PID)
U_x = Kp_pos(1) * ex + Kd_pos(1) * ex_dot + Ki_pos(1) * integral_error_x + x_d_ddot;

% Calcolo dell'errore e della sua derivata
ey = y_d - y;
ey_dot = y_d_dot - y_dot;

% Aggiornamento dell'integrale (supponendo dt definito)
persistent integral_error_y;
if isempty(integral_error_y)
    integral_error_y = 0;
end
integral_error_y = integral_error_y + ey * dt;
% integral_max = 10.0;  % Valore arbitrario
% integral_error_y = max(min(integral_error_y, integral_max), -integral_max);

% Calcolo del termine di controllo lungo y (PID)
U_y = Kp_pos(2) * ey + Kd_pos(2) * ey_dot + Ki_pos(2) * integral_error_y + y_d_ddot;



%U_x and U_y need sto be bounded
% if we do not bound U_x and U_y we can obtain an unfeasable control

% Calculate maximum allowable accelerations based on physics
max_tilt_angle = 0.4; % About 40 degrees, arbitrary
max_lateral_accel = g * tan(max_tilt_angle); % Maximum possible lateral acceleration
asinboundary=0.99 

%BOUNDING IN UX AND UY temporarluy commented out
U_x=saturate(U_x,-max_lateral_accel,max_lateral_accel)
U_y=saturate(U_y,-max_lateral_accel,max_lateral_accel)



T_x = (m/T) * U_x;
T_y = (m/T) * U_y;


phi_d = asin(saturate(T_y*cos(psi) - T_x*sin(psi),-asinboundary,asinboundary));
%phi_d = asin(T_y);
theta_d = -asin(saturate((T_y*cos(psi) + T_x*sin(psi))/cos(phi_d),-asinboundary,asinboundary));
%theta_d = -asin((T_x));
%psi_d = atan2(y_d_dot, x_d_dot);
psi_d = 0;

% phi_d = max(min(phi_d, max_angle), -max_angle);
% theta_d = max(min(theta_d, max_angle), -max_angle);

%ref_ang_pos = [phi_d,theta_d,psi_d];

% Memorizza stati precedenti per calcolo velocità angolari
persistent prev_phi_d prev_theta_d prev_psi_d prev_time;
if isempty(prev_phi_d)
    prev_phi_d = phi_d;
    prev_theta_d = theta_d;
    prev_psi_d = psi_d;
    prev_time = 0;
end

% Calcolo delle velocità angolari con filtro passa-basso
time_constant = 0.05;  % Costante di tempo per il filtro
phi_dot_d = (phi_d - prev_phi_d) / dt;
theta_dot_d = (theta_d - prev_theta_d) / dt;
psi_dot_d = (psi_d - prev_psi_d) / dt;

% Filtraggio delle velocità angolari
persistent filtered_phi_dot_d filtered_theta_dot_d filtered_psi_dot_d;
if isempty(filtered_phi_dot_d)
    filtered_phi_dot_d = phi_dot_d;
    filtered_theta_dot_d = theta_dot_d;
    filtered_psi_dot_d = psi_dot_d;
end

alpha = dt / (time_constant + dt);  % Coefficiente filtro passa-basso
filtered_phi_dot_d = (1-alpha) * filtered_phi_dot_d + alpha * phi_dot_d;
filtered_theta_dot_d = (1-alpha) * filtered_theta_dot_d + alpha * theta_dot_d;
filtered_psi_dot_d = (1-alpha) * filtered_psi_dot_d + alpha * psi_dot_d;


% Aggiorna i valori precedenti per il prossimo ciclo
prev_phi_d = phi_d;
prev_theta_d = theta_d;
prev_psi_d = psi_d;
prev_time = prev_time + dt;

% Output
ref_ang_pos = [phi_d, theta_d, psi_d];
ref_ang_vel = [filtered_phi_dot_d, filtered_theta_dot_d, filtered_psi_dot_d];
%FOR SCOPES
Uvec=[U_x,U_y]
Tvec=[T,T_x,T_y,T_z]
end

%UTILITY FUNCTION TO SATURATE SOME VARIABLES
function val_sat = saturate(val, min_val, max_val)
    val_sat = max(min(val, max_val), min_val);
end