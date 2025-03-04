function [T,phi_d,theta_d,psi_d] = position_control(desired_traj,state)
run('C:\Users\loren\Desktop\Mag\Elective 1\quadrotor_landing_on_mobile_platform\setup\model_n_control_param.m')

phi = state(7);
theta = state(8);
psi = state(9);
x = state(1);
y = state(2);
z = state(3);
x_dot = state(4);
y_dot = state(5);
z_dot = state(6);
x_d = desired_traj(1);
y_d = desired_traj(2);
z_d = desired_traj(3);
x_d_dot = desired_traj(4);
y_d_dot = desired_traj(5);
z_d_dot = desired_traj(6);
x_d_ddot = desired_traj(7);
y_d_ddot = desired_traj(8);
z_d_ddot = desired_traj(9);

% Parametri (m, g, guadagni, dt, ecc.) devono essere definiti o caricati
% z_d, z, z_d_dot, z_dot, z_d_ddot sono rispettivamente la posizione, la velocità e
% l'accelerazione desiderate e attuali lungo l'asse z.

% Calcolo dell'errore e della sua derivata
ez = z_d - z;
ez_dot = z_d_dot - z_dot;

% Aggiornamento dell'integrale (supponendo dt definito)
persistent integral_error_z;
if isempty(integral_error_z)
    integral_error_z = 0;
end
integral_error_z = integral_error_z + ez * dt;

% Calcolo del termine di controllo lungo z (PID)
T_z = Kp_pos(3) * ez + Kd_pos(3) * ez_dot + Ki_pos(3) * integral_error_z + z_d_ddot + g;

% Calcolo della spinta totale, considerando la proiezione
T = m * T_z / (cos(phi) * cos(theta));

% Calcolo dell'errore e della sua derivata
ex = x_d - x;
ex_dot = x_d_dot - x_dot;

% Aggiornamento dell'integrale (supponendo dt definito)
persistent integral_error_x;
if isempty(integral_error_x)
    integral_error_x = 0;
end
integral_error_x = integral_error_x + ex * dt;

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

% Calcolo del termine di controllo lungo x (PID)
U_y = Kp_pos(2) * ey + Kd_pos(2) * ey_dot + Ki_pos(2) * integral_error_y + y_d_ddot;

T_x = (m/T) * U_x;
T_y = (m/T) * U_y;

phi_d = asin(T_y*cos(psi) - T_x*sin(psi));
theta_d = -asin((T_y*cos(psi) + T_x*sin(psi))/cos(phi_d));
psi_d = atan(y_d_dot/x_d_dot);
