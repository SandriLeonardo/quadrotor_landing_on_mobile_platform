function accelerations = dynamic_model(state, u, params)
% dynamic_model - Calcola le accelerazioni lineari e angolari del drone
% Input:
%   state: vettore dello stato [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
%   u: vettore di controllo [Tvec, tau_phi, tau_theta, tau_psi]
%   --  Tvec: [T,Tx,Ty,Tz]
%   params: parametri [m, g, Ix, Iy, Iz]
% Output:
%   accelerations: [x_ddot, y_ddot, z_ddot, phi_ddot, theta_ddot, psi_ddot]

% Estrai parametri
m = params(1);
g = params(2);
Ix = params(3);
Iy = params(4);
Iz = params(5);

% Estrai angoli di Eulero dallo stato
phi = state(7);     %ROLL
theta = state(8);   %PITCH
psi = state(9);     %YAW

% Estrai comandi di controllo
T = u(1,1);           %THRUST CONTROL, comes from POSITION CONTROLLER
Tx = u(1,2);
Ty = u(1,3);
Tz = u(1,4);

tau_phi = u(2);     %Control for desired torque for Phi (along x)
tau_theta = u(3);   %Control for desired torque for Pitch (along y)
tau_psi = u(4);     %Control for desired torque for Yaw (along z)

%ACCELERAZIONI CALCOLATE COME DA SLIDE [Slides QuadModeling-01 pg.23]

% Calcola accelerazioni lineari 
% x_ddot = -(cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi)) * T / m;
% y_ddot = -(sin(psi) * sin(theta) * cos(phi) - sin(phi) * cos(psi)) * T / m;
x_ddot = Tx * T / m;
y_ddot = Ty * T / m;
z_ddot = -cos(theta) * cos(phi) * T / m + g;

% Calcola accelerazioni angolari (eq. dalla foto)
phi_ddot = tau_phi / Ix;
theta_ddot = tau_theta / Iy;
psi_ddot = tau_psi / Iz;

% Output: accelerazioni lineari e angolari
accelerations = [x_ddot, y_ddot, z_ddot, phi_ddot, theta_ddot, psi_ddot];
end