function accelerations = dynamic_model(state, u, params)
% dynamic_model - Calcola le accelerazioni lineari e angolari del drone
% Input:
%   state: vettore dello stato [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
%   u: vettore di controllo [T, tau_phi, tau_theta, tau_psi]
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
phi = state(7);
theta = state(8);
psi = state(9);

% Estrai comandi di controllo
T = u(1);
tau_phi = u(2);
tau_theta = u(3);
tau_psi = u(4);

% Calcola accelerazioni lineari (eq. dalla foto)
x_ddot = -(cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi)) * T / m;
y_ddot = -(sin(psi) * sin(theta) * cos(phi) - sin(phi) * cos(psi)) * T / m;
z_ddot = -cos(theta) * cos(phi) * T / m + g;

% Calcola accelerazioni angolari (eq. dalla foto)
phi_ddot = tau_phi / Ix;
theta_ddot = tau_theta / Iy;
psi_ddot = tau_psi / Iz;

% Output: accelerazioni lineari e angolari
accelerations = [x_ddot, y_ddot, z_ddot, phi_ddot, theta_ddot, psi_ddot];
end