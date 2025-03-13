function [m,g,Ix,Iy,Iz,Kp_pos,Kd_pos,Ki_pos,Kp_ang,Kd_ang,Ki_ang,dt] = model_n_control_param()
%% Parametri del drone
m = 1.0;        % Massa [kg]
g = 9.81;       % Gravità [m/s²]
Ix = 0.1;      % Momento d'inerzia asse x [kg·m²]
Iy = 0.1;      % Momento d'inerzia asse y [kg·m²]
Iz = 0.2;      % Momento d'inerzia asse z [kg·m²]

%% Guadagni PID ridotti per maggiore stabilità
% Controllo Posizione (x, y, z)
Kp_pos = [0.001024,0.001024, 5];   % Proporzionale ridotto
Kd_pos = [0.23171, 0.23171, 0.1];    % Derivativo ridotto
Ki_pos = [0.006, 0.006, 0.0592879];  % Integrativo ridotto

% Controllo Angoli (φ, θ, ψ)
Kp_ang = [0.022841,0.022841,0.04568];
Kd_ang = [0.09868, 0.09868, 0.19736];
Ki_ang = [0.00132, 0.00132, 0.00264];

%% Time step
dt = 0.001;
end