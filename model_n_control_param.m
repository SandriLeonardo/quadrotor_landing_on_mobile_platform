function [m,g,Ix,Iy,Iz,Kp_pos,Kd_pos,Ki_pos,Kp_ang,Kd_ang,Ki_ang,dt] = model_n_control_param()
%% Parametri del drone
m = 1.0;        % Massa [kg]
g = 9.81;       % Gravità [m/s²]
Ix = 0.01;      % Momento d'inerzia asse x [kg·m²]
Iy = 0.01;      % Momento d'inerzia asse y [kg·m²]
Iz = 0.02;      % Momento d'inerzia asse z [kg·m²]

%% Guadagni PID
% Controllo Posizione (x, y, z)
Kp_pos = [1.5; 1.5; 10];   % Proporzionale
Kd_pos = [0.8; 0.8; 5];    % Derivativo
Ki_pos = [0.1; 0.1; 0.5];  % Integrativo

% Controllo Angoli (φ, θ, ψ)
Kp_ang = [10; 10; 10];     % Proporzionale
Kd_ang = [2; 2; 2];        % Derivativo
Ki_ang = [0.1; 0.1; 0.1];  % Integrativo

%% Time step
dt = 0.01;