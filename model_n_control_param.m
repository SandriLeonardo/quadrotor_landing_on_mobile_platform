function [m,g,Ix,Iy,Iz,Kp_pos,Kd_pos,Ki_pos,Kp_ang,Kd_ang,Ki_ang,dt] = model_n_control_param()
%% Parametri del drone
m = 1.0;        % Massa [kg]
g = 9.81;       % Gravità [m/s²]
Ix = 0.01;      % Momento d'inerzia asse x [kg·m²]
Iy = 0.01;      % Momento d'inerzia asse y [kg·m²]
Iz = 0.02;      % Momento d'inerzia asse z [kg·m²]

%% Guadagni PID
% Controllo Posizione (x, y, z)
Kp_pos = [2, 2, 15];   % Proporzionale
Kd_pos = [1.2, 1.2, 8];    % Derivativo
Ki_pos = [0.2, 0.2, 1];  % Integrativo

% Controllo Angoli (φ, θ, ψ)
Kp_ang = [8, 8, 5];     % Prima era [25; 25; 15]
Kd_ang = [4, 4, 3];      % Prima era [6; 6; 4]
Ki_ang = [0.1, 0.1, 0.1]; % Mantenuto basso

%% Time step
dt = 0.001;