function [m,g,Ix,Iy,Iz,Kp_pos,Kd_pos,Ki_pos,Kp_ang,Kd_ang,Ki_ang,dt] = model_n_control_param_tuned()
%m = 1.000000;        % Massa [kg]
g = 9.810000;       % Gravità [m/s²]
Ix = 0.100000;      % Momento d'inerzia asse x [kg·m²]
Iy = 0.100000;      % Momento d'inerzia asse y [kg·m²]
Iz = 0.200000;      % Momento d'inerzia asse z [kg·m²]

%% Controllo Posizione (x, y, z)
Kp_pos = [0.001024, 0.001024, 0.981223];
Kd_pos = [0.043171, 0.043171, 0.000000];
Ki_pos = [0.000006, 0.000006, 0.192879];

% Controllo Angoli (φ, θ, ψ)
Kp_ang = [0.022841, 0.022841, 0.045681];
Kd_ang = [0.098678, 0.098678, 0.197357];
Ki_ang = [0.001322, 0.001322, 0.002643];

%dt = 0.010000;
end