function [m,g,Ix,Iy,Iz,Kp_pos,Kd_pos,Ki_pos,Kp_ang,Kd_ang,Ki_ang,dt] = model_n_control_param()
%% Parametri del drone
m = 1.0;        % Massa [kg]
g = 9.81;       % Gravità [m/s²]
Ix = 0.1;       % Momento d'inerzia asse x [kg·m²]
Iy = 0.1;       % Momento d'inerzia asse y [kg·m²]
Iz = 0.2;       % Momento d'inerzia asse z [kg·m²]

%% GUADAGNI PID
%Controllo Posizione (x, y, z)
Kp_pos = [2.4,2.4,15];   
Kd_pos = [1,1,1];    
Ki_pos = [0.01,0.01,6];

% Controllo Angoli (φ, θ, ψ)
Kp_ang = [1,1,1];
Kd_ang = [0.2,0.2,1];
Ki_ang = [0.01,0.01,0.05];


%% Time step (Overwritten by Simulink Sample_Time)
dt = 0.0001;
end