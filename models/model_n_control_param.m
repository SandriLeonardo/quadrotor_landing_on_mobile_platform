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

%% Traiettoria desiderata (esempio: cerchio nel piano xy)
t_sim = 10;         % Tempo di simulazione [s]
r = 2.0;            % Raggio [m]
omega = 1.0;        % Velocità angolare [rad/s]
z_ref = 1.0;        % Altezza costante [m]

% Genera segnali di riferimento
time = 0:0.01:t_sim;
x_ref = r*cos(omega*time);
y_ref = r*sin(omega*time);
z_ref = z_ref * ones(size(time));