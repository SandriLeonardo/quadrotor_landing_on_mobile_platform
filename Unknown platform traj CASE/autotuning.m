% Script per l'autotuning dei controllori PID del quadricottero

%% Configurazione iniziale
clear;
clc;

% Carica parametri iniziali
[m,g,Ix,Iy,Iz,Kp_pos,Kd_pos,Ki_pos,Kp_ang,Kd_ang,Ki_ang,dt] = model_n_control_param();
params = [m,g,Ix,Iy,Iz,Kp_pos,Kd_pos,Ki_pos,Kp_ang,Kd_ang,Ki_ang,dt];

%% PARTE 1: TUNING DEL CONTROLLORE DI ASSETTO

% Crea modelli di trasferimento per i tre assi di rotazione
s = tf('s');
G_phi = 1/(Ix*s^2);    % Funzione di trasferimento per roll (phi)
G_theta = 1/(Iy*s^2);  % Funzione di trasferimento per pitch (theta)
G_psi = 1/(Iz*s^2);    % Funzione di trasferimento per yaw (psi)

% Definisci requisiti di design
tuning_options = pidtuneOptions('DesignFocus', 'balanced', 'PhaseMargin', 60);

% Tune controllore roll (phi)
C_phi = pidtune(G_phi, 'PID', tuning_options);
fprintf('PID Phi Tuned: Kp=%f, Ki=%f, Kd=%f\n', C_phi.Kp, C_phi.Ki, C_phi.Kd);

% Tune controllore pitch (theta)
C_theta = pidtune(G_theta, 'PID', tuning_options);
fprintf('PID Theta Tuned: Kp=%f, Ki=%f, Kd=%f\n', C_theta.Kp, C_theta.Ki, C_theta.Kd);

% Tune controllore yaw (psi)
C_psi = pidtune(G_psi, 'PID', tuning_options);
fprintf('PID Psi Tuned: Kp=%f, Ki=%f, Kd=%f\n', C_psi.Kp, C_psi.Ki, C_psi.Kd);

% Aggiorna i parametri del controllore di attitude
Kp_ang_tuned = [C_phi.Kp, C_theta.Kp, C_psi.Kp];
Ki_ang_tuned = [C_phi.Ki, C_theta.Ki, C_psi.Ki];
Kd_ang_tuned = [C_phi.Kd, C_theta.Kd, C_psi.Kd];

%% PARTE 2: TUNING DEL CONTROLLORE DI POSIZIONE
% Nota: Per il controllore di posizione, dobbiamo considerare la dinamica 
% in loop chiuso del controllore di assetto

% Crea i sistemi in anello chiuso per gli angoli
CL_phi = feedback(C_phi*G_phi, 1);
CL_theta = feedback(C_theta*G_theta, 1);
CL_psi = feedback(C_psi*G_psi, 1);

% Approssimazione del sistema di posizione 
% (assumendo piccoli angoli e linearizzazione)
G_x = tf(g, [1, 0, 0]); % Approssimazione per x (via theta)
G_y = tf(g, [1, 0, 0]); % Approssimazione per y (via phi)
G_z = tf(1/m, [1, 0]);  % Dinamica verticale

% Tune PID X (dipende da theta)
C_x = pidtune(G_x*CL_theta, 'PID', tuning_options);
fprintf('PID X Tuned: Kp=%f, Ki=%f, Kd=%f\n', C_x.Kp, C_x.Ki, C_x.Kd);

% Tune PID Y (dipende da phi)
C_y = pidtune(G_y*CL_phi, 'PID', tuning_options);
fprintf('PID Y Tuned: Kp=%f, Ki=%f, Kd=%f\n', C_y.Kp, C_y.Ki, C_y.Kd);

% Tune PID Z
C_z = pidtune(G_z, 'PID', tuning_options);
fprintf('PID Z Tuned: Kp=%f, Ki=%f, Kd=%f\n', C_z.Kp, C_z.Ki, C_z.Kd);

% Aggiorna i parametri del controllore di posizione
Kp_pos_tuned = [C_x.Kp, C_y.Kp, C_z.Kp];
Ki_pos_tuned = [C_x.Ki, C_y.Ki, C_z.Ki];
Kd_pos_tuned = [C_x.Kd, C_y.Kd, C_z.Kd];

%% Visualizza i risultati
figure(1);
subplot(3,1,1);
step(feedback(C_phi*G_phi, 1));
title('Roll (Phi) Step Response');
grid on;

subplot(3,1,2);
step(feedback(C_theta*G_theta, 1));
title('Pitch (Theta) Step Response');
grid on;

subplot(3,1,3);
step(feedback(C_psi*G_psi, 1));
title('Yaw (Psi) Step Response');
grid on;

figure(2);
subplot(3,1,1);
step(feedback(C_x*G_x, 1));
title('X Position Step Response');
grid on;

subplot(3,1,2);
step(feedback(C_y*G_y, 1));
title('Y Position Step Response');
grid on;

subplot(3,1,3);
step(feedback(C_z*G_z, 1));
title('Z Position Step Response');
grid on;

%% Salva i nuovi parametri in un file
% Confronto tra parametri originali e parametri tuned
fprintf('\n--- CONFRONTO PARAMETRI ---\n');
fprintf('Parametri originali posizione (Kp): [%f, %f, %f]\n', Kp_pos);
fprintf('Parametri tuned posizione (Kp): [%f, %f, %f]\n', Kp_pos_tuned);
fprintf('Parametri originali attitude (Kp): [%f, %f, %f]\n', Kp_ang);
fprintf('Parametri tuned attitude (Kp): [%f, %f, %f]\n', Kp_ang_tuned);

% Genera un nuovo file model_n_control_param_tuned.m
fid = fopen('model_n_control_param_tuned.m', 'w');
fprintf(fid, 'function [m,g,Ix,Iy,Iz,Kp_pos,Kd_pos,Ki_pos,Kp_ang,Kd_ang,Ki_ang,dt] = model_n_control_param_tuned()\n');
fprintf(fid, '%%% Parametri del drone\n');
fprintf(fid, 'm = %f;        %% Massa [kg]\n', m);
fprintf(fid, 'g = %f;       %% Gravità [m/s²]\n', g);
fprintf(fid, 'Ix = %f;      %% Momento d''inerzia asse x [kg·m²]\n', Ix);
fprintf(fid, 'Iy = %f;      %% Momento d''inerzia asse y [kg·m²]\n', Iy);
fprintf(fid, 'Iz = %f;      %% Momento d''inerzia asse z [kg·m²]\n', Iz);
fprintf(fid, '\n%%% Guadagni PID (autotuned)\n');
fprintf(fid, '%% Controllo Posizione (x, y, z)\n');
fprintf(fid, 'Kp_pos = [%f, %f, %f];\n', Kp_pos_tuned);
fprintf(fid, 'Kd_pos = [%f, %f, %f];\n', Kd_pos_tuned);
fprintf(fid, 'Ki_pos = [%f, %f, %f];\n', Ki_pos_tuned);
fprintf(fid, '\n%% Controllo Angoli (φ, θ, ψ)\n');
fprintf(fid, 'Kp_ang = [%f, %f, %f];\n', Kp_ang_tuned);
fprintf(fid, 'Kd_ang = [%f, %f, %f];\n', Kd_ang_tuned);
fprintf(fid, 'Ki_ang = [%f, %f, %f];\n', Ki_ang_tuned);
fprintf(fid, '\n%%% Time step\n');
fprintf(fid, 'dt = %f;\n', dt);
fprintf(fid, 'end');
fclose(fid);

fprintf('\nParametri salvati nel file model_n_control_param_tuned.m\n');