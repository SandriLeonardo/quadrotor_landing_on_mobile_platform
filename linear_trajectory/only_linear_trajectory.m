function [pos, vel, acc] = only_linear_trajectory(t)
    % Definizione parametri direttamente nella funzione
    pos_A = [0, 0, 0];    % Posizione iniziale [x, y, z]
    pos_B = [5, 3, 2];     % Posizione finale
    T_tot = 10;            % Tempo totale del movimento
    
    % Calcolo della traiettoria
    if t <= T_tot
        pos = pos_A + (pos_B - pos_A) * (t / T_tot); % Posizione
        vel = (pos_B - pos_A) / T_tot;               % Velocità costante
    else
        pos = pos_B;        % Mantieni posizione finale
        vel = [0, 0, 0];    % Velocità zero
    end
    acc = [0, 0, 0];        % Accelerazione zero
    
    % Output come array singoli (Simulink-friendly)
    pos = pos(:)'; % Formato [x, y, z] come vettore riga
    vel = vel(:)';
    acc = acc(:)';
end