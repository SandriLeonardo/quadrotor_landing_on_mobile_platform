% function [pos, vel, acc] = only_linear_trajectory(t)
%     % Parametri del profilo bang-coast-bang
%     pos_A = [0, 0, 0];      % Posizione iniziale [x, y, z]
%     pos_B = [2, 2, ];      % Posizione finale
%     T_tot = 10;             % Tempo totale del movimento
%     a_max = 1;              % Accelerazione massima (m/s²)
% 
%     % Calcola la distanza totale
%     delta_pos = pos_B - pos_A;
%     distanza_totale = norm(delta_pos); % Distanza euclidea
% 
%     % Calcola la durata delle fasi di accelerazione/decelerazione (T_acc)
%     % Equazione del moto: distanza = area trapezio velocità
%     % Soluzione per T_acc: T_acc = (a_max * T_tot - sqrt(a_max^2 * T_tot^2 - 2 * a_max * distanza)) / a_max
%     T_acc = T_tot/2 - sqrt((T_tot/2)^2 - distanza_totale/a_max);
% 
%     % Verifica che T_acc sia valido (se no, usa accelerazione costante)
%     if imag(T_acc) ~= 0 || T_acc > T_tot/2
%         error('Impossibile raggiungere la posizione finale con i parametri dati. Aumenta a_max o T_tot.');
%     end
% 
%     % Velocità massima durante la fase "coast"
%     v_max = a_max * T_acc;
% 
%     % Calcola posizione, velocità e accelerazione in base al tempo t
%     if t < 0
%         % Tempo negativo: posizione iniziale
%         pos = pos_A;
%         vel = [0, 0, 0];
%         acc = [0, 0, 0];
% 
%     elseif t <= T_acc
%         % Fase 1: Accelerazione (bang)
%         acc = (delta_pos / distanza_totale) * a_max; % Vettore accelerazione direzione pos_B
%         vel = acc * t;
%         pos = pos_A + 0.5 * acc * t^2;
% 
%     elseif t <= T_tot - T_acc
%         % Fase 2: Velocità costante (coast)
%         acc = [0, 0, 0];
%         vel = (delta_pos / distanza_totale) * v_max; % Velocità costante
%         pos = pos_A + 0.5 * acc * T_acc^2 + vel * (t - T_acc);
% 
%     elseif t <= T_tot
%         % Fase 3: Decelerazione (bang)
%         acc = -(delta_pos / distanza_totale) * a_max; % Accelerazione opposta
%         vel = (delta_pos / distanza_totale) * v_max + acc * (t - (T_tot - T_acc));
%         pos = pos_B - 0.5 * (-acc) * (T_tot - t)^2;
% 
%     else
%         % Tempo superiore a T_tot: mantieni posizione finale
%         pos = pos_B;
%         vel = [0, 0, 0];
%         acc = [0, 0, 0];
%     end
% 
%     % Formato Simulink-friendly (vettori riga)
%     pos = pos(:)'; 
%     vel = vel(:)';
%     acc = acc(:)';
% end

function [pos, vel, acc] = only_linear_trajectory(t)
    % Parametri della traiettoria
    pos_A = [0, 0, 0]; % Posizione iniziale [x, y, z]
    pos_B = [3, 5, 8]; % Posizione finale [x, y, z]
    Time_tot = 10; % Tempo totale del movimento
    
    % Calcola la distanza totale
    delta_pos = pos_B - pos_A;
    distanza_totale = norm(delta_pos); % Distanza euclidea 3D
    
    % Calcola la velocità costante necessaria
    v_const = distanza_totale / Time_tot;
    vel_vector = (delta_pos / distanza_totale) * v_const; % Vettore velocità direzione pos_B
    
    % Calcola posizione, velocità e accelerazione in base al tempo t
    if t < 0
        % Tempo negativo: posizione iniziale
        pos = pos_A;
        vel = [0, 0, 0];
        acc = [0, 0, 0];
    elseif t <= Time_tot
        % Movimento a velocità costante
        pos = pos_A + vel_vector * t;
        vel = vel_vector;
        acc = [0, 0, 0];
    else
        % Tempo superiore a T_tot: mantieni posizione finale
        pos = pos_B;
        vel = [0, 0, 0];
        acc = [0, 0, 0];
    end
    
    % Formato Simulink-friendly (vettori riga)
    pos = pos(:)';
    vel = vel(:)';
    acc = acc(:)';
end