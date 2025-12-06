function enable_update = compute_kalman_update_flag(roll, pitch)
% COMPUTE_KALMAN_UPDATE_FLAG Determina se abilitare l'update del Kalman filter
%
% Inputs:
%   roll - angolo di roll in radianti
%   pitch - angolo di pitch in radianti
%
% Output:
%   enable_update - 1 se update abilitato, 0 altrimenti

persistent hysteresis_state

% Inizializza al primo utilizzo
if isempty(hysteresis_state)
    hysteresis_state = 1;  % Inizia abilitato
end

% Soglie in gradi
pitch_threshold = 10.0;
roll_threshold = 10.0;
hysteresis_margin = 2.0;

% Converti in gradi
roll_deg = abs(rad2deg(roll));
pitch_deg = abs(rad2deg(pitch));

% Logica con isteresi
if hysteresis_state == 1
    % Attualmente abilitato - disabilita se supera soglia + margine
    if (pitch_deg > pitch_threshold + hysteresis_margin) || ...
       (roll_deg > roll_threshold + hysteresis_margin)
        hysteresis_state = 0;
    end
else
    % Attualmente disabilitato - abilita se sotto soglia - margine
    if (pitch_deg < pitch_threshold - hysteresis_margin) && ...
       (roll_deg < roll_threshold - hysteresis_margin)
        hysteresis_state = 1;
    end
end

enable_update = hysteresis_state;

end
