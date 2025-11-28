function [pos, vel, acc] = hovering_takeoff(t)
    % Outputs must always be 1x3 row vectors
    pos = zeros(1,3);
    vel = zeros(1,3);
    acc = zeros(1,3);

    target_height = 3; % Va cambiato anche da stateManager (TODO)
    v_const = 0.25;   % Ascent speed [m/s]
    
    % Calculate expected position based on time
    expected_z = v_const * t;
    
    if expected_z < target_height
        % Rising phase - time-based position for smooth motion
        pos = [0, 0, expected_z];
        vel = [0, 0, v_const];
        acc = [0, 0, 0];
    else
        % Hovering phase - maintain target height
        pos = [0, 0, target_height];
        vel = [0, 0, 0];
        acc = [0, 0, 0];
    end
end