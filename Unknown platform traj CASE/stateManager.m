function [state, takeoff_traj] = stateManager(position, platform_position, t)
    
    % Persistent variables (maintain values between time steps)
    persistent current_state hover_start_time last_t position_error_start_time
    
    % Initialize on first run
    if isempty(current_state)
        current_state = 1;  % Start in takeoff state
        hover_start_time = -1;
        last_t = 0;
        position_error_start_time = -1;
    end
    
    % Calculate dt automatically
    dt = t - last_t;
    last_t = t;
    
    % Parameters
    altitude_threshold = 0.1;
    hover_time_required = 10.0;
    position_error_threshold = 0.2;
    position_error_time_required = 15.0;
    landing_altitude = 0.05;
    target_hover_height = 3.0;  

    altitude = position(3);
    position_error = norm(position(1:2) - platform_position(1:2));
    
    % Initialize outputs
    pos = zeros(1,3);
    vel = zeros(1,3);
    acc = zeros(1,3);
    takeoff_traj = zeros(1,9);
    
    % State machine logic
    switch current_state
        case 1  % TAKEOFF STATE
            % Hovering takeoff trajectory logic
            v_const = 0.25;   % Ascent speed [m/s]
            expected_z = v_const * t;
            
            if expected_z < target_hover_height
                % Rising phase
                pos = [0, 0, expected_z];
                vel = [0, 0, v_const];
                acc = [0, 0, 0];
                takeoff_traj = [pos,vel,acc];
            else
                % Hovering phase
                pos = [0, 0, target_hover_height];
                vel = [0, 0, 0];
                acc = [0, 0, 0];
                takeoff_traj = [pos,vel,acc];
            end
            
            % Check if we've reached hover height
            if altitude >= (target_hover_height - altitude_threshold)
                if hover_start_time < 0
                    hover_start_time = t;
                end
                
                time_at_hover = t - hover_start_time;
                
                if time_at_hover >= hover_time_required
                    current_state = 2;
                    hover_start_time = -1;
                end
            else
                if altitude < (target_hover_height - 2*altitude_threshold)
                    hover_start_time = -1;
                end
            end
            
        case 2  % FOLLOW STATE
            % Add your follow trajectory here
            
            if position_error < position_error_threshold
                if position_error_start_time < 0
                    position_error_start_time = t;
                end
                
                time_at_low_error = t - position_error_start_time;
                
                if time_at_low_error >= position_error_time_required
                    current_state = 3;
                    position_error_start_time = -1;
                end
            else
                position_error_start_time = -1;
            end
            
        case 3  % LANDING STATE
            % Add your landing trajectory here
            
            if altitude < landing_altitude
                % Stay in landing state
            end
    end
    
    % Outputs
    state = current_state;
end