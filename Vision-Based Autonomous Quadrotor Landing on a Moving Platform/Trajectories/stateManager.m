function [state, traj] = stateManager(position, platform_position, detected, t)

    % Persistent variables (maintain values between time steps)
    persistent current_state hover_start_time last_t position_error_start_time detection_start_time state3_start_time state3_start_pos 

    
    % Initialize on first run
    if isempty(current_state)
        current_state = 1;  % Start in takeoff state
        hover_start_time = -1;
        last_t = 0;
        position_error_start_time = -1;
        detection_start_time = -1;
        
        state3_start_time = -1;
        state3_start_pos = [0,0,0];
    end
    
    % Calculate dt automatically
    dt = t - last_t;
    last_t = t;
    
    % Parameters
    altitude_threshold = 0.1;
    hover_time_required = 10.0;
    position_error_threshold = 0.8;
    position_error_time_required = 15.0;
    landing_altitude = 0.05;
    target_hover_height = 2.5;  

    % Ensure position is a row vector for consistent math
    position = reshape(position, 1, 3);
    altitude = position(3);
    position_error = norm(position(1:2) - platform_position(1:2));
    
    % Initialize outputs
    pos = zeros(1,3);
    vel = zeros(1,3);
    acc = zeros(1,3);
    traj = zeros(1,9);

    % Random walk traj
    center = [0, 0];
    radius = 6.0;
    angular_velocity = 0.2;  % rad/s 

    angle = angular_velocity * t;
            
    target_x = center(1) + radius * cos(angle);
    target_y = center(2) + radius * sin(angle);
    
    target_vx = -radius * angular_velocity * sin(angle);
    target_vy = radius * angular_velocity * cos(angle);
    
    target_pos = [target_x, target_y, target_hover_height];
    target_vel = [target_vx, target_vy, 0];
    target_acc = [-radius * angular_velocity^2 * cos(angle), ...
                  -radius * angular_velocity^2 * sin(angle), 0];

    % State machine logic
    switch current_state
        %% First case: Takeoff and hovering
        case 1  % TAKEOFF STATE
            % Hovering takeoff trajectory logic
            v_const = 0.75;   % Ascent speed [m/s]
            expected_z = v_const * t;
            
            if expected_z < target_hover_height
                % Rising phase
                pos = [0, 0, expected_z];
                vel = [0, 0, v_const];
                acc = [0, 0, 0];
                traj = [pos,vel,acc];
            else
                % Hovering phase
                pos = [0, 0, target_hover_height];
                vel = [0, 0, 0];
                acc = [0, 0, 0];
                traj = [pos,vel,acc];
            end
            
            if altitude >= (target_hover_height - altitude_threshold)
            
                %--- Hover start timer ---
                if hover_start_time < 0
                    hover_start_time = t;
                end
            
                %--- Detection timer ---
                if detected
                    if detection_start_time < 0
                        detection_start_time = t;   % Start counting
                    end
                else
                    detection_start_time = -1;      % Reset if detection lost
                end
            
                time_at_hover = t - hover_start_time;
                time_detected = (detection_start_time > 0) * (t - detection_start_time);
            
                %--- Transition to state 2 if detection ≥ 3 seconds ---
                if time_detected >= 3.0
                    current_state = 2;
                    fprintf('Switched to STATE 2 at time %f \n',t);
                    hover_start_time = -1;
                    detection_start_time = -1;
                end
            
                %--- Transition to state 3 if hover ≥ 10 seconds ---
                if time_at_hover >= hover_time_required
                    current_state = 3;
                    hover_start_time = -1;
                    detection_start_time = -1;
                    
                    % Reset State 3 smoothing timers so they start fresh
                    state3_start_time = -1; 
                    fprintf('Switched to STATE 3 at time %f \n',t);
                end   
            
            else
                % If we're not stable at hover height, reset timers
                hover_start_time = -1;
                detection_start_time = -1;
            end   

            

         %% SECOND CASE: FOLLOWING
        case 2  % FOLLOW STATE
            % Reset State 3 timer just in case we came from there
            state3_start_time = -1;

            % Add your follow trajectory here
            if mod(t,2) == 0
                fprintf('Position tracking error: %g\n', position_error);
            end
            if position_error < position_error_threshold
                if position_error_start_time < 0
                    position_error_start_time = t;
                end
                
                time_at_low_error = t - position_error_start_time;
                if mod(t,1) == 0
                    fprintf('Time inside the bound:%g\n',time_at_low_error);
                end
                
                if time_at_low_error >= position_error_time_required
                    current_state = 4;
                    fprintf('Switched to STATE 4: Landing!\n');
                    position_error_start_time = -1;
                end
            else
                position_error_start_time = -1;
            end

            
        %% THIRD CASE: LOOKING FOR THE PLATFORM (RANDOM WALK)    
        case 3  % Go on a circular trajectory while trying to find the moving platform

            if state3_start_time < 0
                state3_start_time = t;
                state3_start_pos  = position; % Snapshot current actual position (e.g., [0,0,2.5])
            end
            
            % 3. Apply Smoothing Logic (First 3 seconds)
            T_trans = 5.0; % Transition duration
            t_local = t - state3_start_time;

            if t_local < T_trans
                angular_velocity =0.05;
                % Normalized time x in [0, 1]
                x = t_local / T_trans;
                
                % Polynomial weights
                alpha_tr   = 3*x^2 - 2*x^3;
                dalpha_tr  = (6*x - 6*x^2) / T_trans;
                ddalpha_tr = (6 - 12*x) / T_trans^2;
            
                diff_PA = (target_pos - state3_start_pos);
                
                % Blended outputs
                pos = state3_start_pos + alpha_tr * diff_PA;
                vel = dalpha_tr * diff_PA + alpha_tr * target_vel;
                acc = ddalpha_tr * diff_PA + dalpha_tr * target_vel + alpha_tr * target_acc;
            else
                % Transition complete: track circle directly
                pos = target_pos;
                vel = target_vel;
                acc = target_acc;
            end

            traj = [pos, vel, acc];

            % --- Detection Logic ---
            if detected
                if detection_start_time < 0
                    detection_start_time = t;   % Start counting
                end
            else
                detection_start_time = -1;      % Reset if detection lost
            end

            time_detected = (detection_start_time > 0) * (t - detection_start_time);
            
            % if time_detected > 0
            %     angular_velocity = 0.2/exp(time_detected);
            % else
            %     angular_velocity = 0.2;
            % end

            if  time_detected >= 2.0
                current_state = 2;
                fprintf('Finally found, starting Tracking')
                detection_start_time = -1;
                state3_start_time = -1; 
            end

            
        %% LANDING
        case 4 % Landing
            fprintf('LANDING');
            if altitude < landing_altitude
            % Stay in landing state
            end
            
    end
    
    % Outputs
    state = current_state;
end