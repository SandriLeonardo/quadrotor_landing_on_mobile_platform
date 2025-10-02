function [pos, vel, acc] = only_linear_trajectory(t, hovering_position, state)
    % only_linear_trajectory - Generates a smooth linear trajectory
    %
    % Inputs:
    %   t - current simulation time
    %   hovering_position - current hover position [x,y,z]
    %   state - current state (1=takeoff, 2=follow, 3=landing)
    %
    % Outputs:
    %   pos - position [x,y,z]
    %   vel - velocity [x,y,z]
    %   acc - acceleration [x,y,z]
    
    persistent t_start trajectory_active pos_A_saved
    
    % Initialize persistent variables on first call
    if isempty(trajectory_active)
        trajectory_active = false;
        t_start = 0;
        pos_A_saved = hovering_position; % default starting point
    end
    
    trajectory_type = 'platform_tracking'; % Options: 'trajectory_tracking' , 'platform_tracking'
    
    switch trajectory_type
        case 'trajectory_tracking'
            % Trajectory parameters
            pos_B = [-4.6, -2.7, 5];   % Target position
            Time_tot = 60;             % Total trajectory time
            
            % Default outputs (stay at hover)
            pos = hovering_position(:)';
            vel = [0, 0, 0];
            acc = [0, 0, 0];
            
            % Activate trajectory when state becomes 2
            if state == 2 && ~trajectory_active
                t_start = t;
                trajectory_active = true;
                pos_A_saved = hovering_position(:)';  % Save starting point
                % Optional debug
                fprintf('Trajectory started at t=%.2f, start pos=[%.3f %.3f %.3f]\n', ...
                        t, pos_A_saved(1), pos_A_saved(2), pos_A_saved(3));
            end
            
            % Compute trajectory if active
            if trajectory_active
                % Local time, clamped to [0, Time_tot]
                t_local = t - t_start;
                t_local = max(0, min(t_local, Time_tot)); % Without it we would have overshoot
                
                % Linear interpolation factor
                alpha = t_local / Time_tot;
                
                % Compute position, velocity, acceleration
                pos = pos_A_saved + alpha * (pos_B - pos_A_saved);
                vel = (pos_B - pos_A_saved) / Time_tot;
                acc = [0, 0, 0];  % Constant velocity, no acceleration
            end
            
            % Ensure row vectors for Simulink
            pos = pos(:)';
            vel = vel(:)';
            acc = acc(:)';
            
        case 'platform_tracking'
            % Platform tracking parameters
            tracking_height = 2.0;  % Height above platform to track
            
            % Get platform trajectory at current time
            [platform_pos, platform_vel] = platform_trajectory(t);
            
            % Set desired position: follow platform's x-y, maintain fixed height
            pos = [platform_pos(1), platform_pos(2), tracking_height];
            
            % Set desired velocity: match platform's x-y velocity, zero z velocity
            vel = [platform_vel(1), platform_vel(2), 0];
            
            % Zero acceleration (velocity reference tracking)
            acc = [0, 0, 0];
            
            % Ensure row vectors for Simulink
            pos = pos(:)';
            vel = vel(:)';
            acc = acc(:)';
    end
end