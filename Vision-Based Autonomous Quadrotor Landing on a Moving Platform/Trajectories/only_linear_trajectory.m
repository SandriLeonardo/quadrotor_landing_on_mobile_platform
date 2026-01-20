function [pos, vel, acc] = only_linear_trajectory(t, hovering_position, state)
    % only_linear_trajectory - Generates a smooth linear trajectory with transition
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
    
    persistent t_start trajectory_active pos_A_saved transition_complete
    
    % Initialize persistent variables on first call
    if isempty(trajectory_active)
        trajectory_active = false;
        transition_complete = false;
        t_start = 0;
        pos_A_saved = hovering_position;
    end
    
    % Transition parameters
    transition_time = 5.0;  % Time to smoothly transition from hover to platform tracking
    
    trajectory_type = 'platform_tracking';
    
    switch trajectory_type
        case 'trajectory_tracking'
            % Trajectory parameters
            pos_B = [-4.6, -2.7, 5];
            Time_tot = 60;
            
            % Default outputs (stay at hover)
            pos = hovering_position(:)';
            vel = [0, 0, 0];
            acc = [0, 0, 0];
            
            % Activate trajectory when state becomes 2
            if state == 2 && ~trajectory_active
                t_start = t;
                trajectory_active = true;
                transition_complete = false;
                pos_A_saved = hovering_position(:)';
                fprintf('Trajectory started at t=%.2f, start pos=[%.3f %.3f %.3f]\n', ...
                        t, pos_A_saved(1), pos_A_saved(2), pos_A_saved(3));
            end
            
            % Compute trajectory if active
            if trajectory_active
                t_local = t - t_start;
                t_local = max(0, min(t_local, Time_tot));
                
                % Smooth transition phase
                if t_local < transition_time
                    % Sigmoid-based smooth interpolation (S-curve)
                    alpha = t_local / transition_time;
                    % Smooth acceleration/deceleration using sigmoid
                    alpha_smooth = 3*alpha^2 - 2*alpha^3;  % Smoothstep function
                    
                    pos = pos_A_saved + alpha_smooth * (pos_B - pos_A_saved);
                    
                    % Velocity during transition (derivative of smoothstep)
                    dalpha_dt = 1 / transition_time;
                    dalpha_smooth = (6*alpha - 6*alpha^2) * dalpha_dt;
                    vel = (pos_B - pos_A_saved) * dalpha_smooth;
                    
                    acc = [0, 0, 0];  % Approximate
                else
                    % Normal linear trajectory after transition
                    alpha = t_local / Time_tot;
                    pos = pos_A_saved + alpha * (pos_B - pos_A_saved);
                    vel = (pos_B - pos_A_saved) / Time_tot;
                    acc = [0, 0, 0];
                end
            end
            
            pos = pos(:)';
            vel = vel(:)';
            acc = acc(:)';
            
        case 'platform_tracking'
            % Platform tracking with smooth transition
            tracking_height = 2.0;
            
            % Get platform trajectory at current time
            [platform_pos, platform_vel] = platform_trajectory(t);
            
            % Target position and velocity
            target_pos = [platform_pos(1), platform_pos(2), tracking_height];
            target_vel = [platform_vel(1), platform_vel(2), 0];
            
            % Activate trajectory when state becomes 2
            if state == 2 && ~trajectory_active
                t_start = t;
                trajectory_active = true;
                transition_complete = false;
                pos_A_saved = hovering_position(:)';
                fprintf('Platform tracking started at t=%.2f from [%.3f %.3f %.3f]\n', ...
                        t, pos_A_saved(1), pos_A_saved(2), pos_A_saved(3));
            end
            
            % Compute trajectory
            if trajectory_active && ~transition_complete
                t_local = t - t_start;
                
                if t_local < transition_time
                    % Smooth transition using smoothstep function
                    alpha = t_local / transition_time;
                    alpha_smooth = 3*alpha^2 - 2*alpha^3;  % C1 continuous
                    
                    % Interpolate position
                    pos = (1 - alpha_smooth) * pos_A_saved + alpha_smooth * target_pos;
                    
                    % Interpolate velocity (derivative of position)
                    dalpha_dt = 1 / transition_time;
                    dalpha_smooth = (6*alpha - 6*alpha^2) * dalpha_dt;
                    
                    % Velocity is blend of transition motion and platform velocity
                    vel_transition = (target_pos - pos_A_saved) * dalpha_smooth;
                    vel = (1 - alpha_smooth) * [0, 0, 0] + alpha_smooth * target_vel + vel_transition;
                    
                    acc = [0, 0, 0];
                else
                    % Transition complete, now fully track platform
                    transition_complete = true;
                    pos = target_pos;
                    vel = target_vel;
                    acc = [0, 0, 0];
                end
            elseif trajectory_active && transition_complete
                % Pure platform tracking after transition
                pos = target_pos;
                vel = target_vel;
                acc = [0, 0, 0];
            else
                % Before activation, stay at hover
                pos = hovering_position(:)';
                vel = [0, 0, 0];
                acc = [0, 0, 0];
            end
            
            pos = pos(:)';
            vel = vel(:)';
            acc = acc(:)';
    end
end