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
    
    % Trajectory parameters
    pos_B = [-2.6, -0.7, 4];   % Target position
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
end
