function [pos, vel, acc] = landing_trajectory(t, current_pos, state)
    % landing_trajectory - Follows platform trajectory while descending smoothly
    %
    % Inputs:
    %   t            - current simulation time
    %   current_pos  - starting position [x,y,z]
    %   landing_time - time duration (s) to complete landing
    %
    % Outputs:
    %   pos - position [x,y,z]
    %   vel - velocity [x,y,z]
    %   acc - acceleration [x,y,z]

    persistent t_start pos_start trajectory_active

    % ---- Default outputs (always defined for Simulink) ----
    pos = current_pos;   
    vel = [0,0,0];       
    acc = [0,0,0]; 

    % Initialize persistent variables on first call
    if isempty(trajectory_active)
        trajectory_active = false;
        t_start = 0;
        pos_start = current_pos; % default start position
    end


    % ---- Landing trigger ----
    if state == 3 && ~trajectory_active
        trajectory_active = true;
        t_start = t;
        pos_start = current_pos;
        fprintf('Landing trajectory started at t=%.2f from [%.2f %.2f %.2f]\n', ...
                 t, pos_start(1), pos_start(2), pos_start(3));
    end

    % Time for landing
    landing_time = 10;

    % Time since landing started
    if trajectory_active
        t_local = t - t_start;
        alpha = min(t_local / landing_time, 1.0);

        % Platform target at current time
        [platform_pos, platform_vel] = platform_trajectory(t);
        target_pos = [platform_pos(1), platform_pos(2), (platform_pos(3)+0.32)];  %% I ADDED 0.32 SINCE IT'S THE HEIGHT OF THE MOBILE PLATFORM ON COPPELIA

        % Smooth diagonal interpolation
        pos = (1 - alpha) * pos_start + alpha * target_pos;

        % Approx velocity
        vel_interp = (target_pos - pos_start) / landing_time;
        vel = vel_interp + [platform_vel(1), platform_vel(2), 0];

        % Simple zero acceleration
        acc = [0,0,0];
    end

    % Row vectors for Simulink
    pos = pos(:)';
    vel = vel(:)';
    acc = acc(:)';
end