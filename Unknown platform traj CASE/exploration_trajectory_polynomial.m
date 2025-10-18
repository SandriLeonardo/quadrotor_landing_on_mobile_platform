function [pos, vel, acc] = exploration_trajectory_polynomial(position, t)
% EXPLORATION_TRAJECTORY_POLYNOMIAL Generates exploration trajectory through waypoints
%
% INPUTS:
%   position  - Current quadrotor position [x, y, z] (1x3)
%   t         - Current time [s]
%
% OUTPUTS:
%   pos - Desired position [x, y, z] (1x3)
%   vel - Desired velocity [vx, vy, vz] (1x3)
%   acc - Desired acceleration [ax, ay, az] (1x3)
%
% TRAJECTORY: 5th-order polynomial through predefined waypoints
% Automatically manages waypoint sequence and restart logic

    persistent waypoint_index waypoint_start_time waypoint_start_pos
    persistent waypoints transition_time
    
    % Initialize on first call
    if isempty(waypoint_index)
        waypoint_index = 1;
        waypoint_start_time = -1;
        waypoint_start_pos = [0, 0, 0];
        transition_time = 3.0;  % 3 seconds per waypoint
        
        % Define exploration waypoints (x, y, z)
        waypoints = [
            0, 0, 3.0;      % Start position
            2, 0, 3.0;      % Move forward
            2, 2, 3.0;      % Move right
            0, 2, 3.0;      % Move back
            -2, 2, 3.0;     % Move left
            -2, 0, 3.0;     % Continue
            -2, -2, 3.0;    % More exploration
            0, -2, 3.0;     % Return path
            0, 0, 3.0;      % Back to center
        ];
    end
    
    % Get current target waypoint
    target_waypoint = waypoints(waypoint_index, :);
    
    % Initialize waypoint trajectory on first entry to this waypoint
    if waypoint_start_time < 0
        waypoint_start_time = t;
        waypoint_start_pos = position;
    end
    
    % Elapsed time
    tau = t - waypoint_start_time;
    
    % Check if trajectory to current waypoint is completed
    if tau >= transition_time
        % Move to next waypoint
        waypoint_index = waypoint_index + 1;
        
        % Check if all waypoints completed - restart sequence
        if waypoint_index > size(waypoints, 1)
            waypoint_index = 1;
        end
        
        % Reset for next waypoint
        waypoint_start_time = -1;
        
        % Return current waypoint position (will update next step)
        pos = target_waypoint;
        vel = [0, 0, 0];
        acc = [0, 0, 0];
        return;
    end
    
    % Normalized time [0, 1]
    s = tau / transition_time;
    
    % 5th-order polynomial coefficients (zero vel/acc at start and end)
    % p(s) = 10s^3 - 15s^4 + 6s^5
    % v(s) = (30s^2 - 60s^3 + 30s^4) / T
    % a(s) = (60s - 180s^2 + 120s^3) / T^2
    
    p_s = 10*s^3 - 15*s^4 + 6*s^5;
    v_s = (30*s^2 - 60*s^3 + 30*s^4) / transition_time;
    a_s = (60*s - 180*s^2 + 120*s^3) / (transition_time^2);
    
    % Apply to each axis
    delta = target_waypoint - waypoint_start_pos;
    
    pos = waypoint_start_pos + p_s * delta;
    vel = v_s * delta;
    acc = a_s * delta;
end