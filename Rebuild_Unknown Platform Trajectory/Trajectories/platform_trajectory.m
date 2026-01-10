function [platform_pos, platform_vel] = platform_trajectory(t)
    % platform_trajectory - Generates a 2D trajectory for the moving platform
    %
    % Inputs:
    %   t - current simulation time
    %
    % Outputs:
    %   platform_pos - platform position [x, y, z]
    %   platform_vel - platform velocity [vx, vy, vz]
    
    persistent waypoints_cache;

    % Platform parameters
    platform_height = 0;  % On the ground
    
    % Trajectory type selection
    trajectory_type = 'circle';  % Options: 'circle', 'linear', 'figure8', 'stationary', 'linear_with_ending', 'spline'
    
    switch trajectory_type
        case 'stationary'
            % Stationary platform
            platform_pos = [0, 0, platform_height];
            platform_vel = [0, 0, 0];
            
        case 'linear'
            % Linear trajectory (infinite)
            start_pos = [0, 0];   % Initial position
            speed = 0.1;          % m/s
            direction = [-5, -3]; % Movement direction vector
        
            % Normalize direction
            direction = direction / norm(direction);
        
            % Infinite motion along direction
            xy_pos = start_pos + direction * speed * t;
            xy_vel = direction * speed;
        
            platform_pos = [xy_pos, platform_height];
            platform_vel = [xy_vel, 0];

            
        case 'linear_with_ending'
            % Linear trajectory with end point
            start_pos = [0, 0]; % Initial position
            end_pos = [-5, -3]; % End position
            speed = 0.1; % m/s
            
            % Calculate direction and total distance
            direction = end_pos - start_pos;
            total_distance = norm(direction);
            direction = direction / total_distance; % Normalize
            
            % Calculate time to reach end point
            t_end = total_distance / speed;
            
            % Clamp position to end point after t_end
            if t <= t_end
                % Moving phase
                xy_pos = start_pos + direction * speed * t;
                xy_vel = direction * speed;
            else
                % Stopped at end point
                xy_pos = end_pos;
                xy_vel = [0, 0];
            end
            
            platform_pos = [xy_pos, platform_height];
            platform_vel = [xy_vel, 0];

        case 'circle'
            % Circular trajectory
            center = [4, 2]; %[4 2] to see random walk
            radius = 4.0;
            angular_velocity = 0.05;  % rad/s (one revolution in ~60 seconds)
            
            angle = angular_velocity * t;
            
            x = center(1) + radius * cos(angle);
            y = center(2) + radius * sin(angle);
            
            vx = -radius * angular_velocity * sin(angle);
            vy = radius * angular_velocity * cos(angle);
            
            platform_pos = [x, y, platform_height];
            platform_vel = [vx, vy, 0];
            
        case 'figure8'
            % Figure-8 trajectory (Lemniscate)
            center = [-2, -1];
            scale = 2.0;
            angular_velocity = 0.12;  % rad/s
            
            angle = angular_velocity * t;
            
            % Lemniscate parametric equations
            sin_t = sin(angle);
            cos_t = cos(angle);
            denom = 1 + sin_t^2;
            
            x = center(1) + scale * cos_t / denom;
            y = center(2) + scale * sin_t * cos_t / denom;
            
            % Velocities (derivatives)
            denom_sq = denom^2;
            vx = scale * angular_velocity * (-sin_t * denom - cos_t * 2 * sin_t * cos_t) / denom_sq;
            vy = scale * angular_velocity * ((cos_t^2 - sin_t^2) * denom - sin_t * cos_t * 2 * sin_t * cos_t) / denom_sq;
            
            platform_pos = [x, y, platform_height];
            platform_vel = [vx, vy, 0];

        case 'spline'
            % Hardcoded waypoints (modify these as needed)
            t_total = 200;
            if isempty(waypoints_cache)
                waypoints_cache = [
                    -1.5,  0.8, 0;
                     0.2, -1.2, 0;
                     1.8,  0.5, 0;
                     0.5,  1.6, 0;
                    -0.8, -0.5, 0;
                     1.2,  0.0, 0
                ];
            end
            
            % Create smooth trajectory
            [pos, vel, ~] = generate_spline_trajectory(3*waypoints_cache, t, t_total);

            platform_pos = pos;
            platform_vel = vel;
            
        otherwise
            % Default to stationary
            platform_pos = [0, 0, platform_height];
            platform_vel = [0, 0, 0];
    end
    
    % Ensure row vectors
    platform_pos = platform_pos(:)';
    platform_vel = platform_vel(:)';
end