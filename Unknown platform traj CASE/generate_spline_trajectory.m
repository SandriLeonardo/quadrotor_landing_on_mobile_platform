function [pos, vel, acc] = generate_spline_trajectory(waypoints, t, t_total)
    % Generates smooth trajectory through waypoints using cubic splines
    %
    % Inputs:
    %   waypoints - [N x 3] array of positions to pass through
    %   t - time vector or scalar
    %   t_total - total duration of trajectory
    %
    % Outputs:
    %   pos - [length(t) x 3] positions
    %   vel - [length(t) x 3] velocities
    %   acc - [length(t) x 3] accelerations
    
    % Ensure t is a column vector
    t = t(:);
    N = size(waypoints, 1);
    
    % Handle edge case: single waypoint (stationary)
    if N == 1
        n_samples = length(t);
        pos = repmat(waypoints, n_samples, 1);
        vel = zeros(n_samples, 3);
        acc = zeros(n_samples, 3);
        return;
    end
    
    % Distribute time based on distances between waypoints
    distances = vecnorm(diff(waypoints, 1, 1), 2, 2);
    total_distance = sum(distances);
    
    % Time allocation proportional to distance
    if total_distance > 0
        segment_durations = distances / total_distance * t_total;
        segment_times = [0; cumsum(segment_durations)];
    else
        % All waypoints at same location (edge case)
        segment_times = linspace(0, t_total, N)';
    end
    
    % Create cubic splines for each dimension
    spline_x = spline(segment_times, waypoints(:, 1));
    spline_y = spline(segment_times, waypoints(:, 2));
    spline_z = spline(segment_times, waypoints(:, 3));
    
    % Clamp time to valid range [0, t_total]
    t_clamped = min(max(t, 0), t_total);
    
    % Evaluate position
    pos = [ppval(spline_x, t_clamped), ...
           ppval(spline_y, t_clamped), ...
           ppval(spline_z, t_clamped)];
    
    % Evaluate velocity (1st derivative)
    n_samples = length(t);
    pos = zeros(n_samples, 3);
    vel = zeros(n_samples, 3);
    acc = zeros(n_samples, 3);
    
    % Compute for each dimension
    for dim = 1:3
        pp = spline(segment_times, waypoints(:, dim));
        pos(:, dim) = ppval(pp, t_clamped);
        [vel(:, dim), acc(:, dim)] = compute_derivatives(pp, t_clamped);
    end
end

function [vel, acc] = compute_derivatives(pp, t)
    % Manually compute derivatives from spline coefficients
    n = length(t);
    vel = zeros(n, 1);
    acc = zeros(n, 1);
    
    breaks = pp.breaks;
    coefs = pp.coefs;  % [a b c d] per segment
    
    for i = 1:n
        segment = find(t(i) >= breaks, 1, 'last');
        if segment >= length(breaks)
            segment = length(breaks) - 1;
        end
        
        dt = t(i) - breaks(segment);
        a = coefs(segment, 1);
        b = coefs(segment, 2);
        c = coefs(segment, 3);
        
        % Derivative formulas
        vel(i) = 3*a*dt^2 + 2*b*dt + c;      % 1st derivative
        acc(i) = 6*a*dt + 2*b;                % 2nd derivative
    end
end