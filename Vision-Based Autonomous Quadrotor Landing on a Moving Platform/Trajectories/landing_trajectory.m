function [pos, vel, acc] = landing_trajectory(t, current_pos, platform_pos, platform_vel, platform_acc, state)
%#codegen
% landing_trajectory - Follows platform trajectory while descending smoothly
% Uses first-order low-pass filter for smooth tracking (same as bypass behavior)
%
% Inputs:
%   t            - current simulation time
%   current_pos  - current position [x,y,z]
%   platform_pos - platform position [x,y,z]
%   platform_vel - platform velocity [x,y,z]
%   platform_acc - platform acceleration [x,y,z]
%   state        - state flag (3 = landing active)
%
% Outputs:
%   pos - position [x,y,z]
%   vel - velocity [x,y,z]
%   acc - acceleration [x,y,z]

%% ------------------- FIXED SIZE OUTPUTS ------------------------
pos = zeros(1,3);
vel = zeros(1,3);
acc = zeros(1,3);

%% ------------------- FORCE INPUTS TO FIXED SIZE ----------------
current_pos  = reshape(current_pos, 1, 3);
platform_pos = reshape(platform_pos, 1, 3);
platform_vel = reshape(platform_vel, 1, 3);
platform_acc = reshape(platform_acc, 1, 3);

%% ------------------- PERSISTENT VARIABLES -----------------------
persistent t_start trajectory_active pos_start
persistent pos_f vel_f acc_f initialized t_prev
persistent last_pos_out last_vel_out last_acc_out

if isempty(trajectory_active)
    trajectory_active = 0;
    t_start = 0;
    pos_start = zeros(1,3);
    
    pos_f = zeros(1,3);
    vel_f = zeros(1,3);
    acc_f = zeros(1,3);
    initialized = 0;
    
    t_prev = t;
    last_pos_out = zeros(1,3);
    last_vel_out = zeros(1,3);
    last_acc_out = zeros(1,3);
end

%% ------------------- PARAMETERS --------------------------------
landing_time = 10.0;  % total landing duration (s)
% REMOVED THRESHOLD: set to 0 to land exactly on the platform center
platform_height_offset = 0.0;  
tau = 5.0;  % time constant for smoothing (s)
min_dt = 1e-6;  % guard for dt ~ 0

%% ------------------- LANDING TRIGGER ---------------------------
if state == 4 && trajectory_active == 0
    trajectory_active = 1;
    t_start = t;
    pos_start = current_pos;
    initialized = 0;
    fprintf('Landing trajectory started at t=%.2f from [%.2f %.2f %.2f]\n', ...
            t, pos_start(1), pos_start(2), pos_start(3));
end

%% ------------------- BEFORE ACTIVATION -------------------------
if trajectory_active == 0
    pos = current_pos;
    vel = [0 0 0];
    acc = [0 0 0];
    
    last_pos_out = pos;
    last_vel_out = vel;
    last_acc_out = acc;
    t_prev = t;
    return;
end

%% ------------------- DURING LANDING (WITH SMOOTHING) -----------
t_local = t - t_start;
alpha = min(t_local / landing_time, 1.0);

% ------------------- FEED-FORWARD CORRECTION -------------------
% Add velocity feed-forward to compensate for the lag caused by tau
% We predict where the platform will be in 'tau' seconds.
pred_x = platform_pos(1) + platform_vel(1) * tau;
pred_y = platform_pos(2) + platform_vel(2) * tau;
pred_z = platform_pos(3) + platform_vel(3) * tau; % Added Z feed-forward

% Platform target with height offset (now 0) using PREDICTED POS
target_pos = [pred_x, pred_y, pred_z + platform_height_offset];
% ---------------------------------------------------------------

% Compute velocity: includes platform horizontal motion + descent velocity
vel_interp = (target_pos - pos_start) / landing_time;
target_vel = vel_interp + [platform_vel(1), platform_vel(2), platform_vel(3)];

% Use platform acceleration for smoother tracking
target_acc = platform_acc;

% Compute unsmoothed interpolated position
pos_interp = (1 - alpha) * pos_start + alpha * target_pos;

% Compute dt safely
dt = t - t_prev;
if dt <= 0
    dt = min_dt;
end
t_prev = t;

% Compute alpha from tau with stable formula: alpha = 1 - exp(-dt/tau)
alpha_lp = 1 - exp(-dt / tau);

% Initialize filter on first call (use last outputs to avoid jump)
if initialized == 0
    pos_f = last_pos_out;
    vel_f = last_vel_out;
    acc_f = last_acc_out;
    initialized = 1;
end

% Apply first-order low-pass filter (causal, sampling-rate aware)
% Filter the interpolated position and target velocities/accelerations
pos_f(1) = pos_f(1) + alpha_lp * (pos_interp(1) - pos_f(1));
pos_f(2) = pos_f(2) + alpha_lp * (pos_interp(2) - pos_f(2));
pos_f(3) = pos_f(3) + alpha_lp * (pos_interp(3) - pos_f(3));

vel_f(1) = vel_f(1) + alpha_lp * (target_vel(1) - vel_f(1));
vel_f(2) = vel_f(2) + alpha_lp * (target_vel(2) - vel_f(2));
vel_f(3) = vel_f(3) + alpha_lp * (target_vel(3) - vel_f(3));

acc_f(1) = acc_f(1) + alpha_lp * (target_acc(1) - acc_f(1));
acc_f(2) = acc_f(2) + alpha_lp * (target_acc(2) - acc_f(2));
acc_f(3) = acc_f(3) + alpha_lp * (target_acc(3) - acc_f(3));

% Output
pos = pos_f;
vel = vel_f;
acc = acc_f;

% Update last outputs
last_pos_out = pos;
last_vel_out = vel;
last_acc_out = acc;

end