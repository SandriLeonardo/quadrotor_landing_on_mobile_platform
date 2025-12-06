function [pos, vel, acc, enabled] = smooth_start_tracking( ...
    t, hovering_position, platform_pos, platform_vel, platform_acc, state)
%#codegen
% Smooth start tracking with bypass smoothing using first-order low-pass
% tau = time constant (s). Smoothing is causal and sampling-rate aware.

%% ------------------- FIXED SIZE OUTPUTS ------------------------
pos = zeros(1,3);
vel = zeros(1,3);
acc = zeros(1,3);
enabled = 0;

%% ------------------- FORCE INPUTS TO FIXED SIZE ----------------
hovering_position = reshape(hovering_position, 1, 3);
platform_pos      = reshape(platform_pos, 1, 3);
platform_vel      = reshape(platform_vel, 1, 3);
platform_acc      = reshape(platform_acc, 1, 3);

%% ------------------- PERSISTENT VARIABLES -----------------------
persistent t_start trajectory_active transition_complete pos_A
persistent pos_f vel_f acc_f initialized t_prev
persistent last_pos_out last_vel_out last_acc_out

if isempty(trajectory_active)
    t_start             = 0;
    trajectory_active   = 0;
    transition_complete = 0;
    pos_A               = zeros(1, 3);

    pos_f       = zeros(1,3);
    vel_f       = zeros(1,3);
    acc_f       = zeros(1,3);
    initialized = 0;

    t_prev = t; % initialize previous time to current call
    last_pos_out = zeros(1,3);
    last_vel_out = zeros(1,3);
    last_acc_out = zeros(1,3);
end

%% ------------------- PARAMETERS --------------------------------
T      = 4.0;     % transition duration (s)
offset = 1.5;
tau    = 5;     % time constant for bypass smoothing (s) -- tune this
% note: smaller tau -> faster response (less smoothing)
%       larger  tau -> smoother but slower response
min_dt = 1e-6;    % guard for dt ~ 0

%% ------------------- TARGET WITH HEIGHT OFFSET -----------------
% Add velocity feed-forward to compensate for the lag caused by tau
pred_x = platform_pos(1) + platform_vel(1) * tau;
pred_y = platform_pos(2) + platform_vel(2) * tau;

target_pos = [pred_x, pred_y, 0 + offset];
target_vel = platform_vel;
target_acc = platform_acc;

if state == 1
    enabled = 1;
end

if state == 3
    enabled = 1;
end

%% ------------------- ACTIVATE TRANSITION ------------------------
if state == 2 && trajectory_active == 0
    trajectory_active   = 1;
    transition_complete = 0;
    t_start             = t;
    pos_A               = hovering_position;
    initialized         = 0;
end

%% ------------------- BEFORE ACTIVATION --------------------------
if trajectory_active == 0
    pos = hovering_position;
    vel = [0 0 0];
    acc = [0 0 0];

    % update last outputs
    last_pos_out = pos;
    last_vel_out = vel;
    last_acc_out = acc;
    t_prev = t;
    return;
end

%% ------------------- DURING TRANSITION --------------------------
t_local = t - t_start;

if transition_complete == 0 && t_local < T
    x = t_local / T;
    alpha_tr   = 3*x^2 - 2*x^3;
    dalpha_tr  = (6*x - 6*x^2) / T;
    ddalpha_tr = (6 - 12*x) / T^2;

    diff_PA = (target_pos - pos_A);
    pos = pos_A + alpha_tr * diff_PA;
    vel = dalpha_tr * diff_PA + alpha_tr * target_vel;
    acc = ddalpha_tr * diff_PA + dalpha_tr * target_vel + alpha_tr * target_acc;
    enabled = 0;

    % update last outputs and time
    last_pos_out = pos;
    last_vel_out = vel;
    last_acc_out = acc;
    t_prev = t;
    return;
end

%% ------------------- AFTER TRANSITION (BYPASS WITH SMOOTHING) ---
transition_complete = 1;

% compute dt safely
dt = t - t_prev;
if dt <= 0
    dt = min_dt;
end
t_prev = t;

% compute alpha from tau with stable formula: alpha = 1 - exp(-dt/tau)
% avoid calling exp with negative or zero tau (tau>0 assumed)
alpha_lp = 1 - exp(-dt / tau);

% initialize filter on first bypass call using last outputs (avoid jump)
if initialized == 0
    % Use last_pos_out etc. to preserve continuity (do not set to target)
    pos_f = last_pos_out;
    vel_f = last_vel_out;
    acc_f = last_acc_out;
    initialized = 1;
end

% Apply first-order low-pass (causal, sampling-rate aware)
% pos_f <- pos_f + alpha_lp * (target_pos - pos_f)
pos_f(1) = pos_f(1) + alpha_lp * (target_pos(1) - pos_f(1));
pos_f(2) = pos_f(2) + alpha_lp * (target_pos(2) - pos_f(2));
pos_f(3) = pos_f(3) + alpha_lp * (target_pos(3) - pos_f(3));

vel_f(1) = vel_f(1) + alpha_lp * (target_vel(1) - vel_f(1));
vel_f(2) = vel_f(2) + alpha_lp * (target_vel(2) - vel_f(2));
vel_f(3) = vel_f(3) + alpha_lp * (target_vel(3) - vel_f(3));

acc_f(1) = acc_f(1) + alpha_lp * (target_acc(1) - acc_f(1));
acc_f(2) = acc_f(2) + alpha_lp * (target_acc(2) - acc_f(2));
acc_f(3) = acc_f(3) + alpha_lp * (target_acc(3) - acc_f(3));

% output
pos = pos_f;
vel = vel_f;
acc = acc_f;
enabled = 1;

% update last outputs
last_pos_out = pos;
last_vel_out = vel;
last_acc_out = acc;

end
