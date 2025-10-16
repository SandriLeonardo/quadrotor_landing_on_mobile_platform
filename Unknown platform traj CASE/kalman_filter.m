function [platform_state_est, P_out] = kalman_filter(detected, measurement, dt, P_in)
% Extended Kalman Filter per platform tracking
% Inputs:
%   detected: 0/1 flag
%   measurement: [px, py, pz] se detected=1
%   dt: sample time
%   P_in: covariance matrix (5x5) dal passo precedente
% Outputs:
%   platform_state_est: [px, py, pz, theta, vt]
%   P_out: updated covariance matrix

persistent x_est initialized

% Parametri
if isempty(initialized)
    % Initialize state: [px, py, pz, theta, vt]
    x_est = [0; 0; 0; 0; 0.5];  % vt iniziale 0.5 m/s
    initialized = true;
end

% Process noise
Q = diag([0.01, 0.01, 0.01, 0.1, 0.05]);  % TODO: Tuning required

% Measurement noise
R = diag([0.1, 0.1, 0.1]);  % TODO: Tuning required

%% Prediction step (sempre)
% Motion model: TODO: vedi Eq. 2 del paper
px = x_est(1);
py = x_est(2);
pz = x_est(3);
theta = x_est(4);
vt = x_est(5);

% State transition
x_pred = [px + vt*cos(theta)*dt;
          py + vt*sin(theta)*dt;
          pz;  % Z costante
          theta;  % u1=0 (constant velocity)
          vt];     % u2=0

% Jacobian della transizione
F = [1, 0, 0, -vt*sin(theta)*dt, cos(theta)*dt;
     0, 1, 0,  vt*cos(theta)*dt, sin(theta)*dt;
     0, 0, 1,  0,                0;
     0, 0, 0,  1,                0;
     0, 0, 0,  0,                1];

% Covariance prediction
P_pred = F * P_in * F' + Q;

%% Update step (solo se detected=1)
if detected == 1
    % Measurement model: H = [I_3x3, 0_3x2]
    H = [eye(3), zeros(3,2)];
    
    % Innovation
    z = measurement(:);
    y = z - H * x_pred;
    
    % Kalman gain
    S = H * P_pred * H' + R;
    K = P_pred * H' / S;
    
    % State update
    x_est = x_pred + K * y;
    
    % Covariance update
    P_out = (eye(5) - K * H) * P_pred;
else
    % No measurement: use prediction only
    x_est = x_pred;
    P_out = P_pred;
end

% Output
platform_state_est = x_est;

end