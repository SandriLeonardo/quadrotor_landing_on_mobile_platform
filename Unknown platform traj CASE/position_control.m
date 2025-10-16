function [Tvec,ref_ang_pos,ref_ang_vel,ref_ang_accel] = position_control(desired_traj,state,params)

%------------------------------------------------------------------------
%                        STATE PARAMETERS IMPORT
%------------------------------------------------------------------------

x = state(1);
y = state(2);
z = state(3);
phi = state(4);
theta = state(5);
psi = state(6);
x_dot = state(7);
y_dot = state(8);
z_dot = state(9);

%------------------------------------------------------------------------
%                        TRAJECTORY PARAMETERS IMPORT
%------------------------------------------------------------------------
% DESIRED END POSITIONS, VELOCITIES AND ACCELERATION ALONG THE AXES

x_d = desired_traj(1);
y_d = desired_traj(2);
z_d = desired_traj(3);
x_d_dot = desired_traj(4);
y_d_dot = desired_traj(5);
z_d_dot = desired_traj(6);
x_d_ddot = desired_traj(7);
y_d_ddot = desired_traj(8);
z_d_ddot = desired_traj(9);

% ++++++++++ JERK TERMS (Correction 7 - OPTIONAL)
% If your trajectory includes jerk (third derivative), uncomment these:
% and make sure desired_traj has 12 elements
% x_d_dddot = desired_traj(10);
% y_d_dddot = desired_traj(11);
% z_d_dddot = desired_traj(12);
% Otherwise, jerk is set to zero:
x_d_dddot = 0;
y_d_dddot = 0;
z_d_dddot = 0;
%-------------------------------------------------------------------------

%------------------------------------------------------------------------
%                         OTHER PARAMETERS IMPORT
%------------------------------------------------------------------------

m = params(1);              % MASS
g = params(2);              % GRAVITY ACCELERATION
Kp_pos = params(6:8);       % POSITIONAL GAINS: PROPORTIONAL (X,Y,Z)
Kd_pos = params(9:11);      % POSITIONAL GAINS: DERIVATIVE (X,Y,Z)
Ki_pos = params(12:14);     % POSITIONAL GAINS: INTEGRAL (X,Y,Z)
dt = params(24);

%------------------------------------------------------------------------
%                    VERTICAL THRUST CONTROL CALCULATIONS
%------------------------------------------------------------------------

% ---------- ERRORS ALONG THE Z AXIS (DISTANCE FROM DESIRED)
ez = z_d - z;                   %PROPORTIONAL ERROR
ez_dot = z_d_dot - z_dot;       %DERIVATIVE ERROR

persistent integral_error_z;    %INTEGRAL ERROR INITIALIZATION
if isempty(integral_error_z)
    integral_error_z = 0;
end

% ---------- CONTROL ALONG Z-AXIS WITH A PID CONTROLLER
T_z_pid = -(Kp_pos(3) * ez + Kd_pos(3) * ez_dot + Ki_pos(3) * integral_error_z + z_d_ddot);

% ++++++++++ JERK FEEDFORWARD (Correction 7 - COMMENTED)
% Uncomment if jerk is available and set above:
% T_z_pid = -(Kp_pos(3) * ez + Kd_pos(3) * ez_dot + Ki_pos(3) * integral_error_z + ...
%             z_d_ddot + (1/Kp_pos(3)) * z_d_dddot);
%-------------------------------------------------------------------------

% ---------- TOTAL THRUST CALCULATION
cos_product = cos(phi) * cos(theta);
if abs(cos_product) < 0.1
    cos_product = sign(cos_product) * 0.1;
end
T_total = (m / cos_product) * (g + T_z_pid);

% ---------- ANTI-WINDUP FOR Z-AXIS (Correction 2)
T_min = 0.1 * m * g;
T_max = 4.0 * m * g;
T = saturate(T_total, T_min, T_max);

% Only integrate if not saturated OR if error helps unsaturate
if (T_total >= T_min && T_total <= T_max) || (sign(ez) ~= sign(T))
    integral_error_z = integral_error_z + ez * dt;
end
%-------------------------------------------------------------------------

%------------------------------------------------------------------------
%                    LATERAL THRUST CONTROL CALCULATIONS
%------------------------------------------------------------------------

% ---------- ERRORS ALONG THE X AXIS (DISTANCE FROM DESIRED)
ex = x_d - x;                                  %PROPORTIONAL ERROR
ex_dot = x_d_dot - x_dot;                      %DERIVATIVE ERROR

persistent integral_error_x;                   %INTEGRAL ERROR INITIALIZ.
if isempty(integral_error_x)
    integral_error_x = 0;
end

% ---------- CONTROL ALONG X-AXIS WITH A PID CONTROLLER
U_x = Kp_pos(1) * ex + Kd_pos(1) * ex_dot + Ki_pos(1) * integral_error_x + x_d_ddot;

% ++++++++++ JERK FEEDFORWARD (Correction 7 - COMMENTED)
% Uncomment if jerk is available and set above:
% U_x = Kp_pos(1) * ex + Kd_pos(1) * ex_dot + Ki_pos(1) * integral_error_x + ...
%       x_d_ddot + (1/Kp_pos(1)) * x_d_dddot;
%-------------------------------------------------------------------------

% ---------- ERRORS ALONG THE Y AXIS (DISTANCE FROM DESIRED)
ey = y_d - y;                                   %PROPORTIONAL ERROR
ey_dot = y_d_dot - y_dot;                       %DERIVATIVE ERROR

persistent integral_error_y;                    %INTEGRAL ERROR INITIALIZ.
if isempty(integral_error_y)
    integral_error_y = 0;
end

% ---------- CONTROL ALONG Y-AXIS WITH A PID CONTROLLER
U_y = Kp_pos(2) * ey + Kd_pos(2) * ey_dot + Ki_pos(2) * integral_error_y + y_d_ddot;

% ++++++++++ JERK FEEDFORWARD (Correction 7 - COMMENTED)
% Uncomment if jerk is available and set above:
% U_y = Kp_pos(2) * ey + Kd_pos(2) * ey_dot + Ki_pos(2) * integral_error_y + ...
%       y_d_ddot + (1/Kp_pos(2)) * y_d_dddot;
%-------------------------------------------------------------------------

%------------------------------------------------------------------------
%                           CONTROL INPUT BOUNDS
%------------------------------------------------------------------------

max_tilt_angle = 0.4;                        %ARBITRARY ANGLE (ABOUT 23°)
max_lateral_accel = g * tan(max_tilt_angle); %MAX LATERAL ACCELERATION
asinboundary = 0.99;                         %FORCING TO AVOID SINGULARITY

U_x_sat = saturate(U_x, -max_lateral_accel, max_lateral_accel);
U_y_sat = saturate(U_y, -max_lateral_accel, max_lateral_accel);

% ---------- ANTI-WINDUP FOR X AND Y AXES (Correction 2)
% Only integrate if not saturated OR if error helps unsaturate
if (abs(U_x) <= max_lateral_accel) || (sign(ex) ~= sign(U_x_sat))
    integral_error_x = integral_error_x + ex * dt;
end

if (abs(U_y) <= max_lateral_accel) || (sign(ey) ~= sign(U_y_sat))
    integral_error_y = integral_error_y + ey * dt;
end
%-------------------------------------------------------------------------

T_x = (m/T) * U_x_sat;  %Actual Thrust on X-Axis
T_y = (m/T) * U_y_sat;  %Actual Thrust on Y-Axis

%------------------------------------------------------------------------
%                      DESIRED TILT ANGLES CALCULATION
%------------------------------------------------------------------------

%DESIRED ROLL ANGLE (X-AXIS)
phi_d = asin(saturate(T_y*cos(psi) - T_x*sin(psi), -asinboundary, asinboundary));

%DESIRED PITCH ANGLE (Y-AXIS)
theta_d = -asin(saturate((T_y*sin(psi) + T_x*cos(psi))/cos(phi_d), -asinboundary, asinboundary));

%DESIRED YAW ANGLE (Z-AXIS)
psi_d = atan2(y_d_dot, x_d_dot);  

%------------------------------------------------------------------------
%         ANGULAR VELOCITIES & ACCELERATIONS (Correction 4)
%------------------------------------------------------------------------
% Calculate derivatives for better reference tracking

% Initialize persistent variables for storing previous values
persistent prev_phi_d prev_theta_d prev_psi_d;
persistent prev_phi_dot_d prev_theta_dot_d prev_psi_dot_d;

if isempty(prev_phi_d)
    prev_phi_d = phi_d;
    prev_theta_d = theta_d;
    prev_psi_d = psi_d;
    prev_phi_dot_d = 0;
    prev_theta_dot_d = 0;
    prev_psi_dot_d = 0;
end

% ---------- NUMERICAL ANGULAR VELOCITIES
% Using finite differences
phi_dot_d = (phi_d - prev_phi_d) / dt;
theta_dot_d = (theta_d - prev_theta_d) / dt;
psi_dot_d = (psi_d - prev_psi_d) / dt;

% ---------- LOW-PASS FILTER FOR SMOOTHER DERIVATIVES (Correction 4)
% Second-order Butterworth-style filter for better noise rejection
persistent filtered_phi_dot_d filtered_theta_dot_d filtered_psi_dot_d;
if isempty(filtered_phi_dot_d)
    filtered_phi_dot_d = 0;
    filtered_theta_dot_d = 0;
    filtered_psi_dot_d = 0;
end

% Filter parameters (can be tuned)
omega_c = 10;  % Cutoff frequency [rad/s]
zeta = 0.7;    % Damping ratio
alpha = dt * omega_c / (dt * omega_c + 2 * zeta);

filtered_phi_dot_d = (1 - alpha) * filtered_phi_dot_d + alpha * phi_dot_d;
filtered_theta_dot_d = (1 - alpha) * filtered_theta_dot_d + alpha * theta_dot_d;
filtered_psi_dot_d = (1 - alpha) * filtered_psi_dot_d + alpha * psi_dot_d;

% ---------- ANGULAR ACCELERATIONS (Correction 4)
% Compute accelerations for inverse dynamics feedforward
phi_ddot_d = (filtered_phi_dot_d - prev_phi_dot_d) / dt;
theta_ddot_d = (filtered_theta_dot_d - prev_theta_dot_d) / dt;
psi_ddot_d = (filtered_psi_dot_d - prev_psi_dot_d) / dt;

% Apply light filtering to accelerations
persistent filtered_phi_ddot_d filtered_theta_ddot_d filtered_psi_ddot_d;
if isempty(filtered_phi_ddot_d)
    filtered_phi_ddot_d = 0;
    filtered_theta_ddot_d = 0;
    filtered_psi_ddot_d = 0;
end

alpha_accel = 0.3;  % More aggressive filtering for accelerations
filtered_phi_ddot_d = (1 - alpha_accel) * filtered_phi_ddot_d + alpha_accel * phi_ddot_d;
filtered_theta_ddot_d = (1 - alpha_accel) * filtered_theta_ddot_d + alpha_accel * theta_ddot_d;
filtered_psi_ddot_d = (1 - alpha_accel) * filtered_psi_ddot_d + alpha_accel * psi_ddot_d;

% Update previous values for next iteration
prev_phi_d = phi_d;
prev_theta_d = theta_d;
prev_psi_d = psi_d;
prev_phi_dot_d = filtered_phi_dot_d;
prev_theta_dot_d = filtered_theta_dot_d;
prev_psi_dot_d = filtered_psi_dot_d;

%------------------------------------------------------------------------
%                                   OUTPUTs
%------------------------------------------------------------------------
ref_ang_pos = [phi_d, theta_d, psi_d];
ref_ang_vel = [filtered_phi_dot_d, filtered_theta_dot_d, filtered_psi_dot_d];
ref_ang_accel = [filtered_phi_ddot_d, filtered_theta_ddot_d, filtered_psi_ddot_d];
Tvec = [T, T_x, T_y, T_z_pid];

end

%UTILITY FUNCTION TO SATURATE SOME VARIABLES
function val_sat = saturate(val, min_val, max_val)
    val_sat = max(min(val, max_val), min_val);
end