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
x_d = desired_traj(1);
y_d = desired_traj(2);
z_d = desired_traj(3);
x_d_dot = desired_traj(4);
y_d_dot = desired_traj(5);
z_d_dot = desired_traj(6);
x_d_ddot = desired_traj(7);
y_d_ddot = desired_traj(8);
z_d_ddot = desired_traj(9);

%------------------------------------------------------------------------
%                         OTHER PARAMETERS IMPORT
%------------------------------------------------------------------------
m = params(1);              % MASS
g = params(2);              % GRAVITY ACCELERATION
Kp_pos = params(6:8);       % POSITIONAL GAINS
Kd_pos = params(9:11);      % DERIVATIVE GAINS
Ki_pos = params(12:14);     % INTEGRAL GAINS
dt = params(24);

%------------------------------------------------------------------------
%                    VERTICAL THRUST CONTROL CALCULATIONS
%------------------------------------------------------------------------
ez = z_d - z;                   
ez_dot = z_d_dot - z_dot;       

persistent integral_error_z;    
if isempty(integral_error_z)
    integral_error_z = 0;
end

T_z_pid = -(Kp_pos(3) * ez + Kd_pos(3) * ez_dot + Ki_pos(3) * integral_error_z + z_d_ddot);

cos_product = cos(phi) * cos(theta);
if abs(cos_product) < 0.1
    cos_product = sign(cos_product) * 0.1;
end
T_total = (m / cos_product) * (g + T_z_pid);

% Anti-Windup Z
T_min = 0.1 * m * g;
T_max = 4.0 * m * g;
T = saturate(T_total, T_min, T_max);

is_saturated_max = (T_total > T_max);
is_saturated_min = (T_total < T_min);

stop_integration = false;
if is_saturated_max && (ez < 0), stop_integration = true;
elseif is_saturated_min && (ez > 0), stop_integration = true;
end

if ~stop_integration
    integral_error_z = integral_error_z + ez * dt;
end

%------------------------------------------------------------------------
%                    LATERAL THRUST CONTROL CALCULATIONS
%------------------------------------------------------------------------
ex = x_d - x;                                  
ex_dot = x_d_dot - x_dot;                      

persistent integral_error_x;                   
if isempty(integral_error_x), integral_error_x = 0; end

U_x = Kp_pos(1) * ex + Kd_pos(1) * ex_dot + Ki_pos(1) * integral_error_x + x_d_ddot;

ey = y_d - y;                                   
ey_dot = y_d_dot - y_dot;                       

persistent integral_error_y;                    
if isempty(integral_error_y), integral_error_y = 0; end

U_y = Kp_pos(2) * ey + Kd_pos(2) * ey_dot + Ki_pos(2) * integral_error_y + y_d_ddot;

%------------------------------------------------------------------------
%                           CONTROL INPUT BOUNDS
%------------------------------------------------------------------------
max_tilt_angle = 0.7;                     
max_lateral_accel = g * tan(max_tilt_angle); 
asinboundary = 0.99;                         

U_x_sat = saturate(U_x, -max_lateral_accel, max_lateral_accel);
U_y_sat = saturate(U_y, -max_lateral_accel, max_lateral_accel);

% Anti-Windup X/Y (No Clamping)
if (abs(U_x) <= max_lateral_accel) || (sign(ex) ~= sign(U_x_sat))
    integral_error_x = integral_error_x + ex * dt;
end

if (abs(U_y) <= max_lateral_accel) || (sign(ey) ~= sign(U_y_sat))
    integral_error_y = integral_error_y + ey * dt;
end

T_x = (m/T) * U_x_sat;  
T_y = (m/T) * U_y_sat;  

%------------------------------------------------------------------------
%                      DESIRED TILT ANGLES CALCULATION
%------------------------------------------------------------------------
phi_d = asin(saturate(T_y*cos(psi) - T_x*sin(psi), -asinboundary, asinboundary));
theta_d = -asin(saturate((T_y*sin(psi) + T_x*cos(psi))/cos(phi_d), -asinboundary, asinboundary));

%------------------------------------------------------------------------
%         DESIRED YAW ANGLE (Z-AXIS) - CONTINUOUS UNWRAPPING
%------------------------------------------------------------------------

% 1. Calculate Velocity Norm
vel_norm = norm([x_d_dot, y_d_dot]);
velocity_threshold = 0.1; 

persistent continuous_psi_d; % Stores the unwrapped, smooth angle
if isempty(continuous_psi_d)
    continuous_psi_d = 0; 
end

% 2. Calculate Target Yaw
if vel_norm > velocity_threshold
    % Calculate Raw Angle from atan2 [-pi, pi]
    raw_psi_d = atan2(y_d_dot, x_d_dot);
    
    % --- THE FIX: UNWRAP THE SIGNAL ---
    % Compare new raw angle with the accumulated continuous angle
    % We want the "jump" to be small (e.g. 3.15 -> 3.20), not large (3.15 -> -3.10)
    
    % Get the current "phase" of the continuous angle
    % We bring continuous_psi_d back to [-pi, pi] just for comparison
    current_wrapped = atan2(sin(continuous_psi_d), cos(continuous_psi_d));
    
    diff = raw_psi_d - current_wrapped;
    
    % Handle the wrap-around
    while diff > pi
        diff = diff - 2*pi;
    end
    while diff < -pi
        diff = diff + 2*pi;
    end
    
    % Add the smooth difference to the accumulator
    continuous_psi_d = continuous_psi_d + diff;
    
    % Analytical Derivative
    psi_dot_d = (x_d_dot * y_d_ddot - y_d_dot * x_d_ddot) / (vel_norm^2);
    
else
    % If stopped, hold the last continuous angle
    psi_dot_d = 0;
end 

% Use the smooth, unwrapped angle as the command
psi_d = continuous_psi_d;


%------------------------------------------------------------------------
%         ANGULAR VELOCITIES & ACCELERATIONS (NO FILTER)
%------------------------------------------------------------------------
persistent prev_phi_d prev_theta_d;
persistent prev_phi_dot_d prev_theta_dot_d prev_psi_dot_d;

if isempty(prev_phi_d)
    prev_phi_d = phi_d;
    prev_theta_d = theta_d;
    prev_phi_dot_d = 0;
    prev_theta_dot_d = 0;
    prev_psi_dot_d = 0;
end

% Numerical Derivatives for Roll/Pitch (Feedback dependent)
phi_dot_d = (phi_d - prev_phi_d) / dt;
theta_dot_d = (theta_d - prev_theta_d) / dt;

% Angular Accelerations
phi_ddot_d = (phi_dot_d - prev_phi_dot_d) / dt;
theta_ddot_d = (theta_dot_d - prev_theta_dot_d) / dt;
psi_ddot_d = (psi_dot_d - prev_psi_dot_d) / dt;

% Update
prev_phi_d = phi_d;
prev_theta_d = theta_d;
prev_phi_dot_d = phi_dot_d;
prev_theta_dot_d = theta_dot_d;
prev_psi_dot_d = psi_dot_d;

%------------------------------------------------------------------------
%                                   OUTPUTs
%------------------------------------------------------------------------
ref_ang_pos = [phi_d, theta_d, psi_d];
ref_ang_vel = [phi_dot_d, theta_dot_d, psi_dot_d];
ref_ang_accel = [phi_ddot_d, theta_ddot_d, psi_ddot_d];
Tvec = [T, T_x, T_y, T_z_pid];

end

function val_sat = saturate(val, min_val, max_val)
    val_sat = max(min(val, max_val), min_val);
end