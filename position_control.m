function [Tvec,ref_ang_pos,ref_ang_vel] = position_control(desired_traj,state,params)
%run('C:\Users\loren\Desktop\Mag\Elective 1\quadrotor_landing_on_mobile_platform\setup\model_n_control_param.m')

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
% Summary:
%
% THE TOTAL THRUST IS ONE OF FOUR CONTROL INPUT OF OUR SYSTEM
% THAT IS CALCULATED MAINLY ON THE Z-AXIS THRUST BUT THE X AND Y COMPONENTS
% ARE ALSO USEFUL FOR DETERMINING THE DESIRED ANGLES TO TRACK TO PRODUCE
% SUCH THRUST, THAT TRACKING IS MADE BY THE ATTITUDE CONTROLLER


% ---------- ERRORS ALONG THE Z AXIS (DISTANCE FROM DESIRED)
ez = z_d - z;                   %PROPORTIONAL ERROR
ez_dot = z_d_dot - z_dot;       %DERIVATIVE ERROR

persistent integral_error_z;    %INTEGRAL ERROR INITIALIZATION
if isempty(integral_error_z)
    integral_error_z = 0;
end

% ---------- EXPLAINATION INTEGRAL ERROR CALCULATION
% In the discrete formulation (which is out approximation since we have a
% sampling of a time variable) the integral error is a SUM (basically via
% Euler Integration) and represent HOW MUCH we have been off target
% The quantity summed is the proportional error.
%-------------------------------------------------------------------------

integral_error_z = integral_error_z + ez * dt;  %INTEGRAL ERROR

% ++++++++++ VARIATIONS: UPPER BOUND FOR INTEGRAL ERROR ALONG Z-AXIS
% These are things COMMENTED OUT but kept on the code just in case needed
%
% integral_max = 10.0;  % ARBITRARY VALUE
% integral_error_z = saturate(integral_error_z,-integral_max,integral_max);
%-------------------------------------------------------------------------

% ---------- CONTROL ALONG Z-AXIS WITH A PID CONTROLLER
T_z = -(Kp_pos(3) * ez + Kd_pos(3) * ez_dot + Ki_pos(3) * integral_error_z + z_d_ddot);

% ---------- TOTAL THRUST CALCULATION
T = (m / (cos(phi) * cos(theta))) * (g + T_z);

%-------------------------------------------------------------------------
% Explaination:
% -T is the total Thrust (which is a FORCE: Mass*Acceleration)
% That force will be our control + g (if it were just g, the robot would 
% just hover at 0 nullifying gravity but without vertical acceleration)
%
% -When the quadrotor tilts, only part of the total thrust points upward
% From Trigonometry we got this dividing for cos(phi)*cos(theta)
%
%- Multiply by mass to convert to force
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
integral_error_x = integral_error_x + ex * dt; %INTEGRAL ERROR

% ++++++++++ VARIATIONS: UPPER BOUND FOR INTEGRAL ERROR ALONG X-AXIS
% These are things COMMENTED OUT but kept on the code just in case needed
%
% integral_max defined before
% integral_error_x = saturate(integral_error_x,-integral_max,integral_max);

% ---------- CONTROL ALONG X-AXIS WITH A PID CONTROLLER
U_x = Kp_pos(1) * ex + Kd_pos(1) * ex_dot + Ki_pos(1) * integral_error_x + x_d_ddot;

% ---------- ERRORS ALONG THE X AXIS (DISTANCE FROM DESIRED)
ey = y_d - y;                                   %PROPORTIONAL ERROR
ey_dot = y_d_dot - y_dot;                       %DERIVATIVE ERROR

persistent integral_error_y;                    %INTEGRAL ERROR INITIALIZ.
if isempty(integral_error_y)
    integral_error_y = 0;
end
integral_error_y = integral_error_y + ey * dt;  %INTEGRAL ERROR

% ++++++++++ VARIATIONS: UPPER BOUND FOR INTEGRAL ERROR ALONG Y-AXIS
% These are things COMMENTED OUT but kept on the code just in case needed
%
% integral_max defined before
% integral_error_y = saturate(integral_error_x,-integral_max,integral_max);


% ---------- CONTROL ALONG Y-AXIS WITH A PID CONTROLLER
U_y = Kp_pos(2) * ey + Kd_pos(2) * ey_dot + Ki_pos(2) * integral_error_y + y_d_ddot;

%------------------------------------------------------------------------
%                           CONTROL INPUT BOUNDS
%------------------------------------------------------------------------
% Summary:
%
% U_x and U_y need to be bounded, otherwise we can obtain an unfeasable 
% control, to bound them we calculate MAX ACCELERATION OBTAINABLE basing
% on physics

max_tilt_angle = 0.4;                        %ARBITRARY ANGLE (ABOUT 23°)
max_lateral_accel = g * tan(max_tilt_angle); %MAX LATERAL ACCELERATION
asinboundary=0.99;                           %FORCING TO AVOID SINGULARITY

U_x=saturate(U_x,-max_lateral_accel,max_lateral_accel); %UPPER BOUND UX
U_y=saturate(U_y,-max_lateral_accel,max_lateral_accel); %UPPER BOUND UY

T_x = (m/T) * U_x;  %Actual Thrust on X-Axis
T_y = (m/T) * U_y;  %Actual Thrust on Y-Axis

%------------------------------------------------------------------------
%                      DESIRED TILT ANGLES CALCULATION
%------------------------------------------------------------------------

%DESIRED ROLL ANGLE (X-AXIS)
phi_d = asin(saturate(T_y*cos(psi) - T_x*sin(psi),-asinboundary,asinboundary));

%DESIRED PITCH ANGLE (Y-AXIS)
theta_d = -asin(saturate((T_y*cos(psi) + T_x*sin(psi))/cos(phi_d),-asinboundary,asinboundary));

%DESIRED YAW ANGLE (Z-AXIS) : Suggested by Claude
%If gives problem we can saturate the arguments
psi_d = atan2(y_d_dot, x_d_dot);  

%------------------------------------------------------------------------
%                    ANGULAR VELOCITIES CALCULATION (To Review)
%------------------------------------------------------------------------
% Questo l'ho lasciato come prima perché non ho riscontrato problemi ma
% è in To-Do da ricontrollare tutto e rivedere, in caso, soprattutto il
% passabasso

% Memorizza stati precedenti per calcolo velocità angolari
persistent prev_phi_d prev_theta_d prev_psi_d prev_time;
if isempty(prev_phi_d)
    prev_phi_d = phi_d;
    prev_theta_d = theta_d;
    prev_psi_d = psi_d;
    prev_time = 0;
end

% Calcolo delle velocità angolari con filtro passa-basso
time_constant = 0.05;  % Costante di tempo per il filtro
phi_dot_d = (phi_d - prev_phi_d) / dt;
theta_dot_d = (theta_d - prev_theta_d) / dt;
psi_dot_d = (psi_d - prev_psi_d) / dt;

% Filtraggio delle velocità angolari
persistent filtered_phi_dot_d filtered_theta_dot_d filtered_psi_dot_d;
if isempty(filtered_phi_dot_d)
    filtered_phi_dot_d = phi_dot_d;
    filtered_theta_dot_d = theta_dot_d;
    filtered_psi_dot_d = psi_dot_d;
end

alpha = dt / (time_constant + dt);  % Coefficiente filtro passa-basso
filtered_phi_dot_d = (1-alpha) * filtered_phi_dot_d + alpha * phi_dot_d;
filtered_theta_dot_d = (1-alpha) * filtered_theta_dot_d + alpha * theta_dot_d;
filtered_psi_dot_d = (1-alpha) * filtered_psi_dot_d + alpha * psi_dot_d;


% Aggiorna i valori precedenti per il prossimo ciclo
prev_phi_d = phi_d;
prev_theta_d = theta_d;
prev_psi_d = psi_d;
prev_time = prev_time + dt;

%------------------------------------------------------------------------
%                                   OUTPUTs
%------------------------------------------------------------------------
ref_ang_pos = [phi_d, theta_d, psi_d];
ref_ang_vel = [filtered_phi_dot_d, filtered_theta_dot_d, filtered_psi_dot_d];
Tvec=[T,T_x,T_y,T_z];
end

%UTILITY FUNCTION TO SATURATE SOME VARIABLES
function val_sat = saturate(val, min_val, max_val)
    val_sat = max(min(val, max_val), min_val);
end
