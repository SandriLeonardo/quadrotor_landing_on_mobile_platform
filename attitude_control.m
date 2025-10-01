function [tau_phi,tau_theta,tau_psi] = attitude_control(ref_ang_pos,ref_ang_vel,state,params)

%------------------------------------------------------------------------
%                        STATE PARAMETERS IMPORT
%------------------------------------------------------------------------

phi = state(4);
theta = state(5);
psi = state(6);
phi_dot = state(10);
theta_dot = state(11);
psi_dot = state(12);


%------------------------------------------------------------------------
%                         OTHER PARAMETERS IMPORT
%------------------------------------------------------------------------

Kp_ang = params(15:17);
Kd_ang = params(18:20);
Ki_ang = params(21:23);
dt = params(24);


%------------------------------------------------------------------------
%                           RPY CONTROL INPUTS
%------------------------------------------------------------------------


% ---------- PHI ANGLE ERRORS (ALONG THE X AXIS) 
e_phi = ref_ang_pos(1) - phi;                         %PROPORTIONAL ERROR
e_phi_dot = ref_ang_vel(1) - phi_dot;                 %DERIVATIVE ERROR


persistent integral_error_phi;                        %INTEGRAL ERROR INITIAL.
if isempty(integral_error_phi)
    integral_error_phi = 0;
end

integral_error_phi = integral_error_phi + e_phi * dt; %INTEGRAL ERROR

% ++++++++++ VARIATIONS: UPPER BOUND FOR INTEGRAL ERROR FOR PHI
% These are things COMMENTED OUT but kept on the code just in case needed
%
% integral_max = 10.0;  % ARBITRARY VALUE
% integral_error_phi = saturate(integral_error_phi,-integral_max,integral_max);
%-------------------------------------------------------------------------

% ---------- CONTROL PHI (ROLL, ALONG X-AXIS) WITH A PID
tau_phi = Kp_ang(1) * e_phi + Kd_ang(1) * e_phi_dot + Ki_ang(1) * integral_error_phi;


% ---------- THETA ANGLE ERRORS (ALONG THE Y AXIS) 
e_theta = ref_ang_pos(2) - theta;                  %PROPORTIONAL ERROR
e_theta_dot = ref_ang_vel(2) - theta_dot;          %DERIVATIVE ERROR


persistent integral_error_theta;                   %INTEGRAL ERROR INITIAL.
if isempty(integral_error_theta)
    integral_error_theta = 0;
end
integral_error_theta = integral_error_theta + e_theta * dt; %INTEGRAL ERROR


% ++++++++++ VARIATIONS: UPPER BOUND FOR INTEGRAL ERROR FOR THETA
% These are things COMMENTED OUT but kept on the code just in case needed
%
% 
% integral_error_theta = saturate(integral_error_theta,-integral_max,integral_max);
%-------------------------------------------------------------------------

% ---------- CONTROL THETA (PITCH, ALONG Y-AXIS) WITH A PID
tau_theta = Kp_ang(2) * e_theta + Kd_ang(2) * e_theta_dot + Ki_ang(2) * integral_error_theta;

% ---------- PSI ANGLE ERRORS (ALONG THE Z AXIS) 
e_psi = ref_ang_pos(3) - psi;                       %PROPORTIONAL ERROR
e_psi_dot = ref_ang_vel(3) - psi_dot;               %DERIVATIVE ERROR


persistent integral_error_psi;                      %INTEGRAL ERROR INITIAL.
if isempty(integral_error_psi)
    integral_error_psi = 0;
end

integral_error_psi = integral_error_psi + e_psi * dt; %INTEGRAL ERROR


% ++++++++++ VARIATIONS: UPPER BOUND FOR INTEGRAL ERROR FOR PSI
% These are things COMMENTED OUT but kept on the code just in case needed
%
%
% integral_error_psi = saturate(integral_error_psi,-integral_max,integral_max);
%-------------------------------------------------------------------------

% ---------- CONTROL PSI (YAW, ALONG Z-AXIS) WITH A PID
tau_psi = Kp_ang(3) * e_psi + Kd_ang(3) * e_psi_dot + Ki_ang(3) * integral_error_psi;

end
%UTILITY FUNCTION TO SATURATE SOME VARIABLES
function val_sat = saturate(val, min_val, max_val)
    val_sat = max(min(val, max_val), min_val);
end
