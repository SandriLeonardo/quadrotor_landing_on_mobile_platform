function [cspar, ccpar, ckpar] = s_params()
%INITIALIZES S-FUNCTION PARAMETERS

    %Coppelia_Sync parameters
    cs_sample_time = 0.05;
    cs_enable_sync = 1;
    cs_enable_logging = 1;
    
    %Coppelia_Camera parameters
    cc_sample_time = 0.05;
    cc_tag_size = 0.4;
    cc_enable_logging = 1;
    cc_enable_adaptive_focus = 1;
    
    %Coppelia Kalman parameters
    ck_sample_time = 0.05;
    ck_q_pos = 0.1;
    ck_q_vel = 1.0;
    ck_r_meas = 0.05;
    ck_enable_logging = 1;
    
    cspar = [cs_sample_time, cs_enable_sync, cs_enable_logging];
    ccpar = [cc_sample_time, cc_tag_size, cc_enable_logging, cc_enable_adaptive_focus];
    ckpar = [ck_sample_time, ck_q_pos, ck_q_vel, ck_r_meas, ck_enable_logging];

    % Assign to base workspace
    assignin('base', 'cspar', cspar);
    assignin('base', 'ccpar', ccpar);
    assignin('base', 'ckpar', ckpar);
end