function [sys,x0,str,ts,simStateCompliance] = coppelia_kalman(t,x,u,flag,varargin)

persistent x_hat P initialized
persistent F H Q R sample_time enable_logging frame_count



switch flag
    
    case 0  % Initialization
        if nargin >= 5 && ~isempty(varargin{1})
            sample_time = varargin{1}(1);
            q_pos = (length(varargin{1}) >= 2) && varargin{1}(2) || 0.01;
            q_vel = (length(varargin{1}) >= 3) && varargin{1}(3) || 0.1;
            r_meas = (length(varargin{1}) >= 4) && varargin{1}(4) || 0.05;
            enable_logging = (length(varargin{1}) >= 5) && varargin{1}(5);
        else
            sample_time = 0.05;
            q_pos = 0.1;
            q_vel = 1.0;
            r_meas = 0.05;
            enable_logging = 0;
        end
        
        dt = sample_time;
        F = [1 0 0 dt 0  0;
             0 1 0 0  dt 0;
             0 0 1 0  0  dt;
             0 0 0 1  0  0;
             0 0 0 0  1  0;
             0 0 0 0  0  1];
        
        H = [1 0 0 0 0 0;
             0 1 0 0 0 0;
             0 0 1 0 0 0];
        
        Q = diag([q_pos, q_pos, q_pos, q_vel, q_vel, q_vel]);
        R = diag([r_meas, r_meas, r_meas]);
        
        x_hat = zeros(6, 1);
        P = eye(6) * 10;
        initialized = false;

        frame_count = 0;
        
        sizes = simsizes;
        sizes.NumContStates = 0;
        sizes.NumDiscStates = 0;
        sizes.NumOutputs = 8;
        sizes.NumInputs = 5;
        sizes.DirFeedthrough = 1;
        sizes.NumSampleTimes = 1;
        
        sys = simsizes(sizes);
        x0 = [];
        str = [];
        ts = [sample_time 0];
        simStateCompliance = 'UnknownSimState';
        
        fprintf('[Kalman Filter] Initialized (dt=%.3f, Q_pos=%.3f, Q_vel=%.3f, R=%.3f)\n', ...
            sample_time, q_pos, q_vel, r_meas);
        
    case 3  % Output
        frame_count = frame_count + 1;
        
        z_meas = u(1:3);
        detected = u(4);
        %fov = u(5); % Not used in Kalman filter
        
        if enable_logging && mod(frame_count, 20) == 0
            fprintf('[KALMAN] Frame %d, detected=%d\n', frame_count, detected);
        end
        
        % Initialize on first detection
        if ~initialized && detected == 1
            x_hat = [z_meas; 0; 0; 0];
            P = eye(6) * 10;
            initialized = true;
            fprintf('[Kalman] INITIALIZED at [%.3f, %.3f, %.3f]\n', ...
                z_meas(1), z_meas(2), z_meas(3));
        end
        
        if ~initialized
            sys = [0; 0; 0; 0; 0; 0; 0; 0];
            return;
        end
        
        % Prediction
        x_hat_pred = F * x_hat;
        P_pred = F * P * F' + Q;
        
        if enable_logging && detected == 1
            fprintf('  [PREDICT] pos=[%.3f, %.3f, %.3f], vel=[%.3f, %.3f, %.3f]\n', ...
                x_hat_pred(1), x_hat_pred(2), x_hat_pred(3), ...
                x_hat_pred(4), x_hat_pred(5), x_hat_pred(6));
        end
        
        % Update
        if detected == 1
            y = z_meas - H * x_hat_pred;
            S = H * P_pred * H' + R;
            K = P_pred * H' / S;
            x_hat = x_hat_pred + K * y;
            P = (eye(6) - K * H) * P_pred;
            
            if enable_logging
                fprintf('  [UPDATE] innovation=[%.3f, %.3f, %.3f]\n', y(1), y(2), y(3));
                fprintf('  [CORRECTED] pos=[%.3f, %.3f, %.3f], vel=[%.3f, %.3f, %.3f]\n', ...
                    x_hat(1), x_hat(2), x_hat(3), ...
                    x_hat(4), x_hat(5), x_hat(6));
            end
        else
            x_hat = x_hat_pred;
            P = P_pred;
            if enable_logging
                fprintf('  [NO UPDATE] Using prediction only\n');
            end
        end
        
        sys = [x_hat; detected;initialized];
        
    case 9
        fprintf('[Kalman Filter] Terminated (frames: %d)\n', frame_count);
        sys = [];
        
    otherwise
        sys = [];
end

end