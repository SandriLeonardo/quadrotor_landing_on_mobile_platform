function [sys,x0,str,ts,simStateCompliance] = coppelia_visualizer(t,x,u,flag,varargin)

persistent fig_handle

switch flag
    case 0  % Initialization
        if nargin >= 5 && ~isempty(varargin{1})
            sample_time = varargin{1}(1);
        else
            sample_time = 0.5;  % Update every 0.5s
        end
        
        sizes = simsizes;
        sizes.NumContStates = 0;
        sizes.NumDiscStates = 0;
        sizes.NumOutputs = 0;
        sizes.NumInputs = 3;  %Coppelia Detection Flag + FOV + Kalman Initialized Flag
        sizes.DirFeedthrough = 1;
        sizes.NumSampleTimes = 1;
        
        sys = simsizes(sizes);
        x0 = [];
        str = [];
        ts = [sample_time 0];
        simStateCompliance = 'UnknownSimState';
        
    case 3  % Update
        detected = u(1);
        fov = u(2);
        kalman_initialized = u(3);
        
        % Check if data exists
        try
            debug_imgs = evalin('base', 'camera_debug');
        catch
            sys = [];
            return;  % Skip visualization if no data yet
        end
        
        if isempty(fig_handle) || ~ishandle(fig_handle)
            fig_handle = figure('Name', 'Landing Platform Detection', ...
                'NumberTitle', 'off', 'Position', [100, 100, 1200, 400]);
        end
        
        figure(fig_handle);
        subplot(1, 3, 1); 
        imshow(debug_imgs.original); 
        title('Original Image');
        
        subplot(1, 3, 2); 
        imshow(debug_imgs.binary); 
        title('Binary Image');
        
        subplot(1, 3, 3); 
        imshow(debug_imgs.detection);
        if detected
            title({'Detection: SUCCESS', sprintf('FOV: %.1f°', fov)}, 'Color', 'green');
        else
            title({'Detection: FAILED', sprintf('FOV: %.1f°', fov)}, 'Color', 'red');
            if kalman_initialized
                xlabel('Using Kalman Filter Prediction', 'Color', 'yellow', 'FontSize', 10);
            end
        end
        drawnow;
        
        sys = [];
        
    case 9  % Termination
        if ishandle(fig_handle)
            close(fig_handle);
        end
        sys = [];
        
    otherwise
        sys = [];
end