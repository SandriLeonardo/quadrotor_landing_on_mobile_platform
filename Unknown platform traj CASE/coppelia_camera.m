function [sys,x0,str,ts,simStateCompliance] = coppelia_camera(t,x,u,flag,varargin)
% S-Function per acquisizione immagini e detection platform da CoppeliaSim
%
% OUTPUTS: [detection_flag, px, py, pz, quad_x, quad_y, quad_z] - 7 valori
%   - detection_flag: 1 se platform rilevata, 0 altrimenti
%   - px, py, pz: posizione platform in world frame [m]
%   - quad_x, quad_y, quad_z: posizione quadrotor (per logging)
%
% PARAMETERS in Simulink S-Function block:
%   S-function parameters: [sample_time, enable_sync, enable_logging]
%   Example: [0.05, 1, 0]  → 20Hz, sync mode ON, logging OFF
%
% USAGE:
%   1. Aggiungi S-Function block in Simulink
%   2. S-function name: 'coppelia_camera_sfunc'
%   3. S-function parameters: [0.05, 1, 0]
%   4. Output del blocco: 7 elementi → usa Demux(7)

%% Persistent variables (mantengono stato tra chiamate)
persistent sim clientID camHandle quadHandle connected
persistent init_done frame_count last_detection_time
persistent enable_logging enable_sync

%% Main switch per gestione flags Simulink
switch flag
    case 0  % ==================== INITIALIZATION ====================
        % Parse parameters
        if nargin >= 5 && ~isempty(varargin{1})
            sample_time = varargin{1}(1);
            if length(varargin{1}) >= 2
                enable_sync = varargin{1}(2);
            else
                enable_sync = 1;  % Default: sync mode
            end
            if length(varargin{1}) >= 3
                enable_logging = varargin{1}(3);
            else
                enable_logging = 0;  % Default: no logging
            end
        else
            sample_time = 0.05;
            enable_sync = 1;
            enable_logging = 0;
        end
        
        % Setup sizes structure
        sizes = simsizes;
        sizes.NumContStates  = 0;
        sizes.NumDiscStates  = 0;
        sizes.NumOutputs     = 7;  % [detected, px, py, pz, qx, qy, qz]
        sizes.NumInputs      = 12;
        sizes.DirFeedthrough = 0;
        sizes.NumSampleTimes = 1;
        
        sys = simsizes(sizes);
        x0  = [];
        str = [];
        ts  = [sample_time 0];  % Discrete sample time
        simStateCompliance = 'UnknownSimState';
        
        % Initialize counters
        frame_count = 0;
        last_detection_time = 0;
        init_done = false;
        
        % Connect to CoppeliaSim
        fprintf('\n========================================\n');
        fprintf('[CoppeliaSim Camera] Initialization\n');
        fprintf('========================================\n');
        
        try
            % Initialize Remote API
            sim = remApi('remoteApi');
            sim.simxFinish(-1);  % Close all opened connections
            
            fprintf('[INFO] Connecting to CoppeliaSim (127.0.0.1:19997)...\n');
            clientID = sim.simxStart('127.0.0.1', 19997, true, true, 5000, 5);
            
            if clientID > -1
                fprintf('[OK] Connected! ClientID: %d\n', clientID);
                
                % Get object handles
                fprintf('[INFO] Getting object handles...\n');
                [res_cam, camHandle] = sim.simxGetObjectHandle(clientID, ...
                    'visionSensor', sim.simx_opmode_blocking);
                [res_quad, quadHandle] = sim.simxGetObjectHandle(clientID, ...
                    'Quadcopter', sim.simx_opmode_blocking);
                
                if res_cam ~= sim.simx_return_ok
                    error('Cannot find visionSensor object in CoppeliaSim scene');
                end
                if res_quad ~= sim.simx_return_ok
                    error('Cannot find Quadcopter object in CoppeliaSim scene');
                end
                fprintf('[OK] Handles obtained (cam=%d, quad=%d)\n', camHandle, quadHandle);
                
                % Initialize streaming mode for camera
                fprintf('[INFO] Initializing camera streaming...\n');
                sim.simxGetVisionSensorImage(clientID, camHandle, 0, sim.simx_opmode_streaming);
                sim.simxGetObjectPosition(clientID, quadHandle, -1, sim.simx_opmode_streaming);
                sim.simxGetObjectOrientation(clientID, quadHandle, -1, sim.simx_opmode_streaming);
                pause(0.1);  % Wait for buffer initialization
                
                % Enable synchronous mode if requested
                if enable_sync
                    fprintf('[INFO] Enabling synchronous mode...\n');
                    sim.simxSynchronous(clientID, true);
                end
                
                % Start simulation
                fprintf('[INFO] Starting simulation...\n');
                sim.simxStartSimulation(clientID, sim.simx_opmode_blocking);
                pause(0.5);
                
                connected = true;
                init_done = true;
                fprintf('[OK] Camera S-Function ready!\n');
                fprintf('========================================\n\n');
            else
                error('Failed to connect to CoppeliaSim on port 19997');
            end
            
        catch ME
            fprintf('[ERROR] Initialization failed: %s\n', ME.message);
            fprintf('        Make sure CoppeliaSim is running with scene loaded\n');
            connected = false;
            init_done = false;
        end
        
    case 2  % ==================== UPDATE ====================
        % Discrete state update (non usato qui)
        sys = [];
        
    case 3  % ==================== OUTPUTS ====================
        if connected && init_done
            try
                % Trigger sync
                if enable_sync
                    sim.simxSynchronousTrigger(clientID);
                    sim.simxGetPingTime(clientID);  % Wait completion
                end
                
                frame_count = frame_count + 1;

                % Update quad pose in CoppeliaSim
                if ~isempty(u) && length(u) >= 6
                    sim.simxSetObjectPosition(clientID, quadHandle, -1, u(1:3), sim.simx_opmode_oneshot);
                    sim.simxSetObjectOrientation(clientID, quadHandle, -1, u(4:6), sim.simx_opmode_oneshot);
                end
                
                % Get quadrotor position
                [res_pos, quad_pos] = sim.simxGetObjectPosition(clientID, quadHandle, -1, sim.simx_opmode_buffer);
                
                % Get camera image
                [res_img, resolution, imagePtr] = sim.simxGetVisionSensorImage(clientID, camHandle, 0, sim.simx_opmode_buffer);
                
                if res_img == sim.simx_return_ok && ~isempty(resolution)
                    % Dereference pointer (workaround Legacy API)
                    w = resolution(1);
                    h = resolution(2);
                    setdatatype(imagePtr, 'uint8Ptr', w*h*3);
                    imageData = imagePtr.Value;
                    
                    % Reshape to image matrix [H x W x 3]
                    img = reshape(uint8(imageData), [w, h, 3]);
                    img = permute(img, [2, 1, 3]);
                    img = flip(img, 1);

                    % Setup camera parameters (reference in coppeliasim
                    % visionsensor)
                    cam_params.fx = 222;
                    cam_params.fy = 222;
                    cam_params.cx = 128;
                    cam_params.cy = 128;
                    cam_params.tag_size = 0.4;
                    
                    % Platform detection (Algorithm 1 from paper)
                    [detected, platform_pos_camera] = detect_platform(img, cam_params);
                    
                    if detected && res_pos == sim.simx_return_ok
                        % Transform from camera frame to world frame
                        % Get quad orientation
                        [~, quad_orient] = sim.simxGetObjectOrientation(clientID, quadHandle, -1, sim.simx_opmode_buffer);
                        phi = quad_orient(1); theta = quad_orient(2); psi = quad_orient(3);
                        
                        % Rotation body → world
                        R_WB = eul2rotm([psi, theta, phi], 'ZYX');
                        
                        % Camera tilt 45° (pitch)
                        R_BC = eul2rotm([0, deg2rad(45), 0], 'ZYX');
                        
                        % Transform
                        platform_pos_world = quad_pos' + R_WB * R_BC * platform_pos_camera';
                        
                        sys = [1, platform_pos_world, quad_pos'];
                        last_detection_time = frame_count;
                        
                        if enable_logging
                            fprintf('[%d] Platform detected at [%.2f, %.2f, %.2f]\n', frame_count, platform_pos_world);
                        end
                    else
                        % No detection
                        sys = [0, 0, 0, 0, quad_pos'];
                    end
                else
                    % Image acquisition failed
                    sys = [0, 0, 0, 0, 0, 0, 0];
                    if enable_logging && mod(frame_count, 100) == 0
                        fprintf('[WARNING] No image data (frame %d)\n', frame_count);
                    end
                end
                
            catch ME
                fprintf('[ERROR] Camera acquisition failed: %s\n', ME.message);
                sys = [0, 0, 0, 0, 0, 0, 0];
            end
        else
            % Not connected, output zeros
            sys = [0, 0, 0, 0, 0, 0, 0];
        end
        
    case 9  % ==================== TERMINATION ====================
        if connected
            fprintf('\n[CoppeliaSim] Shutting down...\n');
            fprintf('  Total frames processed: %d\n', frame_count);
            fprintf('  Detections: %d\n', last_detection_time);
            
            try
                sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);
                pause(0.5);
                sim.simxFinish(clientID);
                fprintf('[OK] Disconnected cleanly\n');
            catch ME
                fprintf('[WARNING] Cleanup error: %s\n', ME.message);
            end
            
            connected = false;
        end
        sys = [];
        
    otherwise
        sys = [];
end

end