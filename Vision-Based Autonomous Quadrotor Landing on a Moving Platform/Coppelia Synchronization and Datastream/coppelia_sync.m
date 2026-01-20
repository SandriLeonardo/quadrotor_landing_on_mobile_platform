function [sys,x0,str,ts,simStateCompliance] = coppelia_sync(t,x,u,flag,varargin)
%==========================================================================
% COPPELIA_SYNC - S-Function for CoppeliaSim Synchronization
%==========================================================================
% PURPOSE:
%   Synchronizes Simulink simulation with CoppeliaSim visualization in
%   real-time. Updates quadrotor and platform positions at each timestep.
%
% INPUTS (6 elements):
%   u = [x, y, z, roll, pitch, yaw] - Quadrotor state from Simulink
%       [px, py, pz]                - Platform position from Simulink
%
% OUTPUTS (1 element):
%   sys = connection_status - 1 if connected to CoppeliaSim, 0 otherwise
%
% PARAMETERS (varargin):
%   [sample_time, enable_sync, enable_logging]
%   - sample_time: Update rate in seconds (default: 0.05s)
%   - enable_sync: 1=synchronous mode, 0=async
%   - enable_logging: 1=print debug info, 0=silent
%
%==========================================================================

%--------------------------------------------------------------------------
% PERSISTENT VARIABLES
%--------------------------------------------------------------------------
% These variables maintain their values between function calls.

persistent sim                % CoppeliaSim Remote API object
persistent clientID           % Connection ID returned by CoppeliaSim
persistent quadHandle         % Handle to Quadcopter object in scene
persistent platformHandle     % Handle to OmniPlatform object in scene
persistent connected          % Boolean: true if connected to CoppeliaSim
persistent enable_logging     % Boolean: true if debug logging enabled
persistent enable_sync        % Boolean: true if synchronous mode enabled
persistent frame_count        % Counter for number of frames processed

%--------------------------------------------------------------------------
% MAIN SWITCH
%--------------------------------------------------------------------------
% Standard Simulink calls this function with different 'flag' values to control
% S-Function behavior at different stages of simulation.
%
% flag = 0: INITIALIZATION - Called once at start
% flag = 3: OUTPUTS - Called every timestep to compute outputs
% flag = 9: TERMINATION - Called once at end to cleanup

switch flag
    
    case 0 %INITIALIZATION

        %======================================================================
        % CASE 0: INITIALIZATION
        %======================================================================
        % Called ONCE when simulation starts. 
        %------------------------------------------------------------------




        %------------------------------------------------------------------
        % S-FUNCTION PARAMETERS EXTRACTION
        %------------------------------------------------------------------
        % Extracts users parameters from simulink block dialog or sets defaults
        
        if nargin >= 5 && ~isempty(varargin{1})
            sample_time = varargin{1}(1);                                   %first param
            enable_sync = (length(varargin{1}) >= 2) && varargin{1}(2);     %if second param exists
            enable_logging = (length(varargin{1}) >= 3) && varargin{1}(3);  %if third param exists
        else
            
            sample_time = 0.05;      % 20Hz update rate
            enable_sync = 1;         % Synchronous mode ON
            enable_logging = 0;      % Logging OFF
        end

        %------------------------------------------------------------------
        % DEFINE S-FUNCTION STRUCTURE
        %------------------------------------------------------------------
        % Tell Simulink about this S-Function's characteristics
        
        sizes = simsizes;   %Use the built-in simsizes function, this gives us a structure to fill in
    
        sizes.NumContStates  = 0; % No continuous states, it means there are no differential equations to solve
        sizes.NumDiscStates  = 0; % No discrete states, it means there is no memory/state
        sizes.NumOutputs     = 1; % One output expected: connection status (1=connected, 0=not)
        sizes.NumInputs      = 9; % Six inputs expected: quadrotor state [x,y,z,roll,pitch,yaw] platform state [x,y,z]
        
        % Direct feedthrough: output depends on current input 'u'
        % It's important that this is set to 1, so that Simulink, executes
        % the OUTPUTS function only after the input 'u' is available, otherwise
        % the quadrotor may get NaN values

        sizes.DirFeedthrough = 1;
        sizes.NumSampleTimes = 1; % One sample time (discrete)
        
        % Assign to system output
        sys = simsizes(sizes);                  % Convert our sizes structure to a vector
        x0  = [];                               % No initial states
        str = [];                               % No state names
        ts  = [sample_time 0];                  % Period, offset (start at t=0)
        simStateCompliance = 'UnknownSimState'; % No SimState compliance
        
        
        frame_count = 0;                        % Initialize frame counter
        
        %------------------------------------------------------------------
        % COPPELIA SIM CONNECTION SETUP
        %------------------------------------------------------------------
        % Establish connection to CoppeliaSim via Legacy Remote API
        
        fprintf('\n[CoppeliaSim Sync] Connecting...\n'); % Start connection process
        
        try
            
            sim = remApi('remoteApi'); % Create Remote API object
            sim.simxFinish(-1);        % Clear any existing connections
            
            % Attempt connection to CoppeliaSim
            % Parameters: (IP, port, waitUntilConnected, doNotReconnect, timeout, cycle)
            clientID = sim.simxStart('127.0.0.1', 19997, true, true, 5000, 5); %19997 is default port for legacy API

            
            % Check if connection successful
            if clientID > -1
                
                %----------------------------------------------------------
                % GET OBJECT HANDLES
                %----------------------------------------------------------
                % Retrieve handles used to identify objects in API calls.
                
                % Get handles in BLOCKING MODE, that means the function
                % will wait until the handle is retrieved before continuing.
                [res_quad, quadHandle] = sim.simxGetObjectHandle(clientID, ...
                    'Quadcopter', sim.simx_opmode_blocking);
                [res_plat, platformHandle] = sim.simxGetObjectHandle(clientID, ...
                    'OmniPlatform', sim.simx_opmode_blocking);

                % res_quad and res_plat contain the return code of the functions
                % sim.simx_return_ok (is 0) and indicates success
                
                % Check if handles were retrieved successfully
                if res_quad ~= sim.simx_return_ok || res_plat ~= sim.simx_return_ok
                    error('Cannot find objects in scene');
                end
                
                %----------------------------------------------------------
                % START COPPELIA SIM SIMULATION
                %----------------------------------------------------------
                % Start the simulation in CoppeliaSim. This begins the
                % physics and rendering loop.
                
                sim.simxStartSimulation(clientID, sim.simx_opmode_blocking); % Blocking mode
                pause(0.5);  %delay to ensure simulation starts properly
                
                %----------------------------------------------------------
                % CONFIGURE SIMULATION PARAMETERS
                %----------------------------------------------------------
                
                % Disable real-time mode (run as fast as possible)
                % The real time mode is not good if we take time to compute
                % things in Simulink, better to run as fast as possible 
                % if real-time is disabled, we control the timing from Simulink
                sim.simxSetBooleanParameter(clientID, ...
                    sim.sim_boolparam_realtime_simulation, false, ...
                    sim.simx_opmode_oneshot);

                
                % Set simulation timestep to 10ms (100Hz internal)
                sim.simxSetFloatingParameter(clientID, ...
                    sim.sim_floatparam_simulation_time_step, 0.01, ...
                    sim.simx_opmode_oneshot);


                % Store in workspace BEFORE camera tries to connect
                assignin('base', 'COPPELIA_SIM', sim);                  % Remote API object
                assignin('base', 'COPPELIA_CLIENTID', clientID);    
                assignin('base', 'COPPELIA_QUADHANDLE', quadHandle);
                assignin('base', 'COPPELIA_CONNECTED', true);
                                
                %----------------------------------------------------------
                % ENABLE SYNCHRONOUS MODE
                %----------------------------------------------------------
                % Synchronous mode makes CoppeliaSim wait for Simulink.
                % Each Simulink timestep triggers exactly one CoppeliaSim step.
                
                if enable_sync
                    sim.simxSynchronous(clientID, true); % Enable synchronous mode
                    fprintf('[INFO] Synchronous mode enabled\n');
                end
                
                connected = true; % Mark as connected
                fprintf('[OK] Connected! (ClientID: %d, Quad: %d, Platform: %d)\n', ...
                    clientID, quadHandle, platformHandle);
                
            else
                error('Connection failed');
            end
            
        catch ME
            fprintf('[ERROR] %s\n', ME.message);
            connected = false;
        end
        

    
    case 3 %OUTPUTS

        %======================================================================
        % CASE 3: OUTPUTS
        %======================================================================
        % Called EVERY TIMESTEP during simulation.
        %
        % Read input 'u' (quadrotor state from Simulink)
        % Update quadrotor position/orientation in CoppeliaSim
        % Update platform position in CoppeliaSim
        % Trigger synchronous simulation step
        % Return connection status
        
        if connected 
            try
                frame_count = frame_count + 1; % Increment frame counter
                
                % READ INPUT and UPDATE QUADROTOR POSITION/ORIENTATION
                if ~isempty(u) && length(u) >= 9
                    pos = reshape(u(1:3), 1, 3); % Position: [x, y, z]
                    ori = reshape(u(4:6), 1, 3); % Orientation: [roll, pitch, yaw]
                    
                    sim.simxSetObjectPosition(clientID, quadHandle, -1, pos, sim.simx_opmode_oneshot);
                    sim.simxSetObjectOrientation(clientID, quadHandle, -1, ori, sim.simx_opmode_oneshot);
                    
                    if enable_logging && mod(frame_count, 100) == 0
                        fprintf('[Frame %d] t=%.2f | Quad: [%.2f, %.2f, %.2f]\n', frame_count, t, pos);
                    end
                end
                
                % READ INPUT and UPDATE PLATFORM POSITION
                if ~isempty(u) && length(u) >= 9
                    plat_pos = reshape(u(7:9), 1, 3); % Platform position: [x, y, z]
                    
                    sim.simxSetObjectPosition(clientID, platformHandle, -1, plat_pos, sim.simx_opmode_oneshot);
                    
                    if enable_logging && mod(frame_count, 100) == 0
                        fprintf('[Frame %d] t=%.2f | Platform: [%.2f, %.2f, %.2f]\n', frame_count, t, plat_pos);
                    end
                end
                
                pause(0.001);
                
                % TRIGGER SIMULATION STEP IN SYNCHRONOUS MODE, that means
                % CoppeliaSim will execute one simulation step and wait for
                % the next trigger from Simulink

                if enable_sync
                    sim.simxSynchronousTrigger(clientID);   % Trigger next step
                    sim.simxGetPingTime(clientID);          % Wait for render completion
                end
                
                sys = 1;                                    % Return connection status: 1=connected
                
            catch ME
                fprintf('[ERROR] %s\n', ME.message);
                sys = 0;                                    % Return connection status: 0=not connected
            end
        else
            sys = 0;
        end
        
   
    
    case 9 %TERMINATION
        
        %======================================================================
        % CASE 9: TERMINATION
        %======================================================================
        % Called ONCE when simulation stops. 
        %======================================================================
        
        
        if connected
            fprintf('\n[CoppeliaSim Sync] Disconnecting (frames: %d)...\n', frame_count);
            
            try
                % Stop the CoppeliaSim simulation
                sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);
                pause(0.5);  % Delay for simulation to fully stop
                sim.simxFinish(clientID); % Close connection
                fprintf('[OK] Disconnected\n');
            catch ME
                fprintf('[WARNING] %s\n', ME.message);
            end
            
            % Clear shared variables from workspace
            % This prevents coppelia_camera from trying to use a
            % closed connection if simulation runs again
            evalin('base', 'clear COPPELIA_SIM COPPELIA_CLIENTID COPPELIA_QUADHANDLE COPPELIA_CONNECTED');
            connected = false;
        end
        
        % Return empty (no output needed on termination)
        sys = []; 
        
    %======================================================================
    % DEFAULT CASE
    %======================================================================
    % Handle any other flag values (case 2 for discrete state update, etc.)
    % We don't use these, so return empty.
    
    otherwise
        sys = []; 
end

end