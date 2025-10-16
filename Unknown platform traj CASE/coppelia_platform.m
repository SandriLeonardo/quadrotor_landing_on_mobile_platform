function [sys,x0,str,ts,simStateCompliance] = coppelia_platform(t,x,u,flag,varargin)
% S-Function per inviare posizione platform a CoppeliaSim
% 
% INPUT: [px, py, pz] - posizione platform da platform_trajectory.m
% OUTPUT: [] (nessuno, solo side-effect in CoppeliaSim)

persistent sim clientID platformHandle connected

switch flag
    case 0  % Initialization
        sizes = simsizes;
        sizes.NumContStates  = 0;
        sizes.NumDiscStates  = 0;
        sizes.NumOutputs     = 0;  % Nessun output
        sizes.NumInputs      = 3;  % [px, py, pz]
        sizes.DirFeedthrough = 1;
        sizes.NumSampleTimes = 1;
        
        sys = simsizes(sizes);
        x0  = [];
        str = [];
        
        % Sample time
        if nargin >= 5 && ~isempty(varargin{1})
            sample_time = varargin{1};
        else
            sample_time = 0.01;  % 100Hz
        end
        ts = [sample_time 0];
        simStateCompliance = 'UnknownSimState';
        
        % Connect to CoppeliaSim
        try
            fprintf('[Platform] Connecting to CoppeliaSim...\n');
            sim = remApi('remoteApi');
            sim.simxFinish(-1);
            clientID = sim.simxStart('127.0.0.1', 19997, true, true, 5000, 5);
            
            if clientID > -1
                % Get platform handle
                [res, platformHandle] = sim.simxGetObjectHandle(clientID, ...
                    'OmniPlatform', sim.simx_opmode_blocking);
                
                if res ~= sim.simx_return_ok
                    error('Cannot find Platform object in CoppeliaSim');
                end
                
                connected = true;
                fprintf('[Platform] Connected, handle: %d\n', platformHandle);
            else
                error('Cannot connect to CoppeliaSim');
            end
        catch ME
            fprintf('[ERROR Platform] %s\n', ME.message);
            connected = false;
        end
        
    case 3  % Outputs
        if connected && ~isempty(u) && length(u) >= 3
            % Invia posizione platform a CoppeliaSim
            sim.simxSetObjectPosition(clientID, platformHandle, -1, ...
                u(1:3), sim.simx_opmode_oneshot);
        end
        sys = [];  % Nessun output
        
    case 9  % Termination
        if connected
            fprintf('[Platform] Disconnecting...\n');
            sim.simxFinish(clientID);
        end
        sys = [];
        
    otherwise
        sys = [];
end
end