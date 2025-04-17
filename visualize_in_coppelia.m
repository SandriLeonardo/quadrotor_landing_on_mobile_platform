% Load simulation results
% (Save position and orientation data)

% Start Coppelia connection
sim = remApi('remoteApi');
sim.simxFinish(-1);
clientID = sim.simxStart('127.0.0.1', 19997, true, true, 5000, 5);

if (clientID > -1)
    % Get quadcopter handle 
    [res, quadHandle] = sim.simxGetObjectHandle(clientID, 'Quadcopter', sim.simx_opmode_blocking);

    % Start simulation in CoppeliaSim
    sim.simxStartSimulation(clientID, sim.simx_opmode_blocking);

        % Replay trajectory
    for i = 1:length(quadrotor_states.time)
        position = quadrotor_states.signals.values(i, 1:3);
        orientation = quadrotor_states.signals.values(i, 4:6);
        
        % Send to CoppeliaSim
        sim.simxSetObjectPosition(clientID, quadHandle, -1, position, sim.simx_opmode_oneshot);
        sim.simxSetObjectOrientation(clientID, quadHandle, -1, orientation, sim.simx_opmode_oneshot);
        
        % Add delay to match simulation rate
        pause(0.01); % Adjust based on simulation step
    end

    % Stop simulation
    sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);
end