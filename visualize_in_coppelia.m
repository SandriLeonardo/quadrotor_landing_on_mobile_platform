% Start Coppelia connection
sim = remApi('remoteApi');
sim.simxFinish(-1);
clientID = sim.simxStart('127.0.0.1', 19997, true, true, 5000, 5);

if (clientID > -1)
    % Get quadcopter handle 
    [res, quadHandle] = sim.simxGetObjectHandle(clientID, 'Quadcopter', sim.simx_opmode_blocking);

    % Start simulation in CoppeliaSim
    sim.simxStartSimulation(clientID, sim.simx_opmode_blocking);

    % Get data dimensions and reshape if needed
    data = out.quadrotor_states.signals.values;
    time = out.quadrotor_states.time;

    % Subsample for visualization (every 100th point)
    step = 50;

    % Replay trajectory
    for i = 1:step:length(time)
        % Check bounds
        if i <= size(data, 1)
            position = data(i, 1:3);
            orientation = data(i, 4:6); 
            % Send to CoppeliaSim
            sim.simxSetObjectPosition(clientID, quadHandle, -1, position, sim.simx_opmode_oneshot);
            sim.simxSetObjectOrientation(clientID, quadHandle, -1, orientation, sim.simx_opmode_oneshot);
            
            % Add delay to make visualization visible
            pause(1);
        end
    end

    % Stop simulation
    sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);
    sim.delete();
end