% Start Coppelia connection
sim = remApi('remoteApi');
sim.simxFinish(-1);
clientID = sim.simxStart('127.0.0.1', 19997, true, true, 5000, 5);

if (clientID > -1)
    disp('Connected to CoppeliaSim');
    
    % Get quadcopter handle with error checking
    [res, quadHandle] = sim.simxGetObjectHandle(clientID, 'Quadcopter', sim.simx_opmode_blocking);
    if (res ~= sim.simx_return_ok)
        disp('ERROR: Cannot find "Quadcopter" object in CoppeliaSim');
        disp('Check that the object name matches exactly in Scene Hierarchy');
        sim.simxFinish(clientID);
        sim.delete();
        return;
    end

    [res, platformHandle] = sim.simxGetObjectHandle(clientID, 'OmniPlatform', sim.simx_opmode_blocking);
    if (res ~= sim.simx_return_ok)
        disp('ERROR: Cannot find "OmniPlatform" object in CoppeliaSim');
        sim.simxFinish(clientID);
        sim.delete();
        return;
    end

    
    % Start simulation in CoppeliaSim
    [res] = sim.simxStartSimulation(clientID, sim.simx_opmode_blocking);
    sim.simxSetBooleanParameter(clientID, sim.sim_boolparam_realtime_simulation, false, sim.simx_opmode_oneshot);
    sim.simxSetFloatingParameter(clientID, sim.sim_floatparam_simulation_time_step, 0.01, sim.simx_opmode_oneshot);
    if (res ~= sim.simx_return_ok)
        disp('ERROR: Failed to start CoppeliaSim simulation');
        sim.simxFinish(clientID);
        sim.delete();
        return;
    end
    
    % Get simulation data and handle 3D array format
    data_raw = out.quadrotor_states.signals.values;
    time = out.quadrotor_states.time;
    
    % Convert 3D array [1,6,N] to 2D array [N,6] 
    if ndims(data_raw) == 3
        data = squeeze(data_raw)'; % Convert [1,6,N] to [N,6]
        fprintf('Converted 3D data to 2D format\n');
    else
        data = data_raw;
    end
    
    % DEBUG: Check trajectory data
    fprintf('=== TRAJECTORY DEBUG INFO ===\n');
    fprintf('Data size: %dx%d\n', size(data,1), size(data,2));
    fprintf('Time range: %.2f to %.2f seconds\n', time(1), time(end));
    fprintf('Start position: [%.3f, %.3f, %.3f]\n', data(1,1), data(1,2), data(1,3));
    fprintf('End position: [%.3f, %.3f, %.3f]\n', data(end,1), data(end,2), data(end,3));
    fprintf('Max position: [%.3f, %.3f, %.3f]\n', max(data(:,1)), max(data(:,2)), max(data(:,3)));
    fprintf('Min position: [%.3f, %.3f, %.3f]\n', min(data(:,1)), min(data(:,2)), min(data(:,3)));
    fprintf('=============================\n');
    
    % Calculate proper timing
    dt_sim = time(2) - time(1);
    step = 6000; % Increase step size to reduce data points and prevent looping effect
    
    fprintf('Starting visualization: %d total points, using every %dth point\n', length(time), step);
    
    % Replay trajectory with interpolation for smooth visualization
    tic;
    for i = 1:step:length(time)-step
        if i+step <= size(data, 1)
            % Linear interpolation between points
            for j = 0:10
                alpha = j/10;
                % --- UAV interpolation ---
                interp_pos = (1-alpha)*data(i,1:3) + alpha*data(i+step,1:3);
                interp_ang = (1-alpha)*data(i,4:6) + alpha*data(i+step,4:6);
            
                sim.simxSetObjectPosition(clientID, quadHandle, -1, interp_pos, sim.simx_opmode_oneshot);
                sim.simxSetObjectOrientation(clientID, quadHandle, -1, interp_ang, sim.simx_opmode_oneshot);
            
                % --- Platform update (sync with time) ---
                t_now = (1-alpha)*time(i) + alpha*time(i+step);
                [platform_pos, ~] = platform_trajectory(t_now);
                sim.simxSetObjectPosition(clientID, platformHandle, -1, platform_pos(:)', sim.simx_opmode_oneshot);
            
                pause(0.01);
            end
            
            if mod(i, 6000) == 0
                fprintf('Progress: %.1f%%\n', 100*i/length(time));
            end
        end
    end
    
    fprintf('Visualization completed in %.2f seconds\n', toc);
    
    % Stop simulation
    sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);
    sim.simxFinish(clientID);
    sim.delete();
else
    disp('FAILED to connect to CoppeliaSim');
    disp('Check: 1) CoppeliaSim running 2) Scene loaded 3) Legacy Remote API enabled');
end