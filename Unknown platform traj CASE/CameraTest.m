% WORKING CAMERA SOLUTION - Dereference pointer
clear; clc; close all;

sim = remApi('remoteApi');
sim.simxFinish(-1);
clientID = sim.simxStart('127.0.0.1', 19997, true, true, 5000, 5);

[~, camHandle] = sim.simxGetObjectHandle(clientID, 'visionSensor', sim.simx_opmode_blocking);
sim.simxStartSimulation(clientID, sim.simx_opmode_blocking);
pause(2);

% Initialize streaming
sim.simxGetVisionSensorImage(clientID, camHandle, 0, sim.simx_opmode_streaming);
pause(1);

% Capture and display
fprintf('Capturing images...\n');
figure('Name', 'Camera Feed');

for frame = 1:50
    [res, resolution, imagePtr] = sim.simxGetVisionSensorImage(clientID, camHandle, 0, sim.simx_opmode_buffer);
    
    if res == sim.simx_return_ok && isa(imagePtr, 'lib.pointer')
        % Dereference pointer
        w = resolution(1);
        h = resolution(2);
        setdatatype(imagePtr, 'uint8Ptr', w*h*3);
        imageData = imagePtr.Value;
        
        % Reshape to image
        img = reshape(uint8(imageData), [w, h, 3]);
        img = permute(img, [2, 1, 3]);
        img = flip(img, 1);
        
        % Display
        imshow(img);
        title(sprintf('Frame %d/50', frame));
        drawnow;
    end
    
    pause(0.05);
end

fprintf('✓ Camera working!\n');

sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);
sim.simxFinish(clientID);
sim.delete();