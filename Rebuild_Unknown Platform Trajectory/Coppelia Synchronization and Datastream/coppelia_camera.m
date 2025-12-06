function [sys,x0,str,ts,simStateCompliance] = coppelia_camera(t,x,u,flag,varargin)
% REMEMBER
% - Image Processing Toolbox is required for some functions (like imfindcircles)

%THINGS THAT COULD BE DONE
% - Initializing in Blocking mode directly instead of streaming then blocking (?) could slow down
% - Obtain quadrotor pose from simulink model instead of querying CoppeliaSim (?) could speed up
% - Adaptive focus should use the vertical distance from quadrotor to platform instead of altitude (needs changes in Simulink model to provide that info)
%   but for now it works





%==========================================================================
% COPPELIA_CAMERA - S-Function for Camera Handling and processing 
%==========================================================================
%
% INPUTS (1 elements):
%   u = connection_status - 1 if connected to CoppeliaSim, 0 otherwise
%
% OUTPUTS (5 elements):
%   [platform_x, platform_y, platform_z, detection_flag, fov]
%
% PARAMETERS (varargin):
%   varargin = [sample_time, tag_size, enable_logging, enable_adaptive_focus]
%%==========================================================================

persistent sim clientID quadHandle cameraHandle connected                           % Camera connection status
persistent enable_logging enable_adaptive_focus tag_size frame_count fov            % Camera settings
persistent camera_params                                                            % Camera intrinsic parameters  
persistent ransac_fail_count                                                        % RANSAC failure counter, couldnt find enough corners

% ADAPTIVE FOCUS PARAMETERS
default_fov=90;             % degrees, default field of view
max_fov=135;                % degrees, maximum field of view for adaptive focus
min_altitude=0.3;           % meters,  minimum altitude for adaptive focus
altitude_fov_switch=0.6;    % meters,  altitude at which to start changing FOV



switch flag
    
    case 0

        %======================================================================
        % CASE 0: INITIALIZATION
        %======================================================================
        % Called ONCE when simulation starts. 
        %------------------------------------------------------------------

        percentage_without_black_outline=0.875; % What percentage of the tag is the white portion (to account for black outline)
        ransac_fail_count = 0;                  % Initialize RANSAC failure counter


        %------------------------------------------------------------------
        % S-FUNCTION PARAMETERS EXTRACTION
        %------------------------------------------------------------------
        % Extracts users parameters from simulink block dialog or sets defaults

        if nargin >= 4 && ~isempty(varargin{1}) 
            sample_time = varargin{1}(1);
            
            if length(varargin{1}) >= 2 && varargin{1}(2) ~= 0
                complete_tag_size = varargin{1}(2);
            else
                complete_tag_size = 0.4;
            end

            %BLACK OUTLINE MAKES THE WHITE PORTION OF THE TAG ACTUALLY
            tag_size = complete_tag_size * percentage_without_black_outline;       % effective size due to black outline

            enable_logging = (length(varargin{1}) >= 3) && varargin{1}(3);         
            enable_adaptive_focus = (length(varargin{1}) >= 4) && varargin{1}(4);  

        else
            sample_time = 0.05;
            tag_size = 0.4;
            enable_logging = 0;
            enable_adaptive_focus = 0;
        end
        
        %------------------------------------------------------------------
        % DEFINE S-FUNCTION STRUCTURE
        %------------------------------------------------------------------
        % Tell Simulink about this S-Function's characteristics

        sizes = simsizes;               %Use the built-in simsizes function, this gives us a structure to fill in
        sizes.NumContStates  = 0;       % No continuous states, it means there are no differential equations to solve
        sizes.NumDiscStates  = 0;       % No discrete states, it means there is no memory/state
        sizes.NumOutputs = 5;           % Output vector has 5 elements, [x,y,z,detected_flag,fov]  
        sizes.NumInputs = 1;            % Input vector has 1 element, connection status
        sizes.DirFeedthrough = 1;       % The output depends on the input directly
        sizes.NumSampleTimes = 1;       % We have one sample time
        
        % Assign to system output
        sys = simsizes(sizes);                  % Convert our sizes structure to a vector
        x0  = [];                               % No initial states
        str = [];                               % No state names
        ts  = [sample_time 0];                  % Period, offset (start at t=0)
        simStateCompliance = 'UnknownSimState'; % No SimState compliance
        
        fov = default_fov;              % initialize to avoid undefined and dimensions issues
        frame_count = 0;                % Frame counter initialization
        connected = false;
        
        fprintf('\n[CoppeliaSim Camera] Initialized\n');
        
    case 3

        %======================================================================
        % CASE 3: OUTPUTS
        %======================================================================
        % Called EVERY TIMESTEP during simulation.
        %------------------------------------------------------------------

        sys = [0; 0; 0; 0; fov];    % default initialization of output

        % Early exit if not enough time has passed, to allow connection setup
        % for the first second, do not attempt to connect, that gives time to
        % Coppelia to start

        if t < 1.0
            sys = [];
            return;
        end
        
        sync_connected = (length(u) >= 1) && (u(1) == 1); % input connection status
        
        % Early exit if not connected
        if ~sync_connected  
            return;
        end


        %IF NOT CONNECTED, TRY TO CONNECT -----------------------------------------------------------------------
        
        if ~connected
            fprintf('[CoppeliaSim Camera] Connecting...\n');
            
            try
                sim = evalin('base', 'COPPELIA_SIM');                       % Get sim object from base workspace
                clientID = evalin('base', 'COPPELIA_CLIENTID');             % Get clientID from base workspace
                quadHandle = evalin('base', 'COPPELIA_QUADHANDLE');         % Get quadHandle from base workspace
                

                % Get camera handle
                [res, cameraHandle] = sim.simxGetObjectHandle(clientID, ...
                    'visionSensor', sim.simx_opmode_blocking);
                
                % Error check
                if res ~= sim.simx_return_ok
                    error('Cannot find visionSensor');
                end
                
                % Initialize streaming 
                sim.simxGetVisionSensorImage(clientID, cameraHandle, 0, sim.simx_opmode_streaming);
                % opmode_streaming for continuous image retrieval

                % Get camera parameters
                camera_params = get_camera_parameters(sim, clientID, cameraHandle);

                connected = true;
                fprintf('[OK] Camera connected (Handle: %d) - Using blocking mode\n', cameraHandle);   

                % Set default FOV in case it's changed
                % We need to apply degtorad conversion cause Coppelia uses radians
                sim.simxSetObjectFloatParameter(clientID, cameraHandle, ...
                                        sim.sim_visionfloatparam_perspective_angle, deg2rad(default_fov), ...
                                        sim.simx_opmode_oneshot);
                fov = default_fov; % Initialize fov variable

                % Update intrinsics to match the FOV we just set. We update the camera_params struct
                % based on the new FOV, so that PnP uses consistent intrinsics.
                try
                    if ~isempty(camera_params) && isfield(camera_params, 'width') && isfield(camera_params, 'height')
                        camera_params.focal_length = (camera_params.width/2) / tan(deg2rad(fov)/2);     % Update focal length
                        camera_params.K = [camera_params.focal_length, 0, camera_params.width/2;        % Update Camera calibration matrix
                                            0, camera_params.focal_length, camera_params.height/2; 
                                            0, 0, 1];
                    else
                        camera_params = get_camera_parameters(sim, clientID, cameraHandle);             % Update from API if previous failed
                    end
                catch
                    % Keep existing camera_params on failure
                end
            


            catch ME
                fprintf('[ERROR] %s\n', ME.message);
                connected = false;
                sys = [0; 0; 0; 0; fov];        % default output
                return;
            end
        end
        
        %IF CONNECTED, CAPTURE IMAGE AND PROCESS ----------------------------------------------------------------


        if connected
            try
                frame_count = frame_count + 1;                                      % Increment frame counter    
                
                % CAPTURE IMAGE, blocking mode
                [res, resolution, imagePtr] = sim.simxGetVisionSensorImage(...
                    clientID, cameraHandle, 0, sim.simx_opmode_blocking);
                % We use blocking mode to ensure we get the latest image
                % assuring synchronization with simulation time, but we initialized
                % in streaming mode to reduce latency, we probably could try to 
                % initialize in blocking mode too, but this way seems to work fine
                
                if res == sim.simx_return_ok && ~isempty(imagePtr)
                    
                    w = resolution(1);  
                    h = resolution(2);

                    % We retrived a pointer, we need to convert to uint8 array
                    setdatatype(imagePtr, 'uint8Ptr', w*h*3);  %function to set pointer datatype
                    imageData = imagePtr.Value;                %Get data from pointer

                    %We now have the image data in a 1D array, we need to reshape and convert to RGB format

                    % ============ CORRECT RESHAPE ORDER ============
                    % CoppeliaSim sends data as: [R1,G1,B1,R2,G2,B2,...] 
                    % in row-major order (width varies fastest)
                    
                    image = reshape(uint8(imageData), [3, w, h]); % Reshape to [3, width, height (we collapse the R,G,B channels first)]
                    image = permute(image, [3, 2, 1]);            % Permute to [height, width, 3]
                    image = flipud(image);                        % Flip vertically (For some reasons Coppelia gives upside-down images)
        
                    % Ensure uint8
                    % Image can be in [0,1] or [0,255] depending on CoppeliaSim settings ??
                    % We convert to [0,255] uint8 for consistency
                    % Jusr a safety check

                    if max(image(:)) <= 1
                        image = uint8(image * 255);
                    else
                        image = uint8(image);
                    end
                    % ==============================================

                    %THE PLATFORM DETECTION WILL BE DONE IN THE FUNCTION BELOW
                    [platform_pos_cam, detected, debug_imgs] = detect_landing_platform(...
                        image, camera_params, tag_size, enable_logging, frame_count);
                    %platform_pos_cam is the position with respect to the camera reference frame
                    %needs to be transformed to world frame later

                    

                    if detected
                        % We need quadrotor pose to transform to world frame, cause the camera is attached to the quadrotor
                        % We get that from CoppeliaSim, potentially we could obtain it from the Simulink model directly to speed up
                        [res_pos, quad_pos] = sim.simxGetObjectPosition(...
                            clientID, quadHandle, -1, sim.simx_opmode_blocking);
                        [res_ori, quad_ori] = sim.simxGetObjectOrientation(...
                            clientID, quadHandle, -1, sim.simx_opmode_blocking);

                        

                        if res_pos == sim.simx_return_ok && res_ori == sim.simx_return_ok
                            if enable_logging
                                %fprintf('Quad: pos=[%.3f,%.3f,%.3f] ori=[%.3f,%.3f,%.3f]\n', ...
                                    %quad_pos(1), quad_pos(2), quad_pos(3), ...
                                    %quad_ori(1), quad_ori(2), quad_ori(3));
                                %fprintf('Plat cam: [%.3f,%.3f,%.3f]\n', ...
                                    %platform_pos_cam(1), platform_pos_cam(2), platform_pos_cam(3));
                            end
                            
                            if enable_logging && frame_count >= 10
                                %fprintf('===TRANSFORM DEBUG (Frame %d) ===\n', frame_count);
                                %fprintf('Platform CAMERA frame: [%.3f, %.3f, %.3f]\n', ...
                                    %platform_pos_cam(1), platform_pos_cam(2), platform_pos_cam(3));
                                %fprintf('Quad altitude: %.3f m\n', quad_pos(3));
                                %fprintf('Platform depth (Z_cam): %.3f m\n', platform_pos_cam(3));
                                %fprintf('Depth ratio (Z_cam/alt): %.3f (should≈1 if level)\n', ...
                                    %platform_pos_cam(3)/quad_pos(3));
                                %fprintf('=======================\n\n');
                            end


                            % Transform to world coordinates
                            platform_pos_world = real(transform_to_world(...
                                platform_pos_cam, quad_pos, quad_ori));         % Ensure real
                            platform_pos_world = platform_pos_world(:);         % Force column vector

                            % Safety check for avoiding NaN/Inf
                            if any(isnan(platform_pos_world)) || any(isinf(platform_pos_world))
                                %fprintf('[ERROR] platform_pos_world contains NaN/Inf\n');
                                sys = [0; 0; 0; 0; fov];                    % default output
                            else
                                sys = double([platform_pos_world; 1; fov]); % detected flag = 1
                            end
                            
                            if enable_logging
                                %fprintf('Plat world: [%.3f,%.3f,%.3f]\n', ...
                                    %platform_pos_world(1), platform_pos_world(2), platform_pos_world(3));
                            end
                            
                            % Convert to double for Simulink compatibility, in case platform_pos_world is not
                            % Could maybe be avoided but just to be safe
                            try
                                sys = double([platform_pos_world; 1; fov]);   
                            catch ME
                                %fprintf('[ERROR] Converting to double failed: %s\n', ME.message);
                                sys = [0; 0; 0; 0; fov];
                            end

                            % ADAPTIVE FOCUS LOGIC

                            if enable_adaptive_focus
                                quadrotor_height = quad_pos(3); % i should use vertical distance but i have to fix some things first so that is mhe
                                % now i have to interpolate betweem altitude fov swithc and min_altitude and make the fov scale from 90 to 135

                                if quadrotor_height < altitude_fov_switch
                                    % Linear interpolation
                                    fov = default_fov + (max_fov - default_fov) * ...
                                        (altitude_fov_switch - quadrotor_height) / (altitude_fov_switch - min_altitude);
                                    fov = min(max(fov, default_fov), max_fov); % Clamp between default and max
                                    
                                    % Set new FOV
                                    sim.simxSetObjectFloatParameter(clientID, cameraHandle, ...
                                        sim.sim_visionfloatparam_perspective_angle, deg2rad(fov), ...
                                        sim.simx_opmode_oneshot);
                                    
                                    if enable_logging
                                        %fprintf('[ADAPTIVE FOCUS] Altitude: %.3f m, FOV set to %.2f degrees\n', ...
                                            %quadrotor_height, fov);
                                    end

                                    % Update intrinsics after changing FOV so PnP
                                    % uses consistent camera intrinsics.
                                    try
                                        if ~isempty(camera_params) && isfield(camera_params, 'width') && isfield(camera_params, 'height')
                                            camera_params.focal_length = (camera_params.width/2) / tan(deg2rad(fov)/2);
                                            camera_params.K = [camera_params.focal_length, 0, camera_params.width/2; 
                                                                0, camera_params.focal_length, camera_params.height/2; 
                                                                0, 0, 1];
                                        else
                                            camera_params = get_camera_parameters(sim, clientID, cameraHandle);
                                        end
                                    catch
                                        % non-fatal: leave camera_params as-is
                                    end

                                else
                                    fov = default_fov;
                                end
                            end

    
                            
                        else
                            sys = [0; 0; 0; 0; fov];
                        end
                    else
                        sys = [0; 0; 0; 0; fov];
                    end

                    
                    % Visualization and Debugging Outputs 
                    % Save to base workspace for inspection in another function
                    assignin('base', 'camera_debug', debug_imgs);

                    
                else
                    sys = [0; 0; 0; 0; fov];
                end
                
            catch ME
                %fprintf('[ERROR] %s\n', ME.message);
                sys = [0; 0; 0; 0; fov];
            end
        else
            sys = [0; 0; 0; 0; fov];
        end


        
    case 9
        %======================================================================
        % CASE 9: TERMINATION
        %======================================================================
        % Called ONCE when simulation stops. 
        %======================================================================

        if connected
            %fprintf('\n[CoppeliaSim Camera] Shutting down...\n');
            %fprintf('[OK] Camera terminated (frames: %d)\n', frame_count);
        end
        sys = [];
        
    otherwise
        sys = [];
end

end

%==========================================================================
% HELPER FUNCTIONS 
%==========================================================================


% FUNCTION TO GET CAMERA INTRINSIC PARAMETERS -------------------------------

function camera_params = get_camera_parameters(sim, clientID, cameraHandle)
    [~, fov] = sim.simxGetObjectFloatParameter(clientID, cameraHandle, ...
        sim.sim_visionfloatparam_perspective_angle, sim.simx_opmode_blocking);
    [~, resolution] = sim.simxGetVisionSensorImage(clientID, cameraHandle, ...
        0, sim.simx_opmode_blocking);
    
    width  = double(resolution(1));
    height = double(resolution(2));
    fov    = double(fov);
    
    focal_length = (width / 2) / tan(fov / 2);
    cx = width / 2;     % principal point at center
    cy = height / 2;    % principal point at center
    
    camera_params.K = [focal_length, 0, cx; 0, focal_length, cy; 0, 0, 1];
    camera_params.width = width;
    camera_params.height = height;
    camera_params.focal_length = focal_length;
end

% FUNCTION TO DETECT LANDING PLATFORM FROM IMAGE -------------------------------
function [platform_pos, detected, debug_imgs] = detect_landing_platform(...
    image, camera_params, tag_size, enable_logging, frame_count)
    
    %initialize outputs
    detected = false;
    platform_pos = [0; 0; 0];
    debug_imgs = struct();
    

    gray_image = rgb2gray(image);                                           %convert to grayscale for processing
    binary_image = imbinarize(gray_image, 'adaptive', 'Sensitivity', 0.20); %adaptive thresholding to binary


    debug_imgs.original = image;                                            % Store original image
    debug_imgs.binary = binary_image;                                       % Store binary image
    
    [landing_tag, tag_corners] = find_largest_quadrangle(binary_image);     % Find largest quadrangle in binary image (using helper function)
    %landing tag is a binary mask of the detected tag region, could be printed for debugging

    % DEBUGGING LOGS
    if isempty(landing_tag)
        %fprintf('[DEBUG] No quadrangle found\n');
        cc = bwconncomp(binary_image);                                      %function to find connected components, from Image Processing Toolbox
        %fprintf('[DEBUG] Found %d regions\n', cc.NumObjects);
    else
        %fprintf('[DEBUG] Quadrangle found, corners: %d\n', size(tag_corners,1));
    end

    % Early exit if no landing tag found
    if isempty(landing_tag)
        debug_imgs.detection = image;
        return;
    end
    

    debug_imgs.tag_region = landing_tag; % Store tag region image

    
    %MODIFICATION WITH DIAGNOSTIC LOGGING OF RATIO BETWEEN CIRCLE AND QUADRANGLE

    [circle_found, circle_corners, circle_center, circle_radius] = detect_circle_pattern(landing_tag, tag_size);

    % THIS IF CIRCLE HAS PRIORITY
    if circle_found
        corners_2d = circle_corners;
        if enable_logging
            %fprintf('  -> Circle pattern detected\n');
        end
    else
        [cross_found, cross_corners] = detect_cross_pattern(landing_tag);
        if cross_found
            corners_2d = cross_corners;
            if enable_logging
                %fprintf('  -> Cross pattern detected\n');
            end
        else
            corners_2d = tag_corners;
            if enable_logging
                %fprintf('  -> Using quadrangle corners\n');
            end
        end
    end

    % Normalize corner order and flip Y

    corners_2d = normalize_corner_order(corners_2d);
    corners_2d(:,2) = camera_params.height - corners_2d(:,2);

    % RANSAC with validation
    num_corners_before = size(corners_2d, 1);
    corners_2d = ransac_filter_corners(corners_2d);
    num_corners_after = size(corners_2d, 1);

    if enable_logging
        %fprintf('  [RANSAC] Corners: %d -> %d (filtered %d)\n', ...
            %num_corners_before, num_corners_after, num_corners_before - num_corners_after);
    end

    % CRITICAL: Validate RANSAC output
    if size(corners_2d, 1) < 4
        ransac_fail_count = ransac_fail_count + 1;
        if enable_logging
            %fprintf('  [RANSAC FAIL] Only %d/4 corners - Frame %d - Total fails: %d\n', ...
             %   num_corners_after, frame_count, ransac_fail_count);
        end
        debug_imgs.detection = image;
        debug_imgs.ransac_fail = true;
        debug_imgs.corners_before = num_corners_before;
        debug_imgs.corners_after = num_corners_after;
        return;
    end
    
    s = tag_size / 2;
    corners_3d = [-s, s, 0; s, s, 0; s, -s, 0; -s, -s, 0]; % Counter-clockwise from top-left
    
    [R, t, success] = solve_pnp(corners_3d, corners_2d, camera_params,enable_logging);

    if success
        corner_spacing = norm(corners_2d(1,:) - corners_2d(2,:)); % pixels
        expected_spacing = (tag_size / t(3)) * camera_params.focal_length;
        %fprintf('Corner spacing: %.1f px (detected) vs %.1f px (expected at Z=%.2f)\n', ...
            %corner_spacing, expected_spacing, t(3));
    end


    if success
        platform_pos = t(:);
        detected = true;
        debug_imgs.detection = image; % Simple - no markers
    else
        debug_imgs.detection = image;
    end
end

function [landing_tag, corners] = find_largest_quadrangle(binary_image)
    landing_tag = [];
    corners = [];
    cc = bwconncomp(binary_image);
    if cc.NumObjects == 0, return; end
    
    stats = regionprops(cc, 'Area', 'PixelIdxList');
    [max_area, idx] = max([stats.Area]);
    %fprintf('[DEBUG] Largest region area: %d\n', max_area);
    
    % Reject if too small (adjust threshold as needed)
    min_area = 140;  % pixels - tune based on your platform size
    if max_area < min_area
        return;  % Too small, reject
    end
    
    % Get largest region
    mask = false(size(binary_image));
    mask(stats(idx).PixelIdxList) = true;
    landing_tag = mask;
    
    % Fit minimum bounding rectangle
    [B, ~] = bwboundaries(mask, 'noholes');
    if isempty(B), return; end
    boundary = B{1};
    
    % Get minimum area rectangle corners
    [rectx, recty] = minboundrect(boundary(:,2), boundary(:,1));
    corners = [rectx(:), recty(:)];
end

% THIS FUNCTION IS TAKEN FORM FILE MINBOUNDRECT.M AVAILABLE ONLINE BY JOHN D'ERRICO
function [rectx, recty] = minboundrect(x, y)
    % Get convex hull
    try
        k = convhull(x, y);
        hull_x = x(k);
        hull_y = y(k);
        %fprintf('[MBR] Convex hull: %d points from %d boundary points\n', length(k), length(x));
    catch ME
        %fprintf('[MBR FAIL] Convex hull failed: %s\n', ME.message);
        rectx = []; recty = [];
        return;
    end
    
    if length(hull_x) < 3
        %fprintf('[MBR FAIL] Hull has only %d points (need ≥3)\n', length(hull_x));
        rectx = []; recty = [];
        return;
    end
    
    min_area = inf;
    best_corners = [];
    
    % Try each edge orientation
    for i = 1:length(hull_x)-1
        dx = hull_x(i+1) - hull_x(i);
        dy = hull_y(i+1) - hull_y(i);
        theta = atan2(dy, dx);
        
        R = [cos(-theta), -sin(-theta); sin(-theta), cos(-theta)];
        rotated = R * [x'; y'];
        
        min_x = min(rotated(1,:));
        max_x = max(rotated(1,:));
        min_y = min(rotated(2,:));
        max_y = max(rotated(2,:));
        
        area = (max_x - min_x) * (max_y - min_y);
        
        if area < min_area
            min_area = area;
            corners_rot = [min_x, min_y; max_x, min_y; max_x, max_y; min_x, max_y]';
            R_inv = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            corners = R_inv * corners_rot;
            best_corners = corners;
        end
    end
    
    if isempty(best_corners)
        %fprintf('[MBR FAIL] No valid rectangle found after %d iterations\n', length(hull_x)-1);
        rectx = []; recty = [];
        return;
    end
    
    %fprintf('[MBR OK] Min area: %.1f px², corners found\n', min_area);
    rectx = best_corners(1,:)';
    recty = best_corners(2,:)';
end

function [found, corners, center, radius] = detect_circle_pattern(tag_region,tag_size)


    found = false;
    corners = [];
    center = [];
    radius = [];
    inverted = ~tag_region;
    [centers, radii] = imfindcircles(inverted, [10, 100], 'ObjectPolarity', 'bright', 'Sensitivity', 0.9);
    if isempty(centers), return; end
    [~, idx] = max(radii);
    center = centers(idx, :);
    radius = radii(idx);
    angle_offset = pi/4;
    corners = zeros(4, 2);
    
    % Dynamic scale factor based on imfindcircles accuracy at different scales
    % After testing, this formula gives better corner placement
    scale_factor = 1.85 + 0.01 * (radius - 15); % Adjust as needed
    scale_factor = min(scale_factor, 2.2); % Cap to avoid excessive scaling
    scale_factor = 1;
    
    percentage_without_black_outline=0.875; % to account for black outline in april tag, change also in helper func
    complete_tag_size=tag_size/percentage_without_black_outline; % to account for black outline, we re-correct
    circle_diameter = complete_tag_size*0.825; % effective size of the circle diameter inside 
    circle_radius = circle_diameter/2;
    pixels_per_meter = radius / circle_radius;

    square_distance_from_center= (tag_size/2) * sqrt(2);
    corner_distance_pixel = square_distance_from_center* pixels_per_meter;
    
    for i = 1:4
        angle = angle_offset + (i-1) * pi/2;
        corners(i, :) = center + corner_distance_pixel * [cos(angle), sin(angle)];
    end
    found = true;
end

function [found, corners] = detect_cross_pattern(tag_region)
    found = false;
    corners = [];
    
    inverted = ~tag_region;
    
    % Adaptive erosion radius based on region size
    region_size = sqrt(sum(tag_region(:)));
    erosion_radius = max(2, round(region_size / 50));
    se = strel('disk', erosion_radius);
    eroded = imerode(inverted, se);
    
    cc = bwconncomp(eroded);
    if cc.NumObjects < 2, return; end
    
    stats = regionprops(cc, 'Area', 'BoundingBox', 'MajorAxisLength', 'MinorAxisLength');
    
    % Find elongated regions (cross arms)
    elongated = [];
    min_area = region_size / 20;  % Adaptive threshold
    
    for i = 1:length(stats)
        major = stats(i).MajorAxisLength;
        minor = stats(i).MinorAxisLength;
        aspect_ratio = major / minor;
        
        if aspect_ratio > 1.5 && stats(i).Area > min_area
            elongated = [elongated, i];
        end
    end
    
    if length(elongated) < 2, return; end
    
    % Get cross center from arm centroids
    arm_stats = regionprops(cc, elongated, 'Centroid', 'MajorAxisLength');
    centroids = reshape([arm_stats.Centroid], 2, [])';
    center = mean(centroids, 1);
    
    % Average arm length for corner placement
    avg_arm_length = mean([arm_stats.MajorAxisLength]);
    corner_offset = avg_arm_length / 4;  % Adaptive offset
    
    % Combine all cross arm pixels
    mask = false(size(tag_region));
    for i = elongated
        mask(cc.PixelIdxList{i}) = true;
    end
    
    % Get boundary extrema
    boundary = bwboundaries(mask, 'noholes');
    if isempty(boundary), return; end
    boundary = boundary{1};
    
    [~, top_idx] = min(boundary(:, 1));
    [~, bottom_idx] = max(boundary(:, 1));
    [~, left_idx] = min(boundary(:, 2));
    [~, right_idx] = max(boundary(:, 2));
    
    % Place corners with adaptive offset
    corners = [
        boundary(top_idx, 2), boundary(top_idx, 1) + corner_offset;
        boundary(right_idx, 2) - corner_offset, boundary(right_idx, 1);
        boundary(bottom_idx, 2), boundary(bottom_idx, 1) - corner_offset;
        boundary(left_idx, 2) + corner_offset, boundary(left_idx, 1)
    ];
    
    found = true;
end

function corners_filtered = ransac_filter_corners(corners)
    if size(corners, 1) <= 4
        corners_filtered = corners;
        return;
    end
    max_iterations = 100;
    inlier_threshold = 5;
    best_inliers = [];
    for iter = 1:max_iterations
        sample_idx = randperm(size(corners, 1), 4);
        sample = corners(sample_idx, :);
        if ~is_convex_quad(sample), continue; end
        inliers = sample_idx;
        for i = 1:size(corners, 1)
            if ~ismember(i, sample_idx)
                dist = point_to_quad_distance(corners(i, :), sample);
                if dist < inlier_threshold
                    inliers = [inliers, i];
                end
            end
        end
        if length(inliers) > length(best_inliers)
            best_inliers = inliers;
        end
    end
    if ~isempty(best_inliers)
        corners_filtered = corners(best_inliers(1:min(4, end)), :);
    else
        corners_filtered = corners(1:min(4, end), :);
    end
end

function [R, t, success] = solve_pnp(points_3d, points_2d, camera_params, enable_logging)
    success = false;
    R = eye(3);
    t = [0; 0; 0];
    
    if size(points_2d, 1) < 4
        return;
    end
    
    % Normalize 2D points by camera intrinsics
    K = camera_params.K;
    K_inv = inv(K);
    
    % Convert to homogeneous coordinates
    points_2d_hom = [points_2d, ones(size(points_2d,1), 1)]';  % 3xN
    points_2d_norm = K_inv * points_2d_hom;  % Normalized image coordinates
    points_2d_norm = points_2d_norm(1:2, :)';  % Back to Nx2
    
    % Tag plane points (z=0)
    points_tag_plane = points_3d(:, 1:2);  % Nx2
    
    % Compute homography from normalized coords to tag plane
    try
        H = fitgeotrans(points_tag_plane, points_2d_norm, 'projective'); %swap arguments
        H = H.T';  % Get 3x3 matrix, transpose to standard form
    catch
        return;
    end
    
    % Decompose homography: H = [h1 h2 h3] = λ[r1 r2 t]
    h1 = H(:, 1);
    h2 = H(:, 2);
    h3 = H(:, 3);
    
    % Estimate scale factor λ
    lambda = 1 / norm(h1);

    if enable_logging
        %fprintf('  [PnP] Lambda: %.4f\n', lambda);
        %fprintf('  [PnP] Recovered depth: %.3f m\n', lambda * norm(h3));
        %fprintf('  [PnP] h3 magnitude: %.4f\n', norm(h3));
    end
    
    % Extract rotation columns and translation
    r1 = lambda * h1;
    r2 = lambda * h2;
    t_recovered = lambda * h3;
    
    % Ensure r1 and r2 are orthonormal
    r1 = r1 / norm(r1);
    r2 = r2 - dot(r1, r2) * r1;  % Gram-Schmidt
    r2 = r2 / norm(r2);
    
    % Compute third column of rotation matrix
    r3 = cross(r1, r2);
    
    % Construct rotation matrix
    R = [r1, r2, r3];
    
    % Enforce orthogonality via SVD
    [U, ~, V] = svd(R);
    R = U * V';
    
    % Ensure det(R) = 1 (proper rotation, not reflection)
    if det(R) < 0
        R = -R;
        t_recovered = -t_recovered;
    end
    
    % Check if solution is valid (positive depth)
    t = t_recovered;
    if t(3) <= 0
        % Try flipping solution
        R = R * diag([1, 1, -1]);
        t(3) = -t(3);
    end
    
    % Final validation
    if t(3) > 0 && t(3) < 10  % Reasonable depth range
        success = true;
        if enable_logging
            %fprintf('  -> PnP SUCCESS: pos=[%.3f, %.3f, %.3f], depth=%.3f\n', ...
                %t(1), t(2), t(3), norm(t));
        end
    else
        if enable_logging
            %fprintf('  -> PnP FAILED (invalid depth: %.3f)\n', t(3));
        end
    end
end

function platform_world = transform_to_world(platform_cam, quad_pos, quad_ori)
    roll = quad_ori(1);
    pitch = quad_ori(2);
    yaw = quad_ori(3);
    Rx = [1, 0, 0; 0, cos(roll), -sin(roll); 0, sin(roll), cos(roll)];
    Ry = [cos(pitch), 0, sin(pitch); 0, 1, 0; -sin(pitch), 0, cos(pitch)];
    Rz = [cos(yaw), -sin(yaw), 0; sin(yaw), cos(yaw), 0; 0, 0, 1];
    R_quad = Rz * Ry * Rx; 
    R_cam_to_body = [1, 0, 0; 0, 1, 0; 0, 0, -1]; %flips Z axis to obtain the word 

    cam_offset_body = [0; 0; -0.060];  % Camera is 6cm below quad center
    platform_world = quad_pos(:) + R_quad * (R_cam_to_body * platform_cam + cam_offset_body);
end

function len = arcLength(curve)
    len = sum(sqrt(sum(diff(curve).^2, 2)));
end

function approx = approxPolyDP(curve, epsilon)
    try
        k = convhull(curve(:, 2), curve(:, 1));
        approx = curve(k, :);
        approx = unique(approx, 'rows', 'stable');
    catch
        % Not enough unique points for convex hull
        approx = unique(curve, 'rows', 'stable');
        if size(approx, 1) < 4
            approx = [];
        end
    end
end
function convex = is_convex_quad(points)
    convex = false;
    if size(points, 1) ~= 4, return; end
    for i = 1:4
        p1 = points(i, :);
        p2 = points(mod(i, 4) + 1, :);
        p3 = points(mod(i + 1, 4) + 1, :);
        v1 = p2 - p1;
        v2 = p3 - p2;
        cross_prod = v1(1) * v2(2) - v1(2) * v2(1);
        if cross_prod < 0, return; end
    end
    convex = true;
end

function dist = point_to_quad_distance(point, quad)
    min_dist = inf;
    for i = 1:4
        p1 = quad(i, :);
        p2 = quad(mod(i, 4) + 1, :);
        d = point_to_segment_distance(point, p1, p2);
        min_dist = min(min_dist, d);
    end
    dist = min_dist;
end

function dist = point_to_segment_distance(point, p1, p2)
    v = p2 - p1;
    w = point - p1;
    c1 = dot(w, v);
    if c1 <= 0, dist = norm(point - p1); return; end
    c2 = dot(v, v);
    if c1 >= c2, dist = norm(point - p2); return; end
    b = c1 / c2;
    pb = p1 + b * v;
    dist = norm(point - pb);
end

function corners_ordered = normalize_corner_order(corners)
    % Ensure consistent counter-clockwise ordering starting from top-left
    if size(corners, 1) < 4
        corners_ordered = corners;
        return;
    end
    
    % Compute centroid
    centroid = mean(corners, 1);
    
    % Compute angles from centroid to each corner
    angles = atan2(corners(:,2) - centroid(2), corners(:,1) - centroid(1));
    
    % Sort by angle (counter-clockwise from -π)
    [~, idx] = sort(angles);
    corners_sorted = corners(idx, :);
    
    % Find top-left corner (smallest x+y sum)
    sums = sum(corners_sorted, 2);
    [~, top_left_idx] = min(sums);
    
    % Rotate array so top-left is first
    corners_ordered = circshift(corners_sorted, -top_left_idx + 1, 1);
end