function [detected, position_camera] = detect_platform(img, cam_params)
% Algorithm 1 from paper - Complete implementation
% Input: RGB image [H x W x 3]
% Output: detected (0/1), position_camera [x,y,z] in camera frame

detected = 0;
position_camera = [0, 0, 0];

try
    %% 1. Binary thresholding
    gray = rgb2gray(img);
    bw = gray > 200;  % White detection
    
    % Preprocessing
    bw = imfill(bw, 'holes');
    bw = bwareaopen(bw, 500);
    bw = imclose(bw, strel('disk', 5));
    
    %% 2. Find largest white quadrangle
    [B, L] = bwboundaries(bw, 'noholes');
    
    if isempty(B)
        return;
    end
    
    % Select largest region
    areas = cellfun(@length, B);
    [~, idx] = max(areas);
    boundary = B{idx};
    
    % Approximate with polygon
    epsilon = 0.02 * arcLength(boundary, true);
    approx = approxPolyDP(boundary, epsilon, true);
    
    if size(approx, 1) ~= 4
        return;  % Not a quadrangle
    end
    
    quadrangle_corners = approx;
    
    %% 3. Extract circle or cross corners inside quadrangle
    % Create mask for quadrangle interior
    mask = poly2mask(quadrangle_corners(:,2), quadrangle_corners(:,1), size(bw,1), size(bw,2));
    bw_inside = bw & mask;
    
    % Invert to find black features (circle/cross)
    bw_black = ~bw_inside & mask;
    
    % Try circle detection
    [centers, radii] = imfindcircles(bw_black, [10 50], 'ObjectPolarity', 'dark', 'Sensitivity', 0.9);
    
    if ~isempty(centers)
        % Use circle: approximate with polygon for PnP
        theta = linspace(0, 2*pi, 12);
        r = radii(1);
        corners_px = [centers(1,1) + r*cos(theta)', centers(1,2) + r*sin(theta)'];
    else
        % Try cross detection
        se = strel('line', 20, 0);  % Horizontal line
        cross_h = imopen(bw_black, se);
        se = strel('line', 20, 90);  % Vertical line
        cross_v = imopen(bw_black, se);
        cross = cross_h | cross_v;
        
        if sum(cross(:)) > 100
            % Find cross corners (4 endpoints)
            cross_pts = bwmorph(cross, 'endpoints');
            [y, x] = find(cross_pts);
            if length(x) >= 4
                corners_px = [x(1:4), y(1:4)];
            else
                % Fallback: use quadrangle corners
                corners_px = [quadrangle_corners(:,2), quadrangle_corners(:,1)];
            end
        else
            % Fallback: use quadrangle corners
            corners_px = [quadrangle_corners(:,2), quadrangle_corners(:,1)];
        end
    end
    
    %% 4. Solve PnP
    if size(corners_px, 1) >= 4
        detected = 1;
        position_camera = solvePnP(corners_px, cam_params);
    end
    
catch ME
    fprintf('[WARNING] Detection failed: %s\n', ME.message);
end

end

%% Helper: arcLength
function len = arcLength(boundary, closed)
    if closed
        boundary = [boundary; boundary(1,:)];
    end
    diffs = diff(boundary);
    len = sum(sqrt(sum(diffs.^2, 2)));
end

%% Helper: approxPolyDP (Douglas-Peucker)
function approx = approxPolyDP(curve, epsilon, closed)
    if closed
        curve = [curve; curve(1,:)];
    end
    
    % Simplified implementation
    n = size(curve, 1);
    keep = false(n, 1);
    keep([1, n]) = true;
    
    simplifyDP(1, n);
    approx = curve(keep, :);
    
    function simplifyDP(start_idx, end_idx)
        if end_idx - start_idx < 2
            return;
        end
        
        % Find point farthest from line
        max_dist = 0;
        max_idx = start_idx;
        
        for i = start_idx+1:end_idx-1
            d = pointToLineDistance(curve(i,:), curve(start_idx,:), curve(end_idx,:));
            if d > max_dist
                max_dist = d;
                max_idx = i;
            end
        end
        
        if max_dist > epsilon
            keep(max_idx) = true;
            simplifyDP(start_idx, max_idx);
            simplifyDP(max_idx, end_idx);
        end
    end
end

function d = pointToLineDistance(pt, line_start, line_end)
    num = abs((line_end(1)-line_start(1))*(line_start(2)-pt(2)) - ...
              (line_start(1)-pt(1))*(line_end(2)-line_start(2)));
    den = sqrt((line_end(1)-line_start(1))^2 + (line_end(2)-line_start(2))^2);
    d = num / (den + eps);
end

%% Helper: Solve PnP (simplified)
function pos_cam = solvePnP(corners_px, cam_params)
    % Camera intrinsics (esempio, calibra per tua camera)
    fx = cam_params.fx;
    fy = cam_params.fy;
    cx = cam_params.cx;
    cy = cam_params.cy;
    tag_size = cam_params.tag_size;
    
    % 3D corners of tag (in tag frame, Z=0)
    corners_3d = tag_size * [-1, -1, 0;
                              1, -1, 0;
                              1,  1, 0;
                             -1,  1, 0];
    
    % Use only first 4 corners
    corners_px = corners_px(1:4, :);
    
    % Estimate pose with POSIT algorithm (simplified)
    % Real implementation: use estimateWorldCameraPose or similar
    
    % Approximation: use centroid + area
    centroid_px = mean(corners_px, 1);
    area_px = polyarea(corners_px(:,1), corners_px(:,2));
    
    % Estimate Z from area
    tag_area_m2 = (2*tag_size)^2;
    focal_avg = (fx + fy) / 2;
    Z = sqrt(tag_area_m2 * focal_avg^2 / area_px);
    
    % Estimate X, Y from centroid
    X = (centroid_px(1) - cx) * Z / fx;
    Y = (centroid_px(2) - cy) * Z / fy;
    
    pos_cam = [X, Y, Z];
end