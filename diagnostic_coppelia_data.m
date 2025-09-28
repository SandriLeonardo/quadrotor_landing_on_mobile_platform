% FIXED DIAGNOSTIC - Handle 3D array format correctly
if ~exist('out', 'var')
    error('Run Simulink first!');
end

data_raw = out.quadrotor_states.signals.values;
time = out.quadrotor_states.time;

fprintf('=== FIXED DIAGNOSIS ===\n');
fprintf('Raw data size: %s\n', mat2str(size(data_raw)));

% Handle 3D array: [1, 6, N] -> [N, 6]
if ndims(data_raw) == 3
    data = squeeze(data_raw)'; % Convert [1,6,N] to [N,6]
    fprintf('✓ Converted 3D array to 2D: %dx%d\n', size(data,1), size(data,2));
else
    data = data_raw;
end

fprintf('Time range: %.2f to %.2f sec\n', time(1), time(end));
fprintf('Start pos: [%.3f, %.3f, %.3f]\n', data(1,1), data(1,2), data(1,3));
fprintf('End pos: [%.3f, %.3f, %.3f]\n', data(end,1), data(end,2), data(end,3));

if size(data,1) < 10
    fprintf('❌ PROBLEM: Only %d data points!\n', size(data,1));
else
    fprintf('✅ Data looks good: %d trajectory points\n', size(data,1));
    fprintf('Position ranges:\n');
    fprintf('  X: %.3f to %.3f\n', min(data(:,1)), max(data(:,1)));
    fprintf('  Y: %.3f to %.3f\n', min(data(:,2)), max(data(:,2)));
    fprintf('  Z: %.3f to %.3f\n', min(data(:,3)), max(data(:,3)));
end