data = squeeze(out.quadrotor_states.signals.values)';
figure;
plot3(data(:,1), data(:,2), data(:,3), 'b-', 'LineWidth', 2);
grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('3D Trajectory');