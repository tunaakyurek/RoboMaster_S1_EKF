function visualize_trajectory(csv_file, output_dir)
% VISUALIZE_TRAJECTORY - Comprehensive iPhone EKF trajectory visualization
% 
% Usage:
%   visualize_trajectory('data/iphone_ekf_log_20250812_150708.csv')
%   visualize_trajectory('data/iphone_ekf_log_20250812_150708.csv', 'trajectory_plots')
%
% Creates comprehensive 3D visualizations including:
% - Interactive 3D trajectory with time coloring
% - Multiple view perspectives (top, side)
% - Orientation analysis
% - Performance metrics
% - Animation (optional)

if nargin < 2
    output_dir = 'matlab_trajectory_plots';
end

if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

fprintf('📊 Loading iPhone EKF trajectory data...\n');

% Load data
try
    data = readtable(csv_file);
    fprintf('✅ Loaded %d data points from %s\n', height(data), csv_file);
catch ME
    error('❌ Failed to load CSV file: %s', ME.message);
end

% Process data
time_rel = data.timestamp - data.timestamp(1);
distance = cumsum([0; sqrt(diff(data.x).^2 + diff(data.y).^2 + diff(data.z).^2)]);

% Convert angles to degrees
roll_deg = rad2deg(data.roll);
pitch_deg = rad2deg(data.pitch);
yaw_deg = rad2deg(data.yaw);

fprintf('📏 Total distance: %.2f meters over %.1f seconds\n', distance(end), time_rel(end));
fprintf('📍 Position range: X=[%.1f, %.1f], Y=[%.1f, %.1f], Z=[%.1f, %.1f]\n', ...
        min(data.x), max(data.x), min(data.y), max(data.y), min(data.z), max(data.z));

%% Create comprehensive trajectory analysis
fprintf('🎨 Creating comprehensive trajectory visualization...\n');

fig1 = figure('Position', [100, 100, 1600, 1200], 'Name', 'iPhone EKF Trajectory Analysis');

% 3D trajectory (main plot)
subplot(2, 3, 1);
scatter3(data.x, data.y, -data.z, 30, time_rel, 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 0.5);
hold on;
plot3(data.x, data.y, -data.z, 'b-', 'LineWidth', 2, 'Color', [0, 0, 1, 0.6]);

% Mark start and end
scatter3(data.x(1), data.y(1), -data.z(1), 150, 'g', 'filled', 's', ...
         'MarkerEdgeColor', 'k', 'LineWidth', 2, 'DisplayName', 'Start');
scatter3(data.x(end), data.y(end), -data.z(end), 150, 'r', 'filled', 'o', ...
         'MarkerEdgeColor', 'k', 'LineWidth', 2, 'DisplayName', 'End');

colormap(gca, 'viridis');
c = colorbar;
c.Label.String = 'Time (s)';
c.Label.FontSize = 11;

xlabel('X Position (m)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Y Position (m)', 'FontSize', 12, 'FontWeight', 'bold');
zlabel('Altitude (m)', 'FontSize', 12, 'FontWeight', 'bold');
title({'3D iPhone Trajectory', '(EKF State Estimates)'}, 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best');
grid on;
view(45, 30);

% Top view (X-Y)
subplot(2, 3, 2);
plot(data.x, data.y, 'b-', 'LineWidth', 3);
hold on;

% Add direction arrows
skip = max(1, floor(length(data.x) / 15));
for i = 1:skip:(length(data.x)-skip)
    dx = data.x(i+skip) - data.x(i);
    dy = data.y(i+skip) - data.y(i);
    if abs(dx) > 0.05 || abs(dy) > 0.05
        quiver(data.x(i), data.y(i), dx*0.7, dy*0.7, 0, 'r', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
    end
end

scatter(data.x(1), data.y(1), 100, 'g', 'filled', 's', 'MarkerEdgeColor', 'k', 'LineWidth', 1);
scatter(data.x(end), data.y(end), 100, 'r', 'filled', 'o', 'MarkerEdgeColor', 'k', 'LineWidth', 1);

xlabel('X Position (m)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Y Position (m)', 'FontSize', 12, 'FontWeight', 'bold');
title({'Top View (X-Y Plane)', 'with Movement Direction'}, 'FontSize', 14, 'FontWeight', 'bold');
grid on;
axis equal;

% Side view (X-Z)
subplot(2, 3, 3);
plot(data.x, data.z, 'b-', 'LineWidth', 3);
hold on;
scatter(data.x(1), data.z(1), 100, 'g', 'filled', 's', 'MarkerEdgeColor', 'k');
scatter(data.x(end), data.z(end), 100, 'r', 'filled', 'o', 'MarkerEdgeColor', 'k');
xlabel('X Position (m)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Z Position (m)', 'FontSize', 12, 'FontWeight', 'bold');
title('Side View (X-Z Plane)', 'FontSize', 14, 'FontWeight', 'bold');
grid on;
set(gca, 'YDir', 'reverse'); % NED convention

% Position time series
subplot(2, 3, 4);
plot(time_rel, data.x, 'r-', 'LineWidth', 2, 'DisplayName', 'X');
hold on;
plot(time_rel, data.y, 'g-', 'LineWidth', 2, 'DisplayName', 'Y');
plot(time_rel, data.z, 'b-', 'LineWidth', 2, 'DisplayName', 'Z');
xlabel('Time (s)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Position (m)', 'FontSize', 12, 'FontWeight', 'bold');
title('Position vs Time', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best');
grid on;

% Orientation time series
subplot(2, 3, 5);
plot(time_rel, roll_deg, 'r-', 'LineWidth', 2, 'DisplayName', 'Roll');
hold on;
plot(time_rel, pitch_deg, 'g-', 'LineWidth', 2, 'DisplayName', 'Pitch');
plot(time_rel, yaw_deg, 'b-', 'LineWidth', 2, 'DisplayName', 'Yaw');
xlabel('Time (s)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Angle (degrees)', 'FontSize', 12, 'FontWeight', 'bold');
title('Orientation vs Time', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best');
grid on;

% Distance and covariance
subplot(2, 3, 6);
yyaxis left;
plot(time_rel, distance, 'Color', [0.5, 0, 0.5], 'LineWidth', 3);
ylabel('Cumulative Distance (m)', 'Color', [0.5, 0, 0.5], 'FontSize', 12, 'FontWeight', 'bold');

yyaxis right;
plot(time_rel, data.cov_trace, 'Color', [1, 0.5, 0], 'LineWidth', 2);
ylabel('EKF Covariance Trace', 'Color', [1, 0.5, 0], 'FontSize', 12, 'FontWeight', 'bold');

xlabel('Time (s)', 'FontSize', 12, 'FontWeight', 'bold');
title('Distance & EKF Convergence', 'FontSize', 14, 'FontWeight', 'bold');
grid on;

% Overall title
sgtitle(sprintf('iPhone-EKF Trajectory Analysis\n%s', strrep(csv_file, '_', '\_')), ...
        'FontSize', 16, 'FontWeight', 'bold');

% Save comprehensive plot
[~, name, ~] = fileparts(csv_file);
save_path = fullfile(output_dir, [name '_complete_analysis.png']);
saveas(fig1, save_path);
fprintf('📁 Complete analysis saved to %s\n', save_path);

%% Create focused 3D plot
fprintf('🎯 Creating focused 3D trajectory plot...\n');

fig2 = figure('Position', [200, 200, 1000, 800], 'Name', '3D Trajectory Focus');

scatter3(data.x, data.y, -data.z, 40, time_rel, 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 0.5);
hold on;
plot3(data.x, data.y, -data.z, 'navy', 'LineWidth', 4);

% Enhanced start/end markers
scatter3(data.x(1), data.y(1), -data.z(1), 200, 'lime', 'filled', 's', ...
         'MarkerEdgeColor', 'k', 'LineWidth', 3, 'DisplayName', 'Start');
scatter3(data.x(end), data.y(end), -data.z(end), 200, 'red', 'filled', 'o', ...
         'MarkerEdgeColor', 'k', 'LineWidth', 3, 'DisplayName', 'End');

colormap('plasma');
c = colorbar;
c.Label.String = 'Time (seconds)';
c.Label.FontSize = 12;
c.Label.FontWeight = 'bold';

xlabel('X Position (m)', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Y Position (m)', 'FontSize', 14, 'FontWeight', 'bold');
zlabel('Altitude (m)', 'FontSize', 14, 'FontWeight', 'bold');
title(sprintf('3D iPhone Trajectory\nTotal Distance: %.2fm over %.1fs', distance(end), time_rel(end)), ...
      'FontSize', 16, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 12);
grid on;
view(45, 20);

% Save 3D plot
save_path_3d = fullfile(output_dir, [name '_3d_trajectory.png']);
saveas(fig2, save_path_3d);
fprintf('📁 3D trajectory saved to %s\n', save_path_3d);

%% Create orientation analysis
fprintf('🧭 Creating orientation analysis...\n');

fig3 = figure('Position', [300, 300, 1200, 900], 'Name', 'Orientation Analysis');

% Roll, Pitch, Yaw over time
subplot(2, 2, 1);
plot(time_rel, roll_deg, 'r-', 'LineWidth', 2, 'DisplayName', 'Roll');
hold on;
plot(time_rel, pitch_deg, 'g-', 'LineWidth', 2, 'DisplayName', 'Pitch');
plot(time_rel, yaw_deg, 'b-', 'LineWidth', 2, 'DisplayName', 'Yaw');
xlabel('Time (s)', 'FontSize', 12);
ylabel('Angle (degrees)', 'FontSize', 12);
title('Orientation vs Time', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best');
grid on;

% Orientation distribution
subplot(2, 2, 2);
histogram(roll_deg, 30, 'FaceColor', 'r', 'FaceAlpha', 0.7, 'DisplayName', 'Roll');
hold on;
histogram(pitch_deg, 30, 'FaceColor', 'g', 'FaceAlpha', 0.7, 'DisplayName', 'Pitch');
histogram(yaw_deg, 30, 'FaceColor', 'b', 'FaceAlpha', 0.7, 'DisplayName', 'Yaw');
xlabel('Angle (degrees)', 'FontSize', 12);
ylabel('Frequency', 'FontSize', 12);
title('Orientation Distribution', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best');
grid on;

% 3D orientation sphere projection
subplot(2, 2, 3);
phi = deg2rad(roll_deg);
theta = deg2rad(pitch_deg);
x_sphere = cos(theta) .* cos(phi);
y_sphere = cos(theta) .* sin(phi);
scatter(x_sphere, y_sphere, 20, time_rel, 'filled');
colormap(gca, 'viridis');
colorbar;
xlabel('Roll Component', 'FontSize', 12);
ylabel('Pitch Component', 'FontSize', 12);
title('Orientation Sphere Projection', 'FontSize', 14, 'FontWeight', 'bold');
axis equal;
grid on;

% Angular rates
subplot(2, 2, 4);
plot(time_rel, rad2deg(data.yaw_rate), 'purple', 'LineWidth', 2);
xlabel('Time (s)', 'FontSize', 12);
ylabel('Yaw Rate (deg/s)', 'FontSize', 12);
title('Angular Velocity', 'FontSize', 14, 'FontWeight', 'bold');
grid on;

sgtitle('iPhone Orientation Analysis', 'FontSize', 16, 'FontWeight', 'bold');

% Save orientation analysis
save_path_orient = fullfile(output_dir, [name '_orientation_analysis.png']);
saveas(fig3, save_path_orient);
fprintf('📁 Orientation analysis saved to %s\n', save_path_orient);

%% Print summary statistics
fprintf('\n📊 TRAJECTORY ANALYSIS SUMMARY\n');
fprintf('===============================================\n');
fprintf('Duration:           %.1f seconds\n', time_rel(end));
fprintf('Total Distance:     %.2f meters\n', distance(end));
fprintf('Average Speed:      %.2f m/s\n', distance(end) / time_rel(end));
fprintf('Position Range:\n');
fprintf('  X: [%.2f, %.2f] meters (%.2f m span)\n', min(data.x), max(data.x), range(data.x));
fprintf('  Y: [%.2f, %.2f] meters (%.2f m span)\n', min(data.y), max(data.y), range(data.y));
fprintf('  Z: [%.2f, %.2f] meters (%.2f m span)\n', min(data.z), max(data.z), range(data.z));
fprintf('Orientation Stats:\n');
fprintf('  Roll:  μ=%.1f°, σ=%.1f°, range=[%.1f°, %.1f°]\n', mean(roll_deg), std(roll_deg), min(roll_deg), max(roll_deg));
fprintf('  Pitch: μ=%.1f°, σ=%.1f°, range=[%.1f°, %.1f°]\n', mean(pitch_deg), std(pitch_deg), min(pitch_deg), max(pitch_deg));
fprintf('  Yaw:   μ=%.1f°, σ=%.1f°, range=[%.1f°, %.1f°]\n', mean(yaw_deg), std(yaw_deg), min(yaw_deg), max(yaw_deg));
fprintf('EKF Performance:\n');
fprintf('  Initial covariance: %.6f\n', data.cov_trace(1));
fprintf('  Final covariance:   %.6f\n', data.cov_trace(end));
fprintf('  Convergence:        %.1f%% reduction\n', (1 - data.cov_trace(end)/data.cov_trace(1)) * 100);
fprintf('===============================================\n');
fprintf('✅ All visualizations saved to: %s\n', output_dir);

end
