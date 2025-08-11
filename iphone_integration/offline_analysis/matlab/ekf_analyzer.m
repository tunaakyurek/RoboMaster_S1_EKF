% EKF Data Analyzer for MATLAB
% =============================
% Comprehensive analysis tools for iPhone-EKF data
% Following RoboMaster EKF Formulary specifications
%
% Author: RoboMaster EKF Integration System
% Date: 2025

classdef ekf_analyzer < handle
    % EKF_ANALYZER Analyzes logged EKF data from iPhone integration
    
    properties
        data            % Table containing logged data
        log_file        % Path to log file
        results         % Analysis results structure
        fig_handles     % Figure handles for plots
    end
    
    methods
        function obj = ekf_analyzer(log_file)
            % Constructor - loads data from CSV file
            %
            % Args:
            %   log_file: Path to CSV log file
            
            obj.log_file = log_file;
            obj.fig_handles = [];
            obj.results = struct();
            
            % Load data
            obj.load_data();
            
            fprintf('Loaded %d samples from %s\n', height(obj.data), log_file);
        end
        
        function load_data(obj)
            % Load data from CSV file
            
            try
                obj.data = readtable(obj.log_file);
                
                % Add computed columns
                if ismember('x', obj.data.Properties.VariableNames) && ...
                   ismember('y', obj.data.Properties.VariableNames)
                    % Calculate distance traveled
                    dx = diff([0; obj.data.x]);
                    dy = diff([0; obj.data.y]);
                    obj.data.distance = cumsum(sqrt(dx.^2 + dy.^2));
                    
                    % Calculate horizontal velocity
                    dt = diff([0; obj.data.timestamp]);
                    dt(dt == 0) = 0.02; % Avoid division by zero
                    obj.data.vx = dx ./ dt;
                    obj.data.vy = dy ./ dt;
                    obj.data.v_horizontal = sqrt(obj.data.vx.^2 + obj.data.vy.^2);
                end
                
                % Convert angles to degrees
                angle_cols = {'roll', 'pitch', 'yaw'};
                for i = 1:length(angle_cols)
                    col = angle_cols{i};
                    if ismember(col, obj.data.Properties.VariableNames)
                        obj.data.([col '_deg']) = rad2deg(obj.data.(col));
                    end
                end
                
            catch ME
                fprintf('Error loading data: %s\n', ME.message);
                obj.data = table();
            end
        end
        
        function plot_trajectory_2d(obj, save_path)
            % Plot 2D trajectory views
            %
            % Args:
            %   save_path: Optional path to save figure
            
            if height(obj.data) == 0
                fprintf('No data to plot\n');
                return;
            end
            
            figure('Name', '2D Trajectory Analysis', 'NumberTitle', 'off', ...
                   'Position', [100, 100, 1200, 900]);
            
            % X-Y trajectory
            subplot(2, 2, 1);
            plot(obj.data.x, obj.data.y, 'b-', 'LineWidth', 1.5);
            hold on;
            plot(obj.data.x(1), obj.data.y(1), 'go', 'MarkerSize', 10, ...
                 'MarkerFaceColor', 'g');
            plot(obj.data.x(end), obj.data.y(end), 'rs', 'MarkerSize', 10, ...
                 'MarkerFaceColor', 'r');
            xlabel('X Position (m)');
            ylabel('Y Position (m)');
            title('2D Trajectory (Top View)');
            grid on;
            legend('Trajectory', 'Start', 'End', 'Location', 'best');
            axis equal;
            
            % X-Z trajectory
            subplot(2, 2, 2);
            plot(obj.data.x, obj.data.z, 'b-', 'LineWidth', 1.5);
            xlabel('X Position (m)');
            ylabel('Z Position (m)');
            title('Side View (X-Z)');
            grid on;
            set(gca, 'YDir', 'reverse'); % NED convention
            
            % Position time series
            subplot(2, 2, 3);
            time = obj.data.timestamp - obj.data.timestamp(1);
            plot(time, obj.data.x, 'r-', 'LineWidth', 1.5);
            hold on;
            plot(time, obj.data.y, 'g-', 'LineWidth', 1.5);
            plot(time, obj.data.z, 'b-', 'LineWidth', 1.5);
            xlabel('Time (s)');
            ylabel('Position (m)');
            title('Position vs Time');
            grid on;
            legend('X', 'Y', 'Z', 'Location', 'best');
            
            % Orientation time series
            subplot(2, 2, 4);
            if ismember('roll_deg', obj.data.Properties.VariableNames)
                plot(time, obj.data.roll_deg, 'r-', 'LineWidth', 1.5);
                hold on;
                plot(time, obj.data.pitch_deg, 'g-', 'LineWidth', 1.5);
                plot(time, obj.data.yaw_deg, 'b-', 'LineWidth', 1.5);
                xlabel('Time (s)');
                ylabel('Angle (degrees)');
                title('Orientation vs Time');
                grid on;
                legend('Roll', 'Pitch', 'Yaw', 'Location', 'best');
            end
            
            if nargin > 1 && ~isempty(save_path)
                saveas(gcf, save_path);
                fprintf('Figure saved to %s\n', save_path);
            end
        end
        
        function plot_trajectory_3d(obj, save_path)
            % Plot 3D trajectory
            %
            % Args:
            %   save_path: Optional path to save figure
            
            if height(obj.data) == 0
                fprintf('No data to plot\n');
                return;
            end
            
            figure('Name', '3D Trajectory', 'NumberTitle', 'off', ...
                   'Position', [200, 200, 1000, 800]);
            
            % 3D trajectory
            plot3(obj.data.x, obj.data.y, -obj.data.z, 'b-', 'LineWidth', 1.5);
            hold on;
            
            % Start and end points
            plot3(obj.data.x(1), obj.data.y(1), -obj.data.z(1), ...
                  'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
            plot3(obj.data.x(end), obj.data.y(end), -obj.data.z(end), ...
                  'rs', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
            
            % Color by time
            time = obj.data.timestamp - obj.data.timestamp(1);
            skip = max(1, floor(height(obj.data) / 100)); % Subsample for performance
            scatter3(obj.data.x(1:skip:end), obj.data.y(1:skip:end), ...
                    -obj.data.z(1:skip:end), 20, time(1:skip:end), 'filled');
            
            xlabel('X Position (m)');
            ylabel('Y Position (m)');
            zlabel('Altitude (m)');
            title('3D Trajectory');
            grid on;
            colorbar;
            colormap('jet');
            legend('Trajectory', 'Start', 'End', 'Location', 'best');
            view(45, 30);
            
            if nargin > 1 && ~isempty(save_path)
                saveas(gcf, save_path);
                fprintf('Figure saved to %s\n', save_path);
            end
        end
        
        function plot_sensor_data(obj, save_path)
            % Plot sensor data analysis
            %
            % Args:
            %   save_path: Optional path to save figure
            
            if height(obj.data) == 0
                fprintf('No data to plot\n');
                return;
            end
            
            figure('Name', 'Sensor Data Analysis', 'NumberTitle', 'off', ...
                   'Position', [300, 100, 1400, 1000]);
            
            time = obj.data.timestamp - obj.data.timestamp(1);
            
            % Accelerometer data
            subplot(2, 3, 1);
            if ismember('accel_x', obj.data.Properties.VariableNames)
                plot(time, obj.data.accel_x, 'r-', 'LineWidth', 1);
                hold on;
                plot(time, obj.data.accel_y, 'g-', 'LineWidth', 1);
                plot(time, obj.data.accel_z, 'b-', 'LineWidth', 1);
                xlabel('Time (s)');
                ylabel('Acceleration (m/s²)');
                title('Accelerometer Data');
                grid on;
                legend('X', 'Y', 'Z', 'Location', 'best');
            end
            
            % Gyroscope data
            subplot(2, 3, 2);
            if ismember('gyro_x', obj.data.Properties.VariableNames)
                plot(time, rad2deg(obj.data.gyro_x), 'r-', 'LineWidth', 1);
                hold on;
                plot(time, rad2deg(obj.data.gyro_y), 'g-', 'LineWidth', 1);
                plot(time, rad2deg(obj.data.gyro_z), 'b-', 'LineWidth', 1);
                xlabel('Time (s)');
                ylabel('Angular Velocity (deg/s)');
                title('Gyroscope Data');
                grid on;
                legend('X', 'Y', 'Z', 'Location', 'best');
            end
            
            % Velocity estimates
            subplot(2, 3, 3);
            if ismember('vz', obj.data.Properties.VariableNames)
                plot(time, obj.data.vz, 'b-', 'LineWidth', 1.5);
                hold on;
            end
            if ismember('v_horizontal', obj.data.Properties.VariableNames)
                plot(time, obj.data.v_horizontal, 'r-', 'LineWidth', 1.5);
            end
            xlabel('Time (s)');
            ylabel('Velocity (m/s)');
            title('Velocity Estimates');
            grid on;
            legend('Vertical', 'Horizontal', 'Location', 'best');
            
            % Covariance trace
            subplot(2, 3, 4);
            if ismember('cov_trace', obj.data.Properties.VariableNames)
                semilogy(time, obj.data.cov_trace, 'Color', [0.5, 0, 0.5], ...
                        'LineWidth', 1.5);
                xlabel('Time (s)');
                ylabel('Covariance Trace');
                title('Filter Uncertainty');
                grid on;
            end
            
            % Histogram of accelerometer noise
            subplot(2, 3, 5);
            if ismember('accel_z', obj.data.Properties.VariableNames)
                % Estimate noise from Z-axis (should be ~9.81 when stationary)
                accel_z_corrected = obj.data.accel_z - mean(obj.data.accel_z);
                histogram(accel_z_corrected, 50, 'FaceColor', 'b', 'EdgeColor', 'none');
                xlabel('Acceleration Error (m/s²)');
                ylabel('Count');
                title('Accelerometer Z Noise Distribution');
                grid on;
                
                % Add normal fit
                hold on;
                pd = fitdist(accel_z_corrected, 'Normal');
                x_values = linspace(min(accel_z_corrected), max(accel_z_corrected), 100);
                y_values = pdf(pd, x_values) * height(obj.data) * ...
                          (max(accel_z_corrected) - min(accel_z_corrected)) / 50;
                plot(x_values, y_values, 'r-', 'LineWidth', 2);
                legend('Data', sprintf('Normal (σ=%.3f)', pd.sigma), 'Location', 'best');
            end
            
            % Power spectral density
            subplot(2, 3, 6);
            if ismember('gyro_z', obj.data.Properties.VariableNames)
                Fs = 1 / mean(diff(obj.data.timestamp)); % Sampling frequency
                [pxx, f] = pwelch(obj.data.gyro_z, [], [], [], Fs);
                loglog(f, pxx, 'b-', 'LineWidth', 1.5);
                xlabel('Frequency (Hz)');
                ylabel('Power/Frequency (rad²/s²/Hz)');
                title('Gyro Z Power Spectral Density');
                grid on;
            end
            
            if nargin > 1 && ~isempty(save_path)
                saveas(gcf, save_path);
                fprintf('Figure saved to %s\n', save_path);
            end
        end
        
        function results = analyze_performance(obj)
            % Analyze EKF performance metrics
            %
            % Returns:
            %   results: Structure containing performance metrics
            
            results = struct();
            
            if height(obj.data) == 0
                obj.results = results;
                return;
            end
            
            % Basic statistics
            results.duration = obj.data.timestamp(end) - obj.data.timestamp(1);
            results.samples = height(obj.data);
            results.sample_rate = results.samples / results.duration;
            
            % Position metrics
            if ismember('distance', obj.data.Properties.VariableNames)
                results.total_distance = obj.data.distance(end);
            end
            
            if ismember('x', obj.data.Properties.VariableNames)
                results.position_range = struct(...
                    'x', [min(obj.data.x), max(obj.data.x)], ...
                    'y', [min(obj.data.y), max(obj.data.y)], ...
                    'z', [min(obj.data.z), max(obj.data.z)]);
            end
            
            % Orientation statistics
            if ismember('roll_deg', obj.data.Properties.VariableNames)
                results.orientation_stats = struct(...
                    'roll', struct('mean', mean(obj.data.roll_deg), ...
                                  'std', std(obj.data.roll_deg), ...
                                  'range', [min(obj.data.roll_deg), max(obj.data.roll_deg)]), ...
                    'pitch', struct('mean', mean(obj.data.pitch_deg), ...
                                   'std', std(obj.data.pitch_deg), ...
                                   'range', [min(obj.data.pitch_deg), max(obj.data.pitch_deg)]), ...
                    'yaw', struct('mean', mean(obj.data.yaw_deg), ...
                                 'std', std(obj.data.yaw_deg), ...
                                 'range', [min(obj.data.yaw_deg), max(obj.data.yaw_deg)]));
            end
            
            % Sensor noise analysis
            if ismember('v_horizontal', obj.data.Properties.VariableNames)
                % Find stationary periods
                velocity_threshold = 0.1; % m/s
                stationary_mask = obj.data.v_horizontal < velocity_threshold;
                
                if sum(stationary_mask) > 10
                    results.sensor_noise = struct(...
                        'accel_x_std', std(obj.data.accel_x(stationary_mask)), ...
                        'accel_y_std', std(obj.data.accel_y(stationary_mask)), ...
                        'accel_z_std', std(obj.data.accel_z(stationary_mask)), ...
                        'gyro_x_std', std(obj.data.gyro_x(stationary_mask)), ...
                        'gyro_y_std', std(obj.data.gyro_y(stationary_mask)), ...
                        'gyro_z_std', std(obj.data.gyro_z(stationary_mask)));
                end
            end
            
            % Covariance analysis
            if ismember('cov_trace', obj.data.Properties.VariableNames)
                results.covariance = struct(...
                    'initial', obj.data.cov_trace(1), ...
                    'final', obj.data.cov_trace(end), ...
                    'mean', mean(obj.data.cov_trace), ...
                    'converged', obj.data.cov_trace(end) < obj.data.cov_trace(1));
            end
            
            obj.results = results;
        end
        
        function comparison = compare_with_ground_truth(obj, ground_truth_file)
            % Compare EKF estimates with ground truth
            %
            % Args:
            %   ground_truth_file: Path to ground truth CSV file
            %
            % Returns:
            %   comparison: Structure containing comparison metrics
            
            comparison = struct();
            
            try
                gt_data = readtable(ground_truth_file);
                
                % Align data (simplified - assumes same sampling)
                min_len = min(height(obj.data), height(gt_data));
                
                % Position RMSE
                if ismember('x', obj.data.Properties.VariableNames) && ...
                   ismember('x', gt_data.Properties.VariableNames)
                    
                    pos_error = sqrt(...
                        (obj.data.x(1:min_len) - gt_data.x(1:min_len)).^2 + ...
                        (obj.data.y(1:min_len) - gt_data.y(1:min_len)).^2 + ...
                        (obj.data.z(1:min_len) - gt_data.z(1:min_len)).^2);
                    
                    comparison.position_rmse = mean(pos_error);
                    comparison.position_max_error = max(pos_error);
                    comparison.position_std = std(pos_error);
                    
                    % Plot comparison
                    figure('Name', 'Ground Truth Comparison', 'NumberTitle', 'off');
                    
                    subplot(2, 1, 1);
                    plot(obj.data.x(1:min_len), obj.data.y(1:min_len), 'b-', ...
                         'LineWidth', 1.5);
                    hold on;
                    plot(gt_data.x(1:min_len), gt_data.y(1:min_len), 'r--', ...
                         'LineWidth', 1.5);
                    xlabel('X Position (m)');
                    ylabel('Y Position (m)');
                    title('Trajectory Comparison');
                    legend('EKF', 'Ground Truth', 'Location', 'best');
                    grid on;
                    axis equal;
                    
                    subplot(2, 1, 2);
                    time = obj.data.timestamp(1:min_len) - obj.data.timestamp(1);
                    plot(time, pos_error, 'r-', 'LineWidth', 1.5);
                    xlabel('Time (s)');
                    ylabel('Position Error (m)');
                    title('Position Error over Time');
                    grid on;
                end
                
                % Orientation RMSE
                if ismember('roll', obj.data.Properties.VariableNames) && ...
                   ismember('roll', gt_data.Properties.VariableNames)
                    
                    orient_error = sqrt(...
                        (obj.data.roll(1:min_len) - gt_data.roll(1:min_len)).^2 + ...
                        (obj.data.pitch(1:min_len) - gt_data.pitch(1:min_len)).^2 + ...
                        (obj.data.yaw(1:min_len) - gt_data.yaw(1:min_len)).^2);
                    
                    comparison.orientation_rmse = rad2deg(mean(orient_error));
                    comparison.orientation_max_error = rad2deg(max(orient_error));
                    comparison.orientation_std = rad2deg(std(orient_error));
                end
                
            catch ME
                fprintf('Error comparing with ground truth: %s\n', ME.message);
            end
        end
        
        function generate_report(obj, output_dir)
            % Generate comprehensive analysis report
            %
            % Args:
            %   output_dir: Directory to save report
            
            if nargin < 2
                output_dir = '.';
            end
            
            % Ensure output directory exists
            if ~exist(output_dir, 'dir')
                mkdir(output_dir);
            end
            
            % Analyze performance
            results = obj.analyze_performance();
            
            % Create report file
            report_file = fullfile(output_dir, 'analysis_report.txt');
            fid = fopen(report_file, 'w');
            
            fprintf(fid, '============================================================\n');
            fprintf(fid, 'EKF Data Analysis Report\n');
            fprintf(fid, '============================================================\n\n');
            fprintf(fid, 'Log File: %s\n', obj.log_file);
            fprintf(fid, 'Analysis Date: %s\n\n', datestr(now));
            
            fprintf(fid, '----------------------------------------\n');
            fprintf(fid, 'Basic Statistics\n');
            fprintf(fid, '----------------------------------------\n');
            fprintf(fid, 'Duration: %.2f seconds\n', results.duration);
            fprintf(fid, 'Samples: %d\n', results.samples);
            fprintf(fid, 'Sample Rate: %.2f Hz\n', results.sample_rate);
            
            if isfield(results, 'total_distance')
                fprintf(fid, 'Total Distance: %.2f meters\n', results.total_distance);
            end
            
            if isfield(results, 'position_range')
                fprintf(fid, '\nPosition Range:\n');
                fprintf(fid, '  X: [%.2f, %.2f] meters\n', results.position_range.x);
                fprintf(fid, '  Y: [%.2f, %.2f] meters\n', results.position_range.y);
                fprintf(fid, '  Z: [%.2f, %.2f] meters\n', results.position_range.z);
            end
            
            if isfield(results, 'orientation_stats')
                fprintf(fid, '\nOrientation Statistics:\n');
                fprintf(fid, '  Roll:\n');
                fprintf(fid, '    Mean: %.2f°\n', results.orientation_stats.roll.mean);
                fprintf(fid, '    Std: %.2f°\n', results.orientation_stats.roll.std);
                fprintf(fid, '    Range: [%.2f°, %.2f°]\n', results.orientation_stats.roll.range);
                
                fprintf(fid, '  Pitch:\n');
                fprintf(fid, '    Mean: %.2f°\n', results.orientation_stats.pitch.mean);
                fprintf(fid, '    Std: %.2f°\n', results.orientation_stats.pitch.std);
                fprintf(fid, '    Range: [%.2f°, %.2f°]\n', results.orientation_stats.pitch.range);
                
                fprintf(fid, '  Yaw:\n');
                fprintf(fid, '    Mean: %.2f°\n', results.orientation_stats.yaw.mean);
                fprintf(fid, '    Std: %.2f°\n', results.orientation_stats.yaw.std);
                fprintf(fid, '    Range: [%.2f°, %.2f°]\n', results.orientation_stats.yaw.range);
            end
            
            if isfield(results, 'sensor_noise')
                fprintf(fid, '\nSensor Noise (Stationary):\n');
                fprintf(fid, '  Accel X Std: %.6f m/s²\n', results.sensor_noise.accel_x_std);
                fprintf(fid, '  Accel Y Std: %.6f m/s²\n', results.sensor_noise.accel_y_std);
                fprintf(fid, '  Accel Z Std: %.6f m/s²\n', results.sensor_noise.accel_z_std);
                fprintf(fid, '  Gyro X Std: %.6f rad/s\n', results.sensor_noise.gyro_x_std);
                fprintf(fid, '  Gyro Y Std: %.6f rad/s\n', results.sensor_noise.gyro_y_std);
                fprintf(fid, '  Gyro Z Std: %.6f rad/s\n', results.sensor_noise.gyro_z_std);
            end
            
            if isfield(results, 'covariance')
                fprintf(fid, '\nCovariance Analysis:\n');
                fprintf(fid, '  Initial: %.6f\n', results.covariance.initial);
                fprintf(fid, '  Final: %.6f\n', results.covariance.final);
                fprintf(fid, '  Mean: %.6f\n', results.covariance.mean);
                fprintf(fid, '  Converged: %s\n', mat2str(results.covariance.converged));
            end
            
            fclose(fid);
            
            fprintf('Report saved to %s\n', report_file);
            
            % Save results as MAT file
            results_file = fullfile(output_dir, 'analysis_results.mat');
            save(results_file, 'results');
            fprintf('Results saved to %s\n', results_file);
        end
    end
end
