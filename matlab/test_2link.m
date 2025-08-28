%% 2-Link Robot Arm Simulation
clear; clc; close all;

%% Robot Parameters
L1 = 1.0;
L2 = 0.5;
m1 = 6.7;
m2 = 3.35;
q1_0 = pi;
q2_0 = 0.0;

try
    robot = Robot([L1, L2], [m1, m2], [q1_0, q2_0]);
    fprintf('Robot created successfully with %d links\n', robot.numLinks);
catch ME
    fprintf('Error creating robot: %s\n', ME.message);
    return;
end

%% Controller Parameters
num_joints = 2;
input_history_dim = 1;     
output_history_dim = 2;      
lambda = 0.90;             
goal_angle1 = pi;
goal_angle2 = 0.0;
goal_state = [goal_angle1; goal_angle2];
initial_covariance = 10000;

try
    controller = SelfTuningRegulator(num_joints, input_history_dim, output_history_dim, ...
                                     lambda, goal_state, initial_covariance);
    controller.setRobot(robot);
    fprintf('Self-Tuning Regulator created successfully\n');
catch ME
    fprintf('Error creating controller: %s\n', ME.message);
    return;
end

%% Simulation Parameters
dt = 0.001;
T_sim = 10.0;
N_steps = round(T_sim / dt);

fprintf('\nSimulation parameters:\n');
fprintf('  Time step: %.3f s\n', dt);
fprintf('  Simulation time: %.1f s\n', T_sim);
fprintf('  Number of steps: %d\n', N_steps);

q = [q1_0; q2_0];
q_dot = [0.0; 0.0];
tau = [0.0; 0.0];

% Storage for trajectory
q_trajectory = zeros(2, N_steps);
q_dot_trajectory = zeros(2, N_steps);
tau_trajectory = zeros(2, N_steps);
time = zeros(1, N_steps);

%% Run Simulation
for i = 1:N_steps
    % Store current state
    q_trajectory(:, i) = q;
    q_dot_trajectory(:, i) = q_dot;
    tau_trajectory(:, i) = tau;
    time(i) = (i-1) * dt;
    
    % Update robot state
    robot.setJointAngle(q');

    % Compute torque
    try
        current_angles = [q];
        previous_inputs = [tau];
        
        tau_new = controller.computeControl(current_angles, previous_inputs);
        tau = tau_new;
        
        % Limit torques
        max_torque = 50.0; % [Nm]
        tau = max(-max_torque, min(max_torque, tau)); % Element-wise for both joints
        
    catch ME
        fprintf('Controller error at step %d: %s\n', i, ME.message);
        tau = [0.0; 0.0]; % Fallback to no control for both joints
    end
    
    % Simulate one time step
    try
        [q_next, q_dot_next] = robot.simulateStep(q, q_dot, tau, dt);
        q = q_next;
        q_dot = q_dot_next;
    catch ME
        fprintf('Simulation error at step %d: %s\n', i, ME.message);
        break;
    end
    
    % Progress indicator
    if mod(i, round(N_steps/10)) == 0
        error1_deg = rad2deg(abs(q(1) - goal_state(1)));
        error2_deg = rad2deg(abs(q(2) - goal_state(2)));
        fprintf('  Step %d/%d (t=%.2fs), q=[%.2f, %.2f] rad, errors=[%.1f, %.1f] deg, tau=[%.2f, %.2f] Nm\n', ...
                i, N_steps, time(i), q(1), q(2), error1_deg, error2_deg, tau(1), tau(2));
    end
end

fprintf('Simulation completed!\n');

%% Display Results
fprintf('\n=== 2-LINK CONTROL RESULTS ===\n');
fprintf('Final state:\n');
fprintf('  q1_final = %.3f rad (%.1f°)\n', q(1), rad2deg(q(1)));
fprintf('  q2_final = %.3f rad (%.1f°)\n', q(2), rad2deg(q(2)));
fprintf('  q1_dot_final = %.3f rad/s\n', q_dot(1));
fprintf('  q2_dot_final = %.3f rad/s\n', q_dot(2));

% Control performance metrics
final_error1 = abs(q(1) - goal_state(1));
final_error2 = abs(q(2) - goal_state(2));
fprintf('\nControl Performance:\n');
fprintf('  Joint 1 final error: %.3f rad (%.1f°)\n', final_error1, rad2deg(final_error1));
fprintf('  Joint 2 final error: %.3f rad (%.1f°)\n', final_error2, rad2deg(final_error2));

% Settling time analysis
error1_traj = abs(q_trajectory(1,:) - goal_state(1));
error2_traj = abs(q_trajectory(2,:) - goal_state(2));
settling1_idx = find(error1_traj < 0.1, 1, 'first');
settling2_idx = find(error2_traj < 0.1, 1, 'first');

if ~isempty(settling1_idx)
    fprintf('  Joint 1 settling time (±0.1 rad): %.2f s\n', time(settling1_idx));
else
    fprintf('  Joint 1 did not settle within ±0.1 rad\n');
end

if ~isempty(settling2_idx)
    fprintf('  Joint 2 settling time (±0.1 rad): %.2f s\n', time(settling2_idx));
else
    fprintf('  Joint 2 did not settle within ±0.1 rad\n');
end

% Maximum torques used
max_tau1 = max(abs(tau_trajectory(1,:)));
max_tau2 = max(abs(tau_trajectory(2,:)));
fprintf('\nMaximum torques used:\n');
fprintf('  Joint 1: %.2f Nm\n', max_tau1);
fprintf('  Joint 2: %.2f Nm\n', max_tau2);

% Motion ranges
q1_range = max(q_trajectory(1,:)) - min(q_trajectory(1,:));
q2_range = max(q_trajectory(2,:)) - min(q_trajectory(2,:));
fprintf('\nMotion ranges:\n');
fprintf('  Joint 1: %.3f rad (%.1f°)\n', q1_range, rad2deg(q1_range));
fprintf('  Joint 2: %.3f rad (%.1f°)\n', q2_range, rad2deg(q2_range));

%% Plot Results
figure('Name', '2-Link Self-Tuning Regulator Results', 'Position', [100, 100, 1400, 900]);

% Joint angles with goals
subplot(3,2,1);
plot(time, rad2deg(q_trajectory(1,:)), 'b-', 'LineWidth', 2);
hold on;
plot(time, rad2deg(goal_state(1)) * ones(size(time)), 'b--', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Joint 1 Angle [deg]');
title('Joint 1 Angle vs Time');
legend('Actual', 'Goal', 'Location', 'best');
xlim([0, T_sim]);

subplot(3,2,2);
plot(time, rad2deg(q_trajectory(2,:)), 'r-', 'LineWidth', 2);
hold on;
plot(time, rad2deg(goal_state(2)) * ones(size(time)), 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Joint 2 Angle [deg]');
title('Joint 2 Angle vs Time');
legend('Actual', 'Goal', 'Location', 'best');
xlim([0, T_sim]);

% Joint velocities
subplot(3,2,3);
plot(time, q_dot_trajectory(1,:), 'b-', 'LineWidth', 2);
grid on;
xlabel('Time [s]');
ylabel('Joint 1 Velocity [rad/s]');
title('Joint 1 Velocity vs Time');
xlim([0, T_sim]);

subplot(3,2,4);
plot(time, q_dot_trajectory(2,:), 'r-', 'LineWidth', 2);
grid on;
xlabel('Time [s]');
ylabel('Joint 2 Velocity [rad/s]');
title('Joint 2 Velocity vs Time');
xlim([0, T_sim]);

% Control torques
subplot(3,2,5);
plot(time, tau_trajectory(1,:), 'b-', 'LineWidth', 2);
grid on;
xlabel('Time [s]');
ylabel('Joint 1 Torque [Nm]');
title('Joint 1 Control Torque vs Time');
xlim([0, T_sim]);

subplot(3,2,6);
plot(time, tau_trajectory(2,:), 'r-', 'LineWidth', 2);
grid on;
xlabel('Time [s]');
ylabel('Joint 2 Torque [Nm]');
title('Joint 2 Control Torque vs Time');
xlim([0, T_sim]);

%% Error Analysis
figure('Name', '2-Link Control Error Analysis', 'Position', [150, 150, 1000, 600]);

subplot(2,2,1);
error1_trajectory = rad2deg(abs(q_trajectory(1,:) - goal_state(1)));
semilogy(time, error1_trajectory, 'b-', 'LineWidth', 2);
grid on;
xlabel('Time [s]');
ylabel('Joint 1 Absolute Error [deg]');
title('Joint 1 Control Error vs Time (Log Scale)');
xlim([0, T_sim]);

subplot(2,2,2);
error2_trajectory = rad2deg(abs(q_trajectory(2,:) - goal_state(2)));
semilogy(time, error2_trajectory, 'r-', 'LineWidth', 2);
grid on;
xlabel('Time [s]');
ylabel('Joint 2 Absolute Error [deg]');
title('Joint 2 Control Error vs Time (Log Scale)');
xlim([0, T_sim]);

subplot(2,2,3);
plot(time, rad2deg(q_trajectory(1,:) - goal_state(1)), 'b-', 'LineWidth', 2);
grid on;
xlabel('Time [s]');
ylabel('Joint 1 Tracking Error [deg]');
title('Joint 1 Tracking Error vs Time');
xlim([0, T_sim]);

subplot(2,2,4);
plot(time, rad2deg(q_trajectory(2,:) - goal_state(2)), 'r-', 'LineWidth', 2);
grid on;
xlabel('Time [s]');
ylabel('Joint 2 Tracking Error [deg]');
title('Joint 2 Tracking Error vs Time');
xlim([0, T_sim]);

%% Animation
fprintf('\nStarting 2-link controlled animation...\n');
try
    % Add goal state as target for animation
    target_angles = goal_state';
    robot.animate(q_trajectory, target_angles);
    fprintf('2-link controlled animation completed successfully!\n');
catch ME
    fprintf('Animation error: %s\n', ME.message);
    
    % Debug info
    fprintf('Trajectory stats:\n');
    fprintf('  Size: %dx%d\n', size(q_trajectory));
    fprintf('  q1 range: [%.3f, %.3f] rad\n', min(q_trajectory(1,:)), max(q_trajectory(1,:)));
    fprintf('  q2 range: [%.3f, %.3f] rad\n', min(q_trajectory(2,:)), max(q_trajectory(2,:)));
    fprintf('  Final errors: [%.3f, %.3f] rad\n', final_error1, final_error2);
end

fprintf('\n=== 2-LINK SELF-TUNING CONTROL TEST COMPLETED ===\n');

fprintf('2-link test completed.\n');