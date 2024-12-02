% Parameters
Kp = 1.2; Ki = 0.01; Kd = 0.05; % PID gains
dt = 0.01; % Time step
time = 0:dt:10; % Simulation duration
desired_path = 5 * abs(sin(2 * pi * time / max(time))); % Desired triangular path

% Initialization
position = 0; velocity = 0;
error_sum = 0; last_error = 0;
actual_positions = [];

% PID control simulation
for i = 1:length(time)
    % Compute error
    error = desired_path(i) - position;
    error_sum = error_sum + error * dt;
    error_diff = (error - last_error) / dt;

    % PID control signal
    control_signal = Kp * error + Ki * error_sum + Kd * error_diff;

    % System dynamics
    acceleration = control_signal;
    velocity = velocity + acceleration * dt;
    position = position + velocity * dt;

    % Store results
    actual_positions = [actual_positions, position];
    last_error = error;
end

% Plot results
figure;
plot(time, desired_path, '--', 'DisplayName', 'Desired Path');
hold on;
plot(time, actual_positions, '-', 'DisplayName', 'Actual Path');
xlabel('Time (s)');
ylabel('Position (m)');
title('PID-Controlled Robotic Arm Path Tracking');
legend;
grid on;
