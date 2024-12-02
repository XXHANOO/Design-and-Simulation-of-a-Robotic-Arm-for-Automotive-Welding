import numpy as np
import matplotlib.pyplot as plt

# Define parameters for the robotic arm
Kp = 1.2  # Proportional gain
Ki = 0.01  # Integral gain
Kd = 0.05  # Derivative gain
dt = 0.01  # Time step
simulation_time = 10  # Simulation duration in seconds

# Define the desired trajectory (triangular path)
time = np.arange(0, simulation_time, dt)
desired_path = 5 * np.abs(np.sin(2 * np.pi * time / simulation_time))  # Triangle wave

# Initialize variables
position = 0
velocity = 0
error_sum = 0
last_error = 0
positions = []

# Simulate PID control for robotic arm path tracking
for t, desired_position in zip(time, desired_path):
    error = desired_position - position
    error_sum += error * dt
    error_diff = (error - last_error) / dt

    # PID control signal
    control_signal = Kp * error + Ki * error_sum + Kd * error_diff

    # Update dynamics (assume simple mass-spring-damper system)
    acceleration = control_signal
    velocity += acceleration * dt
    position += velocity * dt

    # Store results
    positions.append(position)
    last_error = error

# Plot the results
plt.figure(figsize=(10, 6))
plt.plot(time, desired_path, label='Desired Path', linestyle='--', linewidth=2)
plt.plot(time, positions, label='Actual Path', linewidth=2)
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.title('Robotic Arm Path Tracking with PID Control')
plt.legend()
plt.grid()
plt.show()
