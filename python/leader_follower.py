import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import solve_continuous_are, inv

# Define system parameters
radius = 5.0  # Radius of the circle
omega_L = 0.1  # Angular velocity of the leader robot
dt = 0.1  # Time step
T = 20  # Total simulation time

# Define cost matrices
Q = np.diag([10, 10, 1])  # Increased weights for position errors
R = np.diag([1, 1])

# Define system matrices for linearized error dynamics
def A(v_L, theta_L):
    return np.array([[0, 0, -v_L * np.sin(theta_L)],
                     [0, 0, v_L * np.cos(theta_L)],
                     [0, 0, 0]])

def B(theta_L):
    return np.array([[np.cos(theta_L), 0],
                     [np.sin(theta_L), 0],
                     [0, 1]])

# Solve Riccati equation
v_L = radius * omega_L  # Linear velocity of the leader robot
P = solve_continuous_are(A(v_L, 0), B(0), Q, R)
K = inv(R) @ B(0).T @ P

# Extract gains
k_1, k_2, k_3 = K[0, 0], K[0, 1], K[0, 2]

# Initialize state vectors
x_L, y_L, theta_L = radius, 0, np.pi / 2  # Leader initial state (starting at (radius, 0))
x_F, y_F, theta_F = radius - 1, -1, np.pi / 2  # Follower initial state

# Lists to store trajectories
leader_trajectory = []
follower_trajectory = []

# Simulation loop
for t in np.arange(0, T, dt):
    # Leader robot dynamics (circular trajectory)
    x_L = radius * np.cos(omega_L * t)
    y_L = radius * np.sin(omega_L * t)
    theta_L = omega_L * t + np.pi / 2  # Ensure theta_L is always tangent to the circle

    # Error states
    e_x = x_F - x_L
    e_y = y_F - y_L
    e_theta = theta_F - theta_L

    # Control inputs for follower robot
    v_F = v_L + k_1 * e_x
    omega_F = k_2 * e_y + k_3 * e_theta

    # Follower robot dynamics
    x_F += v_F * np.cos(theta_F) * dt
    y_F += v_F * np.sin(theta_F) * dt
    theta_F += omega_F * dt

    # Store trajectories
    leader_trajectory.append((x_L, y_L))
    follower_trajectory.append((x_F, y_F))

# Convert trajectories to numpy arrays for plotting
leader_trajectory = np.array(leader_trajectory)
follower_trajectory = np.array(follower_trajectory)

# Plot trajectories
plt.figure()
plt.plot(leader_trajectory[:, 0], leader_trajectory[:, 1], label='Leader Trajectory')
plt.plot(follower_trajectory[:, 0], follower_trajectory[:, 1], label='Follower Trajectory')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.legend()
plt.title('Leader-Follower Robot Trajectories (Circular Path)')
plt.grid()
plt.show()
