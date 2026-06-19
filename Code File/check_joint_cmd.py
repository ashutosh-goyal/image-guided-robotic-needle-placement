import os
import pickle
import numpy as np
import matplotlib.pyplot as plt

script_dir = os.path.dirname(os.path.abspath(__file__))

# Full path to the file
#file_path = os.path.join(script_dir, "theta_initial_optimize.pkl")
file_path_initial = os.path.join(script_dir, "final.pkl")


# Optional: check if file exists



# Load and return
with open(file_path_initial, "rb") as f:
    theta = pickle.load(f)


theta_history = np.array(theta)
#print(theta_history)
#print(len(theta_history))
num_joints = theta_history.shape[1]
duration = 10  # seconds
num_samples = theta_history.shape[0]
time = np.linspace(0, duration, num=num_samples)

plt.figure(figsize=(15, 8))
for i in range(num_joints):
    plt.plot(time, theta_history[:, i], label=f'Joint {i+1}')
plt.title('Joint Angles Over Time')
plt.xlabel('Time (s)')
plt.ylabel('Theta (radians)')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()