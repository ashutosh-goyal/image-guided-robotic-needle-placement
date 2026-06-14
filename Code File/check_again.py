import os
import pickle
import numpy as np
import matplotlib.pyplot as plt

script_dir = os.path.dirname(os.path.abspath(__file__))

# Full path to the file
#file_path = os.path.join(script_dir, "theta_initial_optimize.pkl")
file_path_initial = os.path.join(script_dir, "theta_array_optimize.pkl")
file_path_final = os.path.join(script_dir, "new_line.pkl")

# Optional: check if file exists

theta_initial = []
theta_final = []

theta = []
# Load and return
with open(file_path_initial, "rb") as f:
    theta_initial = pickle.load(f)


with open(file_path_final, "rb") as f:
    theta_final = pickle.load(f)    

for i in range(len(theta_initial[0])):
    theta.append([theta_initial[0][i],theta_initial[1][i],theta_initial[2][i],theta_initial[3][i],theta_initial[4][i],theta_initial[5][i],theta_initial[6][i]])

for i in range(len(theta_final)):
    theta.append([theta_final[9999-i][0],theta_final[9999-i][1],theta_final[9999-i][2],theta_final[9999-i][3],theta_final[9999-i][4],theta_final[9999-i][5],theta_final[9999-i][6]])    


def save_theta_array(theta_array):
    # Get the directory of the current script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Join it with the desired filename
    file_path = os.path.join(script_dir, "final.pkl")

    # Save the pickle file
    with open(file_path, "wb") as f:
        pickle.dump(theta_array, f)   


save_theta_array(theta)        