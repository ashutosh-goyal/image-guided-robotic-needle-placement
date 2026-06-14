import numpy as np
import rospy
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import pickle
import os
from scipy.optimize import minimize

#Joint angle limit
theta_max_deg = [166,101,166,-4,166,215,166]
theta_max_rad = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]
theta_min_deg = [-166,-101,-166,-176,-166,-1,-166]
theta_min_rad = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]

theta_dot_max = [2.1750,2.1750,2.1750,2.1750,2.6100,2.6100,2.6100]
theta_ddot_max= [15,7.5,10,12.5,15,20,20]



def theta_poly(t, coeffs):
    return sum(c * t**i for i, c in enumerate(coeffs))

def dtheta_poly(t, coeffs):
    return sum(i * c * t**(i - 1) for i, c in enumerate(coeffs) if i > 0)

def ddtheta_poly(t, coeffs):
    return sum(i * (i - 1) * c * t**(i - 2) for i, c in enumerate(coeffs) if i > 1)

def generate_quintic_with_constraints(theta_i, theta_f, vel_i, vel_f, acc_i, acc_f,
                                      theta_min, theta_max, T):
    def boundary_constraints(coeffs):
        return [
            coeffs[0] - theta_i,                               # theta(0)
            coeffs[1] - vel_i,                                 # dtheta(0)
            2 * coeffs[2] - acc_i,                             # ddtheta(0)
            theta_poly(T, coeffs) - theta_f,                  # theta(T)
            dtheta_poly(T, coeffs) - vel_f,                   # dtheta(T)
            ddtheta_poly(T, coeffs) - acc_f                   # ddtheta(T)
        ]

    def path_constraints(coeffs):
        t_vals = np.linspace(0, T, 100)
        return [
            theta_poly(t, coeffs) - theta_min for t in t_vals
        ] + [
            theta_max - theta_poly(t, coeffs) for t in t_vals
        ]

    def objective(coeffs):
        # Minimize jerk squared over time (optional: better than just sum of squares)
        t_vals = np.linspace(0, T, 100)
        jerk_vals = [sum(i*(i-1)*(i-2)*c*t**(i-3) for i, c in enumerate(coeffs) if i > 2) for t in t_vals]
        return sum((j**2) for j in jerk_vals)

    cons = (
        [{'type': 'eq', 'fun': lambda c, i=i: boundary_constraints(c)[i]} for i in range(6)] +
        [{'type': 'ineq', 'fun': lambda c, i=i: path_constraints(c)[i]} for i in range(200)]
    )

    res = minimize(
        objective,
        x0=np.zeros(6),  # initial guess
        constraints=cons,
        method='SLSQP',
        options={'disp': False, 'maxiter': 1000}
    )

    if res.success:
        return res.x
    else:
        raise RuntimeError("Trajectory optimization failed: " + res.message)



def plot_joint_trajectories(coeffs_list, T):
    t = np.linspace(0, T, 30000)
    plt.figure(figsize=(10, 6))

    for i, coeffs in enumerate(coeffs_list):
        theta = sum(c * t**j for j, c in enumerate(coeffs))
        theta_array.append(theta)
        plt.plot(t, theta, label=f'Joint {i+1}')

    plt.xlabel("Time [s]")
    plt.ylabel("Theta(t) [rad]")
    plt.title("Quintic Trajectories for 7 Joints")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


def plot_joint_vel_trajectories(coeffs_list, T):
    t = np.linspace(0, T, 100)
    plt.figure(figsize=(10, 6))

    for i, coeffs in enumerate(coeffs_list):
        theta_dot = sum(j * c * t**(j - 1) for j, c in enumerate(coeffs) if j > 0)
        
        plt.plot(t, theta_dot, label=f'Joint {i+1}')

    plt.xlabel("Time [s]")
    plt.ylabel("Velocity(t) [rad/s]")
    plt.title("Quintic velocity Trajectories for 7 Joints")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

def plot_joint_acc_trajectories(coeffs_list, T):
    t = np.linspace(0, T, 100)
    plt.figure(figsize=(10, 6))

    for i, coeffs in enumerate(coeffs_list):
        theta_ddot = sum(j*(j-1) * c * t**(j - 2) for j, c in enumerate(coeffs) if j > 1)
        
        plt.plot(t, theta_ddot, label=f'Joint {i+1}')

    plt.xlabel("Time [s]")
    plt.ylabel("Accerlation(t) [rad/s^2]")
    plt.title("Quintic acceleration Trajectories for 7 Joints")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

def plot_jerk(coeffs_list, T):
    t = np.linspace(0, T, 100)
    plt.figure(figsize=(10, 6))

    for i, coeffs in enumerate(coeffs_list):
        jerk_vals_ = sum(j*(j-1)*(j-2) * c * t**(j - 3) for j, c in enumerate(coeffs) if j > 2)
        
        plt.plot(t, jerk_vals_, label=f'Joint {i+1}')

    plt.xlabel("Time [s]")
    plt.ylabel("Jerk(t) [rad/s^3]")
    plt.title("Quintic jerk Trajectories for 7 Joints")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

def is_trajectory_within_limits(coeffs, theta_min, theta_max, T):
    t = np.linspace(0, T, 100)
    theta = sum(c * t**j for j, c in enumerate(coeffs))
    return np.all(theta >= theta_min) and np.all(theta <= theta_max)


def save_theta_array(theta_array):
    # Get the directory of the current script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Join it with the desired filename
    file_path = os.path.join(script_dir, "theta_array_optimize.pkl")

    # Save the pickle file
    with open(file_path, "wb") as f:
        pickle.dump(theta_array, f)   

def get_current_joint_positions():
    rospy.loginfo("Waiting for /franka_state_controller/joint_states_desired message...")
    joint_msg = rospy.wait_for_message("/franka_state_controller/joint_states_desired", JointState)
    rospy.loginfo("Received /joint_states message.")
    
    # Make sure joint names are ordered as expected
    expected_joints = ['panda_joint1', 'panda_joint2', 'panda_joint3', 
                       'panda_joint4', 'panda_joint5', 'panda_joint6', 
                       'panda_joint7']
    
    joint_position_map = dict(zip(joint_msg.name, joint_msg.position))

    try:
        joint_positions = [joint_position_map[name] for name in expected_joints]
    except KeyError as e:
        rospy.logerr(f"Missing joint in JointState message: {e}")
        raise
    return joint_positions





if __name__ == "__main__":
    T = 30  # duration [s]
    rospy.init_node("quintic_trajectory_node")
    theta_initial = get_current_joint_positions()
  # theta_final = [-1.305847765944419, -0.5837905412593624, 0.9722493137318714, -2.9337114976581775, 0.21761065227241905, 2.6249116693277226, 2.0964363360244103]
    #theta_final = [0.03525317204742065, -0.5316906221456731, -0.014692200273323835, -2.1140818216194144, -0.0723921620588234, 1.4837963331540298, -0.6863871216088581]
    theta_final = [ 0.16449257, -0.36249767, 0.13739677, -2.32537803, -0.45202738, 1.55265721, -0.80380215]


    # Example initial and final values for 7 joints
    theta_i = [theta_initial[0],theta_initial[1],theta_initial[2],theta_initial[3],theta_initial[4],theta_initial[5],theta_initial[6]]
    theta_f = [theta_final[0],theta_final[1],theta_final[2],theta_final[3],theta_final[4],theta_final[5],theta_final[6]]
    vel_i   = [0.0] * 7
    vel_f   = [0.0] * 7
    acc_i   = [0.0] * 7   # you can change per joint if needed
    acc_f   = [0.0] * 7  # same here

    coeffs_list = []
    theta_array = []
    for i in range(7):
        coeffs = generate_quintic_with_constraints(theta_i[i], theta_f[i], vel_i[i], vel_f[i], acc_i[i], acc_f[i],
                                      theta_min_rad[i], theta_max_rad[i], T)
        
        if is_trajectory_within_limits(coeffs, theta_min_rad[i], theta_max_rad[i], T):
            coeffs_list.append(coeffs)
            print(i+1)
            continue   
        else:
            raise ValueError(f"Cannot find valid trajectory for joint {i+1}")
    
    plot_joint_trajectories(coeffs_list, T)
    plot_joint_vel_trajectories(coeffs_list, T)
    plot_joint_acc_trajectories(coeffs_list, T)
    plot_jerk(coeffs_list, T)
    save_theta_array(theta_array)
    # theta_sim_max = [max(theta_array[0]),max(theta_array[1]),max(theta_array[2]),max(theta_array[3]),max(theta_array[4]),max(theta_array[5]),max(theta_array[6])]
    # theta_sim_min = [min(theta_array[0]),min(theta_array[1]),min(theta_array[2]),min(theta_array[3]),min(theta_array[4]),min(theta_array[5]),min(theta_array[6])]
    # print(theta_sim_max)
    # print(theta_sim_min)
    #print(theta_array[0])
