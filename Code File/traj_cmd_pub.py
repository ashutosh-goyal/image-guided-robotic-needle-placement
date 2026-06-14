#!/usr/bin/env python3
import rospy
import numpy as np
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header
from std_msgs.msg import Float64MultiArray
import tf_conversions
import pickle
import os

#### This file is the publisher to the /joint_command topic, which will publish the joint angle array and received be the rviz file which will take the robot to the given 
##angles

# theta = [0.7841690919734665, 0.7841588779057842, 0.784168091797877, -0.0697998582713355, -0.7841657899582453, 0.7841634642067703, 0.7841636375562002]
# theta = [-7.201853051697071e-06, 4.240218114404115e-06, -5.556162024689115e-06, -0.06980008708802465, 3.896703600680951e-06, -1.1387677814056474e-06, -1.0336209088634973e-06]
##theta = [0.83906942, 0.77821194, -0.26351373, -0.06026793, 0.20080255, 0.78723814, 0.75006403]
def load_theta_array():
    # Get the directory of the current script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Full path to the file
    #file_path = os.path.join(script_dir, "theta_array_optimize.pkl")
    file_path = os.path.join(script_dir, "final.pkl")
 
    # Optional: check if file exists
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"theta_array_optimize.pkl not found at: {file_path}")

    # Load and return
    with open(file_path, "rb") as f:
        return pickle.load(f)



def publisher():
    rospy.init_node('joint_traj__angle_publisher')
    pub = rospy.Publisher('/joint_position_example_controller/joint_trajectory_command', Float64MultiArray, queue_size=10)
    rate = rospy.Rate(1000)  # 10 Hz

     # Only 7 joint values sent

    ## rospy.loginfo("Publishing joint angles: %s", theta)

    for i in range(len(theta_array)):
        if rospy.is_shutdown():
            break

        msg = Float64MultiArray()
        #msg.data = [theta_array[0][i],theta_array[1][i],theta_array[2][i],theta_array[3][i],theta_array[4][i],theta_array[5][i],theta_array[6][i]]
        msg.data = [theta_array[i][0],theta_array[i][1],theta_array[i][2],theta_array[i][3],theta_array[i][4],theta_array[i][5],theta_array[i][6]]
        
        pub.publish(msg)
        rate.sleep()

    # while not rospy.is_shutdown():
    #     pub.publish(msg)
        

if __name__ == '__main__':
    theta_array = load_theta_array()
    publisher()

   