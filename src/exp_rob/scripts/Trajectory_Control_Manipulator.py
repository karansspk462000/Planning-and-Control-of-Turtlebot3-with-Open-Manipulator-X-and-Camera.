#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np
import matplotlib.pyplot as plt

class InverseKinematics:
    def __init__(self):
        rospy.init_node('calculate_joint_angless')
        self.joint_angles_pub = rospy.Publisher('/joint_angles', Float64MultiArray, queue_size=10)
        self.end_effector_positions = [
            [0.186,0,0.205,0],
            [0.286,0,0.205,0],
            [0.286,0.1,0.205,0],
            [0.186,0.1,0.205,0],
            [0.186,0,0.205,0],
            [0.186,0,0.205,0]
        ]
        self.current_position_index = 0

    def P3R_inverse_kinematics(self, a1, a2, a3, x3, y3, theta):
        # Inverse Kinematics of planar 3R robot
        x2 = x3 - a3 * np.cos(theta)
        y2 = y3 - a3 * np.sin(theta)

        cos_th2 = (x2**2 + y2**2 - a1**2 - a2**2) / (2 * a1 * a2)
        sin_th2 = [-np.sqrt(1 - cos_th2**2), np.sqrt(1 - cos_th2**2)]

        th2 = [np.arctan2(sin_th2[0], cos_th2), np.arctan2(sin_th2[1], cos_th2)]

        sin_th1 = [(y2 * (a1 + a2 * np.cos(th2[0])) - a2 * np.sin(th2[0]) * x2) / (a1**2 + a2**2 + 2 * a1 * a2 * np.cos(th2[0])),
                   (y2 * (a1 + a2 * np.cos(th2[1])) - a2 * np.sin(th2[1]) * x2) / (a1**2 + a2**2 + 2 * a1 * a2 * np.cos(th2[1]))]
            
        cos_th1 = [(x2 * (a1 + a2 * np.cos(th2[0])) + a2 * np.sin(th2[0]) * y2) / (a1**2 + a2**2 + 2 * a1 * a2 * np.cos(th2[0])),
                   (x2 * (a1 + a2 * np.cos(th2[1])) + a2 * np.sin(th2[1]) * y2) / (a1**2 + a2**2 + 2 * a1 * a2 * np.cos(th2[1]))]
            
        th1 = [np.arctan2(sin_th1[0], cos_th1[0]), np.arctan2(sin_th1[1], cos_th1[1])]

        th3 = [theta - th1[0] - th2[0], theta - th1[1] - th2[1]]

        return th1, th2, th3

    def omx_inverse_kinematics(self, target_pos, target_rot=None, target_phi=None):
        x = target_pos[0]
        y = target_pos[1]
        z = target_pos[2]
        print(x,y,z)
        d1 = 0.077
        a1 = np.sqrt(0.024**2 + 0.128**2)
        alpha_2 = np.arctan(0.024/0.128)
        a2 = 0.124
        a3 = 0.126

        x_new = np.sqrt(x**2 + y**2)
        y_new = z - d1
        if target_phi is None:
            phi = self.calculate_angle_with_xy_plane(target_rot)
        else:
            phi = -target_phi

        sm_th = (phi + np.pi) % (2 * np.pi) - np.pi
        if -np.pi/2 <= sm_th and sm_th <= np.pi/2:
            x_ph = [[x_new, phi],
                    [-x_new, np.pi - phi]]
        else:
            x_ph = [[x_new, np.pi - phi],
                    [-x_new, phi]]
        
        theta_1 = [np.arctan2(y, x), np.arctan2(-y, -x)]
        thetas = []
        for i in range(2):
            th2, th3, th4 = self.P3R_inverse_kinematics(a1=a1,
                                                       a2=a2,
                                                       a3=a3,
                                                       x3=x_ph[i][0],
                                                       y3=y_new,
                                                       theta=x_ph[i][1])

            theta_2 = [np.pi/2 - th2[0] - alpha_2, np.pi/2 - th2[1] - alpha_2]
            theta_3 = [-np.pi/2 - th3[0] + alpha_2, -np.pi/2 - th3[1] + alpha_2]
            theta_4 = [-th4[0], -th4[1]]

            thetas.append([theta_1[i], theta_2[0], theta_3[0], theta_4[0]])
            thetas.append([theta_1[i], theta_2[1], theta_3[1], theta_4[1]])

        return thetas

    def calculate_joint_angles(self, start_pos, end_pos, duration):
        num_steps = 100  # Number of steps for the cycloidal profile
        joint_angles_trajectory = []

        for t in np.linspace(0, 1, num_steps):
            intermediate_pos = [
                start_pos[i] + (end_pos[i] - start_pos[i]) * t for i in range(3)
            ]
            target_phi = end_pos[3]  

            # Calculate the desired joint angles for the intermediate position using the cycloid equation
            t_j = t - np.sin(2 * np.pi * t) / (2 * np.pi)
            joint_angles = np.array(self.omx_inverse_kinematics(intermediate_pos, target_phi=target_phi))
            
            # Append the joint angles to the trajectory
            joint_angles_trajectory.append(joint_angles[0])

        return joint_angles_trajectory

    def plot_joint_angles_trajectory(self, start_pos, end_pos, duration):
        num_steps = 100  # Number of steps for the cycloidal profile
        time_points = np.linspace(0, 1, num_steps)
        joint_angles_trajectory = self.calculate_joint_angles(start_pos, end_pos, duration)

        plt.figure(figsize=(10, 6))
        plt.plot(time_points, joint_angles_trajectory)
        plt.title('Joint Angles vs. Time')
        plt.xlabel('Time')
        plt.ylabel('Joint Angles')
        plt.grid(True)
        # plt.show()

    def run(self):
        rate = rospy.Rate(10) 

        while not rospy.is_shutdown():
            if self.current_position_index < len(self.end_effector_positions) - 1:
                start_pos = self.end_effector_positions[self.current_position_index]
                end_pos = self.end_effector_positions[self.current_position_index + 1]
                duration = 2.0 
                self.plot_joint_angles_trajectory(start_pos, end_pos, duration)

                joint_angles_trajectory = self.calculate_joint_angles(start_pos, end_pos, duration)

                for joint_angles in joint_angles_trajectory:
                    joint_angles_msg = Float64MultiArray(data=joint_angles)
                    self.joint_angles_pub.publish(joint_angles_msg)
                    rospy.sleep(duration / len(joint_angles_trajectory))
                self.plot_joint_angles_trajectory

                self.current_position_index += 1
            else:
                rospy.loginfo("Reached the last position. Shutting down the node.")
                rospy.signal_shutdown("End of positions reached")

if __name__ == '__main__':
    try:
        node = InverseKinematics()
        node.run()
    except rospy.ROSInterruptException:
        pass

