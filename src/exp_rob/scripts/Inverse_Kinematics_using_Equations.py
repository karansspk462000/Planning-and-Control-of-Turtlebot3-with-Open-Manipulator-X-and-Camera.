#!/usr/bin/env python


import rospy
from std_msgs.msg import Float64MultiArray
from math import atan2, sqrt, cos, sin, pi
import numpy as np
import math


     


class InverseKinematics:
   def __init__(self):
       rospy.init_node('calculate_joint_angles')
       # print("1")
       self.joint_angles_pub = rospy.Publisher('/joint_angles', Float64MultiArray, queue_size=10)
       # print("2")
       self.end_effector_sub = rospy.Subscriber('/end_effector_pose', Float64MultiArray, self.end_effector_callback)


  


   def end_effector_callback(self, pose_msg):
       # Extract end effector position (px, py) and angle phi from the message
  
       px, py, pz, phi= pose_msg.data
       
       def P3R_inverse_kinematics(a1, a2, a3, x3, y3, theta):
           ## Inverse Kinematics of planar 3R robot


           x2 = x3 - a3*np.cos(theta)
           y2 = y3 - a3*np.sin(theta)


           cos_th2 = (x2**2+y2**2-a1**2-a2**2)/(2*a1*a2)
           # print(cos_th2)
           sin_th2 = [-np.sqrt(1-cos_th2**2), np.sqrt(1-cos_th2**2)]


           th2 = [np.arctan2(sin_th2[0], cos_th2),np.arctan2(sin_th2[1], cos_th2)]


           sin_th1 = [(y2*(a1+a2*np.cos(th2[0]))-a2*np.sin(th2[0])*x2)/(a1**2+a2**2+2*a1*a2*np.cos(th2[0])),
                   (y2*(a1+a2*np.cos(th2[1]))-a2*np.sin(th2[1])*x2)/(a1**2+a2**2+2*a1*a2*np.cos(th2[1]))]
          
           cos_th1 = [(x2*(a1+a2*np.cos(th2[0]))+a2*np.sin(th2[0])*y2)/(a1**2+a2**2+2*a1*a2*np.cos(th2[0])),
                   (x2*(a1+a2*np.cos(th2[1]))+a2*np.sin(th2[1])*y2)/(a1**2+a2**2+2*a1*a2*np.cos(th2[1]))]
          
           th1 = [np.arctan2(sin_th1[0], cos_th1[0]),np.arctan2(sin_th1[1], cos_th1[1])]


           th3 = [theta-th1[0]-th2[0], theta-th1[1]-th2[1]]


           return th1, th2, th3
      
       def inverse_kinematics(target_pos, target_rot=None, target_phi=None):
  
           x = target_pos[0]
           y = target_pos[1]
           z = target_pos[2]
          
           d1 =0.077
           a1 = np.sqrt(0.024**2+0.128**2)
           alpha_2 = np.arctan(0.024/0.128)
           # print(np.rad2deg(alpha_2))
           a2 = 0.124
           a3 = 0.126


           x_new = np.sqrt(x**2+y**2)
           y_new = z-d1
           if target_phi is None:
               phi = calculate_angle_with_xy_plane(target_rot)
           else:
               phi = -target_phi
          
           theta_1 = [np.arctan2(y, x), np.arctan2(-y, -x)]
           thetas = []
           for i in range(1):
               th2, th3, th4 = P3R_inverse_kinematics(a1=a1,
                                                       a2=a2,
                                                       a3=a3,
                                                       x3=x_new,
                                                       y3=y_new,
                                                       theta=phi)
              
              


               theta_2 = [np.pi/2-th2[0]-alpha_2, np.pi/2-th2[1]-alpha_2]
               theta_3 = [-np.pi/2-th3[0]+alpha_2, -np.pi/2-th3[1]+alpha_2]
               theta_4 = [-th4[0], -th4[1]]


               thetas.append([theta_1[i], theta_2[0], theta_3[0], theta_4[0]])
               thetas.append([theta_1[i], theta_2[1], theta_3[1], theta_4[1]])


           return thetas




       target_phi = phi
       target_pos = [px-0.012,py,pz]
       # target_pos[0] -= 0.012
       # print("Joint_Angles_Calculation:")
       joint_angles = np.array(inverse_kinematics(target_pos, target_phi=target_phi))




       # Create a Float64MultiArray message to publish the joint angles
       joint_angles_msg = Float64MultiArray(data=joint_angles[0])
       # joint_angles_msg = Float64MultiArray(data=[(theta11),(theta21),(theta31)])


       # Publish the joint angles
       # print("Joint_Angles:")
       rospy.sleep(1)
       self.joint_angles_pub.publish(joint_angles_msg)


if __name__ == '__main__':
   try:
       node = InverseKinematics()
       rospy.spin()
   except rospy.ROSInterruptException:
       pass





