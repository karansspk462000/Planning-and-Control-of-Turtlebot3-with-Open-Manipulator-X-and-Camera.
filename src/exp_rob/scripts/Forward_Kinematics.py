#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
from robot_controller.msg import Position
from geometry_msgs.msg import Pose
import math

class Manipulator:
    def __init__(self):
        # sub = rospy.Subscriber("/joint_states1", JointState, self.callback)
        sub = rospy.Subscriber("/joint_angles", Float64MultiArray, self.callback)
        self.pub = rospy.Publisher("/end_effector_position", Pose, queue_size=10)
        self.data_file = "end_effector_positions.txt"

        
        self.end_effector_positions = []

    def callback(self, msg):
        self.arr = msg.data
        self.end_effector_pos(self.arr)
        
    
    # def store_end_effector_position(self, array):
    #     self.end_effector_positions.append(array)
    #     with open(self.data_file, "w") as file:
    #         for position in self.end_effector_positions:
    #             file.write(",".join(map(str, position)) + "\n")
    
    def end_effector_pos(self,array):
        # msg=Position()
        msg=Pose()

        theta=[array[0],(math.pi/2)-0.1853-array[1],-(math.pi/2)-array[2]+0.1853,-array[3]]

        alpha=[0,math.pi/2,0,0]
        a=[0.012,0,0.130,0.124]
        d=[0.077,0,0,0]
        # theta=[array[2],(math.pi/2)-0.1853-array[3],-(math.pi/2)-array[4]+0.1853,-array[5]]
        
        Ttemp=np.eye(4)
        
        for i in range(4):
            Tim=[[np.cos(theta[i]),-np.sin(theta[i]),0,a[i]],
             [np.sin(theta[i])*np.cos(alpha[i]),np.cos(theta[i])*np.cos(alpha[i]),-np.sin(alpha[i]),-d[i]*np.sin(alpha[i])],
             [np.sin(theta[i])*np.sin(alpha[i]),np.cos(theta[i])*np.sin(alpha[i]),np.cos(alpha[i]),d[i]*np.cos(alpha[i])],
             [0,0,0,1]]
           
            Ttemp=np.dot(Ttemp,Tim)


        p54=np.array([[0.126],
                    [0],
                    [0],
                    [1]])   
        p50=np.dot(Ttemp,p54)
        
        msg.position.x=p50[0]
        msg.position.y=p50[1]
        msg.position.z=p50[2]
        pos_arr=[msg.position.x ,msg.position.y ,msg.position.z]
        # self.store_end_effector_position(pos_arr)
        self.pub.publish(msg)
        rospy.sleep(0.001)

if __name__ == '__main__':
    try:
        rospy.init_node("manipulator_F_Kin")
        man = Manipulator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    print("done")
