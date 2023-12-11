#!/usr/bin/env python3
import rospy
import numpy as np
# from math import acos, degrees
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from math import *

class turtle:
    def __init__(self):
        self.pub=rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        # self.pub=rospy.Publisher('/RosAria/cmd_vel',Twist,queue_size=10)

    def updatePose(self,Point):
        self.odom=Point
        self.odom.pose.pose.position.x=round(self.odom.pose.pose.position.x,4)
        self.odom.pose.pose.position.y=round(self.odom.pose.pose.position.y,4)
        return self.odom

    def pose_callback(self, data):
        self.Point = self.updatePose(data)

    def controller(self):

        goal_pos=Odometry()
        goal_pos.pose.pose.position.x=float(input("Enter x coordinate of goal: "))
        goal_pos.pose.pose.position.y=float(input("Enter y coordinate of goal: "))
        
        msg=Twist()

        
        while(1):

            dist=sqrt(pow((goal_pos.pose.pose.position.x - self.Point.pose.pose.position.x), 2) + pow((goal_pos.pose.pose.position.y - self.Point.pose.pose.position.y), 2))
            if dist>0.2:
                theta=atan2(goal_pos.pose.pose.position.y-self.Point.pose.pose.position.y, goal_pos.pose.pose.position.x - self.Point.pose.pose.position.x)
                
                msg.linear.x=0.2*dist
                # msg.linear.x=0    
                msg.linear.y=0
                msg.linear.z=0
                msg.angular.x=0
                msg.angular.y=0 

                desired_angle=atan2(goal_pos.pose.pose.position.y-self.Point.pose.pose.position.y,goal_pos.pose.pose.position.x-self.Point.pose.pose.position.x)

                # Rotation Matrix for desired angle
                R_des=np.array([[math.cos(desired_angle),-math.sin(desired_angle),0],
                                    [math.sin(desired_angle),math.cos(desired_angle),0],
                                    [0,0,1]])


                # msg=Pose()
                q0=self.Point.pose.pose.orientation.w
                q1=self.Point.pose.pose.orientation.x
                q2=self.Point.pose.pose.orientation.y
                q3=self.Point.pose.pose.orientation.z
                quaternion=(q0,0,0,q3)
                quaternion /= np.linalg.norm(quaternion)
                (q0,q1,q2,q3)=quaternion
                # if(q1<=0):
                #     theta=2*acos(q0)
                # elif(q1>0):
                #     theta=-2*acos(q0)

                # # Rotation Matrix for current angle
                # R_curr=np.array([[math.cos(theta),-math.sin(theta),0],
                #                     [math.sin(theta),math.cos(theta),0],
                #                     [0,0,1]])  
                R_curr=np.array([[1-2*q2*q2-2*q3*q3,2*q1*q2-2*q0*q3,2*q1*q3+2*q0*q2],
                                [2*q1*q2+2*q0*q3,1-2*q1*q1-2*q3*q3,2*q2*q3-2*q0*q1],
                                [2*q1*q3-2*q0*q2,2*q2*q3+2*q0*q1,1-2*q1*q1-2*q2*q2]])      

                R_curr_T=np.transpose(R_curr)

                # Error in Angle
                R_error=np.dot(R_des,R_curr_T)
                error_angle=atan2(R_error[1][0],R_error[0][0])



                print("Start")
                print("Distance from goal is: ",dist)
                print("Desired Angle: ",np.rad2deg(desired_angle))
                # print("Current Angle with x axis: ",np.rad2deg(theta))
                print("Error in angle: ",np.rad2deg(error_angle))

                if(-4<np.rad2deg(error_angle)<4):
                    error_angle=0
                msg.angular.z=0.6*(error_angle)
            else:
                msg.linear.x=0
                msg.linear.y=0 
                msg.linear.z=0
                msg.angular.x=0
                msg.angular.y=0
                msg.angular.z=0
                print("Reached")
                self.pub.publish(msg)
                break
                
            self.pub.publish(msg)
            rospy.sleep(0.1)
        print("Reached")
        
        # msg.linear.x=0
        # msg.linear.y=0 
        # msg.linear.z=0
        # msg.angular.x=0
        # msg.angular.y=0
        # msg.angular.z=0
        # print("Reached")
        # self.pub.publish(msg)



if __name__=="__main__":
    try:
        rospy.init_node("turtlesim_desired_pos_node")
        t1=turtle()
        # sub=rospy.Subscriber("/odom",Odometry,t1.pose_callback)
        sub=rospy.Subscriber("/odom",Odometry,t1.pose_callback)
        # sub=rospy.Subscriber("/RosAria/pose",Odometry,t1.pose_callback)
        t1.controller()
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

