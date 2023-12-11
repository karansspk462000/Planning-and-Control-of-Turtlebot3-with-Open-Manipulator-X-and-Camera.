#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import *

class turtle:
    def __init__(self):
        self.pub=rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)

    def updatePose(self,pose):
        pose.x=round(pose.x,4)
        pose.y=round(pose.y,4)
        return pose

    def pose_callback(self, data):
        self.pose = self.updatePose(data)

    def controller(self):

        goal_pos=Pose()
        goal_pos.x=float(input("Enter x coordinate of goal: "))
        goal_pos.y=float(input("Enter y coordinate of goal: "))
        
        msg=Twist()

        angle=atan2(goal_pos.y - self.pose.y, goal_pos.x - self.pose.x)
        while sqrt(pow((goal_pos.x - self.pose.x), 2) + pow((goal_pos.y - self.pose.y), 2))>0.5:
            dist=sqrt(pow((goal_pos.x - self.pose.x), 2) + pow((goal_pos.y - self.pose.y), 2))
            angle=atan2(goal_pos.y - self.pose.y, goal_pos.x - self.pose.x)
            msg.linear.x=1.5*dist
            msg.linear.y=0
            msg.linear.z=0
            msg.angular.x=0
            msg.angular.y=0 
            msg.angular.z=6*(angle-self.pose.theta)
            self.pub.publish(msg)
        msg.linear.x=0
        msg.angular.z=0
        self.pub.publish(msg)
    


if __name__=="__main__":
    try:
        rospy.init_node("turtlesim_desired_pos_node")
        t1=turtle()
        sub=rospy.Subscriber("/turtle1/pose",Pose,t1.pose_callback)
        t1.controller()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

