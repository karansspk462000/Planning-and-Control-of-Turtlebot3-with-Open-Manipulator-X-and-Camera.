#!/usr/bin/env python
            import rospy
from std_msgs.msg import Float64MultiArray
from open_manipulator_msgs.srv import SetJointPosition, SetJointPositionRequest

class JointPositionClient:
    def __init__(self):
        rospy.init_node('joint_position_client')
        
        # Subscribe to the /joint_angles topic
        rospy.Subscriber('/joint_angles', Float64MultiArray, self.joint_angles_callback)
        
        # # Wait for the service to become available
        # rospy.wait_for_service('/goal_joint_space_path')
        
        # Create a service proxy
        self.set_joint_position = rospy.ServiceProxy('/goal_joint_space_path', SetJointPosition)
        
        # Initialize joint angles as None
        self.joint_angles = None
        
    def joint_angles_callback(self, msg):
        self.joint_angles = list(msg.data)
        
    def send_joint_positions(self, max_accelerations_scaling, max_velocity_scaling, path_time):
        if self.joint_angles is not None:
            try:
                # Create a request message
                print("Got Angles")
                self.joint_angles.append(0)
                print(self.joint_angles)
                request = SetJointPositionRequest()
                request.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
                request.joint_position.position = self.joint_angles
                request.joint_position.max_accelerations_scaling_factor = max_accelerations_scaling
                request.joint_position.max_velocity_scaling_factor = max_velocity_scaling
                request.path_time = path_time
                
                # Call the service
                response = self.set_joint_position(request)
                
                if response.is_planned:
                    print("Successfully sent joint positions.")
                    rospy.sleep(2)
                
                # Reset joint_angles to None after sending
                self.joint_angles = None
            
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)
        # else:
        #     rospy.logwarn("No joint angles received from the /joint_angles topic.")

if __name__ == '__main__':
    try:
        
        joint_position_client = JointPositionClient()
        
        # Define the parameters
        max_accelerations_scaling = 0.0
        max_velocity_scaling = 0.0
        path_time = 2.0
        
        
        rate = rospy.Rate(10)  # Adjust the rate as needed
        while not rospy.is_shutdown():
            joint_position_client.send_joint_positions(max_accelerations_scaling, max_velocity_scaling, path_time)
            rate.sleep()
    
    except KeyboardInterrupt:
        rospy.loginfo("Ctrl+C pressed. Shutting down the node.")

