import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge,CvBridgeError
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt

class VS:
    def __init__(self):

        self.des_image=cv2.imread("/home/pc/turtlesim_ws/src/robot_controller/scripts/Reference_image.jpg")
        # cv2.imshow("Colored",des_image)
        # cv2.waitKey(1)
        gray_des=cv2.cvtColor(self.des_image,cv2.COLOR_BGR2GRAY)
        circles1= cv2.HoughCircles(gray_des,cv2.HOUGH_GRADIENT, dp=1, minDist=100, param1=50, param2=30, minRadius=2, maxRadius=1000)
        # print("Circles",circles1)
        # rospy.sleep(10)

        # If circles are found, draw them
        features1 = []
        if circles1 is not None:
            self.circles1 = np.uint16(np.around(circles1))
            
            for i in self.circles1[0, :]:
                # Extract circle features (center_x, center_y, radius)
                features1.append([i[0], i[1], i[2]])
                # Draw the outer circle
                cv2.circle(self.des_image, (i[0], i[1]), i[2], (0, 255, 0), 2)
                # Draw the center of the circle
                cv2.circle(self.des_image, (i[0], i[1]), 2, (0, 0, 255), 3)
       
        # Convert the features to NumPy arrays
        self.desired_features = np.array(features1)
        print("Desired_Features: ",self.desired_features)

        # cv2.imshow("Desired_Image",des_image)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        self.pub=rospy.Publisher("/cmd_vel",Twist,queue_size=10)
        rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)
        rospy.Subscriber("/odom",Odometry,self.pos_callback)
        rospy.sleep(0.1)

    def pos_callback(self,msg):
        self.current_depth=msg.pose.pose.position.x
        self.current_depth=self.current_depth * 1000
    def callback(self,msg):
        
        # print(msg.data)   
        bridge=CvBridge() 
        self.cv_image=bridge.imgmsg_to_cv2(msg,desired_encoding='passthrough')
        gray=cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2GRAY)

        # Use HoughCircles to detect circles in the image
        self.circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT, dp=1, minDist=50, param1=50, param2=30, minRadius=10, maxRadius=100)
        
        # print(circles)

        
    def visual_Ser(self):
        feature_e_norm_array=[]
        iterations_array=[]
        iterations=0
        # zd=174
        zd=229
        pos_e_norm=(np.linalg.norm(self.current_depth-zd))/1000
        feature_e_norm=0
        while( feature_e_norm<680):
            # If circles are found, draw them
            features=[]
            if self.circles is not None:
                self.circles = np.uint16(np.around(self.circles))
                # print("Entered")
                # rospy.sleep(20)
                for i in self.circles[0, :]:
                    features.append([i[0],i[1],i[2]])
                    # Draw the outer circle
                    cv2.circle(self.cv_image, (i[0], i[1]), i[2], (0, 255, 0), 2)
                    # Draw the center of the circle
                    cv2.circle(self.cv_image, (i[0], i[1]), 2, (0, 0, 255), 3)

            current_features=np.array(features)
            # print("Current_Features: ",current_features)

            # # Save the result
            # output_path = 'output_image.jpg'
            # cv2.imwrite(output_path, self.cv_image)
            # rospy.sleep(10)

            # cv2.imshow("Current Image",cv_image)
            # cv2.waitKey(1)
            # return

            # Cast matrices to a larger integer type or to float
            self.desired_features = self.desired_features.astype(np.int64)
            current_features = current_features.astype(np.int64)
            rospy.sleep(0.2)
            # Interaction Matrix
            if(self.desired_features.shape != current_features.shape):
                rospy.loginfo("Out of Field of View of Camera")
                exit()
            e=current_features  - self.desired_features
            # print("Desired Features Shape: ",self.desired_features.shape)
            # print("Current Features Shape: ",current_features.shape)
            # print("Features Error Norm: ",np.linalg.norm(e))
            
            print("Current Features: ",abs(current_features))
            print("Desired Features: ",abs(self.desired_features))

            # Feature 1
            u1=current_features[0][0]
            v1=current_features[0][1]

            # Feature 2
            u2=current_features[1][0]
            v2=current_features[1][1]

            # Feature 3
            u3=current_features[2][0]
            v3=current_features[2][1]

            # Feature 4
            u4=current_features[3][0]
            v4=current_features[3][1]

            # zd=174  #In mm
            f=825   #In mm
            IM=[
                [-f/zd,0,u1/zd,(u1*v1)/f,-(f**2+u1**2)/f,v1],
                [0,-f/zd,v1/zd,(f**2+v1**2)/f,-(u1*v1)/f,-u1],

                [-f/zd,0,u2/zd,(u2*v2)/f,-(f**2+u2**2)/f,v2],
                [0,-f/zd,v2/zd,(f**2+v2**2)/f,-(u2*v2)/f,-u2],

                [-f/zd,0,u3/zd,(u3*v3)/f,-(f**2+u3**2)/f,v3],
                [0,-f/zd,v3/zd,(f**2+v3**2)/f,-(u3*v3)/f,-u3],

                [-f/zd,0,u4/zd,(u4*v4)/f,-(f**2+u4**2)/f,v4],
                [0,-f/zd,v4/zd,(f**2+v4**2)/f,-(u4*v4)/f,-u4]
                ]   
            # print("Interaction Matrix: ",np.linalg.pinv(IM))
            
            e=e[:,0:2]
            # print("error: ",e)
            e_column_vector=e.reshape(-1,1)
            # print("e_column: ",e_column_vector.size)
            print("Error vector: ",e_column_vector)
            ctrl=np.dot(np.linalg.pinv(IM),e_column_vector)
            # print("Control: ",ctrl.shape)
            
            #  Control Law
            prop_ctrl=0.001*ctrl


            # Rx=[[0,-1,0],
            # [1,0,0],
            # [0,0,1]]

            # Ry=[[0,1,0],
            # [-1,0,0],
            # [0,0,1]]
            # ctrl=[ctrl[0][0],
            #       ctrl[1][0],
            #       ctrl[5][0]]
            vx=prop_ctrl[0][0]
            vy=prop_ctrl[1][0]
            omegaz=prop_ctrl[5][0]

            pos=Twist()
            pos.linear.x=vx
            pos.linear.y=vy
            pos.angular.z=10*omegaz
            print("Angular velocity: ",10*omegaz)
            self.pub.publish(pos)
            # rospy.sleep(0.2)
            pos_e_norm=(np.linalg.norm(self.current_depth-zd))/1000
            print("Position_Error_Norm: ",pos_e_norm)
            
            feature_e_norm=np.linalg.norm(e)
            t=rospy.Time.now()
            feature_e_norm_array.append(700-feature_e_norm)
            print("Feature_Error_Norm: ",feature_e_norm)
            print("\n")
            iterations+=1
            iterations_array.append(iterations)
            if(feature_e_norm>700):
                pos.linear.x=0
                pos.linear.y=0
                pos.angular.z=0
                self.pub.publish(pos)
                # rospy.sleep(0.5)
                rospy.loginfo("Reached")
                # Plot
                plt.plot(iterations_array,feature_e_norm_array)
                # Add labels and title
                plt.xlabel('Iterations')
                plt.ylabel('Feature Error Norm')
                plt.title('Errror vs Time')
                plt.show()
                features1=[]
                for i in self.circles1[0, :]:
                    # Extract circle features (center_x, center_y, radius)
                    features1.append([i[0], i[1], i[2]])
                    # Draw the outer circle
                    cv2.circle(self.cv_image, (i[0], i[1]), i[2], (0, 255, 0), 2)
                    # Draw the center of the circle
                    cv2.circle(self.cv_image, (i[0], i[1]), 2, (0, 0, 255), 3)

                if self.circles is not None:
                    self.circles = np.uint16(np.around(self.circles))
                    for i in self.circles[0, :]:
                        features.append([i[0],i[1],i[2]])
                        # Draw the outer circle
                        cv2.circle(self.cv_image, (i[0], i[1]), i[2], (0, 255, 0), 2)
                        # Draw the center of the circle
                        cv2.circle(self.cv_image, (i[0], i[1]), 2, (0, 0, 255), 3)
                    
                cv2.imshow("Current Image",self.cv_image)
                cv2.imshow("Desired Image",self.des_image)
                cv2.waitKey(0)
                # cv2.destroyAllWindows()
                
                exit()
            


if __name__=="__main__":
    try:
        rospy.init_node("Visual_Servoing_Node")
        obj=VS()
        obj.visual_Ser()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Ctrl+C pressed. Shutting down the node.")