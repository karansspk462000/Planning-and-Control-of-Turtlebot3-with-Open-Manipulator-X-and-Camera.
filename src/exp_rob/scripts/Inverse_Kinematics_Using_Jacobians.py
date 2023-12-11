import rospy
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Float64MultiArray
from open_manipulator_msgs.srv import SetJointPosition, SetJointPositionRequest
import numpy as np
from sympy import *
import sympy as sp
import math

class ik:
    def __init__(self):
        rospy.Subscriber("/joint_states",JointState,self.joint_angles_callback)
        rospy.Subscriber("/gazebo/link_states",LinkStates,self.end_eff_position_callback)
        self.joint_pub=rospy.Publisher("/joint_angles",Float64MultiArray,queue_size=10)
        # rospy.Subscriber("/joint_angles",Float64MultiArray,self.desired_angles_callback)
        self.send_joint_angles_to_service=rospy.ServiceProxy("/goal_joint_space_path", SetJointPosition)
        # self.joint_angles
        # self.current_end_eff_position=None
        self.desired_position=[0.286, -0.021024306644566202, 0.205]

    def desired_angles_callback(self,msg):
        self.joint_angles=list(msg.data)

    def joint_angles_callback(self,msg):
        self.current_angles=[msg.position[2],msg.position[3],msg.position[4],msg.position[5]]
        # print("Current Angles: ",self.current_angles)

    def end_eff_position_callback(self,msg):
        self.current_end_eff_position=[msg.pose[7].position.x+0.045,msg.pose[7].position.y,msg.pose[7].position.z]
        # print("Current End Effector Position: ",self.current_end_eff_position)

    def send_joint_angles(self,des_q):
        # print("Service")    
        request = SetJointPositionRequest()
        request.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
        request.path_time=0.01
        # if self.joint_angles is not None:
        # self.joint_angles.append(0)
        # self.joint_angles = np.append(self.joint_angles, 0)
        request.joint_position.position = (des_q)
        # print("Joint Angles: ",des_q)
        response=self.send_joint_angles_to_service(request)
        if response.is_planned:
                    print("Successfully sent joint positions.")
                    rospy.sleep(0.1)
            # self.joint_angles=None

    def main_ik(self):
        msg=Float64MultiArray()
        th1, th2, th3, th4, th0 = symbols('th1 th2 th3 th4 th0') #Defined symbols to use later in calculation
        theta = sp.symbols('theta[0:%d]' % 4)
        
        self.start_position=self.current_end_eff_position
        # exit()
        positions=np.linspace(self.start_position,self.desired_position,200)
        positions=[positions[0],positions[-1]]
        for desired_position in positions:
            error=desired_position-self.current_end_eff_position
            
            while(np.linalg.norm(error)>0.03):
                print("Norm To Exit: ",np.linalg.norm(np.array(self.desired_position)-self.current_end_eff_position))
                if(np.linalg.norm(np.array(self.desired_position)-self.current_end_eff_position) < 0.02):
                    rospy.loginfo("Reached")
                    exit()
                print("Desired Position: ",self.desired_position)
                print("Current Position: ",self.current_end_eff_position)
                print("Error: ",error)
                print("Error Norm: ",np.linalg.norm(error))
                # print("\n")
                # Function call for jacobian calculation
                # J=self.jacobian_calculation()
                J=self.forward_kinematics_inv_Jacobian()
                
                
                
                # print("Jacobian Shape1 :",J.shape)
                # print("Jacobian Shape2 :",J_for.shape)
                # rospy.sleep(30)
                # print("Current Angles: ",self.current_angles)
                # Set joint angles to specific values
                # th1_val, th2_val, th3_val, th4_val, th0_val = self.current_angles[0], self.current_angles[1], self.current_angles[2], self.current_angles[3], 0.1853
                # Substitute numerical values for joint angles
                # J_with_values = J.subs({th0: th0_val, th1: th1_val, th2: th2_val, th3: th3_val, th4: th4_val}) 


                q=[0.1853,self.current_angles[0], self.current_angles[1], self.current_angles[2], self.current_angles[3]]

                theta_values = [q[0], q[1] + math.pi/2 - 0.1853, q[2] - math.pi/2 + 0.1853, q[3]]
                J_with_values=J.subs({theta[0]: theta_values[0], theta[1]: theta_values[1],
                                theta[2]: theta_values[2], theta[3]: theta_values[3]})


                # Calculate the result of the Jacobian matrix with numerical values
                result_J = J_with_values.evalf()
                # print(result_J)
                # rospy.sleep(100)
                # print("Type of matrix: ",result_J.type)
                # Convert to a numeric NumPy array
                result_J_np = np.array(result_J).astype(float)
                
                pseudo_J=np.linalg.pinv(result_J_np)
                # Convert the numpy array to a list

                # Calculate joint positions using the pseudo Jacobian and inverse kinematics(delta q calculation)
                # delta_theta = np.dot(pseudo_J[:,0:3], error)
                delta_theta = np.dot(pseudo_J, error)
                

                # Provide some sleep to update current angles and position
                
                print("Delta Theta:",delta_theta)
                des_q = self.current_angles - 0.05*delta_theta
                print("Current Angles: ",self.current_angles)
                print("Desired q: ",des_q)
                print("\n")
                # des_q[0]=-des_q[0]
                # des_q=-des_q
                self.send_joint_angles(des_q)
                # print("x")
                # print("Q:",q)
                # # self.joint_angles=q
                # msg.data = q  #Value updated
                # self.joint_pub.publish(msg) #Value published to topic
                # rospy.sleep(0.1) #wait for msg to publish
                rospy.sleep(0.1)
                error=desired_position-self.current_end_eff_position
                

                
            
            
            

        
    @staticmethod
    def jacobian_calculation():
        # Defining Symbols
        th1, th2, th3, th4, th0 = symbols('th1 th2 th3 th4 th0')

        # Calculating Transformation Matrices
        T10 = Matrix([[cos(th1), -sin(th1), 0, 0.012],
                    [sin(th1), cos(th1), 0, 0],
                    [0, 0, 1, 0.077],
                    [0, 0, 0, 1]])

        T21 = Matrix([[sin(th0 - th2), -cos(th0 - th2), 0, 0],
                    [0, 0, -1, 0],
                    [cos(th0 - th2), sin(th0 - th2), 0, 0],
                    [0, 0, 0, 1]])

        T32 = Matrix([[sin(th0+th3), cos(th0+th3), 0, 0.130],
                    [-cos(th0+th3), sin(th0+th3), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

        T43 = Matrix([[cos(th4), -sin(th4), 0, 0.124],
                    [sin(th4), cos(th4), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

        TE4 = Matrix([[1, 0, 0, 0.126],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

        # Calculating Transformations wrt global frame
        T20 = T10 * T21

        T30 = T10 * T21 * T32

        T40 = T10 * T21 * T32 * T43

        TE0 = T10 * T21 * T32 * T43 * TE4

        # Positions
        x=TE0[0,3]
        y=TE0[1,3]
        z=TE0[2,3]

        # Finding Partial Derivates
        dx_by_dth1=diff(x, th1)
        dx_by_dth2=diff(x, th2)
        dx_by_dth3=diff(x, th3)
        dx_by_dth4=diff(x, th4)

        dy_by_dth1=diff(y, th1)
        dy_by_dth2=diff(y, th2)
        dy_by_dth3=diff(y, th3)
        dy_by_dth4=diff(y, th4)

        dz_by_dth1=diff(z, th1)
        dz_by_dth2=diff(z, th2)
        dz_by_dth3=diff(z, th3)
        dz_by_dth4=diff(z, th4)

        R10=T10[0:3,2]
        R20=T20[0:3,2]
        R30=T30[0:3,2]
        R40=T40[0:3,2]

        # Defining Jacobians
        J = Matrix([[dx_by_dth1,dx_by_dth2,dx_by_dth3,dx_by_dth4],
            [dy_by_dth1,dy_by_dth2,dy_by_dth3,dy_by_dth4],
            [dz_by_dth1,dz_by_dth2,dz_by_dth3,dz_by_dth4],
            [R10[0,0],R20[0,0],R30[0,0],R40[0,0]],
            [R10[1,0],R20[1,0],R30[1,0],R40[1,0]],
            [R10[2,0],R20[2,0],R30[2,0],R40[2,0]]
            ])

        return J

    @staticmethod
    def forward_kinematics_inv_Jacobian():         
        # Function for forward kinematics and inverse Jacobian calculation
        
        epsilon = sp.Matrix([1, 1, 1, 1])
        n = 4

        # Define symbolic variables
        theta = sp.symbols('theta[0:%d]' % n)
        alpha = np.array([0, np.pi/2, 0, 0])
        a = np.array([0.012, 0, 0.130, 0.125])
        d = np.array([0.077, 0, 0, 0])

        # Initialize the transformation matrices
        Ttemp = sp.eye(4)
        HTM = [sp.eye(4) for _ in range(n)]

        for i in range(n):
            # Calculate the transformation matrix elements using symbolic variables
            t11 = sp.cos(theta[i])
            t12 = -sp.sin(theta[i])
            t13 = 0
            t14 = a[i]

            t21 = sp.sin(theta[i]) * sp.cos(alpha[i])
            t22 = sp.cos(theta[i]) * sp.cos(alpha[i])
            t23 = -sp.sin(alpha[i])
            t24 = -d[i] * sp.sin(alpha[i])

            t31 = sp.sin(theta[i]) * sp.sin(alpha[i])
            t32 = sp.cos(theta[i]) * sp.sin(alpha[i])
            t33 = sp.cos(alpha[i])
            t34 = d[i] * sp.cos(alpha[i])

            Tiim1 = sp.Matrix([[t11, t12, t13, t14],
                            [t21, t22, t23, t24],
                            [t31, t32, t33, t34],
                            [0, 0, 0, 1]])

            Ti0 = Ttemp * Tiim1
            HTM[i] = Ti0
            Ttemp = Ti0

        # Homogeneous Transformation matrix
        Tn0 = HTM[n - 1]
        HTM_list = [HTM[0], HTM[1], HTM[2], HTM[3]]

        # End effector's position
        pE0_h = Tn0 * sp.Matrix([0.126, 0, 0, 1])
        pE0 = sp.Matrix(pE0_h[:3])

        # Initialize the Jacobian matrix
        Jv = sp.Matrix.ones(3, n)

        for i, item in enumerate(HTM_list):
            v1 = epsilon[i] * sp.Matrix(item[:3, 2])
            v2 = pE0 - sp.Matrix(item[:3, 3])
            cross_product = v1.cross(v2)
            Jv[:, i] = cross_product
        
        return Jv

        # # Substitute joint values into the Jacobian matrix
        # theta_values = [q[0], q[1] + math.pi/2 - 0.1853, q[2] - math.pi/2 + 0.1853, q[3]]
        # Jv_substituted = Jv.subs({theta[0]: theta_values[0], theta[1]: theta_values[1],
        #                         theta[2]: theta_values[2], theta[3]: theta_values[3]})
        # Jv_substituted = N(Jv_substituted, 3)
        # Jv_substituted = Jv_substituted.applyfunc(lambda x: round(x, 3))
        
        # # Substitute joint values into end effector position
        # ee_pos = pE0.subs({theta[0]: theta_values[0], theta[1]: theta_values[1],
        #                 theta[2]: theta_values[2], theta[3]: theta_values[3]})
        # Jv = np.array(Jv_substituted, dtype=float)
        # ee_pos = np.array(ee_pos, dtype=float).reshape(3)
        
        # return np.linalg.pinv(Jv)

if __name__=="__main__":
    try:
        rospy.init_node("IK_using_jacobians_method")
        obj=ik()
        rospy.sleep(0.1) #Give some sleep for program to read value from topics
        
        # while not rospy.is_shutdown():
        obj.main_ik()
            # obj.send_joint_angles()
            # rospy.Rate(10)
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard Interrupted!!")