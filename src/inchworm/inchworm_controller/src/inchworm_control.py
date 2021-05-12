#!/usr/bin/env python

###################
##### Imports #####
###################
import numpy as np
import math
# ROS Imports
import rospy
from std_msgs.msg import Float64
from inchworm_project_msgs.msg import JointCmd, EndEffectorCmd

class Inchworm:
    """
        Inchworm class for the project. This will include everything needed to run an inchworm controller
    """
    def __init__(self, debug = False, rate = 10):
        """
            Initialization Function

            Parameters
            ----------
            isDebug : bool
                Sets class in debug mode
                    TRUE  => DEBUG level messages and higher will be printed
                    FALSE => [DEFAULT] WARN level messages and higher will be printed
            rate : float
                Rate at which the controller loops
        """

        # Set up ROS Node
        if debug:
            rospy.init_node('inchworm_controller', log_level=rospy.DEBUG)
        else:
            rospy.init_node('inchworm_controller', log_level=rospy.WARN)
        self.rate = rospy.Rate(rate)

        # Set up Joint Command Publishers
        self.main_b_pub = rospy.Publisher('main_b_joint_controller/command', Float64, queue_size=0)
        self.b_frontleg_pub = rospy.Publisher('b_frontleg_joint_controller/command', Float64, queue_size=0)
        self.frontleg_frontfoot_pub = rospy.Publisher('frontleg_frontfoot_joint_controller/command', Float64, queue_size=0)
        self.main_c_pub = rospy.Publisher('main_c_joint_controller/command', Float64, queue_size=0)
        self.c_backleg_pub = rospy.Publisher('c_backleg_joint_controller/command', Float64, queue_size=0)
        self.backleg_backfoot_pub = rospy.Publisher('backleg_backfoot_joint_controller/command', Float64, queue_size=0)

        # Define Joint Command Variables
        # Note: DO NOT change these by hand! Use cmd_joints() function
        self._main_b_joint_cmd = Float64()
        self._b_frontleg_joint_cmd = Float64()
        self._frontleg_frontfoot_joint_cmd = Float64()
        self._main_c_joint_cmd = Float64()
        self._c_backleg_joint_cmd = Float64()
        self._backleg_backfoot_joint_cmd = Float64()

        rospy.Subscriber('joint_cmd', JointCmd, self._cmd_joints_callback)

        rospy.Subscriber('cmd_endeffector', EndEffectorCmd, self._cmd_ee_callback)
        
        # Set to starting configuration
        # self.cmd_joints(main_joint=1.04620, frontleg_joint=0.52360, backleg_joint=-0.52360, frontfoot_joint=0, backfoot_joint=0)

    def run_controller(self):
        """
            Main controller loop

            Parameters
            ----------
            None

            Returns
            -------
            None
        """
        while not rospy.is_shutdown():
            # self._cmd_joints_ros()
            self.rate.sleep()

    def fwkin(self, q_back_foot_leg, q_leg_c, q_c_b, q_leg_b, q_front_foot_leg):
        """
            FW kin from back foot end effector to front end effector

            Note: Not tested!

            Parameters
            ----------
            q1, q2, q3, q4, q5: joint variables in rad

            Returns
            -------
            tf matrix
        """
        t_dict = dict()

        t_dict['back_ee_foot'] = self._tz(0.0489)
        t_dict['back_foot_leg'] = self._tz(0.0573) * self._rz(q_back_foot_leg) * self._rx(np.deg2rad(-90)) * self._rz(np.deg2rad(90))
        t_dict['back_leg_c'] = self._rz(q_leg_c) * self._tx(-0.1663)
        t_dict['c_b'] = self._rz(q_c_b) * self._tx(0.1663)
        t_dict['b_front_leg'] = self._rz(q_leg_b) * self._tx(0.0573) * self._rz(np.deg2rad(-90)) * self._rx(np.deg2rad(90))
        t_dict['front_leg_foot'] = self._rz(q_front_foot_leg)
        t_dict['front_foot_ee'] = self._tz(-0.0489)
        
    def ikin(self, p):
        """
            Inverse Kinematics function: takes in desired position and outputs angles

            Note: NOT correct!

            Parameters
            ----------
            p: list of EE's position in x y z coordinate frame

            Returns
            -------
            q:  list of joint values theta1 - theta5
        """

        L1 = 1
        L2 = 2
        L3 = 3
        L4 = 3

        px = p(0)
        py = p(1)
        pz = p(2)

        theta1 = -np.arctan2(py,px)
        theta2 = np.arctan2( ( pz-L1 ), ( np.sqrt( (px)^2 + (py)^2 ) ) ) + np.cos( ( (L2)^2 + (L4)^2 - (L3)^2 ) / (2*L2*L4))        
        theta3 = -np.cos(-((L3)^2+(L2)^2-(L4)^2)/(2*L2*L3))
        #theta 4 is orientation
        #assuming that theta 5 does not move
        theta = [theta1, theta2, theta3]
        return theta
    
    def cmd_joints(self, main_front_joint = None, main_back_joint = None, frontleg_joint = None, backleg_joint = None, frontfoot_joint = None, backfoot_joint = None):
        """
        """
        if main_front_joint is not None:
            self.main_b_pub.publish(Float64(-float(main_front_joint)/2))
        if main_back_joint is not None:
            self.main_c_pub.publish(Float64(float(main_back_joint))) 
        if frontleg_joint is not None:
            self.b_frontleg_pub.publish(Float64(float(frontleg_joint)))
        if backleg_joint is not None:
            self.c_backleg_pub.publish(Float64(float(backleg_joint)))
        if frontfoot_joint is not None:
            self.frontleg_frontfoot_pub.publish(Float64(float(frontfoot_joint)))
        if backfoot_joint is not None:
            self.backleg_backfoot_pub.publish(Float64(float(backfoot_joint)))
    
    def _cmd_joints_callback(self, msg):
        self.cmd_joints(main_front_joint = msg.main_front, main_back_joint=msg.main_back, frontleg_joint = msg.frontleg, 
                        backleg_joint = msg.backleg, frontfoot_joint = msg.frontfoot, backfoot_joint = msg.backfoot)
    
    def _cmd_ee_callback(self, msg):
        pass


    def tdh(self,theta, d, a , alpha):
        """
            DH Parameters function: takes in theta, d, a, alpha and outputs transformation matrix

            Parameters
            ----------
            theta, d, a, alpha

            Returns
            -------
            transformation matrix
        """
        tdh = np.array([
                [cos(theta), -sin(theta)*cosd(alpha), sin(theta)*sin(alpha),  a*cos(theta)],
                [sin(theta), cos(theta)*cos(alpha),  -cos(theta)*sin(alpha), a*sin(theta)],
                [0,          sin(alpha),             cos(alpha),             d],
                [0,          0,                      0,                      1]])

        return tdh

    def _tx(self, x):
        """
            Creates transformation matrix with X transformation
            Parameters
            ----------
            x: linear transformation in x direction in meters
        """
        return np.array([
            [1, 0, 0, x],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
    def _ty(self, y):
        """
            Creates transformation matrix with Y transformation
            Parameters
            ----------
            y: linear transformation in y direction in meters
        """
        return np.array([
            [1, 0, 0, 0],
            [0, 1, 0, y],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
    def _tz(self, z):
        """
            Creates transformation matrix with Z transformation
            Parameters
            ----------
            z: linear transformation in z direction in meters
        """
        return np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, z],
            [0, 0, 0, 1]
        ])
    def _rx(self, rx):
        """
            Creates transformation matrix with RX rotation

            Parameters
            ----------
            rx: angular transformation around x axis in rad
        """
        return np.array([
            [1,            0,             0, 0],
            [0, math.cos(rx), -math.sin(rx), 0],
            [0, math.sin(rx),  math.cos(rx), 0],
            [0,            0,             0, 1]
        ])
    def _ry(self, ry):
        """
            Creates transformation matrix with RY rotation

            Parameters
            ----------
            ry: angular transformation around y axis in rad
        """
        return np.array([
            [ math.cos(ry), 0, math.sin(ry), 0],
            [            0, 1,            0, 0],
            [-math.sin(ry), 0, math.cos(ry), 0],
            [            0, 0,            0, 1]
        ])
    def _rz(self, rz):
        """
            Creates transformation matrix with RZ rotation

            Parameters
            ----------
            rz: angular transformation around z axis in rad
        """
        return np.array([
            [math.cos(rz), -math.sin(rz), 0, 0],
            [math.sin(rz),  math.cos(rz), 0, 0],
            [           0,             0, 1, 0],
            [           0,             0, 0, 1]
        ])
    

if __name__ == '__main__':
    try:
        robot = Inchworm(debug=True, rate = 10)
        robot.run_controller()
    except rospy.ROSInterruptException:
        pass