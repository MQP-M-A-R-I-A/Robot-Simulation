#!/usr/bin/env python

###################
##### Imports #####
###################
import numpy as np
# ROS Imports
import rospy
from std_msgs.msg import Float64

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
        
        # Set to starting configuration
        self.cmd_joints(main_joint=1.04620, frontleg_joint=0.52360, backleg_joint=-0.52360, frontfoot_joint=0, backfoot_joint=0)

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
            self._cmd_joints_ros()
            self.rate.sleep()

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
        tdh = [ [cos(theta), -sin(theta)*cosd(alpha), sin(theta)*sin(alpha),  a*cos(theta)],
                [sin(theta), cos(theta)*cos(alpha),  -cos(theta)*sin(alpha), a*sin(theta)],
                [0,          sin(alpha),             cos(alpha),             d],
                [0,          0,                      0,                      1]]

        return tdh

    def fwkin(self, q, movingFoot):
        """
            Forward Kinematics function: takes in angles and outputs position

            Parameters
            ----------
            q: list of joint variables
            movingFoot: string indicating which foot is moving

            Returns
            -------
            None
        """
        L1 = 1
        L2 = 2
        L3 = 3
        L4 = 3
        L5 = 2
        L6 = 1

        # will replace Ls with actual values and also replace everything with 2 precalcualted matrices
        if (movingFoot == "front")
            #front EE moving
            A1 = tdh(0, L1, 0, 0)
            A2 = tdh(theta1, 0, L2, -90)
            A3 = tdh(90, 0, 0, 0)
            A4 = tdh(theta2, 0, -L3, 0)
            A5 = tdh(theta3, 0, L4, 0)
            A6 = tdh(theta4-90, 0, -L5, 90)
            A7 = tdh(theta5, 0, L6, 0)
        else
            #back EE moving
            A1 = tdh(0, L6, 0, 0)
            A2 = tdh(theta6, 0, L5, -90)
            A3 = tdh(90, 0, 0, 0)
            A4 = tdh(theta4, 0, -L4, 0)
            A5 = tdh(theta3, 0, L3, 0)
            A6 = tdh(theta2-90, 0, -L2, 90)
            A7 = tdh(theta1, 0, L1, 0)
        
        return A1*A2*A3*A4*A5*A6*A7
        
    def ikin(self, p):
        """
            Inverse Kinematics function: takes in desired position and outputs angles

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
        theta = [theta1 theta2 theta3]
        return theta

        rospy.WARN("Ikin function not implemented yet!")
    
    def cmd_joints(self, main_joint = None, frontleg_joint = None, backleg_joint = None, frontfoot_joint = None, backfoot_joint = None):
        """
        """
        if main_joint is not None:
            self._main_b_joint_cmd.data = -float(main_joint)/2
            self._main_c_joint_cmd.data = float(main_joint)/2
        if frontleg_joint is not None:
            self._b_frontleg_joint_cmd.data = float(frontleg_joint)
        if backleg_joint is not None:
            self._c_backleg_joint_cmd.data = float(backleg_joint)
        if frontfoot_joint is not None:
            self._frontleg_frontfoot_joint_cmd.data = float(frontfoot_joint)
        if backfoot_joint is not None:
            self._backleg_backfoot_joint_cmd.data = float(backfoot_joint)

    def _cmd_joints_ros(self):
        """
            Sends commands to joints, runs in the main loop
            Note: Do not call this function to control joints! This is only intended to publish things.
                    Use cmd_joints() function instead
            
            Parameters
            ----------
            None

            Returns
            -------
            None
        """
        self.main_b_pub.publish(self._main_b_joint_cmd)
        self.b_frontleg_pub.publish(self._b_frontleg_joint_cmd)
        self.frontleg_frontfoot_pub.publish(self._frontleg_frontfoot_joint_cmd)
        self.main_c_pub.publish(self._main_c_joint_cmd)
        self.c_backleg_pub.publish(self._c_backleg_joint_cmd)
        self.backleg_backfoot_pub.publish(self._backleg_backfoot_joint_cmd)

if __name__ == '__main__':
    try:
        robot = Inchworm(debug=True, rate = 10)
        robot.run_controller()
    except rospy.ROSInterruptException:
        pass