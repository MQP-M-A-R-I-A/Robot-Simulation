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

    def fwkin(self):
        """
            Forward Kinematics function: takes in angles and outputs position

            Parameters
            ----------
            None

            Returns
            -------
            None
        """
        rospy.WARN("Fwkin function not implemented yet!")

    def ikin(self):
        """
            Inverse Kinematics function: takes in desired position and outputs angles

            Parameters
            ----------
            None

            Returns
            -------
            None
        """
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