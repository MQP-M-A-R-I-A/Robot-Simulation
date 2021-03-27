#!/usr/bin/env python

###################
##### Imports #####
###################
import numpy as np
# ROS Imports
import rospy

class Inchworm:
    """
    Inchworm class for the project. This will include everything needed to run an inchworm controller
    """
    def __init__(self, isDebug:bool = False, rate: float = 10):
        """
        Parameters
        ----------
        isDebug : bool
            Sets class in debug mode
                TRUE  => DEBUG level messages and higher will be printed
                FALSE => [DEFAULT] WARN level messages and higher will be printed
        rate : float
            Rate at which the controller loops
        """
        self.rate = rospy.Rate(rate)

        # Set up ROS Node
        if isDebug:
            rospy.init_node('inchworm_controller', log_level=rospy.DEBUG)
        else:
            rospy.init_node('inchworm_controller', log_level=rospy.WARN)

        # Set up subscriber

    def run_controller(self):
        """
        Main controller loop
        """
        while not rospy.is_shutdown():
            rospy.WARN("Nothing implemented in controller loop!")
            self.rate.sleep()

    def fwkin(self):
        """
        Forward Kinematics function: takes in angles and outputs position
        """
        rospy.WARN("Fwkin function not implemented yet!")

    def ikin(self):
        """
        Inverse Kinematics function: takes in desired position and outputs angles
        """
        rospy.WARN("Ikin function not implemented yet!")

if __name__ == '__main__':
    try:
        robot = Inchworm(isDebug=True, rate = 10)
    except rospy.ROSInterruptException:
        pass