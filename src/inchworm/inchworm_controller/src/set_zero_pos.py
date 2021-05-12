#!/usr/bin/env python
import rospy

from std_msgs.msg import Float64
from gazebo_msgs.srv import SetModelConfiguration, SetModelConfigurationRequest

if __name__ == '__main__':
    rospy.init_node('set_inchworm_config')
    rospy.loginfo("Waiting for service")
    try:
        rospy.wait_for_service('/gazebo/set_model_configuration', timeout=10)
    except rospy.ROSException as e:
        rospy.logerr("Could not find the service. Is Gazebo up?")
    
    reset_joints = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)

    reset_request = SetModelConfigurationRequest()
    reset_request.model_name = "inchworm"
    reset_request.urdf_param_name = "inchworm/robot_description"
    joint_names = [
        'main_b_joint',
        'main_c_joint',
        'b_frontleg_joint',
        'c_backleg_joint',
        'frontleg_frontfoot_joint',
        'backleg_backfoot_joint'
    ]

    joint_positions = [
        -0.5,
        0.5,
        0.0,
        0.0,
        0.0,
        0.0
    ]

    reset_joints(model_name="inchworm", urdf_param_name = "/inchworm/robot_description", 
                joint_names=joint_names, joint_positions = joint_positions)
