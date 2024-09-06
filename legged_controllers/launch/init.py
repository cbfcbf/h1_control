#!/bin/python3

import rospy
from gazebo_msgs.srv import SetModelConfiguration
import numpy as np

def set_joint_positions():
    rospy.init_node('init')
    
    # 等待服务可用
    rospy.wait_for_service('/gazebo/set_model_configuration')
    
    p=[-0.1,0.1,-0,0,1.6, 1.6,-0.7,-0.7,-0.9,-0.9]
    try:
        # 创建服务代理
        set_joint_state = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
        
        # 调用服务，设置关节位置
        response = set_joint_state(
            model_name='h1_description',  # 替换为你的模型名称
            urdf_param_name='legged_robot_description',
            joint_names=['right_hip_yaw_joint','left_hip_yaw_joint','right_hip_roll_joint','left_hip_roll_joint','left_knee_joint', 'right_knee_joint','left_hip_pitch_joint','right_hip_pitch_joint','left_ankle_joint','right_ankle_joint'],
            joint_positions=p  # 设置关节的初始位置
        )
        
        if response.success:
            rospy.loginfo("Successfully set initial joint positions.")
        else:
            rospy.logwarn("Failed to set initial joint positions.")
            
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    set_joint_positions()