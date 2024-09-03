#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SetModelConfiguration

def set_joint_positions():
    rospy.init_node('set_joint_positions_node')
    
    # 等待服务可用
    rospy.wait_for_service('/gazebo/set_model_configuration')
    
    try:
        # 创建服务代理
        set_joint_state = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
        
        # 调用服务，设置关节位置
        response = set_joint_state(
            model_name='h1_description',  # 替换为你的模型名称
            urdf_param_name='legged_robot_description',
            joint_names=['left_knee_joint', 'right_knee_joint'],
            joint_positions=[0.5, 0.5]  # 设置关节的初始位置
        )
        
        if response.success:
            rospy.loginfo("Successfully set initial joint positions.")
        else:
            rospy.logwarn("Failed to set initial joint positions.")
            
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    set_joint_positions()