
#! /usr/bin/env python3.6

'''
author: Nooshin Kohli
2022
'''

from numpy.core.numeric import load
import rospy

from controller_manager_msgs.srv import SwitchController,UnloadController,LoadController
from rospy.service import ServiceException



rospy.wait_for_service('/leg/controller_manager/switch_controller')
rospy.wait_for_service('/leg/controller_manager/unload_controller')
rospy.wait_for_service('/leg/controller_manager/load_controller')




        

if __name__ == '__main__':
    try:
        switch_controller = rospy.ServiceProxy('/leg/controller_manager/switch_controller', SwitchController)
        unload_controller = rospy.ServiceProxy('/leg/controller_manager/unload_controller',UnloadController)
        load_controller = rospy.ServiceProxy('/leg/controller_manager/load_controller', LoadController)
        req_1 = switch_controller([],['hip_joint_position_controller','thigh_joint_position_controller','calf_joint_position_controller'], 2,False,0.0)
        req_2 = unload_controller('hip_joint_position_controller')
        req_3 = unload_controller('thigh_joint_position_controller')
        req_4 = unload_controller('calf_joint_position_controller')
        # req_5 = unload_controller('jumper_position_controller')
        req_6 = load_controller('hip_joint_effort_controller')
        req_7 = load_controller('thigh_joint_effort_controller')
        req_8 = load_controller('calf_joint_effort_controller')
        # req_9 = load_controller('jumper_effort_controller')
        req_10 = switch_controller(['hip_joint_effort_controller','thigh_joint_effort_controller','calf_joint_effort_controller'],[],2,False,0.0)

        #########################################################################################################################################
        
        # req_11 = switch_controller([],['hip_joint_effort_controller','thigh_joint_effort_controller','calf_joint_effort_controller','jumper_effort_controller'],2,False,0.0)
        # req_12 = unload_controller('hip_joint_effort_controller')
        # req_13 = unload_controller('thigh_joint_effort_controller')
        # req_14 = unload_controller('calf_joint_effort_controller')
        # req_15 = unload_controller('jumper_effort_controller')
        # req_16 = load_controller('hip_joint_position_controller')
        # req_17 = load_controller('thigh_joint_position_controller')
        # req_18 = load_controller('calf_joint_position_controller')
        # req_19 = load_controller('jumper_position_controller')
        # req_20 = switch_controller(['hip_joint_position_controller','thigh_joint_position_controller','calf_joint_position_controller','jumper_position_controller'],[],2,False,0.0)
        # rospy.spin()
    except rospy.ServiceException:
        print("down!!")