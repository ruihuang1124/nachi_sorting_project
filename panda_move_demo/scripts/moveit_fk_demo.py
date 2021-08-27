#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2019 Wuhan PS-Micro Technology Co., Itd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy, sys
import moveit_commander

class MoveItFkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_fk_demo', anonymous=True)
 
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('panda_arm')
        
        # 设置机械臂运动的允许误差值
        arm.set_goal_joint_tolerance(0.001)

        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(0.3)
        arm.set_max_velocity_scaling_factor(0.3)

        # 设置机械臂的目标位置，使用七轴的位置数据进行描述（单位：弧度）
        #joint_positions = [-0.892622139344918, 0.24846613627985903, 0.6574817291086305, -0.9236269376523841, 0.342400425563999, 2.1, -1.3]
        
        #joint_positions = [-0.9121579568708621, 0.3001651289591081, 0.622793608899702, -0.9202470255650972, 0.3323942793997257, 2.4466586613657824, -1.1591781982627059, 0.02990497812628746, 0.02990497812628746]
        #joint_positions = [-0.9514559009491974, 0.489348688495041, 0.6106576238766052, -0.8485307813025356, 0.33232887535731354, 2.446840000690952, -1.3590851201310444]
        #joint_positions = [-0.3849519871326987, 0.632857540434542, 0.6105062612985309, -0.793841537526386, 0.43365481320087207, 2.4544585551600293, -1.3584586315305678]
        #joint_positions = [0.04777518130693309, 0.27524691574259535, 0.023525366650487243, -0.9146484812630546, 0.2483094807142505, 2.2882689821498944, 1.0022918537569803]
        #joint_positions = [0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4]
        joint_positions = [-0.65, -0.2802554999150728, 0.6816831404933025, -2.2791428322260354, 0.35744897433784273, 1.9455188041767661, -1.350704752111103]

        arm.set_joint_value_target(joint_positions)
                 
        # 控制机械臂完成运动
        arm.go()
        rospy.sleep(1)

        
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItFkDemo()
    except rospy.ROSInterruptException:
        pass
