#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
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
#################################################################################

# Authors: Gilbert #

import rospy
import time
from gazebo_msgs.msg import ModelState, ModelStates

class Combination():
    def __init__(self):
        self.pub_model = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=1)
        self.moving()

    def moving(self):
        state = 0
        while not rospy.is_shutdown():
            model = rospy.wait_for_message('gazebo/model_states', ModelStates)
            for i in range(len(model.name)):
                if model.name[i] == 'obstacle_4':
                    obstacle_4 = ModelState()
                    obstacle_4.model_name = model.name[i]
                    obstacle_4.pose = model.pose[i]
                    base_x = 2.5
                    base_y = -2.5
                    speed = 0.03
                    x = obstacle_4.pose.position.x - base_x
                    y = obstacle_4.pose.position.y - base_y

                    if x >= 0 and y > 0:
                        if x >= 3.0:
                            obstacle_4.pose.position.x = obstacle_4.pose.position.x
                            obstacle_4.pose.position.y -= speed
                        else:
                            obstacle_4.pose.position.x += speed
                            obstacle_4.pose.position.y = obstacle_4.pose.position.y
                                                   
                    elif x > 0 and y <= 0:
                        if y <= -1.5:
                            obstacle_4.pose.position.x -= speed
                            obstacle_4.pose.position.y = obstacle_4.pose.position.y
                        else:
                            obstacle_4.pose.position.x = obstacle_4.pose.position.x
                            obstacle_4.pose.position.y -= speed
                        
                    elif x <= 0 and y < 0:
                        if x <= -3.0:
                            obstacle_4.pose.position.x = obstacle_4.pose.position.x
                            obstacle_4.pose.position.y += speed
                        else:
                            obstacle_4.pose.position.x -= speed
                            obstacle_4.pose.position.y = obstacle_4.pose.position.y
                            
                    elif x < 0 and y >= 0:
                        if y >= 1.5:
                            obstacle_4.pose.position.x += speed
                            obstacle_4.pose.position.y = obstacle_4.pose.position.y                         
                        else:
                            obstacle_4.pose.position.x = obstacle_4.pose.position.x
                            obstacle_4.pose.position.y += speed
                            
                    else:
                        obstacle_4.pose.position.x = obstacle_4.pose.position.x
                        obstacle_4.pose.position.y = obstacle_4.pose.position.y

                    self.pub_model.publish(obstacle_4)
                    time.sleep(0.1)

def main():
    rospy.init_node('combination_obstacle_4')
    try:
        combination = Combination()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
