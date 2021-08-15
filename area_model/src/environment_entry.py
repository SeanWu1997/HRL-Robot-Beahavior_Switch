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
import numpy as np
import math
from math import pi
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from respawnGoal import Respawn, Respawn_subgoal

class Env():
    def __init__(self, action_size):
        self.goal_x = 0
        self.goal_y = 0
        self.heading = 0
        self.action_size = action_size
        self.initGoal = True
        self.get_goalbox = False
        self.get_subgoalbox = False
        self.position = Pose()
        self.subgoal_x = 0
        self.subgoal_y = 0
        self.past_distance = 0.0
        self.past_heading = 0.0
        self.current_distance = 0.0
        self.current_heading = 0.0
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.getOdometry)
        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        self.respawn_goal = Respawn()
        self.respawn_subgoal = Respawn_subgoal()

    def getGoalDistace(self):
        goal_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2)

        return goal_distance

    def getOdometry(self, odom):
        self.position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)

        goal_angle = math.atan2(self.goal_y - self.position.y, self.goal_x - self.position.x)

        heading = goal_angle - yaw
        if heading > pi:
            heading -= 2 * pi

        elif heading < -pi:
            heading += 2 * pi

        self.heading = round(heading, 2)

    def LaserFilter(self, scan):
        scan_range = []
        for i in range(len(scan)):
            if scan[i] > 1:
                scan_range.append(1.0)
            else:
                scan_range.append(scan[i])

        return scan_range

    def GetLaser(self, scan):
        scan_range = []
        # for i in range(len(scan.ranges)):
        for i in range(0,360, 15):
            if scan.ranges[i] == float('Inf'):
                scan_range.append(3.5)
            elif np.isnan(scan.ranges[i]):
                scan_range.append(0)
            else:
                scan_range.append(scan.ranges[i])
                
        return scan_range

    def Normalize(self, data, Max, min):
        if isinstance(data,list):
            nor_data = [round((data[i]-min)/(Max-min), 2) for i in range(len(data))]
        else:
            nor_data = round((data-min)/(Max-min), 2)

        return nor_data

    def subGoal(self, action): 
        if action == 0:
            radius = 0.0#m
            angular = 0.0

        else:
            radius = 1.0#m
            angular = (action - 4) * (pi / 4.0)

        goal_x = self.position.x + radius * math.cos(angular)
        goal_x = self.position.x + radius * math.sin(angular)

        return goal_x, goal_y

    def getState(self, scan):
        self.current_heading = self.heading
        min_range = 0.208

        laser = self.GetLaser(scan)       
        self.current_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y),2)
        if self.current_distance < 0.2:
            self.get_goalbox = True

        laser = self.Normalize(laser, 3.5, 0.0)
        heading = self.Normalize(self.current_heading, pi, -pi)
        current_distance = self.Normalize(self.current_distance, 10.0, 0.0)

        return laser + [heading, current_distance]

    def getsubState(self, scan):
        min_range = 0.208
        done = False

        laser = self.GetLaser(scan)
        laser = self.LaserFilter(laser)

        if min_range > min(laser) > 0:
            done = True
            self.pub_cmd_vel.publish(Twist())

        if self.position.x >= 6 or self.position.y >= 0:
            done = True
            self.pub_cmd_vel.publish(Twist())
        
        current_distance = round(math.hypot(self.subgoal_x - self.position.x, self.subgoal_y - self.position.y),2)

        if current_distance < 0.05:
            self.get_subgoalbox = True
            self.pub_cmd_vel.publish(Twist())
        
        laser = self.Normalize(laser, 1.0, 0.0) 
        heading = self.Normalize(self.heading, pi, -pi) 
        distance = self.getGoalDistace(current_distance, 5.0, 0.0)

        return laser + [heading, distance], done, self.get_subgoalbox

    def setReward(self, state, done):
        distance = self.current_distance
        heading = self.current_heading

        diff_distance = round((self.past_distance - self.current_distance),3)
        """ Entry Reward """
        if self.position.y > -2.5:
            reward = -1 - abs(diff_distance)
        else:
            reward = 2 * diff_distance

        if done:
            rospy.loginfo("Collision!!")
            reward = -5
            self.pub_cmd_vel.publish(Twist())

        if self.get_goalbox:
            rospy.loginfo("Goal!!")
            reward = 5
            self.pub_cmd_vel.publish(Twist())
            self.goal_x, self.goal_y = self.respawn_goal.getPosition(True, delete=True)
            self.goal_distance = self.getGoalDistace()
            self.get_goalbox = False

        self.past_heading = heading
        self.past_distance = distance

        return reward	

    def step(self, action, done):
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        state = self.getState(data)
        reward = self.setReward(state, done)

        return np.asarray(state), reward

    def reset(self):
        rospy.wait_for_service('gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print("gazebo/reset_simulation service call failed")

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        if self.initGoal:
            self.goal_x, self.goal_y = self.respawn_goal.getPosition()
            self.initGoal = False

        #self.goal_distance = self.getGoalDistace()
        state = self.getState(data)

        self.past_distance = self.current_distance
        self.past_heading = self.current_heading

        return np.asarray(state)
    
    def action_step(self, action):
        vel_cmd = Twist()
        vel_cmd.linear.x = action[0]
        vel_cmd.angular.z = action[1]
        self.pub_cmd_vel.publish(vel_cmd)

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        state, done, get = self.getsubState(data)
        self.get_subgoalbox = False

        return np.asarray(state), done, get

    def action_reset(self, action, initSubgoal):
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass
        
        self.subgoal_x, self.subgoal_y = self.subGoal(action)

        if initSubgoal:
            self.respawn_subgoal.getPosition(self.subgoal_x, self.subgoal_y)
        else:
            self.respawn_subgoal.getPosition(self.subgoal_x, self.subgoal_y, delete=True)

        state, done, get = self.getsubState(data)

        return np.asarray(state)