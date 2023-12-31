#!/usr/bin/python
# coding=gbk

# Copyright 2020 Wechange Tech.
# Developer: FuZhi, Liu (liu.fuzhi@wechangetech.com)
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

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
from geometry_msgs.msg import PointStamped,PoseStamped
import actionlib
from move_base_msgs.msg import *
import tf

def status_callback(msg):

    global goal_pub, index,markerArray
    global add_more_point,try_again

    if(msg.status.status == 3):
        try_again = 1
        if add_more_point == 0:
            print ('Goal reached')

        if index < count:

            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = markerArray.markers[index].pose.position.x
            pose.pose.position.y = markerArray.markers[index].pose.position.y
            pose.pose.orientation.w = 1
            goal_pub.publish(pose)

            index += 1
        elif index == count:
            add_more_point = 1
    else:
    # uint8 PENDING         = 0   # The goal has yet to be processed by the action server
    # uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
    # uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
    #                             #   and has since completed its execution (Terminal State)
    # uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
    # uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
    #                             #    to some failure (Terminal State)
    # uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
    #                             #    because the goal was unattainable or invalid (Terminal State)
    # uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
    #                             #    and has not yet completed execution
    # uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
    #                             #    but the action server has not yet confirmed that the goal is canceled
    # uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
    #                             #    and was successfully cancelled (Terminal State)
    # uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
    #                             #    sent over the wire by an action server
        print ('Goal cannot reached has some error :',msg.status.status," try again!!!!")
        if try_again == 1:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = markerArray.markers[index-1].pose.position.x
            pose.pose.position.y = markerArray.markers[index-1].pose.position.y
            pose.pose.orientation.w = 1
            goal_pub.publish(pose)
            try_again = 0
        else:
            if index < len(markerArray.markers):
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = markerArray.markers[index].pose.position.x
                pose.pose.position.y = markerArray.markers[index].pose.position.y
                pose.pose.orientation.w = 1
                goal_pub.publish(pose)
                index += 1


def click_callback(msg):
    global markerArray,count
    global goal_pub,index
    global add_more_point

    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    # marker.type = marker.TEXT_VIEW_FACING
    marker.type = marker.CYLINDER
    marker.action = marker.ADD
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.5
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = msg.pose.position.x
    marker.pose.position.y = msg.pose.position.y
    marker.pose.position.z = msg.pose.position.z

    markerArray.markers.append(marker)

    # Renumber the marker IDs
    id = 0
    for m in markerArray.markers:
       m.id = id
       id += 1

    # Publish the MarkerArray
    mark_pub.publish(markerArray)

    #first goal
    if count==0:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = msg.pose.position.x
        pose.pose.position.y = msg.pose.position.y
        pose.pose.position.z = msg.pose.position.z
        pose.pose.orientation.x = msg.pose.orientation.x
        pose.pose.orientation.y = msg.pose.orientation.y
        pose.pose.orientation.z = msg.pose.orientation.z
        pose.pose.orientation.w = msg.pose.orientation.w
        goal_pub.publish(pose)
        index += 1

    if add_more_point and count > 0:
        add_more_point = 0
        move =MoveBaseActionResult()
        move.status.status = 3
        move.header.stamp = rospy.Time.now()
        goal_status_pub.publish(move)
    quaternion = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
    theta = tf.transformations.euler_from_quaternion( quaternion)[2]
    count += 1
    # print 'add a path goal point %f %f %f'%(msg.pose.position.x,msg.pose.position.y,theta*180.0/3.14)


markerArray = MarkerArray()

count = 0       #total goal num
index = 0       #current goal point index
add_more_point = 0 # after all goal arrive, if add some more goal
try_again = 1  # try the fail goal once again

rospy.init_node('multi_goal_point_demo')

mark_pub = rospy.Publisher('/path_point_array', MarkerArray,queue_size=100)
click_goal_sub = rospy.Subscriber('/goal',PoseStamped,click_callback)
goal_pub = rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=1)
goal_status_sub = rospy.Subscriber('/move_base/result',MoveBaseActionResult,status_callback)
#after all goal arrive, if add some more goal
#we deleberate pub a topic to trig the goal sent
goal_status_pub = rospy.Publisher('/move_base/result',MoveBaseActionResult,queue_size=1)
rospy.spin()
