#!/usr/bin/env python

# This small script subscribes to the velocity command message of move_base node
# and laser scan message.
# Author: lei.zeng@tu-dortmund.de

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

dis_stop = rospy.get_param("stop_distance")
dis_dec = rospy.get_param("deceleration_distance")
planner_nav = rospy.get_param("local_planner_navigation")


def init_vel_cmd_subscribers():
    topic_name = "/cmd_vel_0"
    rospy.Subscriber(topic_name, Twist, detect_vel_cmd_callback, queue_size=10)


def detect_vel_cmd_callback(data_vel):
    global twist_msg
    global twist_pub
    twist_msg = data_vel
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    pub.publish(twist_pub)


def init_obstacle_stop_subscribers():
    topic_name = "/scan"
    rospy.Subscriber(topic_name, LaserScan,
                     detect_obstacle_distance_callback, queue_size=10)


def detect_obstacle_distance_callback(data_scan):
    local_planner = rospy.get_param("/move_base/base_local_planner")
    global twist_msg
    global twist_pub
    size = (data_scan.angle_max - data_scan.angle_min) / \
        data_scan.angle_increment
    mid_idx = int(size/2)
    twist_stop = Twist()
    twist_dec = Twist()
    # from -30 degrees to 30 degrees
    start_idx = mid_idx - int(mid_idx/8)
    end_idx = mid_idx + int(mid_idx/8)
    # print(start_idx, end_idx, data_scan.angle_increment, data_scan.angle_increment/3.14 *180)
    count_stop = 0
    count_dec = 0
    diatance = 0
    wall = 0

    for idx in range(start_idx, end_idx):
        if data_scan.ranges[idx] < dis_stop:
            count_stop = count_stop + 1
        if data_scan.ranges[idx] < dis_dec:
            count_dec = count_dec + 1
            diatance = diatance*(count_dec - 1)/(count_dec) + \
                data_scan.ranges[idx]/(count_dec)
    for idx in range(0, int(mid_idx/3)):
        if data_scan.ranges[idx] < 2.0:
            wall = wall + 1
    for idx in range(int(2*mid_idx - mid_idx/3), 2*mid_idx):
        if data_scan.ranges[idx] < 2.0:
            wall = wall + 1

    if local_planner != planner_nav:
        twist_pub = twist_msg
    else:
        if count_dec > 20 and count_stop <= 20:
            # +0.05*abs(twist_msg.linear.x)/(twist_msg.linear.x)
            twist_dec.linear.x = twist_msg.linear.x * (diatance)/(dis_dec)
            twist_dec.angular.z = twist_msg.angular.z
            twist_pub = twist_dec
            rospy.loginfo("robot decelerating")
        elif count_stop > 20:
            rospy.loginfo("robot stopping")
            # twist_stop.angular.z = 0
            twist_stop.linear.x = 0
            twist_stop.angular.z = twist_msg.angular.z
            twist_pub = twist_stop
        else:
            twist_pub = twist_msg


def main():
    rospy.init_node('vel_rebroadcast')

    init_obstacle_stop_subscribers()
    init_vel_cmd_subscribers()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"


if __name__ == '__main__':
    # Global variables
    global twist_msg
    twist_msg = Twist()
    twist_msg.linear.x = 0
    global twist_pub
    twist_pub = Twist()
    twist_pub.linear.x = 0
    main()


