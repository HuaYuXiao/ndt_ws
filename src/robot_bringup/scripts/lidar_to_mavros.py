#!/usr/bin/python3
"""
Subscribe:  nav_msgs/Odometry (default: /Odometry)
Publish:    geometry_msgs/PoseStamped -> /mavros/vision_pose/pose
"""

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class OdomRepublisher(object):
    def __init__(self):
        self.input_topic = rospy.get_param('~input_topic', '/Odometry')
        self.pose_topic = rospy.get_param('~pose_topic', '/mavros/vision_pose/pose')
        self.queue_size = rospy.get_param('~queue_size', 10)

        self.pose_pub = rospy.Publisher(self.pose_topic, PoseStamped, queue_size=self.queue_size)
        self.sub = rospy.Subscriber(self.input_topic, Odometry, self.odom_cb, queue_size=self.queue_size)

    def odom_cb(self, odom_msg: Odometry):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = odom_msg.header.stamp
        pose_stamped.header.frame_id = odom_msg.header.frame_id if odom_msg.header.frame_id else ""
        pose_stamped.pose = odom_msg.pose.pose
        self.pose_pub.publish(pose_stamped)

def main():
    rospy.init_node('odom_republisher', anonymous=False)
    repub = OdomRepublisher()
    rospy.spin()

if __name__ == "__main__":
    main()