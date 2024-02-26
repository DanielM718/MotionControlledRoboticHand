#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def callback(data):
	rospy.loginfo("test")

def listener():
	rospy.init_node('listener', anonymous=True)

	rospy.Subscriber('test', String, callback)

	rospy.spin()


if __name__ == '__main__':
	listener()
