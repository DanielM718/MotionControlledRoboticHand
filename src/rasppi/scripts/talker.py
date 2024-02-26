#!/usr/bin/env python3
import rospy
from std_mesgs.msg import String

def talker():
	pub = rospy.publisher('test', String, queue_size=10)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(10) #10hz
	while not tospy.is_shutdown():
		rospy.logingo("test")
		pub.publish("test")
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
