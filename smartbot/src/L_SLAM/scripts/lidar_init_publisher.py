#!/usr/bin/python
import tf
import rospy
from geometry_msgs.msg import *


class LidarInitPublisher:
	def __init__(self):
		self.broadcaster = tf.TransformBroadcaster()

	def spin(self):
			self.broadcaster.sendTransform((0, 0, 0), (-0.5, -0.5, -0.5, 0.5), rospy.Time.now(), 'velodyne', 'lidar')
			self.broadcaster.sendTransform((0, 0, 0), (0, 0, 0, 1.0), rospy.Time.now(), 'lidar_rot', 'map')
			return

def main():
	rospy.init_node('lidar_init_publisher')
	node = LidarInitPublisher()

    	rate = rospy.Rate(100.0)
	while not rospy.is_shutdown():
		node.spin()
		rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except:
        print 'shutdown'

