#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg._OccupancyGrid import OccupancyGrid
from move_base_msgs.msg._MoveBaseActionGoal import MoveBaseActionGoal
from sensor_msgs.msg._LaserScan import LaserScan
 
class Explorer: 

	def __init__(self):
		rospy.init_node('explorer', anonymous=True)
		self.mapSub = rospy.Subscriber('map', OccupancyGrid, map_callback)
		self.scanSub = rospy.Subscriber('scan', LaserScan, scan_callback)
		self.movePub = rospy.Publisher('move_base/goal', MoveBaseActionGoal)
		rospy.spin()

	def map_callback(self):
		pass

	def scan_callback(self):
		pass

	def run(self):
		pass
 
if __name__ == '__main__':
	try:
		explorer = Explorer()
		explorer.run()
	except rospy.ROSInterruptException:
		pass