#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg._OccupancyGrid import OccupancyGrid
from move_base_msgs.msg._MoveBaseActionGoal import MoveBaseActionGoal
from move_base_msgs.msg._MoveBaseAction import MoveBaseAction
from nav_msgs.msg._Odometry import Odometry
from sensor_msgs.msg._LaserScan import LaserScan
from move_base_msgs.msg._MoveBaseGoal import MoveBaseGoal
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg._Pose import Pose
from std_msgs.msg._Bool import Bool
import actionlib
from tf import TransformListener
import copy
import math
from path_planing.msg import PathPoint
from path_planing.msg import FullPath
from path_planing.srv import FindUnknown

class Explorer:

    def __init__(self):
        rospy.init_node('explorer', anonymous=True)
        
        self.is_navigating = False
        self.is_searching_unknown_space = False
        self.robot_pose_available = False
        self.reached_goal = False
        self.map_updated = False
        self.scan = []
        self.waypoints = []
        self.last_waypoints = []
        self.robot_radius = 1
        self.blowUpCellNum = 3
        self.map_resolution = 0
        self.robot_x = 0
        self.robot_x_pose = 0
        self.robot_y = 0
        self.robot_y_pose = 0
        self.map_height = 0
        self.map_width = 0
        self.map_offset_x = 0
        self.map_offset_y = 0
        self.map =[[]]
        self.tf = TransformListener()
        self.waypointsAvailable = False
        self.mapComplete  = False
        self.mapCompletePrint = False
        self.pose = Pose()

        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.pose_subscriber = rospy.Subscriber('/simple_odom_pose',Pose, self._update_pose)
        self.mapSub = rospy.Subscriber('/map', OccupancyGrid, self._map_callback)
        self.scanSub = rospy.Subscriber('/scan', LaserScan, self._scan_callback)
        self.pub_point = rospy.Publisher('clicked_point',PointStamped, queue_size=1)

        self.pub_goal = rospy.Publisher('move_to_goal/goal', Pose, queue_size=1)
        self.sub_goal_reached = rospy.Subscriber('/move_to_goal/reached', Bool, self._goal_reached_callback)

        rospy.wait_for_service('find_unkown_service')

        self.srv = rospy.ServiceProxy('find_unkown_service', FindUnknown)

        self.received_map = False

        self.rate = rospy.Rate(20)

        #rospy.spin()

    def _map_callback(self, data):
        self.map_resolution = data.info.resolution
        self.map_height = data.info.height
        self.map_width = data.info.width
        self.map_offset_x = data.info.origin.position.x
        self.map_offset_y = data.info.origin.position.y
        self.map = np.reshape(data.data, (data.info.height, data.info.width))
        self.received_map = True
    
    def run(self):
        while not rospy.is_shutdown():
            if self.mapComplete == False:
                if self.robot_pose_available and not self.is_navigating and not self.is_searching_unknown_space and self.received_map:    
                    if(self.waypointsAvailable == True):
                        self._navigate()
                    else:
                        self._calculate()
                self.rate.sleep()
            else:
                if not self.mapCompletePrint:
                    self.mapCompletePrint = True
                    print "---> MAP COMPLETE <---"

    def _calculate(self):
        print('Calculating freespace')

        self.is_searching_unknown_space = True

        service_response = self.srv(self.blowUpCellNum, self.robot_x, self.robot_y, self.robot_radius)
        self.waypoints = service_response.waypoints.fullpath
        allpoints = service_response.allPoints.fullpath

        if self.waypoints == [] and allpoints == []:
            self.mapComplete = True
            self.move_base_client.cancel_goal()
        else: 
            self.last_waypoints = self.waypoints
            # clear clicked points
            #for _ in range(100):            
            #    self._publish_point(0, 0)
            #    self.rate.sleep()

            #for (x, y) in allpoints:
            #    self._publish_point(x, y)
            #    self.rate.sleep()

            for point in self.waypoints:
                self._publish_point(point.path_x, point.path_y)
                self.rate.sleep()
            
            self.waypointsAvailable = True
            self.is_searching_unknown_space = False

    def _scan_callback(self, scan):
        pass

    def _update_pose(self, data):
        if self.map_resolution > 0:
            try:
                self.pose.position.x = data.position.x
                self.pose.position.y = data.position.y
                self.pose.position.z = data.position.z
                self.pose.orientation.x = data.orientation.x 
                self.pose.orientation.y = data.orientation.y 
                self.pose.orientation.z = data.orientation.z
                self.pose.orientation.w = data.orientation.w 

                self.robot_y_pose = self.pose.position.y
                self.robot_x_pose = self.pose.position.x

                self.robot_x = int(math.floor((self.robot_x_pose - self.map_offset_x)/self.map_resolution))
                self.robot_y = int(math.floor((self.robot_y_pose - self.map_offset_y)/self.map_resolution))
                self.robot_pose_available = True
            except:
                print('transform not ready')
                self.robot_y_pose = data.position.y
                self.robot_x_pose = data.position.x

                self.robot_x = int(math.floor((self.robot_x_pose - self.map_offset_x)/self.map_resolution))
                self.robot_y = int(math.floor((self.robot_y_pose - self.map_offset_y)/self.map_resolution))
  
    def _navigate(self):
        # get waypoint and start moving towards it
        # when success the process next
        self.is_navigating = True

        if(len(self.waypoints) > 0):
            print self.waypoints
            point = self.waypoints.pop(0)
            x = point.path_x
            y = point.path_y

            print self.waypoints

            # -- move to goal --
            self._move(x, y)

        else:
            self.is_navigating = False
            self.waypointsAvailable = False

    def _move(self, x, y):
        """
        Moves the rob2t to a place defined by coordinates x and y.
        """
        print('Navigate to: ' + str(x) + ' | ' + str(y))
        goal = Pose()

        target_x = (x * self.map_resolution) + self.map_offset_x
        target_y = (y * self.map_resolution) + self.map_offset_y

        goal.position.x = target_x
        goal.position.y = target_y
        goal.orientation.w = 1

        self.pub_goal.publish(goal)

    def _publish_point(self, x, y):
        pt_stamped = PointStamped()

        pt_stamped.header.frame_id = "map"
        pt_stamped.header.stamp = rospy.Time.now()

        pt_stamped.point.x = (x * self.map_resolution) + self.map_offset_x
        pt_stamped.point.y = (y * self.map_resolution) + self.map_offset_y
        pt_stamped.point.z = 0

        self.pub_point.publish(pt_stamped)

    def _goal_reached_callback(self, reached):
        print('Reached: ' + str(reached))
        if reached:
            self._navigate()
        else:
            self.waypoints = []
            self.is_navigating = False

if __name__ == '__main__':
    try:
        explorer=Explorer()
        explorer.run()
    except rospy.ROSInterruptException:
        pass
