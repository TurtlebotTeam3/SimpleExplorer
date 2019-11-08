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
from wavefront import Wavefront
import actionlib


class Explorer:

    def __init__(self):
        self.is_navigating = False
        self.is_searching_unknown_space = False
        self.scan = []
        self.waypoints = []
        self.robot_radius = 4
        self.map_resolution = 0
        self.robot_x = 0
        self.robot_y = 0
        self.map_height = 0
        self.map_width = 0
        self.wavefront = Wavefront()

        rospy.init_node('explorer', anonymous=True)
        self.move_base_client = actionlib.SimpleActionClient(
            'move_base', MoveBaseAction)
        self.odomSub = rospy.Subscriber('odom', Odometry, self._odom_callback)
        self.mapSub = rospy.Subscriber(
            'map', OccupancyGrid, self._map_callback)
        self.scanSub = rospy.Subscriber('scan', LaserScan, self._scan_callback)
        
        
        
        rospy.spin()

    def _map_callback(self, data):
        self.map_resolution = data.info.resolution
        self.map_height = data.info.height
        self.map_width = data.info.width
        # reshape the map
        map = np.reshape(data.data, (data.info.height, data.info.width))
        if not self.is_navigating and not self.is_searching_unknown_space:
            self.is_searching_unknown_space = True
            # search for an unkown space
            x, y = self._search_for_unknown_space(map)
            # get the waypoints to the unkown space
            print('Calculating waypoints')
            self.waypoints = self.wavefront.run(
                map, x, y, self.robot_x, self.robot_y, self.robot_radius)
            self.is_searching_unknown_space = False
            #start navigating
            self._navigate()

    def _scan_callback(self, scan):
        pass

    def _odom_callback(self, odom):
        if self.map_resolution > 0:
            # convert from robot coordinates to map coordinates
            self.robot_x = int(self.map_width/2 + odom.pose.pose.position.x/self.map_resolution)
            self.robot_y = int(self.map_height/2 + odom.pose.pose.position.y/self.map_resolution)

    def _search_for_unknown_space(self, map):
        num_rows = len(map)
        num_cols = len(map[0])
        for row in range(1, num_rows - 1):
            for col in range(1, num_cols - 1):
                if map[row][col] == 0:
                    if map[row - 1][col] == -1:
                        return col, row - 1
                    if map[row + 1][col] == -1:
                        return col, row + 1
                    if map[row][col - 1] == -1:
                        return col - 1, row
                    if map[row][col + 1] == -1:
                        return col + 1, row

    def _navigate(self):
        # check if waypoints are available
        if len(self.waypoints) > 0:
            self.is_navigating = True
            # get waypoint and start moving towards it
            # when success the process next
            (x, y) = self.waypoints.pop(0)
            success = self._move(x, y)
            if not success:
                # when not success plan new path
                self.waypoints = []
        else:
            self.is_navigating = False

    def _move(self, x, y):
        """
        Moves the robot to a place defined by coordinates x and y.
        """
        print('Navigate to: ' + str(x) + ' | ' + str(y))
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "base_link"
        goal.target_pose.header.stamp = rospy.Time.now()

        target_x = (x - self.map_width/2.0) * self.map_resolution
        target_y = (y - self.map_height/2.0) * self.map_resolution

        goal.target_pose.pose.position.x = target_x
        goal.target_pose.pose.position.y = target_y
        goal.target_pose.pose.orientation.w = 1
        self.move_base_client.send_goal(goal)
        success = self.move_base_client.wait_for_result()
        #When success then go to next waypoint otherwise stop navigating and check map
        if success:
            print('Reached: ' + str(x) + ' | ' + str(y))
            self._navigate()
        else:
            print('Faild driving to: ' + str(x) + ' | ' + str(y))
            self.is_navigating = False

if __name__ == '__main__':
    try:
        explorer=Explorer()
    except rospy.ROSInterruptException:
        pass
