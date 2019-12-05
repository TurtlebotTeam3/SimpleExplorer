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
from tf import TransformListener
import copy
import math
import threading
import time
import queue

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
        self.waypointsFound = False
        self.robot_radius = 1
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
        self.wavefront = Wavefront()
        self.transformListener = TransformListener()
        self.start = False

        self.move_base_client = actionlib.SimpleActionClient(
            '/move_base', MoveBaseAction)
        self.odomSub = rospy.Subscriber('/odom', Odometry, self._odom_callback)
        self.mapSub = rospy.Subscriber(
            '/map', OccupancyGrid, self._map_callback)
        self.scanSub = rospy.Subscriber('/scan', LaserScan, self._scan_callback)

        self._run()

        rospy.spin()

    def _map_callback(self, data):
        self.map_resolution = data.info.resolution
        self.map_height = data.info.height
        self.map_width = data.info.width
        self.map_offset_x = data.info.origin.position.x
        self.map_offset_y = data.info.origin.position.y
        self.map = np.reshape(data.data, (data.info.height, data.info.width))
    
    def _run(self):
        run = True
        while run == True:
            if self.robot_pose_available and not self.is_navigating and not self.is_searching_unknown_space:    
                if(self.waypointsFound == True):
                    self._navigate()
                else:
                    self._calculate()
        
    def _calculate(self):
        print('Calculating freespace')
        #blow up walls
        blowup = self._blow_up_walls(self.map)
        np.savetxt("map_normal.csv", self.map , delimiter=",", fmt='%1.3f')
        np.savetxt("map_blowup.csv", blowup , delimiter=",", fmt='%1.3f')

        robo_x = self.robot_x
        robo_y = self.robot_y

        self.is_searching_unknown_space = True
        self.waypoints = self._search_for_unknown_space(blowup, robo_x, robo_y)
        # search for an unkown space
        #goal_x, goal_y = self._search_for_unknown_space(blowup, robo_x, robo_y)
        # get the waypoints to the unkown space
        #self.waypoints = self.wavefront.run(blowup, robo_x, robo_y)
        self.waypointsFound = True
        self.is_searching_unknown_space = False

    def _search_for_unknown_space(self, map, robo_x, robo_y):
        start_queue = queue.Queue()
        start_queue.put((robo_x,robo_y))
        BFS_results = self._BFS(start_queue)


    def _BFS(self,map,queue=None):
        
        current_index = queue.get()
        current_x,current_y = current_index[0],current_index[1]

        element = map[current_y,current_x]

        if element == -1: return current_x,current_y

        for n in range(current_x-1,current_x+2):
            for m in range(current_y-1,current_y+2):
                if not (n==current_x and m==current_y) and n>-1 and m>-1 and n<map.shape[0] and m<map.shape[1] and (n,m) not in queue.queue :
                        queue.put((n,m))    
        return self._BFS(queue)
    
    def _blow_up_walls(self, map):
        map_old = copy.deepcopy(map)
        map_new = copy.deepcopy(map)
        num_rows = len(map)
        num_cols = len(map[0])
        for i in range(1): 
            for row in range(self.robot_radius, num_rows - self.robot_radius - 1):
                    for col in range(self.robot_radius, num_cols - self.robot_radius - 1):
                        if(map_old[row,col] == 100):
                            map_new[row - 1,col - 1] = 100
                            map_new[row + 1,col + 1] = 100
                            map_new[row - 1,col + 1] = 100
                            map_new[row + 1,col - 1] = 100
                            map_new[row - 1,col] = 100
                            map_new[row + 1,col] = 100
                            map_new[row,col - 1] = 100
                            map_new[row,col + 1] = 100
            map_old = copy.deepcopy(map_new)
            #print x
        return map_old

    def _scan_callback(self, scan):
        pass

    def _odom_callback(self, odom):
        if self.map_resolution > 0:
            # TODO: this seems not to be correct, robot is somewhere in the nowhere
            # convert from robot coordinates to map coordinates
            self.robot_x_pose = odom.pose.pose.position.x
            self.robot_y_pose = odom.pose.pose.position.y

            self.robot_x = int(math.floor((self.robot_x_pose - self.map_offset_x)/self.map_resolution))
            self.robot_y = int(math.floor((self.robot_y_pose - self.map_offset_y)/self.map_resolution))

            self.robot_pose_available = True     

    def _navigate(self):
            # get waypoint and start moving towards it
            # when success the process next
            self.is_navigating = True

            if(len(self.waypoints) > 0):
                print self.waypoints
                #(x, y, direction) = self.waypoints.pop(0)
                (x, y) = self.waypoints.pop(len(self.waypoints) - 1)
                self.waypoints = []
                print self.waypoints
                self.waypointsFound = True
                success = self._move(x, y)
                if success:
                    # next point
                    self._navigate()
                else:
                    #self._move(self.robot_x, self.robot_y)
                    self._navigate()
            else:
                self.is_navigating = False
                self.waypointsFound = False
                #time.sleep(5)

    def _move(self, x, y):
        """
        Moves the robot to a place defined by coordinates x and y.
        """
        print('Navigate to: ' + str(x) + ' | ' + str(y))
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # TODO: this seems not to be correct, targets are somewhere in the nowhere
        # target_x = (x - self.map_width/2.0 - self.map_offset_x) * self.map_resolution
        # target_y = (y - self.map_height/2.0 - self.map_offset_y) * self.map_resolution
        target_x = (x * self.map_resolution) + self.map_offset_x
        target_y = (y * self.map_resolution) + self.map_offset_y

        goal.target_pose.pose.position.x = target_x
        goal.target_pose.pose.position.y = target_y
        goal.target_pose.pose.orientation.w = 1
        self.move_base_client.send_goal(goal)
        
        success = self.move_base_client.wait_for_result(rospy.Duration(20, 0))
        #When success then go to next waypoint otherwise stop navigating and check map
        if success:
            print('Reached: ' + str(x) + ' | ' + str(y))
            return True
        else:
            print('Faild driving to: ' + str(x) + ' | ' + str(y))
            self.move_base_client.cancel_goal()
            return False

if __name__ == '__main__':
    try:
        explorer=Explorer()
    except rospy.ROSInterruptException:
        pass