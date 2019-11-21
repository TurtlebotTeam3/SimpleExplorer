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
from wavefront import Wavefront
import actionlib
from tf import TransformListener
import copy
import math

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
        self.spam_clicked_point = False

        self.move_base_client = actionlib.SimpleActionClient(
            'move_base', MoveBaseAction)
        self.odomSub = rospy.Subscriber('/odom', Odometry, self._odom_callback)
        self.mapSub = rospy.Subscriber(
            '/map', OccupancyGrid, self._map_callback)
        self.scanSub = rospy.Subscriber('/scan', LaserScan, self._scan_callback)
        self.pub_point = rospy.Publisher('clicked_point',PointStamped, queue_size=1)

        self.pub_goal = rospy.Publisher('move_to_goal/goal', Pose, queue_size=1)
        self.sub_goal_reached = rospy.Subscriber('/move_to_goal/reached', Bool, self._goal_reached_callback)

        self.rate = rospy.Rate(20)

        rospy.spin()

    def _map_callback(self, data):
        self.map_resolution = data.info.resolution
        self.map_height = data.info.height
        self.map_width = data.info.width
        self.map_offset_x = data.info.origin.position.x
        self.map_offset_y = data.info.origin.position.y
        
        # reshape the map
        map = np.reshape(data.data, (data.info.height, data.info.width))
        self._map_update(map)
       

    def _map_update(self, map):
        if self.robot_pose_available and not self.is_navigating and not self.is_searching_unknown_space:
            self.is_searching_unknown_space = True
            # blow up map
            map = self._blow_up_map(map)
            # search for an unkown space
            # x, y = self._search_for_unknown_space(map)
            print('Calculating waypoints')
            self.waypoints, allpoints = self.wavefront.findUnknown(map,self.robot_x, self.robot_y, self.robot_radius)
            # get the waypoints to the unkown space
            
            #self.waypoints = self.wavefront.run(
            #     map, x, y, self.robot_x, self.robot_y, self.robot_radius)
            # self.wavefront.findUnknown(self.map, self.robot_x, self.robot_y, self.robot_radius)
            # self.waypoints.append((x,y))

            # clear clicked points
            for i in range(100):            
                self._publish_point(0, 0)
                self.rate.sleep()

            # for (x, y) in allpoints:
            #    self._publish_point(x, y)
            #    self.rate.sleep()

            for (x, y) in self.waypoints:
                self._publish_point(x, y)
                self.rate.sleep()

            self.is_searching_unknown_space = False
            #start navigating
            self._navigate()

    def _blow_up_map(self, map):
        #blow up walls
        blowUpCellNum = 1
        tmp_map = copy.deepcopy(map)
        for row in range(0, len(tmp_map)):
            for col in range(0 , len(tmp_map[0])):
                if map[row,col] == 100 and row >= blowUpCellNum and col >= blowUpCellNum and row <= len(tmp_map) - blowUpCellNum and col <= len(tmp_map[0]) - blowUpCellNum:
                    tmp_map[row - blowUpCellNum : row + blowUpCellNum, col - blowUpCellNum : col + blowUpCellNum] = 100
        return tmp_map

    def _goal_reached_callback(self, reached):
        print('Reached: ' + str(reached))
        if reached:
            self._navigate()
        else:
            self.waypoints = []
            self.is_navigating = False


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
            # self.robot_x = int(math.ceil(self.map_width/2 + self.map_offset_x + self.robot_x_pose/self.map_resolution))
            # self.robot_y = int(math.ceil(self.map_height/2 + self.map_offset_y + self.robot_y_pose/self.map_resolution))

            self.robot_pose_available = True

    def _search_for_unknown_space(self, map):
        num_rows = len(map)
        num_cols = len(map[0])
        for row in range(self.robot_radius, num_rows - self.robot_radius):
            for col in range(self.robot_radius, num_cols - self.robot_radius):
                if map[row][col] == 0:
                    print('row: ' + str(row) + ' col: ' + str(col))
                    surrounding_any_wall = map[row - self.robot_radius : row + self.robot_radius + 1, col - self.robot_radius : col + self.robot_radius + 1] == 100
                    if map[row - 1][col] == -1 and not any(np.any(surrounding_any_wall, axis = 0)) and not any(np.any(surrounding_any_wall, axis = 1)):
                        return col, row
                    if map[row + 1][col] == -1 and not any(np.any(surrounding_any_wall, axis = 0)) and not any(np.any(surrounding_any_wall, axis = 1)):
                        return col, row
                    if map[row][col - 1] == -1 and not any(np.any(surrounding_any_wall, axis = 0)) and not any(np.any(surrounding_any_wall, axis = 1)):
                        return col , row
                    if map[row][col + 1] == -1 and not any(np.any(surrounding_any_wall, axis = 0)) and not any(np.any(surrounding_any_wall, axis = 1)):
                        return col , row

    def _navigate(self):
        # check if waypoints are available
        if len(self.waypoints) > 0:
            self.is_navigating = True
            # get waypoint and start moving towards it
            # when success the process next
            
            (x, y) = self.waypoints.pop(0) # visit all way points
            # send final goal
            # (x, y) = self.waypoints[len(self.waypoints) - 1]
            # self.waypoints = []

            self._move_2(x,y)
            # success = self._move_1(x, y)
            # if not success:
                # when not success plan new path
            #    self.waypoints = []
        else:
            self.is_navigating = False

    def _move_1(self, x, y):
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
        success = self.move_base_client.wait_for_result(rospy.Duration(60))
        #When success then go to next waypoint otherwise stop navigating and check map
        if success:
            print('Reached: ' + str(x) + ' | ' + str(y))
            self._navigate()
        else:
            print('Faild driving to: ' + str(x) + ' | ' + str(y))
            self.is_navigating = False

    def _move_2(self, x, y):
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

if __name__ == '__main__':
    try:
        explorer=Explorer()
    except rospy.ROSInterruptException:
        pass
