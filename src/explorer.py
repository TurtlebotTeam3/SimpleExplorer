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
        self.waypointsAvailable = False
        self.mapComplete  = False
        self.mapCompletePrint = False

        self.move_base_client = actionlib.SimpleActionClient(
            'move_base', MoveBaseAction)
        self.odomSub = rospy.Subscriber('/odom', Odometry, self._odom_callback)
        self.mapSub = rospy.Subscriber(
            '/map', OccupancyGrid, self._map_callback)
        self.scanSub = rospy.Subscriber('/scan', LaserScan, self._scan_callback)
        self.pub_point = rospy.Publisher('clicked_point',PointStamped, queue_size=1)

        self.pub_goal = rospy.Publisher('move_to_goal/goal', Pose, queue_size=1)
        self.sub_goal_reached = rospy.Subscriber('/move_to_goal/reached', Bool, self._goal_reached_callback)

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
        #blow up walls
        map = self._blow_up_wall(self.map)
        np.savetxt("map_normal.csv", self.map , delimiter=",", fmt='%1.3f')
        np.savetxt("map_blowup.csv", map , delimiter=",", fmt='%1.3f')

        robo_x = self.robot_x
        robo_y = self.robot_y

        self.is_searching_unknown_space = True
        
        self.waypoints, allpoints = self.wavefront.find_unknown(map,self.robot_x, self.robot_y, self.robot_radius)
        if self.waypoints == None and allpoints == None:
            self.mapComplete = True
            self.move_base_client.cancel_goal()
        else: 
            # clear clicked points
            for i in range(100):            
                self._publish_point(0, 0)
                self.rate.sleep()

            #for (x, y) in allpoints:
            #    self._publish_point(x, y)
            #    self.rate.sleep()

            for (x, y) in self.waypoints:
                self._publish_point(x, y)
                self.rate.sleep()
            
            self.waypointsAvailable = True
            self.is_searching_unknown_space = False

    def _blow_up_wall(self, map):
        #blow up walls
        blowUpCellNum = 3
        tmp_map = copy.deepcopy(map)
        for row in range(0, len(tmp_map)):
            for col in range(0 , len(tmp_map[0])):
                if map[row,col] == 100 and row >= blowUpCellNum and col >= blowUpCellNum and row <= len(tmp_map) - blowUpCellNum and col <= len(tmp_map[0]) - blowUpCellNum:
                    tmp_map[row - blowUpCellNum : row +1 + blowUpCellNum, col - blowUpCellNum : col +1 + blowUpCellNum] = 100
        return tmp_map

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

    def _navigate(self):
        # get waypoint and start moving towards it
        # when success the process next
        self.is_navigating = True

        if(len(self.waypoints) > 0):
            print self.waypoints
            (x, y) = self.waypoints.pop(0)
            # (x, y) = self.waypoints.pop(len(self.waypoints) - 1)
            # self.waypoints = []
            print self.waypoints
            # self.waypointsAvailable = True
            
            # -- move base --
            #self._move_1(x, y)

            # -- move to goal --
            self._move_2(x, y)

        else:
            self.is_navigating = False
            self.waypointsAvailable = False

    
    # # # # # # # # # # # # # # # # # # # # 
    #              Move Base
    # # # # # # # # # # # # # # # # # # # #
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
        success = self.move_base_client.wait_for_result(rospy.Duration(25))
        #When success then go to next waypoint otherwise stop navigating and check map
        if success:
            print('Reached: ' + str(x) + ' | ' + str(y))
            self._navigate()
        else:
            print('Faild driving to: ' + str(x) + ' | ' + str(y))
            self.move_base_client.cancel_goal()
            self.is_navigating = False



    # # # # # # # # # # # # # # # # # # # # 
    #              Move to goal
    # # # # # # # # # # # # # # # # # # # #
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
