#!/usr/bin/env python
import rospy
import numpy as np
import time
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
import tf
import copy
import math
from path_planing.msg import PathPoint
from path_planing.msg import FullPath
from path_planing.srv import FindUnknown, FindUnseen


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
        self.robot_x_old = 0
        
        self.robot_x_pose = 0
        self.robot_y = 0
        self.robot_y_old = 0
        self.robot_y_pose = 0

        self.robot_yaw = 0
        self.robot_yaw_old = 0

        self.map_height = 0
        self.map_width = 0
        self.map_offset_x = 0
        self.map_offset_y = 0
        self.map =[[]]
        self.map_info = None
        self.map_complete  = False
        self.map_complete_print = False

        self.waypointsAvailable = False

        self.map_camera = None
        self.map_camera_complete = False
        self.map_camera_seen_seq = 0
        self.map_camera_seen_initialised = False

        self.all_maps_complete_printed = False

        self.pose = Pose()

        self.pub_point = rospy.Publisher('/clicked_point',PointStamped, queue_size=1)
        self.pub_goal = rospy.Publisher('/move_to_goal/goal', Pose, queue_size=1)
        self.pub_seen_map = rospy.Publisher('/camera_seen_map', OccupancyGrid, queue_size=1)

        self.pose_subscriber = rospy.Subscriber('/simple_odom_pose',Pose, self._update_pose)
        self.mapSub = rospy.Subscriber('/map', OccupancyGrid, self._map_callback)
        self.scanSub = rospy.Subscriber('/scan', LaserScan, self._scan_callback)
        self.sub_goal_reached = rospy.Subscriber('/move_to_goal/reached', Bool, self._goal_reached_callback)
        
        rospy.wait_for_service('find_unkown_service')
        rospy.wait_for_service('find_unseen_service')

        self.find_unknown_service = rospy.ServiceProxy('find_unkown_service', FindUnknown)
        self.find_unseen_service = rospy.ServiceProxy('find_unseen_service', FindUnseen)

        self.received_map = False

        self._setup_camera_seen_masks()

        self.rate = rospy.Rate(20)


    def _setup_camera_seen_masks(self):
        self.mask_camera_seen_north = np.array([[1, 1, 1, 1, 1, 1, 1, 1, 1],
                                                [0, 1, 1, 1, 1, 1, 1, 1, 0],
                                                [0, 0, 1, 1, 1, 1, 1, 0, 0],
                                                [0, 0, 0, 1, 1, 1, 0, 0, 0],
                                                [0, 0, 0, 1, 1, 1, 0, 0, 0],
                                                [0, 0, 0, 0, 1, 0, 0, 0, 0]])
        self.mask_camera_seen_north_offset_x = -4
        self.mask_camera_seen_north_offset_y = -5

        self.mask_camera_seen_north_east = np.array([[1, 0, 0, 0, 0, 0, 0],
                                                     [1, 1, 0, 0, 0, 0, 0],
                                                     [1, 1, 1, 0, 0, 0, 0],
                                                     [1, 1, 1, 1, 0, 0, 0],
                                                     [1, 1, 1, 1, 1, 0, 0],
                                                     [1, 1, 1, 1, 1, 1, 0],
                                                     [1, 1, 1, 1, 1, 1, 1]])
        self.mask_camera_seen_north_east_offset_x = 0
        self.mask_camera_seen_north_east_offset_y = -6

        self.mask_camera_seen_east = np.array([ [0, 0, 0, 0, 0, 1],
                                                [0, 0, 0, 0, 1, 1],
                                                [0, 0, 0, 1, 1, 1],
                                                [0, 1, 1, 1, 1, 1],
                                                [1, 1, 1, 1, 1, 1],
                                                [0, 1, 1, 1, 1, 1],
                                                [0, 0, 0, 1, 1, 1],
                                                [0, 0, 0, 0, 1, 1],
                                                [0, 0, 0, 0, 0, 1]])
        self.mask_camera_seen_east_offset_x = 0
        self.mask_camera_seen_east_offset_y = -4

        self.mask_camera_seen_south_east = np.array([[1, 1, 1, 1, 1, 1, 1],
                                                     [1, 1, 1, 1, 1, 1, 0],
                                                     [1, 1, 1, 1, 1, 0, 0],
                                                     [1, 1, 1, 1, 0, 0, 0],
                                                     [1, 1, 1, 0, 0, 0, 0],
                                                     [1, 1, 0, 0, 0, 0, 0],
                                                     [1, 0, 0, 0, 0, 0, 0]])
        self.mask_camera_seen_south_east_offset_x = 0
        self.mask_camera_seen_south_east_offset_y = 0

        self.mask_camera_seen_south = np.array([[0, 0, 0, 0, 1, 0, 0, 0, 0],
                                                [0, 0, 0, 1, 1, 1, 0, 0, 0],
                                                [0, 0, 0, 1, 1, 1, 0, 0, 0],
                                                [0, 0, 1, 1, 1, 1, 1, 0, 0],
                                                [0, 1, 1, 1, 1, 1, 1, 1, 0],
                                                [1, 1, 1, 1, 1, 1, 1, 1, 1]])
        self.mask_camera_seen_south_offset_x = -4
        self.mask_camera_seen_south_offset_y = 0

        self.mask_camera_seen_south_west = np.array([[1, 1, 1, 1, 1, 1, 1],
                                                     [0, 1, 1, 1, 1, 1, 1],
                                                     [0, 0, 1, 1, 1, 1, 1],
                                                     [0, 0, 0, 1, 1, 1, 1],
                                                     [0, 0, 0, 0, 1, 1, 1],
                                                     [0, 0, 0, 0, 0, 1, 1],
                                                     [0, 0, 0, 0, 0, 0, 1]])
        self.mask_camera_seen_south_west_offset_x = -6
        self.mask_camera_seen_south_west_offset_y = 0

        self.mask_camera_seen_west = np.array([ [1, 0, 0, 0, 0 ,0],
                                                [1, 1, 0, 0, 0, 0],
                                                [1, 1, 1, 0, 0, 0],
                                                [1, 1, 1, 1, 1, 0],
                                                [1, 1, 1, 1, 1, 1],
                                                [1, 1, 1, 1, 1, 0],
                                                [1, 1, 1, 0, 0, 0],
                                                [1, 1, 0, 0, 0, 0],
                                                [1, 0, 0, 0, 0, 0]])
        self.mask_camera_seen_west_offset_x = -5
        self.mask_camera_seen_west_offset_y = -4

        self.mask_camera_seen_north_west = np.array([[0, 0, 0, 0, 0, 0, 1],
                                                     [0, 0, 0, 0, 0, 1, 1],
                                                     [0, 0, 0, 0, 1, 1, 1],
                                                     [0, 0, 0, 1, 1, 1, 1],
                                                     [0, 0, 1, 1, 1, 1, 1],
                                                     [0, 1, 1, 1, 1, 1, 1],
                                                     [1, 1, 1, 1, 1, 1, 1]])
        self.mask_camera_seen_north_west_offset_x = -6
        self.mask_camera_seen_north_west_offset_y = -6

    def _map_callback(self, data):
        self.map_resolution = data.info.resolution
        self.map_height = data.info.height
        self.map_width = data.info.width
        self.map_offset_x = data.info.origin.position.x
        self.map_offset_y = data.info.origin.position.y
        self.map_info = data.info
        self.map = np.reshape(data.data, (data.info.height, data.info.width))
        self.received_map = True

        if not self.map_camera_seen_initialised:
            self.map_camera = np.full((self.map_height, self.map_width), -1)
            self._publish_map_camera()
            self.map_camera_seen_initialised = True

    
    def run(self):
        start = time.time()
        while not rospy.is_shutdown():
            if self.map_complete == False or self.map_camera_complete == False:
                if not self.map_complete_print and self.map_complete:
                    self.map_complete_print = True
                    print "---> MAPPING COMPLETE <---"
                
                if self.robot_pose_available and not self.is_navigating and not self.is_searching_unknown_space and self.received_map:    
                    if(self.waypointsAvailable == True):
                        self._navigate()
                    else:
                        self._calculate()
                self.rate.sleep()
            else:
                if not self.all_maps_complete_printed:
                    end = time.time()
                    self.all_maps_complete_printed = True
                    print('--> All complete <--')
                    print('--- Duration: ' + str(end-start) + 'sec ---')

        end = time.time()
        self.all_maps_complete_printed = True
        print('--> All complete <--')
        print('--- Duration: ' + str(end-start) + 'sec ---')


    def _calculate(self):
        print('Calculating freespace')

        self.is_searching_unknown_space = True

        service_response = None
        if not self.map_complete:
            # search for undiscoverd space by lidar
            service_response = self.find_unknown_service(self.blowUpCellNum, self.robot_x, self.robot_y, self.robot_radius)
        else:
            if self.map_camera_seen_initialised:
                # search for unseen space by the camer
                service_response = self.find_unseen_service(self.blowUpCellNum, self.robot_x, self.robot_y, self.robot_radius)
        
        self.waypoints = service_response.waypoints.fullpath
        allpoints = service_response.allPoints.fullpath

        if self.waypoints == [] and allpoints == []:
            # check which map is currently being completed
            if not self.map_complete:
                self.map_complete = True
            else:
                self.map_camera_complete = True
        else: 
            self.last_waypoints = self.waypoints
            # clear clicked points
            for _ in range(100):            
                self._publish_point(0, 0)
                self.rate.sleep()

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
            self.pose.position.x = data.position.x
            self.pose.position.y = data.position.y
            self.pose.position.z = data.position.z
            self.pose.orientation.x = data.orientation.x 
            self.pose.orientation.y = data.orientation.y 
            self.pose.orientation.z = data.orientation.z
            self.pose.orientation.w = data.orientation.w 

            self.robot_y_pose = self.pose.position.y
            self.robot_x_pose = self.pose.position.x

            self.robot_x_old = self.robot_x
            self.robot_y_old = self.robot_y
            self.robot_yaw_old = self.robot_yaw

            self.robot_x = int(math.floor((self.robot_x_pose - self.map_offset_x)/self.map_resolution))
            self.robot_y = int(math.floor((self.robot_y_pose - self.map_offset_y)/self.map_resolution))
            self.robot_yaw = self._robot_angle()
            
            self.robot_pose_available = True

            try:
                if self.robot_x != self.robot_x_old or self.robot_y != self.robot_y_old or abs(self.robot_yaw - self.robot_yaw_old) > 0.1:
                    self._update_map_camera_seen(self.robot_x, self.robot_y, self.robot_yaw)
            except:
                print('update map camera seen failed')
    
    def _update_map_camera_seen(self, x, y, yaw):
        """
        Updates the map that tracks the seen areas from the view of the camera
        """
        #print('Angle: ' + str(yaw))
        yaw = (yaw + math.pi/2.0 + 2*math.pi) % (2*math.pi)
        #print('Angle 2pi: ' + str(yaw))
        
        # North
        if (yaw >= (15.0/16.0 * 2.0 * math.pi) and yaw < (2 * math.pi)) or (yaw >= 0 and yaw < (1.0/16.0 * 2.0 * math.pi)):
            self._map_camera_set_seen(x, y, self.mask_camera_seen_north, self.mask_camera_seen_north_offset_x, self.mask_camera_seen_north_offset_y)

        # North East
        if yaw >= (1.0/16.0 * 2.0 * math.pi) and yaw < (3.0/16.0 * 2.0 * math.pi):
            self._map_camera_set_seen(x, y, self.mask_camera_seen_north_east, self.mask_camera_seen_north_east_offset_x, self.mask_camera_seen_north_east_offset_y)

         # East
        if yaw >= (3.0/16.0 * 2.0 * math.pi) and yaw < (5.0/16.0 * 2.0 * math.pi):
            self._map_camera_set_seen(x, y, self.mask_camera_seen_east, self.mask_camera_seen_east_offset_x, self.mask_camera_seen_east_offset_y)

        # South East
        if yaw >= (5.0/16.0 * 2.0 * math.pi) and yaw < (7.0/16.0 * 2.0 * math.pi):
            self._map_camera_set_seen(x, y, self.mask_camera_seen_south_east, self.mask_camera_seen_south_east_offset_x, self.mask_camera_seen_south_east_offset_y)

        # South
        if yaw >= (7.0/16.0 * 2.0 * math.pi) and yaw < (9.0/16.0 * 2.0 * math.pi):
            self._map_camera_set_seen(x, y, self.mask_camera_seen_south, self.mask_camera_seen_south_offset_x, self.mask_camera_seen_south_offset_y)

        # South West
        if yaw >= (9.0/16.0 * 2.0 * math.pi) and yaw < (11.0/16.0 * 2.0 * math.pi):
            self._map_camera_set_seen(x, y, self.mask_camera_seen_south_west, self.mask_camera_seen_south_west_offset_x, self.mask_camera_seen_south_west_offset_y)

        # West
        if yaw >= (11.0/16.0 * 2.0 * math.pi) and yaw < (13.0/16.0 * 2.0 * math.pi):
            self._map_camera_set_seen(x, y, self.mask_camera_seen_west, self.mask_camera_seen_west_offset_x, self.mask_camera_seen_west_offset_y)

        # North West
        if yaw >= (13.0/16.0 * 2.0 * math.pi) and yaw < (15.0/16.0 * 2.0 * math.pi):
            self._map_camera_set_seen(x, y, self.mask_camera_seen_north_west, self.mask_camera_seen_north_west_offset_x, self.mask_camera_seen_north_west_offset_y)

        self.map_camera_seen_seq = self.map_camera_seen_seq + 1
        
        self._publish_map_camera()

    def _publish_map_camera(self):
        # create occupany grid to publish it
        oG = OccupancyGrid()
        # header
        oG.header.seq = self.map_camera_seen_seq
        oG.header.frame_id = "map"
        oG.header.stamp = rospy.Time.now()
        # set the info like it is in the original map
        oG.info = self.map_info
        # set the map reshaped as array
        oG.data = np.reshape(self.map_camera, self.map_height * self.map_width)

        self.pub_seen_map.publish(oG)

    def _map_camera_set_seen(self, x, y, mask, offset_x, offset_y, positiv_direction = True):
        """
        This method sets the matrix of the map which represents the seen area by camera
        """
        y_mask = 0
        x_mask = 0

        y_start = 0
        y_end = 0
        y_increment = 1
        x_start = 0
        x_start = 0
        x_increment = 1

        if positiv_direction:
            y_start = y + offset_y
            y_end = (y + offset_y + len(mask))
            y_increment = 1

            x_start = x + offset_x
            x_end = (x + offset_x + len(mask[0]))
            x_increment = 1
        else:
            y_end = y + offset_y
            y_start = (y + offset_y + len(mask))
            y_increment = -1

            x_end = x + offset_x
            x_start = (x + offset_x + len(mask[0]))
            x_increment = -1

        # rows
        for y1 in range(y_start, y_end, y_increment):
            x_mask = 0
            # cols
            for x1 in range(x_start, x_end, x_increment):
                # check that not outside of map
                if y1 >= 0 and x1 >= 0 and y1 < len(self.map_camera) and x1 < len(self.map_camera[0]):
                    # set field in camera map to 0 if not wall
                    #if self.map[y1][x1] != 100 and self.map[y1][x1] != -1:
                    # if the mask has a 1 than it is the field of view
                    if mask[y_mask][x_mask] == 1:
                        self.map_camera[y1][x1] = 10
                   # else:
                        # exit for loop because can not view behind wall
                        #break
                x_mask = x_mask + 1

            y_mask = y_mask + 1


    def _robot_angle(self):
        orientation_q = self.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        return yaw

    def _navigate(self):
        # get waypoint and start moving towards it
        # when success the process next
        self.is_navigating = True

        if(len(self.waypoints) > 0):
            print(self.waypoints)
            point = self.waypoints.pop(0)
            x = point.path_x
            y = point.path_y

            print(self.waypoints)

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
