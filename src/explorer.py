#!/usr/bin/env python
import rospy
import numpy as np
import time
import actionlib
import tf
import copy
import math
from std_msgs.msg import String, Bool
from nav_msgs.msg import OccupancyGrid, Odometry, MapMetaData
from sensor_msgs.msg._LaserScan import LaserScan
from geometry_msgs.msg import Pose
from path_planing.msg import PathPoint, FullPath
from path_planing.srv import FindUnknown, FindUnseen
from simple_camera.srv import EnableBlobDetection, EnableTagKnownCheck
from simple_odom.msg import CustomPose, PoseConverted
from path_drive.msg import PathDriveAction, PathDriveActionGoal, PathDriveActionResult


class Explorer:

    def __init__(self):
        rospy.init_node('explorer', anonymous=True)
        
        self.is_navigating = False
        self.is_searching_unknown_space = False
        self.robot_pose_available = False
        
        self.robot_radius = 1
        self.blowUpCellNum = 3
        
        self.map_info = MapMetaData()
        self.map_complete  = False
        self.map_complete_print = False

        self.map_camera_complete = False
        self.all_maps_complete_printed = False

        self.waypoints = []
        self.waypointsAvailable = False

        self.pose = Pose()
        self.pose_converted = PoseConverted()

        self.pub_seen_map = rospy.Publisher('camera_seen_map', OccupancyGrid, queue_size=1)
        self.marker_waypoint_publisher = rospy.Publisher('waypoint_marker_array', MarkerArray, queue_size=1)

        self.pose_subscriber = rospy.Subscriber('simple_odom_pose', CustomPose, self._handle_update_pose)
        
        rospy.wait_for_service('find_unkown_service')
        rospy.wait_for_service('find_unseen_service')

        self.find_unknown_service = rospy.ServiceProxy('find_unkown_service', FindUnknown)
        self.find_unseen_service = rospy.ServiceProxy('find_unseen_service', FindUnseen)

        self.client = actionlib.SimpleActionClient('path_drive_server', PathDriveAction)
        self.client.wait_for_server()

        self._setup()
        rospy.loginfo("--- ready ---")
        self.rate = rospy.Rate(20)

    def _setup(self):
        map = rospy.wait_for_message('map', OccupancyGrid)
        self.map_info = map.info
    
    def run(self):
        start = time.time()
        
        while not rospy.is_shutdown():
            if self.map_complete == False or self.map_camera_complete == False:
                if not self.map_complete_print and self.map_complete:
                    self.map_complete_print = True
                    print "---> MAPPING COMPLETE <---"
                if self.robot_pose_available and not self.is_navigating and not self.is_searching_unknown_space:    
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
            service_response = self.find_unknown_service(self.blowUpCellNum, self.pose_converted.x, self.pose_converted.y, self.robot_radius)
        else:
            # search for unseen space by the camer
            service_response = self.find_unseen_service(self.blowUpCellNum, self.pose_converted.x, self.pose_converted.y, self.robot_radius)
        
        self.waypoints = service_response.waypoints.fullpath
        allpoints = service_response.allPoints.fullpath

        if self.waypoints == [] and allpoints == []:
            # check which map is currently being completed
            if not self.map_complete:
                self.map_complete = True
            else:
                self.map_camera_complete = True
        else:            
            self.waypointsAvailable = True
        
        self.is_searching_unknown_space = False

    def _handle_update_pose(self, data):
        if self.map_info.resolution > 0:
            
            self.pose = data.pose
            self.pose_converted = data.pose_converted
            
            self.robot_pose_available = True

    def _navigate(self):
        """
        Navigating to the waypoints
        """
        self.is_navigating = True

        goal = PathDriveActionGoal()

        goal.goal.waypoints.fullpath = self.waypoints

        self.client.send_goal(goal.goal)

        self.client.wait_for_result()

        self.waypoints = []
        self.is_navigating = False
        self.waypointsAvailable = False

if __name__ == '__main__':
    try:
        explorer=Explorer()
        explorer.run()
    except rospy.ROSInterruptException:
        pass
