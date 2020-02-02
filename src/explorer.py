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
from visualization_msgs.msg import Marker, MarkerArray
from simple_camera.srv import EnableBlobDetection, EnableTagKnownCheck
from simple_odom.msg import CustomPose, PoseConverted

class Explorer:

    def __init__(self):
        rospy.init_node('explorer', anonymous=True)
        
        self.marker_array = None
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

        self.pub_goal = rospy.Publisher('/move_to_goal/goal', Pose, queue_size=1)
        self.pub_seen_map = rospy.Publisher('/camera_seen_map', OccupancyGrid, queue_size=1)
        self.marker_waypoint_publisher = rospy.Publisher('waypoint_marker_array', MarkerArray, queue_size=1)

        self.pose_subscriber = rospy.Subscriber('/simple_odom_pose', CustomPose, self._handle_update_pose)
        self.sub_goal_reached = rospy.Subscriber('/move_to_goal/reached', Bool, self._goal_reached_callback)
        
        rospy.wait_for_service('find_unkown_service')
        rospy.wait_for_service('find_unseen_service')
        #rospy.wait_for_service('enable_blob_detection_service')
        #rospy.wait_for_service('enable_tag_known_check_service')

        self.find_unknown_service = rospy.ServiceProxy('find_unkown_service', FindUnknown)
        self.find_unseen_service = rospy.ServiceProxy('find_unseen_service', FindUnseen)
        #self.enable_blob_detection_service = rospy.ServiceProxy('enable_blob_detection_service', EnableBlobDetection)
        #self.enable_tag_known_check_service = rospy.ServiceProxy('enable_tag_known_check_service', EnableTagKnownCheck)

        self._setup()
        rospy.loginfo("--- ready ---")
        self.rate = rospy.Rate(20)

    def _setup(self):
        map = rospy.wait_for_message('/map', OccupancyGrid)
        self.map_info = map.info
    
    def run(self):
        start = time.time()
        #bool_blob = Bool()
        #bool_blob.data = True
        #bool_tag = Bool()
        #bool_tag.data = True
        #self.enable_blob_detection_service(bool_blob)
        #self.enable_tag_known_check_service(bool_tag)
        
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
            self._publish_list(self.waypoints)
            
            self.waypointsAvailable = True
        
        self.is_searching_unknown_space = False

    def _handle_update_pose(self, data):
        if self.map_info.resolution > 0:
            
            self.pose = data.pose
            self.pose_converted = data.pose_converted
            
            self.robot_pose_available = True

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

        target_x = (x * self.map_info.resolution) + self.map_info.origin.position.x
        target_y = (y * self.map_info.resolution) + self.map_info.origin.position.y

        goal.position.x = target_x
        goal.position.y = target_y
        goal.orientation.w = 1

        self.pub_goal.publish(goal)

    def _goal_reached_callback(self, reached):
        print('Reached: ' + str(reached))
        if reached:
            self._navigate()
        else:
            self.waypoints = []
            self.is_navigating = False

    def _publish_list(self, list):
        
        markerArray = self._create_marker_array(list, 0.075,0.35, 0.35, 0.85)
                
        if self.marker_array != None:
            for oldmarker in self.marker_array.markers:
                oldmarker.action = Marker.DELETE
            self.marker_waypoint_publisher.publish(self.marker_array)
            rospy.sleep(0.01)

        self.marker_array = markerArray
        self.marker_waypoint_publisher.publish(self.marker_array)
        rospy.sleep(0.01)

    def _create_marker_array(self, list, size, red, green, blue):
        markerArray = MarkerArray()
        for point in list:
            try:
                x = point.path_x
                y = point.path_y
            except:
                try:
                    y, x = point
                except:
                    print("An exception occurred")
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale.x = size
            marker.scale.y = size
            marker.scale.z = size
            marker.color.r = red
            marker.color.g = green
            marker.color.b = blue
            marker.color.a = 1.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = (x * self.map_info.resolution) + self.map_info.origin.position.x
            marker.pose.position.y = (y * self.map_info.resolution) + self.map_info.origin.position.y 
            marker.pose.position.z = 1

            markerArray.markers.append(marker)

        id = 0
        for m in markerArray.markers:
            m.id = id
            id += 1

        return markerArray

if __name__ == '__main__':
    try:
        explorer=Explorer()
        explorer.run()
    except rospy.ROSInterruptException:
        pass
