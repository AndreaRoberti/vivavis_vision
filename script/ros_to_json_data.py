#!/usr/bin/env python3

# ROS libraries
import rospy
from sensor_msgs.msg import PointCloud2, Image
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid
import tf
from std_msgs.msg import String, Time, Bool
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Point

# Python libraries
import numpy as np
import open3d as o3d
import math
import pandas as pd
import os
import cv2
from cv_bridge import CvBridge
import glob
import uuid
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull

from vivavis_vision.msg import WallInfo, WallInfoArray

bridge = CvBridge()

class ROS2JsonData:
    def __init__(self):
        super().__init__()

        rospy.init_node('ros_to_json_data')   
        
        self.act_cam_position = []     # store the ACTUAL camera position vector[x,y,z]
        self.walls_ordered_eq_params = [[],[],[],[],[],[]]   # ORDERED equations' parameters of "walls" found inside the scene
       
        
        
        rospy.Subscriber("vivavis_vision/walls_info", WallInfoArray, self.wall_info_callback)
        self.json_walls_equations_pub = rospy.Publisher('out/json_walls_equations', String, queue_size=100)
        self.json_human_workspace_pub = rospy.Publisher('out/json_human_workspace', String, queue_size=100)

        self.walls_info = WallInfoArray()
        
        self.listener = tf.TransformListener()
        self.camera_frame = 'camera_color_optical_frame'
        self.fixed_frame = 'world'
        
        self.act_cam_position = []     # store the ACTUAL camera position vector[x,y,z]
        self.act_cam_orientation = []  # store the ACTUAL camera orientation quaternion [x,y,z,w]
     
    def wall_info_callback(self, data):
        self.walls_info = data

    def update(self):
        self.getCameraPose()
        self.publish_json_df()

    def getCameraPose(self):
        try:
            # Listen for the transform from camera_frame to fixed_frame
            (trans, rot) = self.listener.lookupTransform(self.fixed_frame, self.camera_frame, rospy.Time(0))
            self.act_cam_position = trans
            self.act_cam_orientation = rot
            # rospy.loginfo("Translation: %s" % str(trans))
            # rospy.loginfo("Rotation: %s" % str(rot))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Transform lookup failed!")

    # 0 = left; 1 = right; 2 = ceiling; 3 = floor; 4 = front; 5 = back
    # Create pandas dataframe
    def publish_json_df(self):
        json_data = []
        # wall_names = ['left','right', 'ceiling', 'floor', 'front', 'back']

        # my order : floor left right front back ceiling
        for w in self.walls_info.walls:
            # print(w)
            if w.header.frame_id:
                if len(self.act_cam_position) > 0:
                    cam_x,cam_y,cam_z = self.act_cam_position
                    shortest_dist = self.shortest_point_plane_distance(self.act_cam_position[0], self.act_cam_position[1], self.act_cam_position[2], w.a, w.b, w.c, w.d)
                else:
                    shortest_dist = None  #tf hasn't been received yet

                newlist = [w.header.frame_id, w.a,w.b,w.c,w.d, shortest_dist, 
                        w.num_points, w.pose.position.x, w.pose.position.y, w.pose.position.z, w.color_id]
                json_data.append(newlist)

        if len(json_data) > 0:        
            df_json = pd.DataFrame(json_data, columns=["wall_type", "a", "b", "c", "d", "shortest_distance", "num_points", "plane_center_x", "plane_center_y", "plane_center_z", "color_id"])
            self.json_walls_equations_pub.publish(str(df_json.to_json(orient='index')))

    # Function to find distance from point to plane
    def shortest_point_plane_distance(self, x1, y1, z1, a, b, c, d):        
        d = abs((a * x1 + b * y1 + c * z1 + d))
        e = (math.sqrt(a * a + b * b + c * c))
        # print (e)
        dist = d/e
        # print("Perpendicular distance is", dist)
        return dist

if __name__ == '__main__':
    
    ros_json_data = ROS2JsonData()

    rate = 100 # Hz
    ros_rate = rospy.Rate(rate)
			  
    while not rospy.is_shutdown():
        # ros_json_data.control_loop()
        ros_json_data.update()
        ros_rate.sleep()
