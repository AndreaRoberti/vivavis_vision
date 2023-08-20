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

    def wall_info_callback(self, data):
        self.walls_info = data
        print(self.walls_info)
        # floor left right front back ceiling


    # control loop, executed each "self.timer_period" seconds
    def control_loop(self, time):          
        print("[control loop] Started")
        t1 = rospy.get_time() # init loop time
   
            # Publish json dataframe
        self.publish_json_df()

            # compute and publish (if needed) human's workspace around him, 
            # according to "parameters.py" file in the form of a pointcloud and a bounding box
        # self.publish_human_workspace()

        t2 = rospy.get_time()
        print("[control loop] Ended", t2-t1)

    def update(self):
        self.publish_json_df()



    # self.walls_ordered_eq_params -> 0 = left; 1 = right; 2 = ceiling; 3 = floor; 4 = front; 5 = back
    def publish_json_df(self):
        # Create pandas dataframe from list "self.walls_ordered_eq_params"
        json_data = []
        wall_names = ['left','right', 'ceiling', 'floor', 'front', 'back']

        for w in range(len(self.walls_info.walls)):
            print(w)
        
        # if self.print: 
        #     print("Detected walls and distances:")        


        # for k,eq in enumerate(self.walls_ordered_eq_params):
        #     if eq != []:
        #         a,b,c,d,num_points, plane_center, color_id = list(eq)
        #         center_x, center_y, center_z = plane_center               

        #         if len(self.act_cam_position) > 0:
        #             cam_x,cam_y,cam_z = self.act_cam_position
        #             shortest_dist = self.shortest_point_plane_distance(cam_x,cam_y,cam_z, a,b,c,d)
        #         else:
        #             shortest_dist = None  #tf hasn't been received yet

        #         print(wall_names[k], "wall; distance:", shortest_dist, "m")
        #         newlist = [wall_names[k], a,b,c,d, shortest_dist, num_points, center_x, center_y, center_z, color_id]
        #         json_data.append(newlist)
        # print("") 

        if len(json_data) > 0:        
            df_json = pd.DataFrame(json_data, columns=["wall_type", "a", "b", "c", "d", "shortest_distance", "num_points", "plane_center_x", "plane_center_y", "plane_center_z", "color_id"])
            self.json_walls_equations_pub.publish(str(df_json.to_json(orient='index')))


    # Function to find distance from point to plane
    def shortest_point_plane_distance(self, x1, y1, z1, a, b, c, d):        
        d = abs((a * x1 + b * y1 + c * z1 + d))
        e = (math.sqrt(a * a + b * b + c * c))
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
