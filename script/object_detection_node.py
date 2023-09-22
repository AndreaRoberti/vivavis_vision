#!/usr/bin/env python

# ROS libraries
import rospy
from sensor_msgs.msg import Image, PointCloud2
from darknet_ros_msgs.msg import BoundingBoxes
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import tf
from std_msgs.msg import String, Bool
from nav_msgs.msg import OccupancyGrid
from rosgraph_msgs.msg import Clock
from visualization_msgs.msg import Marker, MarkerArray

# Python libraries
import numpy as np
import open3d as o3d
import cv2
import pandas as pd
import os
from scipy.spatial.transform import Rotation as R
import glob 

# My libraries
from mylib.lib_cloud_conversion_between_Open3D_and_ROS import *
from mylib.draw_poly_marker import create_bbox_points, create_name_marker
from mylib.parameters import *
from mylib.iou3d import box3d_iou, get_3d_box
from mylib.transf_utils import *

bridge = CvBridge()

class ObjectDetectionDistance:

    def __init__(self):
        super().__init__()

        rospy.init_node('obj_det_dist')
        
        # Publishers to visualize names and bboxes of the detected objects
        self.objects_bbox_3d_pub = rospy.Publisher('/out/objects_bbox_3d', MarkerArray, queue_size=100)
        self.object_names_3d_pub = rospy.Publisher('/out/objects_names_3d', MarkerArray, queue_size=100)

        # Publisher to store the object information that will be saved in .jsons files
        self.json_objects_pub = rospy.Publisher('/out/json_objects', String, queue_size=100)

        # Map2D publisher / subscriber
        rospy.Subscriber('/out/map2d_img1', Image, self.map2d_img_callback)    
        # rospy.Subscriber('/camera/color/image_raw', Image, self.map2d_img_callback)   
        self.map2d_1_img = None           
  
        self.map2d_img_pub = rospy.Publisher('/out/map2d_img2', Image, queue_size=100) 

        # Subscriber to the 2D projection map, needed to have the self.origin_cm 
        self.origin_cm = None
        #rospy.Subscriber('/rtabmap/proj_map', OccupancyGrid, self.proj_map_callback)
        rospy.Subscriber('/rtabmap/grid_prob_map', OccupancyGrid, self.proj_map_callback)

        # Pointcloud publishers to have detected objects' pcd
        self.obj_pcd_pub = rospy.Publisher('/out/obj', PointCloud2, queue_size=100) #useful topics for debugging
        self.depth_pub = rospy.Publisher('/out/depth', PointCloud2, queue_size=100) #useful topics for debugging

        # Detection parameters    
        self.detection_th = 0.25 # probability threshold for considering the object
        self.curr_frame_obj_bboxes = []  # list of detected objects in the current frame, solidal to the camera
        self.detected_objects = []  # list of all detected objects in "map" (SLAM) frame
        
        # Variables to know if camera is moving
        self.prev_trans_cam_map = None
        self.prev_rot_eul_cam_map = None
        self.is_moving = False       # bool to know if camera is moving (according to rgb frame callback)

        self.publish_rviz = False

        # Subscribe to the use_bag topic
        self.use_bag = None
        while self.use_bag == None:
            rospy.Subscriber('/use_bag', Bool, self.bag_bool_callback)
            print("Waiting for the /use_bag topic!")

        # If using a bag, create variables, set tf transformations' folder, subcribe to /clock topic
        if self.use_bag:
            print("Using the rosbag...")
            # Init time clock for offline bags
            self.act_time = None # ROS (clock) actual time as a string
            self.last_detection_time = None # time corresponding to detection (img - jsons must be correspond)
            self.act_timestamp = None
            self.T_cam_map, self.trans_cam_map, self.rot_quat_cam_map, self.rot_eul_cam_map = None, None, None, None
            self.T_map_cam, self.trans_map_cam, self.rot_quat_map_cam, self.rot_eul_map_cam = None, None, None, None

            #self.tf_folder = os.path.join(os.path.dirname(__file__)) + "/recorded_bags/6_laboratorio/tf_saved"
            self.tf_folder = "/media/fab/Data/Desktop/ICE_temp/borsaSLAM/17-04-23_bag_altair/1_corridoioAltair/tf_saved"
            if not os.path.exists(self.tf_folder):
                raise TypeError("Tf folder path is wrong!")

            filenames = sorted(glob.glob(self.tf_folder + "/*.json"), key=os.path.getmtime)
            self.all_tf_timestamps = np.array([float(fn.split("_")[-1].replace(".json", "")) for fn in filenames])
 
            rospy.Subscriber('/clock', Clock, self.clock_callback)
        else:
            print("Using real time data")
            self.tf_sub = tf.TransformListener()        

        # CHECK
        self.listener = tf.TransformListener()
        self.camera_frame = 'camera_color_optical_frame'
        self.fixed_frame = 'world'
        
        # Subscribe to the darknet topics
        self.obj_img = None
        while self.obj_img is None:
            # rospy.Subscriber('/darknet_ros/detection_image', Image, self.detection_image_callback)
            rospy.Subscriber('/camera/color/image_raw', Image, self.detection_image_callback)
            rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.detection_bboxes_callback)
            print("Waiting for the object detection image...")

        # Subscribe to the rgb image topic
        self.depth_img_cv2 = None
        while self.depth_img_cv2 is None:            
            rospy.Subscriber('camera/aligned_depth_to_color/image_raw', Image, self.depth_img_callback)
            # rospy.Subscriber('/camera/depth_registered/image_raw', Image, self.depth_img_callback)
            # rospy.Subscriber('/camera/depth/image', Image, self.depth_img_callback)
            print("Waiting for the depth image...")
        
                
        print("\n====================================================================")
        print("Starting object detection node")
        print("====================================================================\n")
        
        timer_period = 0.1 # [seconds]
        self.timer = rospy.Timer(rospy.Duration(timer_period), self.control_loop) 
        
    # END init()


    def getCameraPose(self):
        try:
            # Listen for the transform from camera_frame to fixed_frame
            (trans, rot) = self.listener.lookupTransform(self.fixed_frame, self.camera_frame, rospy.Time(0))
            self.T_cam_map =  np.eye(4)
            # rospy.loginfo("Translation: %s" % str(trans))
            # rospy.loginfo("Rotation: %s" % str(rot))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Transform lookup: no transform from camera_frame to fixed_frame available!")
   

    # Clock callback (if using a rosbag)
    def clock_callback(self, msg):  
        self.act_timestamp = str(msg.clock.secs) + "." + str(msg.clock.nsecs)   # str format
        self.act_time = float(str(msg.clock.secs) + "." + str(msg.clock.nsecs)) # float format


    
    def publish_map2d(self):

        if self.map2d_1_img is not None:  # if initialized
            # Assign color for the objects
            res_target_cm = 0.5 # Warning: this is copied from "walls_detection_node.py", they must match!
            
            if len(self.detected_objects) > 0:  # if at least 1 object detected
                for obj in self.detected_objects:
                    class_name, class_id, _, _, obj_corners, center, distance, inframe = obj
                    print("[map2d]", class_name, "frame?", inframe)

                    c_sort = obj_corners[obj_corners[:, 2].argsort()] # sort by z

                    obj_corners_top2d = c_sort[:4]  # take the first 4 that have same z-coord
                    obj_corners_top2d = obj_corners_top2d[obj_corners_top2d[:, 0].argsort()] # sort by x 

                 
                    obj_corners_top2d_pixels = np.array([ [ int((el[1]*100 - self.origin_cm[1])/res_target_cm),
                                                            int((el[0]*100 - self.origin_cm[0])/res_target_cm) ]
                                                            for el in obj_corners_top2d ])                                   
            
                    # Define the color
                    color = set_of_random_colors_255_bgr[class_id]

                    '''
                    # Draw a circle at top left corner
                    if obj_corners_top2d[0,1] > obj_corners_top2d[1,1]:
                        top_right = obj_corners_top2d[0,:]
                    else:
                        top_right = obj_corners_top2d[1,:]

                    row = int((top_right[0]*100  - self.origin_cm[0])/res_target_cm)
                    col = int((top_right[1]*100  - self.origin_cm[1])/res_target_cm)
                    start = (col, row)
                    cv2.circle(self.map2d_1_img, start, 8, [0,0,0], -1)

                    # Draw a circle at bottom right corner
                    if obj_corners_top2d[2,0] > obj_corners_top2d[3,0]:
                        bottom_left = obj_corners_top2d[2,:]
                    else:
                        bottom_left = obj_corners_top2d[3,:]

                    row = int((bottom_left[0]*100  - self.origin_cm[0])/res_target_cm)
                    col = int((bottom_left[1]*100  - self.origin_cm[1])/res_target_cm)
                    end = (col, row)
                    cv2.circle(self.map2d_1_img, end, 8, [255,255,255], -1)

                    # Draw a rectangle representing the 2D bounding box from the top
                    #print("rect start/end", start, end)
                    #cv2.rectangle(self.map2d_1_img, start, end, color, 4)

                    #(x, y, w, h) = cv2.boundingRect(obj_corners_top2d_pixels)
                    #cv2.rectangle(self.map2d_1_img, (x,y), (x+w,y+h), (255, 0, 0), 2)
                    '''

                    ## Draw rotated rectangle
                    # Source: https://docs.opencv.org/3.4/dd/d49/tutorial_py_contour_features.html
                    # Source: https://stackoverflow.com/questions/36921249/drawing-angled-rectangles-in-opencv
                    rect = cv2.minAreaRect(obj_corners_top2d_pixels)
                    center = (round(rect[0][0]), round(rect[0][1]))
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    cv2.drawContours(self.map2d_1_img, [box], 0, color, 2)
                  
                    # Write the class name                    
                    cv2.putText(self.map2d_1_img, class_name, center, cv2.FONT_HERSHEY_SIMPLEX, 2, color, 3, cv2.LINE_AA)
                                    
                    # Draw a circle at center, green if in camera frame, else red
                    if inframe:
                        color = [0,255,0]
                    else:
                        color = [0,0,255]
                    cv2.circle(self.map2d_1_img, center, 10, color, -1)
                        
                    
                self.map2d_img_pub.publish(bridge.cv2_to_imgmsg(self.map2d_1_img))
                print("Published map2d")
          

    # Function to retrieve pointcloud from depth image and boundaries to crop it
    def get_pcd_from_depth_img(self, bbx_position, show=False):
        # Crop the depth image
        depth = np.asarray(self.depth_img_cv2)
        _,_,_, xmin, xmax, ymin, ymax, _, _ = bbx_position
   
        cropped_depth_img = np.zeros_like(depth)     
        cropped_depth_img[ymin:ymax, xmin:xmax] = depth[ymin:ymax, xmin:xmax]
        cropped_depth_img = o3d.geometry.Image(cropped_depth_img.astype(np.float32))         
        
        if show:
            cv2.imshow("NOT Cropped", self.depth_img_cv2)
            #cv2.imshow("Cropped", cropped_depth)
            cv2.waitKey(1)   
   
        # create the pcd from depth
        obj_pcd = o3d.geometry.PointCloud.create_from_depth_image(cropped_depth_img, intrinsic, depth_trunc=1.0)
        obj_pcd.paint_uniform_color([0,1,0])  #pcd in camera frame

        # Filter by depth
        '''
        z_threshold = 30
        points = np.asarray(obj_pcd.points)

        #points[points[:, 2] > z_threshold] = z_threshold
        #obj_pcd.points = open3d.utility.Vector3dVector(points)

        pcd_sel = obj_pcd.select_by_index(np.where(points[:, 2] < z_threshold)[0])
        pcd_sel.paint_uniform_color([1,0,0])
        #o3d.visualization.draw_geometries([obj_pcd,pcd_sel])
        return pcd_sel
        '''        

        return obj_pcd






    # control loop, executed each 0.2 seconds
    def control_loop(self, time):
        print("contr")
        t1 = rospy.get_time()
            
        # # Retrieve map to camera and camera to map transformations
        # if self.use_bag:
        #     self.T_cam_map, self.trans_cam_map, self.rot_quat_cam_map, self.rot_eul_cam_map = get_T_cam_to_map_bag(self.tf_folder, self.all_tf_timestamps, self.act_timestamp)
        #     self.T_map_cam, self.trans_map_cam, self.rot_quat_map_cam, self.rot_eul_map_cam = get_T_map_to_cam_bag(self.tf_folder, self.all_tf_timestamps, self.act_timestamp)
        # else:      
        #     self.T_cam_map, self.trans_cam_map, self.rot_quat_cam_map, self.rot_eul_cam_map = get_T_cam_to_map_tf(camera_frame, rviz_frame, self.tf_sub)
        #     self.T_map_cam, self.trans_map_cam, self.rot_quat_map_cam, self.rot_eul_map_cam = get_T_map_to_cam_tf(camera_frame, rviz_frame, self.tf_sub)

        # Check that transforms are meaningful
        if np.array_equal(np.array(self.T_cam_map), np.eye(4)):
            print("[Warning] Transformations are not initialized!")

        else: # start the actual control loop

            num_detected_objs = len(self.detected_objects) # number of total detected objects
            
            '''
            # Check if camera is moving
            if self.prev_trans_cam_map is None or self.prev_rot_eul_cam_map is None:
                print("[Warning] Camera pose is not initialized!")
                self.prev_trans_cam_map = self.trans_cam_map
                self.prev_rot_eul_cam_map = self.rot_eul_cam_map
            '''
            
            if num_detected_objs > 0:
                #self.is_moving = self.is_camera_moving()
                #print("is camera moving?", self.is_moving)

                #if self.is_moving:
                #    print("[Warning] Camera is moving! STOP detection, UPDATE distances!")
                #else:
                for k,obj in enumerate(self.detected_objects):
                    _,_, _, _, _, prev_center, prev_distance, prev_inframe = obj
                    x,y,z = prev_center
                    prev_center_omo = [x,y,z,1]
                    # Very important step: center must be transformed to actual CAMERA REFERENCE FRAME!!!!
                    new_center_cam = self.T_map_cam.dot( np.array(prev_center_omo) ) 
                    new_center_cam = new_center_cam[0:3] # remove last coordinate
                    new_distance = np.linalg.norm(new_center_cam) 
                    # update the distance if neeed (rotations don't make any difference)
                    if abs(new_distance*100 - prev_distance*100) > 2: # 2cm threshold to update the distance 
                        self.detected_objects[k][-2] = new_distance

                    cam1 = self.T_cam_map.dot( np.array([0,0,0,1]).T )[:3]
                    cam2 = self.T_cam_map.dot( np.array([0,0,0.4,1]).T )[:3]
                    cam_vector = cam2 - cam1
                    obj_vector = np.array(prev_center) - cam1

                    len_a = np.linalg.norm(cam_vector)
                    len_b = np.linalg.norm(obj_vector)

                    alpha = np.arccos(cam_vector.dot(obj_vector)/(len_a*len_b)) * 180/np.pi

                    #print("alpha: ", alpha)
                    if abs(alpha) < 45:
                        self.detected_objects[k][-1] = True
                    else:
                        self.detected_objects[k][-1] = False

                    # A change was made, so update the map
                    #if self.detected_objects[k][-1] != prev_inframe:
                    #    update_map2d = True

                #self.prev_trans_cam_map = self.trans_cam_map
                #self.prev_rot_eul_cam_map = self.rot_eul_cam_map

            # Classify (as new or old) currently detected objects
            if len(self.curr_frame_obj_bboxes) > 0: 

                for bb in self.curr_frame_obj_bboxes:                
                    # Get object's pointcloud 
                    obj_pcd = self.get_pcd_from_depth_img(bb)
                    if len(obj_pcd.points) > 0:

                        # My computation of the distance (MUST BE DONE IN CAMERA FRAME!):
                        obj_center = obj_pcd.get_center() # get object's pointcloud's center         
                        distance = np.linalg.norm(obj_center)  # square the norm of the center (should be expressed wrt camera center's frame)
                    
                        # Publish the object's pointcloud in MAP frame to debug                
                        obj_pcd.transform(self.T_cam_map)  # accounting for camera pose!
                        if self.publish_rviz:
                            self.obj_pcd_pub.publish(convertCloudFromOpen3dToRos(obj_pcd, "map"))  # useful topics for debugging
                                
                        # Get object's corners in MAP frame (after transformation)!
                        obj_aligned_bbox = obj_pcd.get_axis_aligned_bounding_box() 

                        # Compute 3d object oriented bbox for rviz/map2d                        
                        obj_oriented_bbox = np.asarray( obj_pcd.get_oriented_bounding_box().get_box_points() )
                
                        # Compute new 3d bbox for IoU
                        box_size = (obj_aligned_bbox.get_extent())
                        heading_angle = 0
                        center = (obj_aligned_bbox.get_center())
                        new_bbox_iou  = get_3d_box(box_size, heading_angle, center) 
                        
                        is_new = False # init

                        if num_detected_objs == 0: # no previously detected objects
                            is_new = True
                        else: # at least 1 detected object
                            detected_obj_arr = np.array(self.detected_objects, dtype=object)
                            if bb[1] not in detected_obj_arr[:,1]: # check if new class id
                                is_new = True  
                            else: # there's at least one object with the same class id

                                for obj_2_update_id, obj_same_id in enumerate(detected_obj_arr):                                    
                                    old_bbox_iou = obj_same_id[3]

                                    (IOU_3d,IOU_2d) = box3d_iou(new_bbox_iou, old_bbox_iou)
                                    print("iou", (IOU_3d,IOU_2d))

                                    #print("IOU 3D/2D", IOU_3d,IOU_2d) #3d IoU/ 2d IoU of BEV(bird eye's view)
                                    if IOU_3d > 0.3 or IOU_2d > 0.35: # if there's a sufficient overlapping, not new object
                                        is_new = False

                        if is_new:  # append the new object   
                            inframe = True                   
                            
                            self.detected_objects.append([bb[0], bb[1], bb[2], new_bbox_iou, obj_oriented_bbox, center, distance, inframe])
                            print("- Found object:", bb[0], " at distance [cm]" , int(distance*100))                          
                            print("Number of detected objects:", len(self.detected_objects))
                            #update_map2d = True # a new object was found, so update the map
                                
                        else:  # same object but override the distance, that depends on the current camera pose!
                            self.detected_objects[obj_2_update_id][-1] = True # means it's in frame
                            prev_distance = self.detected_objects[obj_2_update_id][-2]
                            if abs(prev_distance*100-distance*100) > 2: #2 cm threshold
                                self.detected_objects[obj_2_update_id][-2] = distance
                                print("[Update]", bb[0], "distance updated to", str(int(distance*100)), "cm")
                        #--------------------------------------------
                                
                    # Create pandas dataframe from list "self.detected_objects" and convert to "json_object"
                    json_data = []
                    for obj in self.detected_objects:   
                        class_name, class_id, confidence, new_bbox_iou, obj_oriented_bbox, center, distance, inframe = obj
                        new_bbox_iou_str = np.array2string(np.array(new_bbox_iou), formatter={'float_kind':lambda x: "%.8f" % x}).replace(' ',',').replace('\n',',').replace(',,',',')
                        obj_oriented_bbox_str = np.array2string(np.array(obj_oriented_bbox), formatter={'float_kind':lambda x: "%.8f" % x}).replace(' ',',').replace('\n',',').replace(',,',',')
                        center_str = np.array2string(np.array(center), formatter={'float_kind':lambda x: "%.8f" % x}).replace(' ',',').replace('\n',',').replace(',,',',')

                        json_data.append([class_name, class_id, confidence, new_bbox_iou_str, obj_oriented_bbox_str, center_str, distance, inframe])
                    
                    if len(json_data) > 0:
                        df_json = pd.DataFrame(json_data, columns=["class_name", "class_id", "confidence", "corners_iou", "corners_oriented_bbox", "center_3d", "distance", "inframe"])
                        #self.json_objects_pub.publish(df_json.to_csv(sep=";", index=False)) #old method
                        self.json_objects_pub.publish(str(df_json.to_json(orient='index')))

                        print("df", df_json)

            

            self.publish_3d_objects() # fabio 12-04-23
            self.publish_map2d()
         
        t2 = rospy.get_time()
        print("loop time", t2-t1)
        
    # END control_loop()



    def publish_3d_objects(self):
        points_msg, name_msg = MarkerArray(), MarkerArray()

        for k,obj in enumerate(self.detected_objects):
            class_name, class_id, confidence, new_bbox_iou, obj_oriented_bbox, center, distance, inframe = obj

            m_color = set_of_random_colors[class_id]            
            marker = create_bbox_points(k, m_color, obj_oriented_bbox, center, rviz_frame)
            #marker = create_planar_back_bbox(k, m_color, corners_array, rviz_frame)
            points_msg.markers.append(marker)

            text = create_name_marker(class_name, confidence, distance, k, m_color, center, rviz_frame)
            name_msg.markers.append(text)

        self.object_names_3d_pub.publish(name_msg)
        self.objects_bbox_3d_pub.publish(points_msg)



    # Function to check if camera is moving
    def is_camera_moving(self):
        trans_th = 0.1
        angle_th = 15 # deg
                
        theta1, phi1, psi1  = self.rot_eul_cam_map
        theta1, phi1, psi1  = theta1*180/np.pi, phi1*180/np.pi, psi1*180/np.pi
        theta2, phi2, psi2  = self.prev_rot_eul_cam_map
        theta2, phi2, psi2  = theta2*180/np.pi, phi2*180/np.pi, psi2*180/np.pi

        act_trans = np.array(self.trans_cam_map)
        prev_trans = np.array(self.prev_trans_cam_map)

        if np.linalg.norm(act_trans - prev_trans)>trans_th or \
            abs(theta1-theta2)>angle_th or abs(phi1-phi2)>angle_th or abs(psi1-psi2)>angle_th:
            return True
        else:
            return False


    # ----------
    # Callbacks
    # ----------
    def bag_bool_callback(self, msg):
        self.use_bag = msg.data

    def proj_map_callback(self, msg):         
        self.origin_cm = [msg.info.origin.position.x*100, msg.info.origin.position.y*100]

    def detection_image_callback(self, img):                
        self.obj_img = bridge.imgmsg_to_cv2(img)  #convert ROS to OpenCV

    def detection_bboxes_callback(self, msg):
        msg_bboxes = msg.bounding_boxes

        self.curr_frame_obj_bboxes = []
        center = None   # needs to be computed afterwards!
        distance = None # needs to be computed afterwards!
        for msg_bbox in msg_bboxes:
            if msg_bbox.probability > self.detection_th:  # threshold of detection
                if msg_bbox.Class in ["chair", "laptop", "diningtable", "backpack"]:
                    self.curr_frame_obj_bboxes.append([msg_bbox.Class, msg_bbox.id, msg_bbox.probability, msg_bbox.xmin, msg_bbox.xmax, msg_bbox.ymin, msg_bbox.ymax, center, distance])
      
    def depth_img_callback(self, depth_img):
        cv_image = bridge.imgmsg_to_cv2(depth_img, desired_encoding='passthrough')
        
        #ATTENTION: normalization badly affects distance computation!
        #cv_image_norm = cv2.normalize(cv_image, cv_image, 0, 1, cv2.NORM_MINMAX)  
        self.depth_img_cv2 = cv_image  # skipping normalization!
     
        # create the pcd from depth
        if self.T_cam_map is not None:
            depth = np.asarray(cv_image)
            img = o3d.geometry.Image(depth.astype(np.float32))      
            depth_pcd = o3d.geometry.PointCloud.create_from_depth_image(img, intrinsic)
            depth_pcd.paint_uniform_color([0,1,0])  #pcd in camera frame
            depth_pcd.transform(self.T_cam_map) #transform from camera to map frame!
            self.depth_pub.publish(convertCloudFromOpen3dToRos(depth_pcd, rviz_frame))  # useful topic for debugging
      

    def map2d_img_callback(self, msg): 
        self.map2d_1_img = bridge.imgmsg_to_cv2(msg)
    

# START main()
if __name__ == '__main__':
   
    obj_det_dist_node = ObjectDetectionDistance()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("ROSInterruptException!")

    rospy.shutdown()
    





