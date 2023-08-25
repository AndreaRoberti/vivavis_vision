#! /usr/bin/python

# Python libraries
import os
import shutil
import pandas as pd
from io import StringIO
import cv2

# ROS libraries
import rospy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
# from darknet_ros_msgs.msg import BoundingBoxes, ObjectCount
from rosgraph_msgs.msg import Clock

bridge = CvBridge()

class JsonWriter:
    def __init__(self):
        super().__init__()
        rospy.init_node('json_img_writer')

        self.walls_counter = 0
        self.objects_counter = 0
        self.human_counter = 0
        self.img_det_counter = 0
        self.img_rgb_counter = 0
        self.map2d_img_counter = 0

        self.prev_rgb_img_timestamp = 0
        self.prev_map2d_img_timestamp = 0
        self.prev_det_img_timestamp = 0

        self.prev_human_json_timestamp = 0
        self.prev_obj_json_timestamp = 0
        self.prev_walls_json_timestamp = 0
       
        
        # Set up your subscriber and define its callback
        rospy.Subscriber("out/json_walls_equations", String, self.save_walls_json_callback)
        rospy.Subscriber("out/json_objects", String, self.save_objects_json_callback)
        rospy.Subscriber("out/json_human_workspace", String, self.save_human_workspace_json_callback)

        rospy.Subscriber('/darknet_ros/detection_image', Image, self.detection_img_callback)
        # rospy.Subscriber('/darknet_ros/found_object', ObjectCount, self.found_object_callback)

        rospy.Subscriber('/camera/rgb/image_rect_color', Image, self.rgb_img_callback)

        #rospy.Subscriber('/out/map2d_img2', Image, self.map2d_img_callback) # with objects' bounding boxes (CNN results)
        rospy.Subscriber('/out/map2d_img1', Image, self.map2d_img_callback) # without objects, faster

        # Remove all previous jsons...
        for dir_to_remove in ["jsons/detected_walls_jsons", "jsons/detected_objects_jsons", "jsons/human_workspace_jsons", "jsons"]:
            if os.path.exists(os.path.join(os.path.dirname(__file__)) + dir_to_remove):
                    shutil.rmtree(os.path.join(os.path.dirname(__file__)) + dir_to_remove)

        for dir_to_create in ["jsons", "jsons/detected_walls_jsons", "jsons/detected_objects_jsons", "jsons/human_workspace_jsons"]:
            os.mkdir(os.path.join(os.path.dirname(__file__)) + dir_to_create)
       
        # Remove all previous images...   
        for dir_to_remove in ["images/map2d", "images/rgb", "images/detection","images"]:
            if os.path.exists(os.path.join(os.path.dirname(__file__)) + dir_to_remove):
                shutil.rmtree(os.path.join(os.path.dirname(__file__)) + dir_to_remove)
                
        for dir_to_create in ["images", "images/rgb", "images/detection", "images/map2d"]:
            os.mkdir(os.path.join(os.path.dirname(__file__)) + dir_to_create)


        # self.use_bag = None
        # while self.use_bag == None:
            # rospy.Subscriber('/use_bag', Bool, self.bag_bool_callback)
            # print("Waiting for the /use_bag topic!")
            # 
        # if self.use_bag:
            # rospy.Subscriber('/clock', Clock, self.clock_callback)
        # else:
        now = rospy.Time.now()
        self.act_time = str(now.to_sec()) # ROS (clock) actual time as a string
        
        timer_period = 0.01 # [seconds]
        self.timer = rospy.Timer(rospy.Duration(timer_period), self.control_loop) 


    # def bag_bool_callback(self, msg):
    #     self.use_bag = msg.data

    # clock callback
    # def clock_callback(self, msg):        
    #     self.act_time = str(msg.clock.secs) + "." + str(msg.clock.nsecs)
       
    # if not using rosbag
    def control_loop(self, time):  
        now = rospy.Time.now()
        self.act_time = str(now.to_sec()) # ROS (clock) actual time as a string


    def save_walls_json_callback(self,msg):    
        if msg.data != []:          
            if self.prev_walls_json_timestamp < float(self.act_time):
                # To insert the timestamp:
                # json_df = pd.read_csv(StringIO(msg.data), sep='\s+')
                json_df = pd.read_json(StringIO(msg.data), orient='index')  
                json_df.insert(0, "ros_timestamp", self.act_time, True)

                json_path = os.path.join(os.path.dirname(__file__)) + "jsons/detected_walls_jsons/"

                # Writing results to detected_objects.json
                with open(json_path + str(self.walls_counter) +"_" + self.act_time + ".json", "w") as outfile:
                    outfile.write(json_df.to_json(orient='index'))
                    print("Saved json walls!")

                self.walls_counter += 1  # incremental index
                self.prev_walls_json_timestamp = float(self.act_time)


    def save_objects_json_callback(self,msg):
        if msg.data != []:   
            if self.prev_obj_json_timestamp < float(self.act_time):

                # To insert the timestamp:   
                json_df = pd.read_json(StringIO(msg.data), orient='index')                
                json_df.insert(0, "ros_timestamp", self.act_time, True)
              
                json_path = os.path.join(os.path.dirname(__file__)) + "jsons/detected_objects_jsons/"
                                
                # Writing results to detected_objects.json
                with open(json_path + str(self.objects_counter)+"_"  + self.act_time + ".json", "w") as outfile:
                    outfile.write(json_df.to_json(orient='index'))
                    print("Saved json objects!")

                self.objects_counter += 1  # incremental index
                self.prev_obj_json_timestamp = float(self.act_time)


    def save_human_workspace_json_callback(self, msg):            
        if msg.data != []: 
            if self.prev_human_json_timestamp < float(self.act_time):
    
                # To insert the timestamp:   
                json_df = pd.read_json(StringIO(msg.data), orient='index')                
                json_df.insert(1, "ros_timestamp", self.act_time, True)
              
                json_path = os.path.join(os.path.dirname(__file__)) + "jsons/human_workspace_jsons/"
                        
                # Writing results to detected_objects.json
                with open(json_path + str(self.human_counter) +"_" + self.act_time + ".json", "w") as outfile:
                    outfile.write(json_df.to_json(orient='index'))

                print("Saved json human workspace!")
                self.human_counter += 1
                self.prev_human_json_timestamp = float(self.act_time)
                

    
    # def found_object_callback(self, msg):
    #     if msg.count > 0:
    #         self.found_object = True
    #     else:
    #         self.found_object = False


    def detection_img_callback(self, img):                
                
        # required because that topic has also images without detected objects
        if self.found_object: 
            if self.prev_det_img_timestamp < float(self.act_time):
                obj_img = bridge.imgmsg_to_cv2(img)  #convert ROS to OpenCV
                path = os.path.join(os.path.dirname(__file__)) + "/images/detection/" +str(self.img_det_counter) + "_" + self.act_time + ".png"
                cv2.imwrite(path, obj_img)
                self.img_det_counter += 1  # incremental index
                self.prev_det_img_timestamp = float(self.act_time)
                print("Saved detection image!")



    def map2d_img_callback(self, img):    
        
        if self.prev_map2d_img_timestamp < float(self.act_time):
            bgr_img = bridge.imgmsg_to_cv2(img)  #convert ROS to OpenCV
            path = os.path.join(os.path.dirname(__file__)) + "/images/map2d/" + str(self.map2d_img_counter) + "_" + self.act_time + ".png"
            cv2.imwrite(path, bgr_img)
            self.map2d_img_counter += 1  # incremental index
            self.prev_map2d_img_timestamp = float(self.act_time)
            print("Saved map2d!")         

    def rgb_img_callback(self, img):           
        
        if self.prev_rgb_img_timestamp < float(self.act_time):
            bgr_img = bridge.imgmsg_to_cv2(img)  #convert ROS to OpenCV
            path = os.path.join(os.path.dirname(__file__)) + "/images/rgb/" +str(self.img_rgb_counter) + "_" + self.act_time + ".png"
            cv2.imwrite(path, cv2.cvtColor(bgr_img, cv2.COLOR_RGB2BGR))    
            self.img_rgb_counter += 1  # incremental index
            self.prev_rgb_img_timestamp = float(self.act_time)
            

if __name__ == '__main__':
    
    node = JsonWriter()
    rospy.spin()
  
