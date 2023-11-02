# VisavisVision Documentation
 
 This is a ROS package developed for helping blind people to navigate in unknown indoor environments.
 
 It depends on a PointCloud data, obtained through a VSLAM algorithm or directly through a RGB-D camera.
 
 - VSLAM : use the main node *visavis_vision_node*

 - Visual Odometry and Stitching : use *visavis_vision_node* and *visavis_pcloud_stich* nodes.

 *example launch files in the launch folder*


## Requirements

- Ubuntu 20.04

- ROS Noetic , follow [this](http://wiki.ros.org/noetic/Installation/Ubuntu) guide 

## How To Build

- Clone this repo in your *catkin_ws*

```bash
git clone https://github.com/AndreaRoberti/visavis_vision.git`
```

- build and source

```bash
catkin build
source devel/setup.bash
```

## How to Run - Real Environment

- It requires a realsense camera
```bash
    sudo apt-get install ros-noetic-realsense2-*
```

***test in progres***

## How to Run - Simulation

Download CoppeliaSIM [here](https://www.coppeliarobotics.com/downloads)  *(from version 4.4 above)*


- In one terminal run 

```bash
roscore
```
- After unzipped CoppeliaSIM archive, go to the directory and run it by typing 

```bash
./coppeliaSim.sh 
```
- Open one of the simulated scene you can find in *sim_scenes* folder

- Then in your caktin workspace

```bash
roslaunch visavis_node visavis_sim.launch
```

- If you want to save JSONs file

```bash
python3 json_writer_node.py 
```

Once you are ready, you can press PLAY on the simulator.

## How to Connect with the Audio component
   
   - If rosbridge server is not already installed

```bash
    sudo apt-get install ros-noetic-rosbridge-server 
```

```bash
    roslaunch rosbridge_server rosbridge_websocket.launch 
```

## Custom ROS Messages

### WallInfo

The `WallInfoArray` is an array of 6 elements [floor, left, right, front, back, ceiling] of `WallInfo`, each defined by the following structure:

- `Header header`
- `float64 a`
- `float64 b`
- `float64 c`
- `float64 d`
- `float64 num_points`
- `int64 color_id`
- `geometry_msgs/Pose pose`
- `geometry_msgs/Point closest_point`
- `sensor_msgs/PointCloud2 cloud`

### ObstacleInfo

The `ObstacleInfoArray` is an array of `ObstacleInfo`, defined by the following structure:

- `Header header`
- `geometry_msgs/Pose pose`
- `geometry_msgs/Point closest_point`
- `sensor_msgs/PointCloud2 cloud`


## Nodes Description

<details>
<summary><strong>Visavis Vision Node Documentation [C++]</strong></summary>

This is the main node that elaborates the incoming point cloud and seperates the walls from the objects/obstacles.

## Parameters

### Visualization Frames

- `optical_frame` (string, default: ""): The optical frame for visualization.

- `fixed_frame` (string, default: ""): The fixed frame for visualization.

### Clipping Distances

- `near_clip` (float, default: 0.0): The near clipping distance for visualization.

- `far_clip` (float, default: 0.0): The far clipping distance for visualization.

### Point Cloud Voxel Size

- `orig_cld_voxel_size` (float, default: 0.015): The voxel size used for the original point cloud.

### Object Clustering

- `object_cluster_distance` (double, default: 0.5): The distance threshold for object clustering.

- `max_object_cluster_size` (int, default: 500000): The maximum size of an object cluster.

- `min_object_cluster_size` (int, default: 1): The minimum size of an object cluster.

## Subscribers

- `cloud_sub` (topic: `in_cloud`): Subscribes to a point cloud on the `in_cloud` topic. Invokes the `cloudCallback` method.

## Publishers

### Point Cloud Publishers

- `cloud_pub` (topic: `walls_cloud`): Publishes a `sensor_msgs::PointCloud2` message representing wall points.

- `cloud_obs_pub` (topic: `obstacles_cloud`): Publishes a `sensor_msgs::PointCloud2` message representing obstacle points.

- `cloud_nearest_pub_` (topic: `nearest_cloud`): Publishes a `sensor_msgs::PointCloud2` message containing all the closest points respect to the camera.

### Object Publishers

- `obstacles_info_pub_` (topic: `walls_info`): Publishes a `visavis_vision::ObstacleInfoArray` message containing information about obstacles.

- `walls_info_pub_` (topic: `walls_info`): Publishes a `visavis_vision::WallInfoArray` message containing information about walls.

### Visualization Publishers

- `visual_walls_pub` (topic: `visual_walls`): Publishes a `visualization_msgs::MarkerArray` message for visualizing walls.

- `visual_obstacles_pub` (topic: `visual_obstacles`): Publishes a `visualization_msgs::MarkerArray` message for visualizing obstacles.

- `human_ws_pub` (topic: `human_ws`): Publishes a `visualization_msgs::Marker` message for visualizing human workspace.

</details>


<details>
<summary><strong>Point Cloud Stitcher Node Documentation [C++]</strong></summary>

The Point Cloud Stitcher Node is responsible for stitching point clouds together based on various parameters and publishing the stitched point cloud in a designated reference frame.

## Parameters

- `voxel_size_stitching` (float, default: 0.002)

This parameter specifies the voxel size to be used during the stitching process. The point clouds will be downsampled using voxel grid filtering with this voxel size before stitching.

- `voxel_size_input_cloud` (float, default: 0.002)

This parameter defines the voxel size to be applied to the input point clouds before stitching. Similar to `voxel_size_stitching`, the input clouds are downsampled using voxel grid filtering.

- `max_cam_depth` (float, default: 0.3)

The maximum depth of the camera. Points in the input point clouds with depth values beyond this threshold will be ignored during the stitching process.

- `output_reference_frame` (string, default: "")

The reference frame in which the stitched point cloud will be published. This parameter specifies the target reference frame for the output point cloud.

## Subscribers

- `in_pointcloud` (`sensor_msgs::PointCloud2`)

This subscriber listens for incoming point cloud messages of type `sensor_msgs::PointCloud2`. The received point clouds will be used for stitching.

## Publishers

- `stitched_pointcloud` (`sensor_msgs::PointCloud2`)

This publisher is responsible for publishing the stitched point cloud after the stitching process is complete. The output point cloud will be in the reference frame specified by the `output_reference_frame` parameter.

## Methods

- `pointCloudCb`

This method is the callback function for the `in_pointcloud` subscriber. It processes the incoming point clouds, performs stitching using the provided parameters, and publishes the stitched point cloud to the `stitched_pointcloud` topic.

## Description

The Point Cloud Stitcher Node subscribes to incoming point cloud messages (`in_pointcloud` topic) and performs stitching based on the specified parameters (`voxel_size_stitching`, `voxel_size_input_cloud`, `max_cam_depth`). The stitched point cloud is then published in the specified reference frame (`output_reference_frame`) using the `stitched_pointcloud` topic.

Please ensure that the necessary ROS topics and messages are correctly configured to enable the smooth operation of the Point Cloud Stitcher Node.

**Note:** It is important to provide accurate parameter values and ensure that the reference frames are set appropriately to achieve desired results.


</details>


<details>
<summary><strong>RGB-D combiner Node Documentation [C++]</strong></summary>
This node combines the RGB image and the aligned depth image to create the PointCloud

## Publishers

- `output_point_cloud`  (`sensor_msgs::PointCloud2`)

This publisher is responsible for publishing the combined point cloud.

## Subscribers

- `/rgb_d_combiner/image_topic` 
    
    This subscriber listens to image color camera topic.

- `/rgb_d_combiner/depth_topic` 

    This subscriber listens to aligned depth to color camera topic.

- `/rgb_d_combiner/info_topic` 

    This subscriber listens to image color camera info topic.


</details>

<details>
<summary><strong>Vision Data Processing Node Documentation [Python] </strong></summary>

The Vision Data Processing Node is responsible for processing vision-related data and publishing the processed data to specific topics. It utilizes ROS (Robot Operating System) topics for communication and data exchange.

## Publishers

- `out/json_walls_equations` (`rospy.Publisher`)

This publisher is responsible for sending JSON-formatted wall equations as `String` to the topic `out/json_walls_equations`. The wall equations represent the equations describing the detected walls in the environment.

- `out/json_human_workspace` (`rospy.Publisher`)

This publisher is responsible for sending JSON-formatted human workspace information as `String` to the topic `out/json_human_workspace`. The information includes data about the workspace of a human detected in the environment.

## Subscribers

- `visavis_vision/walls_info` (`rospy.Subscriber`)

This subscriber listens to the topic `visavis_vision/walls_info` for incoming messages of type `WallInfoArray`. The provided callback function `wall_info_callback` processes the received wall information.

- Expected Message Type: `WallInfoArray`

- `visavis_vision/obstacles_pose` (`rospy.Subscriber`)

This subscriber listens to the topic `visavis_vision/obstacles_pose` for incoming messages of type `PoseArray`. The provided callback function `obstacles_pose_array_callback` processes the received pose array, which contains information about the poses of obstacles in the environment.

- Expected Message Type: `PoseArray`

- `visavis_vision/human_ws` (`rospy.Subscriber`)

This subscriber listens to the topic `visavis_vision/human_ws` for incoming messages of type `Marker`. The provided callback function `human_ws_callback` processes the received marker data, which represents the workspace of a human detected in the environment.

- Expected Message Type: `Marker`

## Callback Functions

- `wall_info_callback(data)`

This callback function processes the data received from the `visavis_vision/walls_info` topic. It handles the detected wall information contained in the `WallInfoArray` message.

- `obstacles_pose_array_callback(data)`

This callback function processes the data received from the `visavis_vision/obstacles_pose` topic. It handles the detected obstacle poses contained in the `PoseArray` message.

- `human_ws_callback(data)`

This callback function processes the data received from the `visavis_vision/human_ws` topic. It handles the marker data representing the human workspace contained in the `Marker` message.

## Description

The Vision Data Processing Node subscribes to various vision-related topics to receive data about walls, obstacles, and human workspaces detected in the environment. It then processes this data using the provided callback functions and publishes the processed information in JSON format to designated topics. The processed information includes wall equations, human workspace details, and obstacle poses. This processed data can be used for further analysis, visualization, or decision-making in a robotic system or other applications.


</details>

