# VivavisVision Documentation

This documentation provides an overview of the parameters and publishers/subscribers used in the `VivavisVision` class.

## How To

- Clone this repo in your *catkin_ws*

```bash
git clone https://github.com/AndreaRoberti/vivavis_vision.git`
```

- build and source

```bash
catkin build
source devel/setup.bash
```

# vivavis_vision_node

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

- `ellipsoid_cloud_pub` (topic: `ellipsoid_cloud`): Publishes a `sensor_msgs::PointCloud2` message containing ellipsoid points.

### Object Publishers

- `ellipsoid_pub` (topic: `ellipsoid`): Publishes an `vivavis_vision::EllipsoidArray` message representing ellipsoids.

- `walls_info_pub` (topic: `walls_info`): Publishes a `vivavis_vision::WallInfoArray` message containing information about walls.

### Visualization Publishers

- `visual_walls_pub` (topic: `visual_walls`): Publishes a `visualization_msgs::MarkerArray` message for visualizing walls.

- `visual_obstacles_pub` (topic: `visual_obstacles`): Publishes a `visualization_msgs::MarkerArray` message for visualizing obstacles.

- `human_ws_pub` (topic: `human_ws`): Publishes a `visualization_msgs::Marker` message for visualizing human workspace.

### Obstacles Poses Publisher

- `pose_pub` (topic: `obstacles_pose`): Publishes a `geometry_msgs::PoseArray` message for obstacles poses.


## WallInfo Message

The `WallInfoArray` is an array of 6 elements [floor, left, right, front, back, ceiling] of `WallInfo`, each defined by the following structure:

- `Header header`
- `float64 a`
- `float64 b`
- `float64 c`
- `float64 d`
- `float64 num_points`
- `int64 color_id`
- `geometry_msgs/Pose pose`


# vivavis_pcloud_stich


The Point Cloud Stitcher Node is responsible for stitching point clouds together based on various parameters and publishing the stitched point cloud in a designated reference frame.

## Parameters

### `voxel_size_stitching` (float, default: 0.002)

This parameter specifies the voxel size to be used during the stitching process. The point clouds will be downsampled using voxel grid filtering with this voxel size before stitching.

### `voxel_size_input_cloud` (float, default: 0.002)

This parameter defines the voxel size to be applied to the input point clouds before stitching. Similar to `voxel_size_stitching`, the input clouds are downsampled using voxel grid filtering.

### `max_cam_depth` (float, default: 0.3)

The maximum depth of the camera. Points in the input point clouds with depth values beyond this threshold will be ignored during the stitching process.

### `output_reference_frame` (string, default: "")

The reference frame in which the stitched point cloud will be published. This parameter specifies the target reference frame for the output point cloud.

## Subscribers

### `in_pointcloud` (`sensor_msgs::PointCloud2`)

This subscriber listens for incoming point cloud messages of type `sensor_msgs::PointCloud2`. The received point clouds will be used for stitching.

## Publishers

### `stitched_pointcloud` (`sensor_msgs::PointCloud2`)

This publisher is responsible for publishing the stitched point cloud after the stitching process is complete. The output point cloud will be in the reference frame specified by the `output_reference_frame` parameter.

## Methods

### `pointCloudCb`

This method is the callback function for the `in_pointcloud` subscriber. It processes the incoming point clouds, performs stitching using the provided parameters, and publishes the stitched point cloud to the `stitched_pointcloud` topic.

## Description

The Point Cloud Stitcher Node subscribes to incoming point cloud messages (`in_pointcloud` topic) and performs stitching based on the specified parameters (`voxel_size_stitching`, `voxel_size_input_cloud`, `max_cam_depth`). The stitched point cloud is then published in the specified reference frame (`output_reference_frame`) using the `stitched_pointcloud` topic.

Please ensure that the necessary ROS topics and messages are correctly configured to enable the smooth operation of the Point Cloud Stitcher Node.

**Note:** It is important to provide accurate parameter values and ensure that the reference frames are set appropriately to achieve desired results.
