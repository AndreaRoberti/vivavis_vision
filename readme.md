# VivavisVision Documentation

This documentation provides an overview of the parameters and publishers/subscribers used in the `VivavisVision` class.

## How To

- Clone this repo in your *catkin_ws*

`git clone https://github.com/AndreaRoberti/vivavis_vision.git`

- `catkin_build`

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
