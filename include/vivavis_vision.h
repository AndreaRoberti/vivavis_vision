#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <Eigen/Dense>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/common.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>

#include <geometry_msgs/PoseArray.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// #include <vivavis_vision/PointArray.h>
// #include <vivavis_vision/CloudArray.h>
#include <vivavis_vision/Ellipsoid.h>
#include <vivavis_vision/EllipsoidArray.h>
#include <math.h>

#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0) // Converts degrees to radians
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI) // Converts radians to degrees

class VivavisVision
{
public:
    VivavisVision(ros::NodeHandle &nh);
    ~VivavisVision(){};

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &input);
    void update();
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusterObject(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster, int &num_obj);

private:
    // Ros handler
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    tf::TransformBroadcaster *br;
    tf::TransformListener listener;
    tf::StampedTransform optical2map;

    std::string camera_info_topic, cld_topic_name;
    std::string optical_frame, reference_frame, fixed_frame;
    double farClipDistance, nearClipDistance;

    float orig_cld_voxel_size;
    double object_cluster_distance_;
    int max_object_cluster_size_, min_object_cluster_size_;
    int num_obj;

    ros::Publisher cloud_pub, cloud_array_pub, ellipsoid_pub, ellipsoid_cloud_pub, pose_pub, visual_walls_pub;
    ros::Publisher left_wall_info, right_wall_info, floor_wall_info, ceiling_wall_info, front_wall_info, back_wall_info;

    image_transport::ImageTransport it_;
    image_transport::Publisher rendered_image_publisher_;

    ros::Subscriber camera_info_sub, cloud_sub;

    visualization_msgs::MarkerArray visualize_walls;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyz_cld_ptr;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr prev_xyz_cld_ptr;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_planes;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_obstacles;

    cv::Mat getCameraPose();
    void setPlaneTransform(float a, float b, float c, float d, Eigen::Vector4f centroid);
    void createObstacles(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);
    visualization_msgs::Marker addVisualWall(int id, float x_c, float y_c, float z_c);
    void makeEllipsoid(pcl::PointCloud<pcl::PointXYZRGB> &cloud, const Eigen::Vector3f radii, const Eigen::Vector4f &c);

    template <typename PointT>
    boost::shared_ptr<pcl::PointCloud<PointT>>
    voxel_grid_subsample(const boost::shared_ptr<pcl::PointCloud<PointT>> &cld_in, float cell_size);
    void filterRoom(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);
    void processRoom(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);
};