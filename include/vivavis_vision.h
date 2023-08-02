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

#include <geometry_msgs/PoseArray.h>

#include <altair_msgs/PointArray.h>
#include <altair_msgs/CloudArray.h>
#include <altair_msgs/Ellipsoid.h>
#include <altair_msgs/EllipsoidArray.h>
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

    ros::Publisher cloud_pub, cloud_array_pub, ellipsoid_pub, ellipsoid_cloud_pub, pose_pub;

    image_transport::ImageTransport it_;
    image_transport::Publisher rendered_image_publisher_;

    ros::Subscriber camera_info_sub, cloud_sub;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyz_cld_ptr;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr prev_xyz_cld_ptr;

    void makeEllipsoid(pcl::PointCloud<pcl::PointXYZRGB> &cloud, const Eigen::Vector3f radii, const Eigen::Vector4f &c);

    template <typename PointT>
    boost::shared_ptr<pcl::PointCloud<PointT>>
    voxel_grid_subsample(const boost::shared_ptr<pcl::PointCloud<PointT>> &cld_in, float cell_size);
};