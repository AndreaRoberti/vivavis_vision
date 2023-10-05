#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/core/core.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

class CombineDepthRgb
{
public:
    CombineDepthRgb(ros::NodeHandle& nh);
    ~CombineDepthRgb();
    void grabRGBD(const sensor_msgs::ImageConstPtr& msg_rgb, const sensor_msgs::ImageConstPtr& msg_d);
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
    void PublishRenderedImage(cv::Mat image, std::string encoding);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudFrom2D(cv::Mat image_depth, cv::Mat image_rgb, cv::Mat K);

private:
    // Ros handler
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    std::string name_of_node_, camera_info_topic;
    std::string camera_image_frame, reference_frame;

    ros::Publisher cloud_pub;

    image_transport::ImageTransport it_;
    image_transport::Publisher      rendered_image_publisher_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_semantic;

    cv::Mat K, D;

    int sync_time;

    cv::Mat imRGBOut, imDOut, maskOut;
};