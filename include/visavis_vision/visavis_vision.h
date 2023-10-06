/**
 * @file visavis_vision.h
 * @brief Header file for the VisavisVision class.
 */

#pragma once

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
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <visavis_vision/WallInfo.h>
#include <visavis_vision/WallInfoArray.h>
#include <visavis_vision/ObstacleInfo.h>
#include <visavis_vision/ObstacleInfoArray.h>
#include <cmath>

#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0) // Converts degrees to radians
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI) // Converts radians to degrees

/**
 * @namespace visavis
 * @brief The namespace for the VisavisVision class.
 */
namespace visavis
{
    /**
     * @class VisavisVision
     * @brief A class for handling vision-related tasks in the Visavis project.
     */
    class VisavisVision
    {
    public:
        /**
         * @brief Constructor for the VisavisVision class.
         * @param nh A reference to the ROS NodeHandle.
         */
        VisavisVision(ros::NodeHandle &nh);

        /**
         * @brief Destructor for the VisavisVision class.
         */
        ~VisavisVision(){};

        /**
         * @brief Callback function for processing point cloud data.
         * @param input A const pointer to the input point cloud data.
         */
        void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &input);

        /**
         * @brief Update function for the VisavisVision class.
         */
        void update();

        /**
         * @brief Cluster objects in a point cloud.
         * @param cloud_cluster A pointer to the input point cloud containing clusters.
         * @param num_obj An integer reference to store the number of detected objects.
         * @return A vector of pointers to point clouds representing individual objects.
         */
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusterObject(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_cluster, int &num_obj);

    private:
        // Ros handler
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        tf::TransformBroadcaster *broadcaster_;
        tf::TransformListener listener_;
        tf::StampedTransform optical2map_;
        std::string optical_frame_, fixed_frame_;
        double far_clip_distance_, near_clip_distance_;
        float orig_cld_voxel_size_;
        double object_cluster_distance_;
        int max_object_cluster_size_, min_object_cluster_size_;
        int num_obj_;
        ros::Publisher cloud_pub_, cloud_obs_pub_, cloud_nearest_pub_,
            visual_walls_pub_, visual_obstacles_pub_;
        ros::Publisher walls_info_pub_;
        ros::Publisher obstacles_info_pub_;
        ros::Publisher human_ws_pub_;
        image_transport::ImageTransport it_;
        image_transport::Publisher rendered_image_publisher_;
        ros::Subscriber cloud_sub_;
        visualization_msgs::MarkerArray visualize_walls_, visualize_obstacles_;
        visavis_vision::WallInfoArray walls_info_;
        visavis_vision::ObstacleInfoArray obstacles_info_;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyz_cld_ptr_;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_planes_;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_obstacles_;

        /**
         * @brief Get the camera pose as a 4x4 transformation matrix.
         * @return A cv::Mat representing the camera pose.
         */
        cv::Mat getCameraPose() const;

        /**
         * @brief Set the transformation for a plane.
         * @param id The ID of the plane.
         * @param cloud A pointer to the point cloud representing the plane.
         * @param a The coefficient 'a' of the plane equation.
         * @param b The coefficient 'b' of the plane equation.
         * @param c The coefficient 'c' of the plane equation.
         * @param d The coefficient 'd' of the plane equation.
         * @param centroid The centroid of the plane.
         * @param min_p The minimum point of the plane.
         * @param max_p The maximum point of the plane.
         */
        void setPlaneTransform(int id, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, float a, float b, float c, float d,
                               Eigen::Vector4f centroid, Eigen::Vector4f min_p, Eigen::Vector4f max_p);

        /**
         * @brief Create visual representations of obstacles.
         * @param cloud A pointer to the point cloud containing obstacles.
         */
        void createVisualObstacles(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

        /**
         * @brief Add a visual object marker.
         * @param id The ID of the object.
         * @param centroid The centroid of the object.
         * @param min_p The minimum point of the object.
         * @param max_p The maximum point of the object.
         * @param color The color of the object.
         * @param orientation The orientation of the object.
         * @return A visualization_msgs::Marker representing the visual object.
         */
        visualization_msgs::Marker addVisualObject(int id, Eigen::Vector4f centroid, Eigen::Vector4f min_p, Eigen::Vector4f max_p, Eigen::Vector4f color,
                                                   Eigen::Quaternionf orientation = Eigen::Quaternionf(0.0, 0.0, 0.0, 1.0));

        /**
         * @brief Perform voxel grid subsampling on a point cloud.
         * @param cld_in A pointer to the input point cloud.
         * @param cell_size The size of the grid cells for subsampling.
         * @return A shared pointer to the subsampled point cloud.
         */
        template <typename PointT>
        boost::shared_ptr<pcl::PointCloud<PointT>> voxel_grid_subsample(const boost::shared_ptr<pcl::PointCloud<PointT>> &cld_in, float cell_size);

        /**
         * @brief Filter room-related data from a point cloud.
         * @param cloud A pointer to the input point cloud.
         */
        void filterRoom(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

        /**
         * @brief Publishes a Point Cloud to a ROS topic.
         *
         * This function publishes a Point Cloud to a ROS topic using the provided ROS publisher.
         *
         * @param cloud The Point Cloud to be published.
         * @param cloud_pub The ROS publisher used to publish the Point Cloud.
         */
        void publishPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                               const ros::Publisher &cloud_pub);
    };
}
