#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

class Stitcher
{
public:
    Stitcher(ros::NodeHandle* pnh, ros::NodeHandle* nh)
    {
        pnh->param("voxel_size_stiching", voxel_size_stiching, 0.002f);
        pnh->param("voxel_size_input_cloud", voxel_size_input_cloud, 0.002f);
        pnh->param("max_cam_depth", max_cam_depth, 0.3f);
        pnh->param("output_reference_frame", output_reference_frame, std::string(""));

        inCloud =
            boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(pcl::PointCloud<pcl::PointXYZRGB>());
        inFilteredCloud =
            boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(pcl::PointCloud<pcl::PointXYZRGB>());
        outCloud =
            boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(pcl::PointCloud<pcl::PointXYZRGB>());
        outTransformedCloud =
            boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(pcl::PointCloud<pcl::PointXYZRGB>());
        stichedFilteredCloud =
            boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(pcl::PointCloud<pcl::PointXYZRGB>());
        stichedCloud =
            boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(pcl::PointCloud<pcl::PointXYZRGB>());

        xyz_f_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(pcl::PointCloud<pcl::PointXYZRGB>());
        tree2 =
            boost::make_shared<pcl::search::KdTree<pcl::PointXYZRGB>>(pcl::search::KdTree<pcl::PointXYZRGB>());

        pcloud_sub = nh->subscribe<sensor_msgs::PointCloud2>("in_pointcloud", 1, &Stitcher::pointCloudCb, this);
        cloud_pub  = nh->advertise<sensor_msgs::PointCloud2>("stiched_pointcloud", 1);
    };

    void compute()
    {
        ros::spin();
    };

private:
    void
    voxelizeXYZRGB(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cld_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cld_out, float voxel_size)
    {
        pcl::VoxelGrid<pcl::PointXYZRGB> voxelizer;
        voxelizer.setInputCloud(cld_in);
        voxelizer.setDownsampleAllData(true);
        // leaf size in meters
        voxelizer.setLeafSize(voxel_size, voxel_size, voxel_size);
        voxelizer.filter(*cld_out);
    };

    void clusterObject(pcl::PointCloud<pcl::PointXYZRGB>::Ptr               cloud_cluster,
                       std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& obj_surf)
    {
        std::vector<pcl::PointIndices> clusters;
        *xyz_f_ = *cloud_cluster;
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr              xyz_f_(new pcl::PointCloud<pcl::PointXYZRGB>);
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud_cluster, *xyz_f_, indices);
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;

        ec.setInputCloud(xyz_f_);
        ec.setClusterTolerance(0.025);// COMMENT 0.005

        int min_object_cluster_size_ = 100;

        ec.setMinClusterSize(min_object_cluster_size_);
        ec.setMaxClusterSize(std::numeric_limits<int>::max());
        ec.setSearchMethod(tree2);
        ec.extract(clusters);

        pcl::ExtractIndices<pcl::PointXYZRGB> extract_;
        for (unsigned int i = 0; i < clusters.size(); ++i)
        {
            if (clusters[i].indices.size() >= static_cast<unsigned int>(min_object_cluster_size_))
            {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusters_i(new pcl::PointCloud<pcl::PointXYZRGB>);
                extract_.setInputCloud(xyz_f_);
                extract_.setIndices(boost::make_shared<const pcl::PointIndices>(clusters[i]));
                extract_.setNegative(false);
                extract_.filter(*clusters_i);
                obj_surf.push_back(clusters_i);
            }
        }
    };

    void pointCloudCb(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        tf::StampedTransform cloud2World;

        try
        {
            listener.waitForTransform(output_reference_frame, msg->header.frame_id, ros::Time(0), ros::Duration(1.0));
            listener.lookupTransform(output_reference_frame, msg->header.frame_id, ros::Time(0), cloud2World);
        }
        catch (const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }

        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg, pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2, *inCloud);
        // ROS_WARN("TRANSFORM");
        // pcl::fromROSMsg(*msg, *inCloud);

        if (!inCloud->points.size())
        {
            ROS_WARN("NO INPUT CLOUD!!");
            return;
        }
        
        // Create the filtering object
        pcl::PassThrough<pcl::PointXYZRGB> pass;
        pass.setInputCloud(inCloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0, max_cam_depth);
        pass.filter(*inFilteredCloud);


        voxelizeXYZRGB(inFilteredCloud, outCloud, voxel_size_input_cloud);

        pcl_ros::transformPointCloud(*outCloud, *outTransformedCloud, cloud2World);

        *stichedCloud += *outTransformedCloud;


         if (!stichedCloud->points.size())
        {
            ROS_WARN("NO STITCHED CLOUD # 1!");
            return;
        }

        voxelizeXYZRGB(stichedCloud, stichedFilteredCloud, voxel_size_stiching);
        if (!stichedFilteredCloud->points.size())
        {
            ROS_WARN("NO STITCHED CLOUD # 2!");
            return;
        }
        *stichedCloud = *stichedFilteredCloud;



        if (!stichedCloud->points.size())
        {
            ROS_WARN("NO STITCHED CLOUD # 3");
            return;
        }
        // std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusterClouds = clusterObject(stichedCloud, num);
        clusterClouds.clear();
        clusterObject(stichedCloud, clusterClouds);

        size_t max_size = 0;
        for (auto& cloud : clusterClouds)
        {
            if (cloud->points.size() > max_size)
            {
                max_size     = cloud->points.size();
                stichedCloud = cloud;
            }
        }
        if (!stichedCloud->points.size())
        {
            ROS_WARN("NO STITCHED CLOUD # 4");
            return;
        }

        
        sensor_msgs::PointCloud2 stiched_res;
        pcl::toROSMsg(*stichedCloud, stiched_res);

        stiched_res.header.frame_id = output_reference_frame;
        stiched_res.header.stamp    = ros::Time::now();
        
        cloud_pub.publish(stiched_res);
    };

private:
    ros::Publisher                             cloud_pub;
    ros::Subscriber                            pcloud_sub;
    tf::TransformListener                      listener;
    float                                      voxel_size_stiching    = 0.0;
    float                                      voxel_size_input_cloud = 0.0;
    float                                      max_cam_depth          = 0.0;
    std::string                                output_reference_frame = "";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr     inCloud                = nullptr;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr     inFilteredCloud        = nullptr;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr     outCloud               = nullptr;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr     outTransformedCloud    = nullptr;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr     stichedFilteredCloud   = nullptr;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr     stichedCloud           = nullptr;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr     xyz_f_                 = nullptr;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree2                  = nullptr;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusterClouds;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcloud_stich");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    Stitcher s(&pnh, &nh);
    s.compute();

    return 0;
}