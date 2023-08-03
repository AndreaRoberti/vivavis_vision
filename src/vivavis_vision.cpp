#include <vivavis_vision.h>

VivavisVision::VivavisVision(ros::NodeHandle &nh) : nh_(nh), private_nh_("~"),
                                                    it_(nh),
                                                    xyz_cld_ptr(new pcl::PointCloud<pcl::PointXYZRGB>),
                                                    prev_xyz_cld_ptr(new pcl::PointCloud<pcl::PointXYZRGB>),
                                                    num_obj(0)
{

    private_nh_.param("optical_frame", optical_frame, std::string(""));
    private_nh_.param("fixed_frame", fixed_frame, std::string(""));
    private_nh_.param("near_clip", nearClipDistance, 0.0);
    private_nh_.param("far_clip", farClipDistance, 0.0);

    private_nh_.param("orig_cld_voxel_size", orig_cld_voxel_size, 0.008f);
    private_nh_.param<double>("object_cluster_distance", object_cluster_distance_, 0.5);
    private_nh_.param<int>("max_object_cluster_size", max_object_cluster_size_, 500000);
    private_nh_.param<int>("min_object_cluster_size", min_object_cluster_size_, 1);

    cloud_pub = private_nh_.advertise<sensor_msgs::PointCloud2>("out_cloud", 1);
    cloud_array_pub = private_nh_.advertise<altair_msgs::CloudArray>("out_cloud_array", 1);
    ellipsoid_cloud_pub = private_nh_.advertise<sensor_msgs::PointCloud2>("ellipsoid_cloud", 1);
    ellipsoid_pub = private_nh_.advertise<altair_msgs::EllipsoidArray>("ellipsoid", 1);

    pose_pub = nh_.advertise<geometry_msgs::PoseArray>("debug_pose", 1);
    cloud_sub = nh_.subscribe("in_cloud", 1, &VivavisVision::cloudCallback, this);

    br = new tf::TransformBroadcaster();
}

void VivavisVision::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &input)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);
    pcl_pc2.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cld_tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cld_tmp_z(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

    pcl::PassThrough<pcl::PointXYZRGB> pass_z_camera;
    pass_z_camera.setFilterFieldName("z");
    pass_z_camera.setFilterLimits(nearClipDistance, farClipDistance);
    pass_z_camera.setInputCloud(cloud);
    pass_z_camera.setKeepOrganized(true);
    pass_z_camera.filter(*xyz_cld_ptr);
    // pub filter one
    // sensor_msgs::PointCloud2 cloud_msg;
    // pcl::toROSMsg(*xyz_cld_ptr, cloud_msg);
    // cloud_msg.header.frame_id = optical_frame;
    // cloud_pub.publish(cloud_msg);

    /*
        try
        {
            listener.waitForTransform(optical_frame, fixed_frame, input->header.stamp, ros::Duration(5.0));
            listener.lookupTransform(fixed_frame, optical_frame, input->header.stamp, optical2map);

            pcl_ros::transformPointCloud(*cld_tmp_z, *cld_tmp, optical2map);

            pcl::PassThrough<pcl::PointXYZRGB> pass_z;
            pass_z.setFilterFieldName("z");
            pass_z.setFilterLimits(0.01, 1.5);
            pass_z.setInputCloud(cld_tmp);
            pass_z.setKeepOrganized(true);
            pass_z.filter(*xyz_cld_ptr);

            xyz_cld_ptr->header.frame_id = fixed_frame;
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
        }

        */
}

template <typename PointT>
boost::shared_ptr<pcl::PointCloud<PointT>>
VivavisVision::voxel_grid_subsample(const boost::shared_ptr<pcl::PointCloud<PointT>> &cld_in, float cell_size)
{
    // ex_VoxelGrid sor;
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cld_in);
    sor.setLeafSize(cell_size, cell_size, cell_size); // leaf size in meters

    boost::shared_ptr<pcl::PointCloud<PointT>> final_cld(new pcl::PointCloud<PointT>);
    sor.filter(*final_cld);
    return final_cld;
    // return sor.getRemovedIndices();
}

void VivavisVision::makeEllipsoid(pcl::PointCloud<pcl::PointXYZRGB> &cloud, const Eigen::Vector3f radii, const Eigen::Vector4f &c)
{
    pcl::PointXYZRGB p;
    p.r = 180;
    p.g = 180;

    float cz = c.z();
    float cx = c.x();
    float cy = c.y();

    for (int phi = 0; phi <= 180; phi += 2)
    {
        for (int theta = 0; theta <= 360; theta += 2)
        {
            p.x = cx + (radii.x() * cos(theta * M_PI / 180) * sin(phi * M_PI / 180));
            p.y = cy + (radii.y() * sin(theta * M_PI / 180) * sin(phi * M_PI / 180));
            p.z = cz + (radii.z() * cos(phi * M_PI / 180));
            cloud.push_back(p);
        }
    }
}

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> VivavisVision::clusterObject(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster, int &num_obj)
{
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> obj_surf;
    std::vector<pcl::PointIndices> clusters;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyz_f_(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_cluster, *xyz_f_, indices);
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZRGB>());
    ec.setInputCloud(xyz_f_);
    ec.setClusterTolerance(object_cluster_distance_);
    ec.setMinClusterSize(min_object_cluster_size_);
    ec.setMaxClusterSize(max_object_cluster_size_);
    ec.setSearchMethod(tree2);
    ec.extract(clusters);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract_;

    num_obj = clusters.size();

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
    return obj_surf;
}

void VivavisVision::processRoom(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr wall_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;

    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);

    if (coefficients->values[2] > 0.9)
    {
        // Ground plane
        extract.setNegative(false);
        extract.filter(*ground_cloud);
    }
    else
    {
        // Wall plane
        extract.setNegative(false);
        extract.filter(*wall_cloud);
    }

    // Remove the extracted plane from the filtered cloud
    extract.setNegative(true);
    extract.filter(*cloud);

    // pub wall
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*wall_cloud, cloud_msg);
    cloud_msg.header.frame_id = fixed_frame;
    cloud_pub.publish(cloud_msg);
}

void VivavisVision::update()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_cld_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (xyz_cld_ptr->size() > 0)
    {

        // map_cld_ptr = voxel_grid_subsample(xyz_cld_ptr, orig_cld_voxel_size);
        // std::cout << " map_cld_ptr " << map_cld_ptr->size() << std::endl;
        // processRoom(map_cld_ptr);

        /*
        if (prev_xyz_cld_ptr->size() > 0)
        {
            *map_cld_ptr += *prev_xyz_cld_ptr;
            map_cld_ptr = voxel_grid_subsample(map_cld_ptr, orig_cld_voxel_size);
        }

        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*xyz_cld_ptr, cloud_msg);
        cloud_msg.header.frame_id = fixed_frame;
        cloud_pub.publish(cloud_msg);


                prev_xyz_cld_ptr = map_cld_ptr;
                std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> objects = clusterObject(map_cld_ptr, num_obj);

                ROS_INFO_STREAM(ros::this_node::getName() << " HAS " << num_obj << "  objects");

                if (num_obj > 0)
                {
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ellipsoid_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

                    altair_msgs::CloudArray cloud_array;
                    altair_msgs::EllipsoidArray ellipsoid_array;
                    geometry_msgs::PoseArray debug_pose_array;

                    debug_pose_array.poses.resize(num_obj);
                    ellipsoid_array.ellipsoid.resize(num_obj);
                    cloud_array.cloud.resize(num_obj);
                    for (size_t i = 0; i < num_obj; i++)
                    {
                        sensor_msgs::PointCloud2 cloud_msg;
                        pcl::toROSMsg(*objects.at(i), cloud_msg);
                        cloud_array.cloud.at(i).header.frame_id = fixed_frame;
                        cloud_array.cloud.at(i) = cloud_msg;

                        Eigen::Vector4f min_pt, max_pt;
                        pcl::getMinMax3D(*objects.at(i), min_pt, max_pt);

                        Eigen::Vector4f c;
                        Eigen::Vector3f radii;
                        radii.x() = std::abs((max_pt.x() - min_pt.x())) / 2.0f;
                        radii.y() = std::abs((max_pt.y() - min_pt.y())) / 2.0f;
                        radii.z() = std::abs((max_pt.z() - min_pt.z())) / 2.0f;

                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr ellipsoid_cloud_i(new pcl::PointCloud<pcl::PointXYZRGB>);
                        pcl::compute3DCentroid(*objects.at(i), c);

                        // to check radian or degree.
                        double rho = atan((radii.z() - radii.x() + sqrt((radii.z() - radii.x()) * (radii.z() - radii.x()) + radii.y())) / radii.y()) * 180 / M_PI;
                        Eigen::Matrix<double, 3, 3> ellipsoid_rot;
                        ellipsoid_rot << cos(rho), -sin(rho), 0, sin(rho), cos(rho), 0, 0, 0, 1;
                        Eigen::Quaterniond rot(ellipsoid_rot);

                        altair_msgs::Ellipsoid ellipsoid_i;
                        ellipsoid_i.pose.position.x = c.x();
                        ellipsoid_i.pose.position.y = c.y();
                        ellipsoid_i.pose.position.z = c.z();
                        ellipsoid_i.pose.orientation.x = rot.x();
                        ellipsoid_i.pose.orientation.y = rot.y();
                        ellipsoid_i.pose.orientation.z = rot.z();
                        ellipsoid_i.pose.orientation.w = rot.w();
                        ellipsoid_i.radii.x = radii.x();
                        ellipsoid_i.radii.y = radii.y();
                        ellipsoid_i.radii.z = radii.z();
                        ellipsoid_array.ellipsoid.at(i) = ellipsoid_i;
                        //
                        debug_pose_array.poses.at(i) = ellipsoid_i.pose;

                        makeEllipsoid(*ellipsoid_cloud_i, radii, c);
                        *ellipsoid_cloud += *ellipsoid_cloud_i;
                    }

                    sensor_msgs::PointCloud2 ellipsoid_cloud_msg;
                    pcl::toROSMsg(*ellipsoid_cloud, ellipsoid_cloud_msg);
                    ellipsoid_cloud_msg.header.frame_id = fixed_frame;
                    ellipsoid_cloud_pub.publish(ellipsoid_cloud_msg);

                    debug_pose_array.header.frame_id = fixed_frame;
                    pose_pub.publish(debug_pose_array);
                    ellipsoid_pub.publish(ellipsoid_array);
                    cloud_array_pub.publish(cloud_array);
                }

            */
    }
}

//------------------------------------------------------------------------------------------------
//                                         MAIN
//------------------------------------------------------------------------------------------------
int vivavis_vision_main(int argc, char **argv)
{
    ros::init(argc, argv, "vivavis_vision_node");
    ros::NodeHandle nh;
    VivavisVision vv_vision(nh);

    while (ros::ok())
    {
        vv_vision.update();
        ros::spinOnce();
    }
    return 0;
}

int main(int argc, char **argv)
{
    return vivavis_vision_main(argc, argv);
}