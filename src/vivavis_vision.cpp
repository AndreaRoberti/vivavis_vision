#include <vivavis_vision.h>

VivavisVision::VivavisVision(ros::NodeHandle &nh) : nh_(nh), private_nh_("~"),
                                                    it_(nh),
                                                    xyz_cld_ptr(new pcl::PointCloud<pcl::PointXYZRGB>),
                                                    prev_xyz_cld_ptr(new pcl::PointCloud<pcl::PointXYZRGB>),
                                                    cloud_planes(new pcl::PointCloud<pcl::PointXYZRGB>),
                                                    cloud_obstacles(new pcl::PointCloud<pcl::PointXYZRGB>),
                                                    cloud_final_obstacles(new pcl::PointCloud<pcl::PointXYZRGB>),
                                                    num_obj(0)
{

    private_nh_.param("optical_frame", optical_frame, std::string(""));
    private_nh_.param("fixed_frame", fixed_frame, std::string(""));
    private_nh_.param("near_clip", nearClipDistance, 0.0);
    private_nh_.param("far_clip", farClipDistance, 0.0);

    private_nh_.param("orig_cld_voxel_size", orig_cld_voxel_size, 0.015f);
    private_nh_.param<double>("object_cluster_distance", object_cluster_distance_, 0.5);
    private_nh_.param<int>("max_object_cluster_size", max_object_cluster_size_, 500000);
    private_nh_.param<int>("min_object_cluster_size", min_object_cluster_size_, 1);

    // Input
    cloud_sub = nh_.subscribe("in_cloud", 1, &VivavisVision::cloudCallback, this);
    // Output
    cloud_pub = private_nh_.advertise<sensor_msgs::PointCloud2>("walls_cloud", 1);
    cloud_obs_pub = private_nh_.advertise<sensor_msgs::PointCloud2>("obstacles_cloud", 1);
    ellipsoid_cloud_pub = private_nh_.advertise<sensor_msgs::PointCloud2>("ellipsoid_cloud", 1);

    // cloud_array_pub = private_nh_.advertise<vivavis_vision::CloudArray>("out_cloud_array", 1);
    ellipsoid_pub = private_nh_.advertise<vivavis_vision::EllipsoidArray>("ellipsoid", 1);
    walls_info_pub = private_nh_.advertise<vivavis_vision::WallInfoArray>("walls_info", 1);

    visual_walls_pub = private_nh_.advertise<visualization_msgs::MarkerArray>("visual_walls", 1, true);
    visual_obstacles_pub = private_nh_.advertise<visualization_msgs::MarkerArray>("visual_obstacles", 1, true);
    human_ws_pub = private_nh_.advertise<visualization_msgs::Marker>("human_ws", 1, true);

    pose_pub = nh_.advertise<geometry_msgs::PoseArray>("debug_pose", 1);

    br = new tf::TransformBroadcaster();
    walls_info.walls.resize(6);
}

cv::Mat VivavisVision::getCameraPose()
{
    tf::StampedTransform cameraPose_transform;
    try
    {
        listener.waitForTransform(fixed_frame, optical_frame, ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform(fixed_frame, optical_frame, ros::Time(0), cameraPose_transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    // get optical frame transform w.r.t the base frame
    auto rotMat = cameraPose_transform.getBasis();
    auto trans = cameraPose_transform.getOrigin();

    cv::Mat optFramePose = (cv::Mat_<float>(4, 4) << rotMat.getRow(0).x(), rotMat.getRow(0).y(), rotMat.getRow(0).z(), trans.x(),
                            rotMat.getRow(1).x(), rotMat.getRow(1).y(), rotMat.getRow(1).z(), trans.y(),
                            rotMat.getRow(2).x(), rotMat.getRow(2).y(), rotMat.getRow(2).z(), trans.z(),
                            0, 0, 0, 1);

    return optFramePose;
}

void VivavisVision::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &input)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);
    pcl_pc2.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cld_tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cld_tmp_z(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_pc2, *xyz_cld_ptr);

    /*
    // real
    pcl::PassThrough<pcl::PointXYZRGB> pass_z_camera;
    pass_z_camera.setFilterFieldName("z");
    pass_z_camera.setFilterLimits(nearClipDistance, farClipDistance);
    pass_z_camera.setInputCloud(cloud);
    pass_z_camera.setKeepOrganized(true);
    pass_z_camera.filter(*cld_tmp_z);

        try
        {
            listener.waitForTransform(optical_frame, fixed_frame, input->header.stamp, ros::Duration(5.0));
            listener.lookupTransform(fixed_frame, optical_frame, input->header.stamp, optical2map);

            pcl_ros::transformPointCloud(*cld_tmp_z, *xyz_cld_ptr, optical2map);

            // pcl::PassThrough<pcl::PointXYZRGB> pass_z;
            // pass_z.setFilterFieldName("z");
            // pass_z.setFilterLimits(0.01, 1.5);
            // pass_z.setInputCloud(cld_tmp);
            // pass_z.setKeepOrganized(true);
            // pass_z.filter(*xyz_cld_ptr);

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
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cld_in);
    sor.setLeafSize(cell_size, cell_size, cell_size); // leaf size in meters
    boost::shared_ptr<pcl::PointCloud<PointT>> final_cld(new pcl::PointCloud<PointT>);
    sor.filter(*final_cld);
    return final_cld;
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

void VivavisVision::filterRoom(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp_planes(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp_obstacles(new pcl::PointCloud<pcl::PointXYZRGB>());
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.05);

    int idx = 0;
    int nr_points = (int)cloud->size();
    while (cloud->size() > 0.1 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        }
        else
        {
            // Extract the planar inliers from the input cloud
            pcl::ExtractIndices<pcl::PointXYZRGB> extract;
            extract.setInputCloud(cloud);
            extract.setIndices(inliers);
            extract.setNegative(false);

            // Get the points associated with the planar surface
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGB>());
            extract.filter(*cloud_plane);

            // compute bounding box and centroid
            Eigen::Vector4f centroid, minp, maxp;
            pcl::getMinMax3D(*cloud_plane, minp, maxp);
            pcl::compute3DCentroid<pcl::PointXYZRGB>(*cloud_plane, centroid);
            // calculate TFs
            setPlaneTransform(idx, cloud_plane->points.size(), coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3],
                              centroid, minp, maxp);

            // Remove the planar inliers, extract the rest
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZRGB>);
            extract.setNegative(true);
            extract.filter(*cloud_f);
            *cloud = *cloud_f;
            *cloud_temp_obstacles = *cloud_f;
            *cloud_temp_planes += *cloud_plane;

            idx++;
        }
    }

    *cloud_planes += *cloud_temp_planes;
    pcl::copyPointCloud(*voxel_grid_subsample(cloud_planes, 0.2), *cloud_planes);
    // ROS_INFO_STREAM("cloud_planes " << cloud_planes->points.size());

    *cloud_obstacles += *cloud_temp_obstacles;
    pcl::copyPointCloud(*voxel_grid_subsample(cloud_obstacles, 0.15), *cloud_obstacles);

    createVisualObstacles(cloud_obstacles); // create markers for obstacles

    // pub walls
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud_planes, cloud_msg);
    cloud_msg.header.frame_id = fixed_frame; // optical_frame;
    cloud_pub.publish(cloud_msg);
    // pub obstacles
    sensor_msgs::PointCloud2 cloud_msg_2;
    pcl::toROSMsg(*cloud_obstacles, cloud_msg_2);
    cloud_msg_2.header.frame_id = fixed_frame; // optical_frame;
    cloud_obs_pub.publish(cloud_msg_2);

    walls_info_pub.publish(walls_info);
}

void VivavisVision::setPlaneTransform(int id, int num_points, float a, float b, float c, float d,
                                      Eigen::Vector4f centroid, Eigen::Vector4f min_p, Eigen::Vector4f max_p)
{
    Eigen::Vector3f plane_normal(a, b, c);
    // Normalize the normal vector to get the orientation
    plane_normal.normalize();
    // Compute the plane orientation as a quaternion
    Eigen::Quaternionf plane_quaternion;
    Eigen::Vector3f up_vector(0.0, 0.0, 1.0); // Assuming Z-up convention
    plane_quaternion.setFromTwoVectors(up_vector, plane_normal);
    float dot_product = plane_normal.dot(up_vector);
    float angle = acos(dot_product);

    visualize_walls.markers.push_back(addVisualObject(id, centroid, min_p, max_p, Eigen::Vector4f(1.0, 0.0, 0.0, 0.5), plane_quaternion));
    visual_walls_pub.publish(visualize_walls);

    float wall_threshold = 95;
    float floor_threshold = 5;

    tf::Vector3 currentTransform_t(centroid(0), centroid(1), centroid(2));
    tf::Quaternion currentTransform_r(plane_quaternion.x(), plane_quaternion.y(), plane_quaternion.z(), plane_quaternion.w());
    tf::Transform currentTransform = tf::Transform(currentTransform_r, currentTransform_t);

    // Convert quaternion to Euler angles
    Eigen::Vector3f euler_angles = plane_quaternion.toRotationMatrix().eulerAngles(0, 1, 2);

    // Extract the z-rotation (in radians) from the Euler angles
    float z_rotation = euler_angles[2];

    vivavis_vision::WallInfo wall;
    wall.a = a;
    wall.b = b;
    wall.c = c;
    wall.d = d;
    wall.num_points = num_points;
    wall.header.stamp = ros::Time::now();
    wall.pose.position.x = centroid(0);
    wall.pose.position.y = centroid(1);
    wall.pose.position.z = centroid(2);
    wall.pose.orientation.x = plane_quaternion.x();
    wall.pose.orientation.y = plane_quaternion.y();
    wall.pose.orientation.z = plane_quaternion.z();
    wall.pose.orientation.w = plane_quaternion.w();

    if (radiansToDegrees(angle) < floor_threshold)
    {
        wall.header.frame_id = "floor";
        walls_info.walls[0] = wall;
        wall.color_id = 0;
    }
    else if (radiansToDegrees(angle) < wall_threshold && radiansToDegrees(angle) > floor_threshold &&
             centroid[0] < getCameraPose().at<float>(0, 3) &&
             std::fabs(a) > 0.9 &&
             std::fabs(a) < 1.1)
    {
        wall.header.frame_id = "left";
        wall.color_id = 1;
        walls_info.walls[1] = wall;
    }
    else if (radiansToDegrees(angle) < wall_threshold && radiansToDegrees(angle) > floor_threshold &&
             centroid[0] > getCameraPose().at<float>(0, 3) &&
             std::fabs(a) > 0.9 &&
             std::fabs(a) < 1.1)
    {
        wall.header.frame_id = "right";
        wall.color_id = 2;
        walls_info.walls[2] = wall;
    }
    else if (radiansToDegrees(angle) < wall_threshold && radiansToDegrees(angle) > floor_threshold &&
             centroid[1] > getCameraPose().at<float>(1, 3) &&
             std::fabs(b) > 0.9 &&
             std::fabs(b) < 1.1)
    {
        wall.header.frame_id = "front";
        wall.color_id = 3;
        walls_info.walls[3] = wall;
    }
    else if (radiansToDegrees(angle) < wall_threshold && radiansToDegrees(angle) > floor_threshold &&
             centroid[1] < getCameraPose().at<float>(1, 3) &&
             std::fabs(b) > 0.9 &&
             std::fabs(b) < 1.1)
    {
        wall.header.frame_id = "back";
        wall.color_id = 4;
        walls_info.walls[4] = wall;
    }
    else
    {
        if (radiansToDegrees(angle) < floor_threshold)
        {
            wall.header.frame_id = "ceiling";
            wall.color_id = 5;
            walls_info.walls[5] = wall;
        }
    }
    br->sendTransform(tf::StampedTransform(currentTransform, ros::Time::now(), fixed_frame, wall.header.frame_id));
}

visualization_msgs::Marker VivavisVision::addVisualObject(int id, Eigen::Vector4f centroid, Eigen::Vector4f min_p, Eigen::Vector4f max_p,
                                                          Eigen::Vector4f color,
                                                          Eigen::Quaternionf orientation)
{
    visualization_msgs::Marker plane_marker;
    plane_marker.header.frame_id = fixed_frame; // optical_frame;
    plane_marker.header.stamp = ros::Time::now();
    plane_marker.id = id;
    plane_marker.ns = std::to_string(id);
    plane_marker.action = visualization_msgs::Marker::ADD;
    plane_marker.type = visualization_msgs::Marker::CUBE;

    plane_marker.pose.orientation.w = orientation.w();
    plane_marker.pose.orientation.x = orientation.x();
    plane_marker.pose.orientation.y = orientation.y();
    plane_marker.pose.orientation.z = orientation.z();
    plane_marker.pose.position.x = centroid[0];                       // coefficients->values[0];
    plane_marker.pose.position.y = centroid[1];                       // coefficients->values[1];
    plane_marker.pose.position.z = centroid[2];                       // coefficients->values[2];
    plane_marker.scale.x = std::fabs(min_p[0]) + std::fabs(max_p[0]); // Adjust the size of the visualization as needed
    plane_marker.scale.y = std::fabs(min_p[1]) + std::fabs(max_p[1]);
    plane_marker.scale.z = std::fabs(min_p[2]) + std::fabs(max_p[2]); // Adjust the thickness of the  visualization as needed
    plane_marker.color.r = color[0];                                  // 1.0;
    plane_marker.color.g = color[1];                                  // 1.0;
    plane_marker.color.b = color[2];                                  // 0.0;
    plane_marker.color.a = color[3];                                  // 0.5;                                       // Adjust the transparency of the visualization
    return plane_marker;
}

void VivavisVision::createVisualObstacles(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> objects = clusterObject(cloud, num_obj);
    // ROS_INFO_STREAM(ros::this_node::getName() << " HAS " << num_obj << "  objects");
    if (num_obj > 0)
    {
        geometry_msgs::PoseArray obstacles_poses;
        obstacles_poses.header.frame_id = fixed_frame;
        visualize_obstacles.markers.clear();
        for (size_t i = 0; i < num_obj; i++)
        {
            bool discard = false;
            Eigen::Vector4f min_pt, max_pt;
            pcl::getMinMax3D(*objects.at(i), min_pt, max_pt);

            Eigen::Vector4f c;
            pcl::compute3DCentroid(*objects.at(i), c);

            for (auto &w : walls_info.walls)
            {
                if (std::fabs(w.a * c[0] + w.b * c[1] + w.c * c[2] + w.d) < 0.2)
                {
                    if (!w.header.frame_id.empty() && !discard)
                    {
                        ROS_INFO_STREAM(" object " << i << "  is on plane " << w.header.frame_id);
                        discard = true;
                    }
                }
            }
            if (!discard)
            {
                ROS_INFO_STREAM(" false disc , pub object " << i);
                geometry_msgs::Pose pose;
                pose.position.x = c[0];
                pose.position.y = c[1];
                pose.position.z = c[2];
                pose.orientation.x = pose.orientation.y = pose.orientation.z = 0;
                pose.orientation.w = 1;
                obstacles_poses.poses.push_back(pose);
                pose_pub.publish(obstacles_poses);

                visualize_obstacles.markers.push_back(addVisualObject(i, c, min_pt, max_pt, Eigen::Vector4f(1.0, 0.0, 0.0, 0.5)));
                visual_obstacles_pub.publish(visualize_obstacles);
            }
        }
    }
}

void VivavisVision::update()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_cld_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (xyz_cld_ptr->size() > 0)
    {
        human_ws_pub.publish(
            addVisualObject(0, Eigen::Vector4f(getCameraPose().at<float>(0, 3), getCameraPose().at<float>(1, 3), getCameraPose().at<float>(2, 3), 0),
                            Eigen::Vector4f(0.3, 0.3, 1.0, 0.5), Eigen::Vector4f(0.3, 0.3, 1.0, 0.5), Eigen::Vector4f(0.0, 1.0, 0.0, 0.5), Eigen::Quaternionf(0, 0, 0, 1.0))

        );
        map_cld_ptr = voxel_grid_subsample(xyz_cld_ptr, orig_cld_voxel_size);
        filterRoom(map_cld_ptr);
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