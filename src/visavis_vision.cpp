#include <visavis_vision/visavis_vision.h>

namespace visavis
{
    VisavisVision::VisavisVision(ros::NodeHandle &nh) : nh_(nh), private_nh_("~"),
                                                        it_(nh),
                                                        xyz_cld_ptr_(new pcl::PointCloud<pcl::PointXYZRGB>),
                                                        cloud_planes_(new pcl::PointCloud<pcl::PointXYZRGB>),
                                                        cloud_obstacles_(new pcl::PointCloud<pcl::PointXYZRGB>),
                                                        num_obj_(0)
    {

        private_nh_.param("optical_frame", optical_frame_, std::string(""));
        private_nh_.param("fixed_frame", fixed_frame_, std::string(""));
        private_nh_.param("near_clip", near_clip_distance_, 0.0);
        private_nh_.param("far_clip", far_clip_distance_, 0.0);

        private_nh_.param("orig_cld_voxel_size", orig_cld_voxel_size_, 0.015f);
        private_nh_.param<double>("object_cluster_distance", object_cluster_distance_, 0.5);
        private_nh_.param<int>("max_object_cluster_size", max_object_cluster_size_, 500000);
        private_nh_.param<int>("min_object_cluster_size", min_object_cluster_size_, 1);

        // Input
        cloud_sub_ = nh_.subscribe("in_cloud", 1, &VisavisVision::cloudCallback, this);
        // Output
        cloud_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>("walls_cloud", 1);
        cloud_obs_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>("obstacles_cloud", 1);
        cloud_nearest_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>("nearest_cloud", 1);

        walls_info_pub_ = private_nh_.advertise<visavis_vision::WallInfoArray>("walls_info", 1);
        obstacles_info_pub_ = private_nh_.advertise<visavis_vision::ObstacleInfoArray>("obstacles_info", 1);

        visual_walls_pub_ = private_nh_.advertise<visualization_msgs::MarkerArray>("visual_walls", 1, true);
        visual_obstacles_pub_ = private_nh_.advertise<visualization_msgs::MarkerArray>("visual_obstacles", 1, true);
        human_ws_pub_ = private_nh_.advertise<visualization_msgs::Marker>("human_ws", 1, true);

        broadcaster_ = new tf::TransformBroadcaster();
        walls_info_.walls.resize(6);
    }

    cv::Mat VisavisVision::getCameraPose() const
    {
        tf::StampedTransform cameraPose_transform;
        try
        {
            listener_.waitForTransform(fixed_frame_, optical_frame_, ros::Time(0), ros::Duration(10.0));
            listener_.lookupTransform(fixed_frame_, optical_frame_, ros::Time(0), cameraPose_transform);
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

    void VisavisVision::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &input)
    {
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*input, pcl_pc2);
        pcl_pc2.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cld_tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cld_tmp_z(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromPCLPointCloud2(pcl_pc2, *xyz_cld_ptr_);

        /*
        // real
        pcl::PassThrough<pcl::PointXYZRGB> pass_z_camera;
        pass_z_camera.setFilterFieldName("z");
        pass_z_camera.setFilterLimits(near_clip_distance_, far_clip_distance_);
        pass_z_camera.setInputCloud(cloud);
        pass_z_camera.setKeepOrganized(true);
        pass_z_camera.filter(*cld_tmp_z);

            try
            {
                listener_.waitForTransform(optical_frame_, fixed_frame_, input->header.stamp, ros::Duration(5.0));
                listener_.lookupTransform(fixed_frame_, optical_frame_, input->header.stamp, optical2map_);

                pcl_ros::transformPointCloud(*cld_tmp_z, *xyz_cld_ptr_, optical2map_);

                // pcl::PassThrough<pcl::PointXYZRGB> pass_z;
                // pass_z.setFilterFieldName("z");
                // pass_z.setFilterLimits(0.01, 1.5);
                // pass_z.setInputCloud(cld_tmp);
                // pass_z.setKeepOrganized(true);
                // pass_z.filter(*xyz_cld_ptr_);

                xyz_cld_ptr_->header.frame_id = fixed_frame_;
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s", ex.what());
            }
            */
    }

    template <typename PointT>
    boost::shared_ptr<pcl::PointCloud<PointT>>
    VisavisVision::voxel_grid_subsample(const boost::shared_ptr<pcl::PointCloud<PointT>> &cld_in, float cell_size)
    {
        pcl::VoxelGrid<PointT> sor;
        sor.setInputCloud(cld_in);
        sor.setLeafSize(cell_size, cell_size, cell_size); // leaf size in meters
        boost::shared_ptr<pcl::PointCloud<PointT>> final_cld(new pcl::PointCloud<PointT>);
        sor.filter(*final_cld);
        return final_cld;
    }

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> VisavisVision::clusterObject(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_cluster, int &num_obj)
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

        num_obj_ = clusters.size();

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

    void VisavisVision::filterRoom(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
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
                setPlaneTransform(idx, cloud_plane, coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3],
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

        *cloud_planes_ += *cloud_temp_planes;
        pcl::copyPointCloud(*voxel_grid_subsample(cloud_planes_, 0.2), *cloud_planes_);
        // ROS_INFO_STREAM("cloud_planes_ " << cloud_planes_->points.size());

        *cloud_obstacles_ += *cloud_temp_obstacles;
        pcl::copyPointCloud(*voxel_grid_subsample(cloud_obstacles_, 0.15), *cloud_obstacles_);

        createVisualObstacles(cloud_obstacles_); // create markers for obstacles

        publishPointCloud(cloud_planes_, cloud_pub_);
        publishPointCloud(cloud_obstacles_, cloud_obs_pub_);

        walls_info_pub_.publish(walls_info_);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr nearest_pcd(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (auto &w : walls_info_.walls)
        {
            if (w.header.frame_id != "floor" && w.header.frame_id != "ceiling" && w.closest_point.x != 0.0 && w.closest_point.y != 0.0 && w.closest_point.z != 0.0)
            {
                pcl::PointXYZRGB point;
                point.x = w.closest_point.x;
                point.y = w.closest_point.y;
                point.z = w.closest_point.z;
                point.r = 255;
                point.g = 255;
                point.b = 0;
                nearest_pcd->push_back(point);
            }
        }

        for (auto &o : obstacles_info_.obstacles)
        {
            pcl::PointXYZRGB point;
            point.x = o.closest_point.x;
            point.y = o.closest_point.y;
            point.z = o.closest_point.z;
            point.r = 255;
            point.g = 255;
            point.b = 0;
            nearest_pcd->push_back(point);
        }

        publishPointCloud(nearest_pcd, cloud_nearest_pub_);
    }

    void VisavisVision::publishPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                                          const ros::Publisher &cloud_pub)
    {
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.frame_id = fixed_frame_;
        cloud_pub.publish(cloud_msg);
    }

    void VisavisVision::setPlaneTransform(int id, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, float a, float b, float c, float d,
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

        visualize_walls_.markers.push_back(addVisualObject(id, centroid, min_p, max_p, Eigen::Vector4f(1.0, 0.0, 0.0, 0.5), plane_quaternion));
        visual_walls_pub_.publish(visualize_walls_);

        float wall_threshold = 95;
        float floor_threshold = 5;

        tf::Vector3 currentTransform_t(centroid(0), centroid(1), centroid(2));
        tf::Quaternion currentTransform_r(plane_quaternion.x(), plane_quaternion.y(), plane_quaternion.z(), plane_quaternion.w());
        tf::Transform currentTransform = tf::Transform(currentTransform_r, currentTransform_t);

        // Convert quaternion to Euler angles
        Eigen::Vector3f euler_angles = plane_quaternion.toRotationMatrix().eulerAngles(0, 1, 2);

        // Extract the z-rotation (in radians) from the Euler angles
        float z_rotation = euler_angles[2];

        pcl::search::KdTree<pcl::PointXYZRGB> kdtree;
        kdtree.setInputCloud(cloud);
        std::vector<int> closest_point_index(1);
        std::vector<float> closest_point_squared_distance(1);

        pcl::PointXYZRGB query_point;
        query_point.x = getCameraPose().at<float>(0, 3);
        query_point.y = getCameraPose().at<float>(1, 3);
        query_point.z = getCameraPose().at<float>(2, 3);

        kdtree.nearestKSearch(query_point, 1, closest_point_index, closest_point_squared_distance);
        // pcl::PointXYZRGB closest_point = cloud->points[closest_point_index];
        geometry_msgs::Point closest_point;
        closest_point.x = cloud->points[closest_point_index.at(0)].x;
        closest_point.y = cloud->points[closest_point_index.at(0)].y;
        closest_point.z = cloud->points[closest_point_index.at(0)].z;

        visavis_vision::WallInfo wall;
        wall.a = a;
        wall.b = b;
        wall.c = c;
        wall.d = d;
        wall.num_points = cloud->points.size();
        wall.header.stamp = ros::Time::now();
        wall.pose.position.x = centroid(0);
        wall.pose.position.y = centroid(1);
        wall.pose.position.z = centroid(2);
        wall.pose.orientation.x = plane_quaternion.x();
        wall.pose.orientation.y = plane_quaternion.y();
        wall.pose.orientation.z = plane_quaternion.z();
        wall.pose.orientation.w = plane_quaternion.w();

        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.frame_id = fixed_frame_;
        wall.cloud = cloud_msg;
        wall.closest_point = closest_point;

        if (radiansToDegrees(angle) < floor_threshold)
        {
            wall.header.frame_id = "floor";
            wall.color_id = 0;
            walls_info_.walls[0] = wall;
        }
        else if (radiansToDegrees(angle) < wall_threshold && radiansToDegrees(angle) > floor_threshold &&
                 centroid[0] < getCameraPose().at<float>(0, 3) &&
                 std::fabs(a) > 0.9 &&
                 std::fabs(a) < 1.1)
        {
            wall.header.frame_id = "left";
            wall.color_id = 1;
            walls_info_.walls[1] = wall;
        }
        else if (radiansToDegrees(angle) < wall_threshold && radiansToDegrees(angle) > floor_threshold &&
                 centroid[0] > getCameraPose().at<float>(0, 3) &&
                 std::fabs(a) > 0.9 &&
                 std::fabs(a) < 1.1)
        {
            wall.header.frame_id = "right";
            wall.color_id = 2;
            walls_info_.walls[2] = wall;
        }
        else if (radiansToDegrees(angle) < wall_threshold && radiansToDegrees(angle) > floor_threshold &&
                 centroid[1] > getCameraPose().at<float>(1, 3) &&
                 std::fabs(b) > 0.9 &&
                 std::fabs(b) < 1.1)
        {
            wall.header.frame_id = "front";
            wall.color_id = 3;
            walls_info_.walls[3] = wall;
        }
        else if (radiansToDegrees(angle) < wall_threshold && radiansToDegrees(angle) > floor_threshold &&
                 centroid[1] < getCameraPose().at<float>(1, 3) &&
                 std::fabs(b) > 0.9 &&
                 std::fabs(b) < 1.1)
        {
            wall.header.frame_id = "back";
            wall.color_id = 4;
            walls_info_.walls[4] = wall;
        }
        else
        {
            if (radiansToDegrees(angle) < floor_threshold)
            {
                wall.header.frame_id = "ceiling";
                wall.color_id = 5;
                walls_info_.walls[5] = wall;
            }
            else
                wall.header.frame_id = "unknown";
        }
        broadcaster_->sendTransform(tf::StampedTransform(currentTransform, ros::Time::now(), fixed_frame_, wall.header.frame_id));
    }

    visualization_msgs::Marker VisavisVision::addVisualObject(int id, Eigen::Vector4f centroid, Eigen::Vector4f min_p, Eigen::Vector4f max_p,
                                                              Eigen::Vector4f color,
                                                              Eigen::Quaternionf orientation)
    {
        visualization_msgs::Marker plane_marker;
        plane_marker.header.frame_id = fixed_frame_;
        plane_marker.header.stamp = ros::Time::now();
        plane_marker.id = id;
        plane_marker.ns = std::to_string(id);
        plane_marker.action = visualization_msgs::Marker::ADD;
        plane_marker.type = visualization_msgs::Marker::CUBE;

        plane_marker.pose.orientation.w = orientation.w();
        plane_marker.pose.orientation.x = orientation.x();
        plane_marker.pose.orientation.y = orientation.y();
        plane_marker.pose.orientation.z = orientation.z();
        plane_marker.pose.position.x = centroid[0];         // coefficients->values[0];
        plane_marker.pose.position.y = centroid[1];         // coefficients->values[1];
        plane_marker.pose.position.z = centroid[2];         // coefficients->values[2];
        plane_marker.scale.x = 0.1 + (max_p[0] - min_p[0]); //  offset + ()
        plane_marker.scale.y = 0.1 + (max_p[1] - min_p[1]); //  offset + ()
        plane_marker.scale.z = 0.1 + (max_p[2] - min_p[2]); //  offset + ()
        plane_marker.color.r = color[0];                    // 1.0;
        plane_marker.color.g = color[1];                    // 1.0;
        plane_marker.color.b = color[2];                    // 0.0;
        plane_marker.color.a = color[3];                    // 0.5;                                       // Adjust the transparency of the visualization
        return plane_marker;
    }

    void VisavisVision::createVisualObstacles(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
    {
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> objects = clusterObject(cloud, num_obj_);
        // ROS_INFO_STREAM(ros::this_node::getName() << " HAS " << num_obj_ << "  objects");
        if (num_obj_ > 0)
        {
            visualize_obstacles_.markers.clear();
            obstacles_info_.obstacles.clear();

            for (size_t i = 0; i < num_obj_; i++)
            {
                bool discard = false;
                Eigen::Vector4f min_pt, max_pt;
                pcl::getMinMax3D(*objects.at(i), min_pt, max_pt);

                Eigen::Vector4f c;
                pcl::compute3DCentroid(*objects.at(i), c);

                for (auto &w : walls_info_.walls)
                {
                    if (std::fabs(w.a * c[0] + w.b * c[1] + w.c * c[2] + w.d) < 0.2)
                    {
                        if (!w.header.frame_id.empty() && !discard)
                        {
                            // ROS_INFO_STREAM(" object " << i << "  is on plane " << w.header.frame_id);
                            discard = true;
                        }
                    }
                }
                if (!discard)
                {

                    visavis_vision::ObstacleInfo obstacle;
                    obstacle.header.frame_id = fixed_frame_;
                    obstacle.header.stamp = ros::Time::now();
                    obstacle.pose.position.x = c[0];
                    obstacle.pose.position.y = c[1];
                    obstacle.pose.position.z = c[2];
                    obstacle.pose.orientation.x = obstacle.pose.orientation.y = obstacle.pose.orientation.z = 0;
                    obstacle.pose.orientation.w = 1;

                    sensor_msgs::PointCloud2 cloud_msg;
                    pcl::toROSMsg(*objects.at(i), cloud_msg);
                    cloud_msg.header.frame_id = fixed_frame_;
                    obstacle.cloud = cloud_msg;

                    pcl::search::KdTree<pcl::PointXYZRGB> kdtree;
                    kdtree.setInputCloud(objects.at(i));
                    std::vector<int> closest_point_index(1);
                    std::vector<float> closest_point_squared_distance(1);

                    pcl::PointXYZRGB query_point;
                    query_point.x = getCameraPose().at<float>(0, 3);
                    query_point.y = getCameraPose().at<float>(1, 3);
                    query_point.z = getCameraPose().at<float>(2, 3);

                    kdtree.nearestKSearch(query_point, 1, closest_point_index, closest_point_squared_distance);
                    // geometry_msgs::Point closest_point;
                    obstacle.closest_point.x = objects.at(i)->points[closest_point_index.at(0)].x;
                    obstacle.closest_point.y = objects.at(i)->points[closest_point_index.at(0)].y;
                    obstacle.closest_point.z = objects.at(i)->points[closest_point_index.at(0)].z;

                    obstacles_info_.obstacles.push_back(obstacle);

                    visualize_obstacles_.markers.push_back(addVisualObject(i, c, min_pt, max_pt, Eigen::Vector4f(1.0, 0.0, 0.0, 0.5)));
                }
            } // end for i-objs

            ROS_INFO_STREAM("obstacles_info_pub_ HAS " << obstacles_info_.obstacles.size() << "  objects");
            obstacles_info_pub_.publish(obstacles_info_);
            visual_obstacles_pub_.publish(visualize_obstacles_);
        }
    }

    void VisavisVision::update()
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_cld_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

        if (xyz_cld_ptr_->size() > 0)
        {
            human_ws_pub_.publish(
                addVisualObject(0, Eigen::Vector4f(getCameraPose().at<float>(0, 3), getCameraPose().at<float>(1, 3), getCameraPose().at<float>(2, 3), 0),
                                Eigen::Vector4f(0.0, 0.0, 0.0, 0.5), Eigen::Vector4f(0.5, 0.5, 1.7, 0.5), Eigen::Vector4f(0.0, 1.0, 0.0, 0.5), Eigen::Quaternionf(0, 0, 0, 1.0))

            );
            map_cld_ptr = voxel_grid_subsample(xyz_cld_ptr_, orig_cld_voxel_size_);
            filterRoom(map_cld_ptr);
        }
    }
}

//------------------------------------------------------------------------------------------------
//                                         MAIN
//------------------------------------------------------------------------------------------------
int visavis_vision_main(int argc, char **argv)
{
    ros::init(argc, argv, "visavis_vision_node");
    ros::NodeHandle nh;
    visavis::VisavisVision vv_vision(nh);

    while (ros::ok())
    {
        vv_vision.update();
        ros::spinOnce();
    }
    return 0;
}

int main(int argc, char **argv)
{
    return visavis_vision_main(argc, argv);
}