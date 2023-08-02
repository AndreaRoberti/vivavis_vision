#include <rgb_d_combiner.h>

CombineDepthRgb::CombineDepthRgb(ros::NodeHandle& nh) :
        nh_(nh), private_nh_("~"), it_(nh), cloud_semantic(new pcl::PointCloud<pcl::PointXYZRGB>)
{
    name_of_node_ = ros::this_node::getName();

    cloud_pub = private_nh_.advertise<sensor_msgs::PointCloud2>("output_point_cloud", 1);
    // camera_info_sub =
    //     private_nh_.subscribe("/camera/color/camera_info", 1, &CombineDepthRgb::cameraInfoCallback, this);
}

CombineDepthRgb::~CombineDepthRgb() {}

void CombineDepthRgb::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
{
    camera_image_frame = msg->header.frame_id;
    K                  = (cv::Mat_<float>(3, 3) << msg->K[0],
         msg->K[1],
         msg->K[2],
         msg->K[3],
         msg->K[4],
         msg->K[5],
         msg->K[6],
         msg->K[7],
         msg->K[8]);

    D = (cv::Mat_<float>(5, 1) << msg->D[0], msg->D[1], msg->D[2], msg->D[3], msg->D[4], msg->D[5]);
    // std::cout << "OK" << std::endl;
    // camera_info_sub.shutdown(); // Shutdown the sub we want camera_info once
}

void CombineDepthRgb::PublishRenderedImage(cv::Mat image, std::string encoding)
{
    std_msgs::Header header;
    header.stamp                                   = ros::Time::now();
    header.frame_id                                = camera_image_frame;
    const sensor_msgs::ImagePtr rendered_image_msg = cv_bridge::CvImage(header, encoding, image).toImageMsg();
    rendered_image_publisher_.publish(rendered_image_msg);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
CombineDepthRgb::pointCloudFrom2D(cv::Mat image_depth, cv::Mat image_rgb, cv::Mat K)
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cld(new pcl::PointCloud<pcl::PointXYZRGB>());
    cv::Size                               s    = image_depth.size();
    int                                    rows = s.height;
    int                                    cols = s.width;

    float cx     = K.at<float>(0, 2);
    float cy     = K.at<float>(1, 2);
    float fx_inv = 1.0 / K.at<float>(0, 0);
    float fy_inv = 1.0 / K.at<float>(1, 1);

    for (size_t u = 0; u < rows; u++)
        for (size_t v = 0; v < cols; v++) {
            uint16_t z_raw = image_depth.at<uint16_t>(u, v);

            if (z_raw != 0.0) {
                float z_metric = z_raw * 0.001;

                pcl::PointXYZRGB out_points;
                out_points.x = z_metric * ((u - cx) * fx_inv);
                out_points.y = z_metric * ((v - cy) * fy_inv);
                out_points.z = z_metric;

                out_points.r = (int) image_rgb.at<cv::Vec3b>(u, v)[0];
                out_points.g = (int) image_rgb.at<cv::Vec3b>(u, v)[1];
                out_points.b = (int) image_rgb.at<cv::Vec3b>(u, v)[2];

                cld->push_back(out_points);
            }
        }
    return cld;
}

void CombineDepthRgb::grabRGBD(const sensor_msgs::ImageConstPtr& msg_rgb, const sensor_msgs::ImageConstPtr& msg_d)
{
    cv::Mat                    image_rgb, image_depth;
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try {
        cv_ptrRGB = cv_bridge::toCvShare(msg_rgb);
        image_rgb = cv_ptrRGB->image;
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try {
        cv_ptrD     = cv_bridge::toCvShare(msg_d);
        image_depth = cv_ptrD->image;
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    // PublishRenderedImage(mask, "mono8");

    if (!image_depth.empty() && !image_rgb.empty() && !K.empty()) {
     
        // std::cout << "OKsss" << std::endl;   
        cv::rotate(image_depth, image_depth, cv::ROTATE_90_COUNTERCLOCKWISE);
        cv::rotate(image_rgb, image_rgb, cv::ROTATE_90_COUNTERCLOCKWISE);
        cv::flip(image_depth, image_depth, 0);
        cv::flip(image_rgb, image_rgb, 0);

        cloud_semantic = pointCloudFrom2D(image_depth, image_rgb, K);
        // publish Point cloud
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud_semantic, cloud_msg);

        cloud_msg.header.frame_id = camera_image_frame;
        cloud_msg.header.stamp    = ros::Time::now();
        cloud_pub.publish(cloud_msg);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "semantic_registration");
    ros::NodeHandle nodeHandler;

    CombineDepthRgb s_rec(nodeHandler);

    std::string image_topic, depth_topic, info_topic;
    int         sync_time;
    nodeHandler.param("/semantic_registration/image_topic", image_topic, std::string("/camera/color/image_raw"));
    nodeHandler.param("/semantic_registration/depth_topic", depth_topic, std::string("/camera/aligned_depth_to_color/image_raw"));
    nodeHandler.param("/semantic_registration/sync_time", sync_time, 10);
    nodeHandler.param("/semantic_registration/info_topic", info_topic, std::string("/camera/color/camera_info"));

    ros::Subscriber camera_info_sub =
        nodeHandler.subscribe(info_topic, 1, &CombineDepthRgb::cameraInfoCallback, &s_rec);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nodeHandler, image_topic, 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nodeHandler, depth_topic, 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(sync_time), rgb_sub, depth_sub);
    sync.registerCallback(
        std::bind(&CombineDepthRgb::grabRGBD, &s_rec, std::placeholders::_1, std::placeholders::_2));

    // Spin me round
    ros::spin();

    return 0;
}
