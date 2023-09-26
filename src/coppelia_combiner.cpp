#include <string>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Float32MultiArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <typeinfo>
#include <sensor_msgs/CameraInfo.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0) // Converts degrees to radians
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI) // Converts radians to degrees

class PointCloudRGBCombiner
{
    typedef pcl::PointXYZRGB Point;

private:
    ros::NodeHandle nh_, private_nh_;
    ros::Publisher color_pcl_pub_;
    ros::Subscriber image_sub_, depthBuffer_sub_, image_depth_sub_, camera_info_sub;

    sensor_msgs::Image currImg;
    std_msgs::Float32MultiArray depthBuff;

    std::string pcl_topic, vrep_depth_topic, image_topic, camera_color_frame, camera_info_name;
    bool old_plugin;
    int v_res, u_res;
    double focal_length;
    double near_clip, far_clip;
    double view_angle;
    int camera_w, camera_h;
    cv::Mat image_rgb, image_depth, K;

    std::vector<float> camera_K, camera_R, camera_P, camera_D;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mergedCloud;

public:
    PointCloudRGBCombiner(ros::NodeHandle nh) : nh_(nh), private_nh_("~"),
                                                old_plugin(false),
                                                u_res(480),
                                                v_res(640),
                                                near_clip(1.00e-02),
                                                far_clip(5.00e+00),
                                                view_angle(60.0)
    {

        private_nh_.param("camera_color_name", image_topic, std::string("/camera/color/image_rect_color"));
        private_nh_.param("camera_info_name", camera_info_name, std::string("/camera/color/camera_info"));
        private_nh_.param("camera_depth_name", vrep_depth_topic, std::string("camera/depth/image_rect_color"));
        private_nh_.param("point_cloud_out", pcl_topic, std::string("/camera/points"));
        private_nh_.param<bool>("old_plugin", old_plugin, true);

        private_nh_.param("u_res", u_res, 480);
        private_nh_.param("v_res", v_res, 640);
        private_nh_.param("near_clip", near_clip, 1.00e-02);
        private_nh_.param("far_clip", far_clip, 5.00e+00);
        private_nh_.param("view_angle", view_angle, 60.0);

        color_pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pcl_topic, 1);
        image_sub_ = nh_.subscribe(image_topic, 1, &PointCloudRGBCombiner::imageCb, this);
        image_depth_sub_ = nh_.subscribe(vrep_depth_topic, 1, &PointCloudRGBCombiner::imageDepthCb, this);
        camera_info_sub = nh_.subscribe(camera_info_name, 1, &PointCloudRGBCombiner::cameraInfoCallback, this);

        depthBuffer_sub_ = nh_.subscribe("/coppelia/depth_buff", 1, &PointCloudRGBCombiner::detphBuffCb, this);

        mergedCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    }

    ~PointCloudRGBCombiner() {}

    template <typename T>
    cv::Mat vector2Mat(std::vector<T> vec, size_t N)
    {
        cv::Mat M = cv::Mat::eye(N, N, CV_32FC1);
        memcpy(M.data, vec.data(), vec.size() * sizeof(T));
        return M;
    }

    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
    {
        camera_h = msg->height;
        camera_w = msg->width;
        camera_color_frame = msg->header.frame_id;
        for (int i = 0; i < 12; i++)
        {
            if (i < 5)
            {
                camera_D.push_back(msg->D[i]);
                camera_K.push_back(msg->K[i]);
                camera_R.push_back(msg->R[i]);
                camera_P.push_back(msg->P[i]);
            }
            else if (i >= 5 && i < 9)
            {
                camera_K.push_back(msg->K[i]);
                camera_R.push_back(msg->R[i]);
                camera_P.push_back(msg->P[i]);
            }
            else
                camera_P.push_back(msg->P[i]);
        }

        K = vector2Mat(camera_K, 3);
        camera_info_sub.shutdown();
    }

    void imageCb(const sensor_msgs::ImageConstPtr &msg)
    {
        currImg = *msg;

        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); // msg->encoding  sensor_msgs::image_encodings::BGR8) if BGR8 assertion fail for cvtColor. ...dno if 16
            image_rgb = cv_ptr->image;
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

    void imageDepthCb(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1); // msg->encoding  sensor_msgs::image_encodings::BGR8) if BGR8 assertion fail for cvtColor. ...dno if 16
            image_depth = cv_ptr->image;
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

    void detphBuffCb(const std_msgs::Float32MultiArray &msg)
    {
        depthBuff = msg;
    }

    template <typename PointT>
    boost::shared_ptr<pcl::PointCloud<PointT>>
    pointCloudFrom2D(cv::Mat image_depth, cv::Mat image_rgb, cv::Mat K, boost::shared_ptr<pcl::PointCloud<PointT>> &cld)
    {
        cv::Size s = image_depth.size();
        int rows = s.height;
        int cols = s.width;

        int w = rows * cols;
        int h = 1;

        cld->resize(w * h);

        float cx = K.at<float>(0, 2);
        float cy = K.at<float>(1, 2);
        float fx_inv = 1.0 / K.at<float>(0, 0);
        float fy_inv = 1.0 / K.at<float>(1, 1);

        for (size_t u = 0; u < rows; u++)
            for (size_t v = 0; v < cols; v++)
            {
                uint16_t z_raw = image_depth.at<uint16_t>(u, v);

                if (z_raw != 0.0)
                {
                    float z_metric = z_raw * 0.001;

                    PointT out_points;
                    out_points.x = z_metric * ((u - cx) * fx_inv);
                    out_points.y = z_metric * ((v - cy) * fy_inv);
                    out_points.z = z_metric;
                    if (out_points.rgb && !image_rgb.empty())
                    {
                        out_points.r = (int)image_rgb.at<cv::Vec3b>(u, v)[2];
                        out_points.g = (int)image_rgb.at<cv::Vec3b>(u, v)[1];
                        out_points.b = (int)image_rgb.at<cv::Vec3b>(u, v)[0];
                    }

                    cld->push_back(out_points);
                }
            }
        return cld;
    }

    void update()
    {
        cv_bridge::CvImagePtr cv_color;
        std::vector<uint8_t> color_vect;
        ros::Time start_time = ros::Time::now();

        if (old_plugin)
        {
            sensor_msgs::PointCloud2::ConstPtr recent_cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(pcl_topic, nh_, ros::Duration(10.0));

            while (!recent_cloud)
            {
                ROS_ERROR("Waiting for point cloud2 and image");
                sensor_msgs::PointCloud2::ConstPtr recent_cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(pcl_topic, nh_, ros::Duration(3.0));
            }

            // ROS_INFO_STREAM("pcl received after " << ros::Time::now() - start_time << " seconds. Start Combining");

            if (currImg.data.size() > 0)
            {
                merge(*const_cast<sensor_msgs::PointCloud2 *>(recent_cloud.get()), currImg.data);

                color_pcl_pub_.publish(*recent_cloud);
                cv_color.reset();
                recent_cloud.reset();
                color_vect.clear();
            }
        }
        else
        {
            /*
            if (!image_depth.empty() && !image_rgb.empty() && !K.empty())
            {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr test3d2d(new pcl::PointCloud<pcl::PointXYZRGB>);
                sensor_msgs::PointCloud2 cloud_msg;
                pointCloudFrom2D(image_depth, image_rgb, K, test3d2d);
                test3d2d->header.frame_id = "camera_depth_optical_frame";
                pcl::toROSMsg(*test3d2d, cloud_msg);
                color_pcl_pub_.publish(cloud_msg);
            }

             */
            // std::cout << depthBuff.data.size() << "  "  << image_rgb.size() << std::endl;
            if (depthBuff.data.size() > 0 && !image_rgb.empty() && !K.empty())
            {
                sensor_msgs::PointCloud2 cld_msg;
                mergedCloud = img2cloud(image_rgb, depthBuff);
                pcl::toROSMsg(*mergedCloud, cld_msg);
                cld_msg.header.frame_id = camera_color_frame;
                color_pcl_pub_.publish(cld_msg);
            }

            if (!image_depth.empty() && !image_rgb.empty() && !K.empty())
            {
                sensor_msgs::PointCloud2 cld_msg;
                cv::flip(image_depth, image_depth, 0);
                cv::flip(image_rgb, image_rgb, 0);
                pointCloudFrom2D(image_depth, image_rgb, K, mergedCloud);
                pcl::toROSMsg(*mergedCloud, cld_msg);
                cld_msg.header.frame_id = camera_color_frame;
                color_pcl_pub_.publish(cld_msg);
            }
        }
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr img2cloud(cv::Mat rgb, std_msgs::Float32MultiArray depth_msg)
    {
        std::vector<float> depth_raw = depth_msg.data;
        if (rgb.cols != v_res or rgb.rows != u_res or depth_raw.size() != u_res * v_res)
        {
            std::cout << "RGB size " << rgb.cols << ", " << rgb.rows << "   rgb and depth image size mismatch" << '\n';
            exit(0);
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        float cx = K.at<float>(0, 2);
        float cy = K.at<float>(1, 2);
        float fx_inv = 1.0 / K.at<float>(0, 0);
        float fy_inv = 1.0 / K.at<float>(1, 1);
        // ROS_server.cpp , old_ros_plugin
        unsigned int datalen = u_res * v_res;
        float scale = (far_clip - near_clip) / 1.0;
        std::vector<float> x_scale, y_scale;
        // float f = (std::max(u_res, v_res) / 2) / tan(0.785398 / 2); //(v_res / 2.) / tan(view_angle / 2.); //  45 degree
        float f = (std::max(u_res, v_res) / 2) / tan(degreesToRadians(view_angle) / 2);
        for (int j = 0; j < u_res; j++)
        {
            float y = (j - u_res / 2.0);
            for (int i = 0; i < v_res; i++)
            {
                int k = j * v_res + i;
                float x = -(i - v_res / 2.0);
                x_scale.push_back(x / f);
                y_scale.push_back(y / f);

                auto rgb_ints = rgb.at<cv::Vec3b>(j, i);

                float depth = near_clip + scale * depth_raw[k];
                float xyz[3] = {depth * x_scale[k], depth * y_scale[k], depth};
                pcl::PointXYZRGB p;
                p.x = xyz[0];
                p.y = xyz[1];
                p.z = xyz[2];
                p.r = (int)rgb_ints[0];
                p.g = (int)rgb_ints[1];
                p.b = (int)rgb_ints[2];
                cloud->points.push_back(p);
            }
        }
        cloud->width = cloud->points.size();
        cloud->height = 1;
        return cloud;
    }

    void merge(sensor_msgs::PointCloud2 &cloud, const std::vector<uint8_t> &colors)
    {

        size_t size = size_t(colors.size() / 3);
        size_t col = size_t(640);
        size_t row = size_t(480);
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud, std::string("r"));
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud, std::string("g"));
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud, std::string("b"));
        int count = 0;
        for (size_t j = 0; j < row; ++j)
        {
            for (size_t i = 0; i < col; ++i, ++iter_r, ++iter_g, ++iter_b)
            {
                count++;
                *iter_r = colors[3 * (i + (479 - j) * 640) + 0];
                *iter_g = colors[3 * (i + (479 - j) * 640) + 1];
                *iter_b = colors[3 * (i + (479 - j) * 640) + 2];
                // point cloud count from left to right, bottom to up while color image count from left to right, up to bottom
                // above conversion to force counting consistency of color with point cloud
            }
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "coppelia_point_cloud");
    ros::NodeHandle nh;

    PointCloudRGBCombiner node(nh);

    while (ros::ok())
    {
        node.update();
        ros::spinOnce();
    }

    return 0;
}
