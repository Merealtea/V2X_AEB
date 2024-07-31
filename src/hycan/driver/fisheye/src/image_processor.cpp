#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <hycan_msgs/FourImages.h>  // 自定义消息

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage,
                                                         sensor_msgs::CompressedImage, 
                                                         sensor_msgs::CompressedImage, 
                                                         sensor_msgs::CompressedImage,
                                                         sensor_msgs::Imu,
                                                         sensor_msgs::NavSatFix
                                                         > MySyncPolicy;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage,
                                                         sensor_msgs::CompressedImage, 
                                                         sensor_msgs::CompressedImage, 
                                                         sensor_msgs::CompressedImage> MySyncPolicy_test;

class ImageProcessor
{
public:
    ImageProcessor() : it_(nh_)
    {
        // 订阅四个压缩图像话题
        sub1_.subscribe(nh_, "/miivii_gmsl_ros/camera1/compressed", 1);
        sub2_.subscribe(nh_, "/miivii_gmsl_ros/camera2/compressed", 1);
        sub3_.subscribe(nh_, "/miivii_gmsl_ros/camera3/compressed", 1);
        sub4_.subscribe(nh_, "/miivii_gmsl_ros/camera4/compressed", 1);

        // 订阅定位话题
        sub5_.subscribe(nh_,"/wit/imu",1);
        sub6_.subscribe(nh_,"/wit/gps_fix",1);

        // 同步策略
        // sync_.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), sub1_, sub2_, sub3_, sub4_, sub5_, sub6_));
        // sync_->registerCallback(boost::bind(&ImageProcessor::imageCallback, this, _1, _2, _3, _4, _5, _6));

        // 同步策略
        sync_test_.reset(new message_filters::Synchronizer<MySyncPolicy_test>(MySyncPolicy_test(10), sub1_, sub2_, sub3_, sub4_));
        sync_test_->registerCallback(boost::bind(&ImageProcessor::imageCallback_test, this, _1, _2, _3, _4));

        // 发布自定义消息
        pub_ = nh_.advertise<hycan_msgs::FourImages>("hycan_processed_images", 1);
    }

    void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg1, // front
                        const sensor_msgs::CompressedImageConstPtr& msg2, // back
                          const sensor_msgs::CompressedImageConstPtr& msg3, // right
                           const sensor_msgs::CompressedImageConstPtr& msg4, // left
                            const sensor_msgs::ImuConstPtr& msg5, // imu
                                const sensor_msgs::NavSatFixConstPtr& msg6) // gps
    {
        std::vector<cv::Mat> images(4);
        std::vector<sensor_msgs::Image> out_msgs(4);
        std::vector<sensor_msgs::CompressedImageConstPtr> msgs = {msg1, msg2, msg3, msg4};

        // 处理每个压缩图像
        for (int i = 0; i < 4; i++) {
            cv::Mat cv_image = cv::imdecode(cv::Mat(msgs[i]->data), cv::IMREAD_COLOR);

            // 1. 先resize
            cv::resize(cv_image, images[i], cv::Size(640, 480));  // 缩放到640x480

            // 2. 归一化
            // TODO(cxy)

            cv_bridge::CvImage out_cv_image;
            out_cv_image.header = msgs[i]->header;
            out_cv_image.encoding = "bgr8";
            out_cv_image.image = images[i];
            out_msgs[i] = *out_cv_image.toImageMsg();
        }

        // 发布自定义消息
        hycan_msgs::FourImages out_image_msg;
        out_image_msg.header.stamp = ros::Time::now();
        out_image_msg.image_front = out_msgs[0];
        out_image_msg.image_back = out_msgs[1];
        out_image_msg.image_left = out_msgs[2];
        out_image_msg.image_right = out_msgs[3];
        out_image_msg.imu = *msg5;
        out_image_msg.gps = *msg6;
        pub_.publish(out_image_msg);
    }

    void imageCallback_test(const sensor_msgs::CompressedImageConstPtr& msg1, // front
                        const sensor_msgs::CompressedImageConstPtr& msg2, // back
                          const sensor_msgs::CompressedImageConstPtr& msg3, // right
                           const sensor_msgs::CompressedImageConstPtr& msg4) // left
    {
        std::vector<cv::Mat> images(4);
        std::vector<sensor_msgs::Image> out_msgs(4);
        std::vector<sensor_msgs::CompressedImageConstPtr> msgs = {msg1, msg2, msg3, msg4};

        // 处理每个压缩图像
        for (int i = 0; i < 4; i++) {
            cv::Mat cv_image = cv::imdecode(cv::Mat(msgs[i]->data), cv::IMREAD_COLOR);

            // 1. 先resize
            cv::resize(cv_image, images[i], cv::Size(640, 480));  // 缩放到640x480

            // 2. 归一化
            // TODO(cxy)

            cv_bridge::CvImage out_cv_image;
            out_cv_image.header = msgs[i]->header;
            out_cv_image.encoding = "bgr8";
            out_cv_image.image = images[i];
            out_msgs[i] = *out_cv_image.toImageMsg();
        }

        // 发布自定义消息
        hycan_msgs::FourImages out_image_msg;
        out_image_msg.image_front = out_msgs[0];
        out_image_msg.image_back = out_msgs[1];
        out_image_msg.image_left = out_msgs[2];
        out_image_msg.image_right = out_msgs[3];
        pub_.publish(out_image_msg);

        // 打印消息
        ROS_INFO("Processed and published images.");
    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    message_filters::Subscriber<sensor_msgs::CompressedImage> sub1_, sub2_, sub3_, sub4_;
    message_filters::Subscriber<sensor_msgs::Imu> sub5_;
    message_filters::Subscriber<sensor_msgs::NavSatFix> sub6_;
    boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
    boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy_test>> sync_test_;
    ros::Publisher pub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hycan_image_processor");
    ImageProcessor ip;
    ros::spin();
    return 0;
}
