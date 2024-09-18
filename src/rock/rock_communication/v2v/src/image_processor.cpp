#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <hycan_msgs/FourImages.h>  // 自定义消息
#include <hycan_msgs/Localization.h>
#include <hycan_msgs/Image.h>


typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage,
                                                         sensor_msgs::CompressedImage, 
                                                         sensor_msgs::CompressedImage, 
                                                         sensor_msgs::CompressedImage,
                                                         hycan_msgs::Localization> MySyncPolicy;

class ImageProcessor
{
public:
    ImageProcessor() 
    {
        // 订阅四个压缩图像话题
        sub1_.subscribe(nh_, "/driver/fisheye/front/compressed", 1);
        sub2_.subscribe(nh_, "/driver/fisheye/back/compressed", 1);
        sub3_.subscribe(nh_, "/driver/fisheye/right/compressed", 1);
        sub4_.subscribe(nh_, "/driver/fisheye/left/compressed", 1);
        sub5_.subscribe(nh_, "rock_utm_localization", 1);

        // 同步策略
        sync_.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(50), sub1_, sub2_, sub3_, sub4_, sub5_));
        sync_->registerCallback(boost::bind(&ImageProcessor::imageCallback, this, _1, _2, _3, _4, _5));
        sync_->setMaxIntervalDuration(ros::Duration(0.1));  // 设置最大时间间隔
        // 发布自定义消息
        pub_ = nh_.advertise<hycan_msgs::FourImages>("rock_processed_images", 1);
    }

    void imageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg1, // front
                        const sensor_msgs::CompressedImage::ConstPtr& msg2, // back
                          const sensor_msgs::CompressedImage::ConstPtr& msg3, // right
                           const sensor_msgs::CompressedImage::ConstPtr& msg4,  // left
                            const hycan_msgs::Localization::ConstPtr& msg5)
    {
        // 只有在gps有的情况下才更新消息
        std::vector<hycan_msgs::Image> out_msgs(4);
        std::vector<sensor_msgs::CompressedImageConstPtr> msgs = {msg1, msg2, msg3, msg4};

        int original_width;
        int original_height;

        // 处理每个压缩图像
        for (int i = 0; i < 4; i++) {
            cv::Mat cv_image = cv::imdecode(cv::Mat(msgs[i]->data), cv::IMREAD_COLOR);
            original_width = cv_image.rows;
            original_height = cv_image.cols;

            // 转换图像为浮点类型，便于归一化操作
            cv_image.convertTo(cv_image, CV_32F);

            // 1. 先resize
            cv::resize(cv_image, cv_image, cv::Size(640, 480));  // 缩放到640x480

            // 2. 归一化
            cv::Mat mean_mat(cv_image.size(), cv_image.type(), img_mean);
            cv::Mat std_mat(cv_image.size(), cv_image.type(), img_std);

            // 图像减去均值
            cv::subtract(cv_image, mean_mat, cv_image);

            // 图像除以方差
            cv::divide(cv_image, std_mat, cv_image);

            out_msgs[i].header = msgs[i]->header; 
            out_msgs[i].height = cv_image.rows;
            out_msgs[i].width = cv_image.cols;

            cv_image = cv_image.reshape(1, cv_image.total() * cv_image.channels());
            out_msgs[i].data.assign((float*)cv_image.data, (float*)cv_image.data + cv_image.total());

        }

        // 发布自定义消息
        hycan_msgs::FourImages out_image_msg;
        out_image_msg.image_front = out_msgs[0];
        out_image_msg.image_back = out_msgs[1];
        out_image_msg.image_left = out_msgs[2];
        out_image_msg.image_right = out_msgs[3];

        out_image_msg.height = original_height;
        out_image_msg.width = original_width;

        out_image_msg.localization = *msg5;
        pub_.publish(out_image_msg);

        // 打印消息
        ROS_INFO("Processed and published images.");
    }

private:
    ros::NodeHandle nh_;
    message_filters::Subscriber<sensor_msgs::CompressedImage> sub1_, sub2_, sub3_, sub4_;
    message_filters::Subscriber<hycan_msgs::Localization> sub5_;
    boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
    
    // 定义均值和方差，分别对应 BGR 颜色通道
    cv::Scalar img_mean = cv::Scalar(123.675, 116.28, 103.53);
    cv::Scalar img_std = cv::Scalar(58.395, 57.12, 57.375);

    ros::Publisher pub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rock_image_processor");
    ImageProcessor ip;
    ros::spin();
    return 0;
}