#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <hycan_msgs/Localization.h>
#include "common.hpp"
#include <string.h>

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
        // 获取shm
        std::string shm_name = "hycan_image_shm";
        allocate_shm(shm_name, sizeof(IMAGE_ELEMENT) * 4, shm_image_ptr);

        if (shm_image_ptr == nullptr)
        {
            ROS_ERROR("Failed to allocate shared memory");
            exit(1);
        }
        resized_img = new IMAGE_ELEMENT[4];

        // 订阅四个压缩图像话题
        sub1_.subscribe(nh_, "/miivii_gmsl_ros/camera4/compressed", 1);
        sub2_.subscribe(nh_, "/miivii_gmsl_ros/camera3/compressed", 1);
        sub3_.subscribe(nh_, "/miivii_gmsl_ros/camera1/compressed", 1);
        sub4_.subscribe(nh_, "/miivii_gmsl_ros/camera2/compressed", 1);
        sub5_.subscribe(nh_, "hycan_utm_localization", 1);

        // 同步策略
        sync_.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(50), sub1_, sub2_, sub3_, sub4_, sub5_));
        sync_->registerCallback(boost::bind(&ImageProcessor::imageCallback, this, _1, _2, _3, _4, _5));
        sync_->setMaxIntervalDuration(ros::Duration(0.1));  // 设置最大时间间隔
        // 发布自定义消息
        pub_ = nh_.advertise<hycan_msgs::Localization>("hycan_processed_images", 1);
        // print opencv version
        ROS_INFO("OpenCV version: %s", CV_VERSION);
    }

    void imageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg1, // left
                        const sensor_msgs::CompressedImage::ConstPtr& msg2, // right
                          const sensor_msgs::CompressedImage::ConstPtr& msg3, // front
                           const sensor_msgs::CompressedImage::ConstPtr& msg4,  // back
                            const hycan_msgs::Localization::ConstPtr& msg5)
    {
        if(is_odd)
        {
            is_odd = false;
            return;
        }
        is_odd = true;
        // 只有在gps有的情况下才更新消息
        std::vector<sensor_msgs::CompressedImageConstPtr> msgs = {msg1, msg2, msg3, msg4};

        int original_width;
        int original_height;
        int new_width;
        int new_height;
        int pad_width;
        int pad_height;
        float_t ratio;

        ros::Time st_time = ros::Time::now();

        double time_diff = (msg1->header.stamp - last_time_).toSec();
        last_time_ = msg1->header.stamp;
        ROS_INFO("FPS in image processor is %f", 1.0 / time_diff);

        // 处理每个压缩图像
        for (int i = 0; i < 4; i++) {
            cv::Mat cv_image = cv::imdecode(cv::Mat(msgs[i]->data), cv::IMREAD_COLOR);
            original_height = cv_image.rows;
            original_width = cv_image.cols;

            ratio = (float)(target_width_ / original_width) > (float)(target_height_ / original_height)
             ? (float)target_width_ / original_width : (float)target_height_ / original_height; 

            new_width = original_width * ratio;
            new_height = original_height * ratio;

            // 1. 先resize
            cv::resize(cv_image, cv_image, cv::Size(new_width, new_height), 0, 0, cv::INTER_LINEAR);  // 缩放到640x480
            
            // Check the new width and height is the multiple of pad_size
            pad_width = (new_width + pad_size_ - 1) / pad_size_ * pad_size_ - new_width;
            pad_height = (new_height + pad_size_ - 1) / pad_size_ * pad_size_ - new_height;

            // 2. 填充图像
            cv::copyMakeBorder(cv_image, cv_image, 0, pad_height, 0, pad_width, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

            // store the image to shared memory
            resized_img[i].camera_id = i;
            resized_img[i].frame_idx = frame_idx;
            resized_img[i].width = cv_image.cols;
            resized_img[i].height = cv_image.rows;
            resized_img[i].num_channel = 3;
            resized_img[i].width_no_pad = new_width;
            resized_img[i].height_no_pad = new_height;
            resized_img[i].original_width = original_width;
            resized_img[i].original_height = original_height;
            resized_img[i].ratio = ratio;
            resized_img[i].stamp = st_time.toSec(); // msgs[i]->header.stamp.toSec(); // DEBUG

            memcpy(&resized_img[i].data, cv_image.data, RESIZED_IMG_SIZE);
        }
        ROS_INFO("Original Image Size: %d x %d", original_width, original_height);
        ROS_INFO("New Image Size: %d x %d", new_width, new_height);
        ROS_INFO("Pad Size: %d x %d", pad_width, pad_height);
        ROS_INFO("Image: %d x %d",  resized_img[0].width, resized_img[0].height);
        ROS_INFO("Timestamp is %f", resized_img[0].stamp);
        frame_idx++;

        // 将处理后的图像存入共享内存
        ROS_INFO("Writing to shared memory");
        memcpy(shm_image_ptr, resized_img, sizeof(IMAGE_ELEMENT) * 4);

        // 发布自定义消息
        pub_.publish(*msg5);

        // 打印消息
        ROS_INFO("Processed and published images. Processed Time: %f", ros::Time::now().toSec() - st_time.toSec());
    }

private:
    ros::NodeHandle nh_;
    message_filters::Subscriber<sensor_msgs::CompressedImage> sub1_, sub2_, sub3_, sub4_;
    message_filters::Subscriber<hycan_msgs::Localization> sub5_;
    boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

    int frame_idx = 0;
    
    ros::Publisher pub_;
    int target_width_ = 360, target_height_ = 360;
    int pad_size_ = 16;

    ros::Time last_time_ = ros::Time::now();

    IMAGE_ELEMENT* shm_image_ptr = nullptr;
    IMAGE_ELEMENT* resized_img;

    bool is_odd = true;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "hycan_image_processor");
    ImageProcessor ip;
    ros::spin();
    return 0;
}