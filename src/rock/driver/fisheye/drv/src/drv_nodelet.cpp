#include <thread>

#include <boost/endian/conversion.hpp>
#include <boost/smart_ptr/make_shared_object.hpp>
#include <nodelet/nodelet.h>
#include <opencv2/videoio.hpp>

#include <image_transport/image_transport.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>

class drv_nodelet : public nodelet::Nodelet {
 public:
 private:
  virtual void onInit() override;

  virtual ~drv_nodelet() override;

  void run();

 private:
  struct camera {
    std::string topic;
    int number;
    image_transport::Publisher pub;
    cv::VideoCapture cap;
    cv::Mat img;
    //std_msgs::Header header;
    sensor_msgs::ImagePtr msg;

    void reset_msg() {
      msg = boost::make_shared<sensor_msgs::Image>();
      msg->height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
      msg->width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
      msg->encoding = "bgr8";
      msg->is_bigendian =
          (boost::endian::order::native == boost::endian::order::big);
      msg->step = msg->width * 3;
      msg->data.resize(msg->width * msg->height * 3);
      img = cv::Mat(msg->height, msg->width, CV_8UC3, msg->data.data(),
                    msg->step);
    }
  };

  // camera back;
  camera front;
  // camera left;
  // camera right;

  std::thread work_thread;
  bool stop_flag = false;
};

#define PARAM(name, var)                                     \
  do {                                                       \
    if (!nhp.getParam(name, var)) {                          \
      NODELET_ERROR_STREAM("missing parameter '" #name "'"); \
      return;                                                \
    }                                                        \
  } while (0)

void drv_nodelet::onInit() {
  NODELET_INFO_STREAM(__PRETTY_FUNCTION__);

  auto &nhp = getMTPrivateNodeHandle();

  // PARAM("back_topic", back.topic);
  PARAM("front_topic", front.topic);
  // PARAM("left_topic", left.topic);
  // PARAM("right_topic", right.topic);
  // PARAM("back_number", back.number);
  PARAM("front_number", front.number);
  // PARAM("left_number", left.number);
  // PARAM("right_number", right.number);

  image_transport::ImageTransport it(nhp);
  // back.pub = it.advertise(back.topic, 1);
  front.pub = it.advertise(front.topic, 1);
  // left.pub = it.advertise(left.topic, 1);
  // right.pub = it.advertise(right.topic, 1);

  // back.cap.open(back.number);
  front.cap.open(front.number);
  // left.cap.open(left.number);
  // right.cap.open(right.number);

  work_thread = std::thread(&drv_nodelet::run, this);
}

drv_nodelet::~drv_nodelet() {
  stop_flag = true;
  if (work_thread.joinable()) {
    work_thread.join();
  }
}

void drv_nodelet::run() {
  NODELET_INFO_STREAM(__PRETTY_FUNCTION__);
  while (!stop_flag) {
    // <------ prepare buffer ------>
    // back.reset_msg();
    front.reset_msg();
    // left.reset_msg();
    // right.reset_msg();
    // <------ grab and set timestamp ------>
    // back.cap.grab();
    // back.msg->header.stamp = ros::Time::now();
    // back.msg->header.seq++;
    front.cap.grab();
    front.msg->header.stamp = ros::Time::now();
    front.msg->header.seq++;
    // left.cap.grab();
    // left.msg->header.stamp = ros::Time::now();
    // left.msg->header.seq++;
    // right.cap.grab();
    // right.msg->header.stamp = ros::Time::now();
    // right.msg->header.seq++;
    // <------ retrieve image ------>
    // back.cap.retrieve(back.img);
    front.cap.retrieve(front.img);
    // left.cap.retrieve(left.img);
    // right.cap.retrieve(right.img);
    // <------ publish message ------>
    // ROS_ASSERT(back.img.ptr() == back.msg->data.data());
    // back.pub.publish(back.msg);
    ROS_ASSERT(front.img.ptr() == front.msg->data.data());
    front.pub.publish(front.msg);
    // ROS_ASSERT(left.img.ptr() == left.msg->data.data());
    // left.pub.publish(left.msg);
    // ROS_ASSERT(right.img.ptr() == right.msg->data.data());
    // right.pub.publish(right.msg);
  }
}

PLUGINLIB_EXPORT_CLASS(drv_nodelet, nodelet::Nodelet);
