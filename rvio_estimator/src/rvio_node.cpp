#include <stdio.h>
#include <thread>
#include <mutex>
#include <queue>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "estimator/estimator.h"

class rvio_ros_node
{

public:

  rvio_ros_node(){}
  ~rvio_ros_node(){}

  void open(ros::NodeHandle n);
  void processImages();
  cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg);

  estimator estimator_obj_;

protected:
  ros::Subscriber img1_sub_, img2_sub_;
  void img1Callback(const sensor_msgs::ImageConstPtr& img_msg);
  void img2Callback(const sensor_msgs::ImageConstPtr& img_msg);

  std::mutex img_buf_lock_;
  std::queue<sensor_msgs::ImageConstPtr> img0_buf_;
  std::queue<sensor_msgs::ImageConstPtr> img1_buf_;

};

cv::Mat rvio_ros_node::getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
  cv_bridge::CvImageConstPtr ptr;
  if (img_msg->encoding == "8UC1")
  {
    sensor_msgs::Image img;
    img.header = img_msg->header;
    img.height = img_msg->height;
    img.width = img_msg->width;
    img.is_bigendian = img_msg->is_bigendian;
    img.step = img_msg->step;
    img.data = img_msg->data;
    img.encoding = "mono8";
    ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
  }
  else
    ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

  cv::Mat img = ptr->image.clone();
  return img;
}

void rvio_ros_node::open(ros::NodeHandle n)
{
  estimator_obj_.initEstimator();

  img1_sub_ = n.subscribe("image1_topic", 100, &rvio_ros_node::img1Callback, this);
  img2_sub_ = n.subscribe("image2_topic", 100, &rvio_ros_node::img2Callback, this);
}

void rvio_ros_node::img1Callback(const sensor_msgs::ImageConstPtr &img_msg)
{
  img_buf_lock_.lock();
  img0_buf_.push(img_msg);
  img_buf_lock_.unlock();

  return;
}

void rvio_ros_node::img2Callback(const sensor_msgs::ImageConstPtr &img_msg)
{

  img_buf_lock_.lock();
  img1_buf_.push(img_msg);
  img_buf_lock_.unlock();

  return;
}

void rvio_ros_node::processImages()
{
  while(1)
  {

    cv::Mat image0, image1;
    std_msgs::Header header;
    double time = 0;
    img_buf_lock_.lock();
    if(!img0_buf_.empty() && !img1_buf_.empty())
    {
      double time0 = img0_buf_.front()->header.stamp.toSec();
      double time1 = img1_buf_.front()->header.stamp.toSec();

      //tolerance between the images
      if(time0 < time1 - 0.003)
      {
        img0_buf_.pop();
        printf("throw img0\n");

      }
      else if( time0 > time1 + 0.003)
      {
        img1_buf_.pop();
        printf("throw img1\n");
      }
      else
      {
        time   = img0_buf_.front()->header.stamp.toSec();
        header = img0_buf_.front()->header;
        image0 = getImageFromMsg(img0_buf_.front());
        img0_buf_.pop();
        image1  = getImageFromMsg(img1_buf_.front());
        img1_buf_.pop();
      }
    }
    img_buf_lock_.unlock();
    if(!image0.empty())
    {
      estimator_obj_.inputImages(time, image0, image1);
    }

    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "rvio_estimator_node");
  ros::NodeHandle n;

  rvio_ros_node * rvio_ros_node_ptr_ = new rvio_ros_node();
  rvio_ros_node_ptr_->open(n);
  std::thread img_th(&rvio_ros_node::processImages, rvio_ros_node_ptr_);

  ros::spin();

  return 0;

}
