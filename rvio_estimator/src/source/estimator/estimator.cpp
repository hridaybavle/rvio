#include "estimator/estimator.h"

estimator::estimator() : feature_manager_{Rs}
{

}


estimator::~estimator()
{

}


void estimator::initEstimator()
{
  input_img_cnt_ = 0;
  process_thread_ = std::thread(&estimator::processMeasurements, this);
}


void estimator::inputImages(double t, const cv::Mat &img0, const cv::Mat img1)
{
  input_img_cnt_++;
  std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> feature_frame;
  TicToc FeatureTrackerTime;

  if(img1.empty())
    feature_frame = feature_tracker_obj_.trackImage(t, img0);
  else
    feature_frame = feature_tracker_obj_.trackImage(t, img0, img1);

   if(input_img_cnt_ % 2 == 0)
   {
     img_buf_lock_.lock();
     feature_buf_.push(std::make_pair(t, feature_frame));
     img_buf_lock_.unlock();
   }
}

void estimator::processMeasurements()
{
    while(1)
    {
      std::cout << "processing measurements" << std::endl;
    }
}
