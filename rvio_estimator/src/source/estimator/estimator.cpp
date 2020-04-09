#include "estimator/estimator.h"

estimator::estimator() : feature_manager_{Rs}
{

}


estimator::~estimator()
{

}

void estimator::initEstimator()
{
  process_lock_.lock();
  input_img_cnt_ = 0;

   for (int i = 0; i < NUM_OF_CAM; i++)
  {
    tic[i] = TIC[i];
    ric[i] = RIC[i];
    std::cout << " exitrinsic cam " << i << endl  << ric[i] << endl << tic[i].transpose() << endl;
  }

  feature_manager_.setRic(ric);
  time_delay_ = TD;
  g_ = G;

  feature_tracker_obj_.readIntrinsicParameter(CAM_NAMES);
  process_thread_ = std::thread(&estimator::processMeasurements, this);
  process_lock_.unlock();
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

  if(SHOW_TRACK)
  {
    cv::Mat tracked_image = feature_tracker_obj_.getTrackImage();
    pubTrackImage(tracked_image, t);
  }

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
      std::chrono::milliseconds dura(2);
      std::this_thread::sleep_for(dura);
    }
}
