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
  prev_time = -1;
  cur_time = 0;
  init_first_pose_flag_ = false;

  process_lock_.lock();
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
    buf_lock_.lock();
    feature_buf_.push(std::make_pair(t, feature_frame));
    buf_lock_.unlock();
  }
}

void estimator::inputIMU(double t, const Eigen::Vector3d linear_acc, const Eigen::Vector3d ang_vel)
{
  buf_lock_.lock();
  acc_buf_.push(make_pair(t, linear_acc));
  ang_vel_buf_.push(make_pair(t, ang_vel));
  buf_lock_.unlock();
}

void estimator::processMeasurements()
{
  while(1)
  {
    pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > > feature;
    vector<pair<double, Eigen::Vector3d>> acc_vector, ang_vel_vector;

    if(!feature_buf_.empty())
    {
      feature  = feature_buf_.front();
      cur_time = feature.first + time_delay_;

      while(1)
      {
        if((!USE_IMU) || IMUAvailable(feature.first + time_delay_))
        {
          break;
        }
        else {
          printf("wait for imu \n");
          std::chrono::milliseconds dura(5);
          std::this_thread::sleep_for(dura);
        }
      }
      buf_lock_.lock();
      if(USE_IMU){
        getIMUInterval(prev_time, cur_time, acc_vector, ang_vel_vector);
      }
      feature_buf_.pop();
      buf_lock_.unlock();

    }

    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }
}

bool estimator::IMUAvailable(double t)
{
  if(!acc_buf_.empty() && t <= acc_buf_.back().first)
    return true;
  else
    return false;
}

bool estimator::getIMUInterval(double t0, double t1, std::vector<pair<double, Eigen::Vector3d> > &acc_vector, std::vector<pair<double, Eigen::Vector3d> > &ang_vel_vector)
{

  if(acc_buf_.empty())
  {
    printf("imu data empty");
    return false;
  }

  if(t1 <= acc_buf_.back().first)
  {
    while(acc_buf_.front().first <= t0)
    {
      acc_buf_.pop();
      ang_vel_buf_.pop();
    }
    while(acc_buf_.front().first < t1)
    {
      acc_vector.push_back(acc_buf_.front());
      acc_buf_.pop();
      ang_vel_vector.push_back(ang_vel_buf_.front());
      ang_vel_buf_.pop();
    }
    acc_vector.push_back(acc_buf_.front());
    ang_vel_vector.push_back(ang_vel_buf_.front());
  }
  else {
    printf("wait for imu in interval\n");
    return false;
  }

  return true;

}

void estimator::initFirstIMUPose(std::vector<pair<double, Eigen::Vector3d> > &acc_vector)
{
    printf("init first imu pose\n");
    init_first_pose_flag_ = true;
    Eigen::Vector3d avg_acc(0,0,0);
    int n = (int) acc_vector.size();

    for(size_t i = 0; i < acc_vector.size(); i++)
    {
      avg_acc = avg_acc + acc_vector[i].second;
    }
    avg_acc = avg_acc / n;
    printf("averge acc %f %f %f\n", avg_acc.x(), avg_acc.y(), avg_acc.z());
    Matrix3d R0 = Utility::g2R(avg_acc);
    double yaw = Utility::R2ypr(R0).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    Rs[0] = R0;
}
