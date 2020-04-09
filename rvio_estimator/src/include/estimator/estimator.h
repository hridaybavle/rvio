#include <stdio.h>
#include <mutex>
#include <thread>
#include "eigen3/Eigen/Eigen"
#include <opencv2/opencv.hpp>

#include "parameters.h"
#include "utility/tic_toc.h"
#include "estimator/feature_manager.h"
#include "featureTracker/feature_tracker.h"

class estimator
{
public:

  estimator();
  ~estimator();

  void initEstimator();
  void inputImages(double t, const cv::Mat& img0, const cv::Mat img1);

protected:
  FeatureTracker feature_tracker_obj_;
  FeatureManager feature_manager_;

  std::thread process_thread_;
  std::queue<pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > > > feature_buf_;
  std::mutex img_buf_lock_;

private:
 Eigen::Matrix3d Rs[(WINDOW_SIZE + 1)];


private:
  void processMeasurements();
  int input_img_cnt_;

};
