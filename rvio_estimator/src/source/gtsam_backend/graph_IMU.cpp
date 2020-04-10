#include "gtsam_backend/graph.h"
#include "estimator/parameters.h"

bool graph_solver::set_imu_preintegration(const gtsam::State& init_state) {

  // Create GTSAM preintegration parameters for use with Foster's version
  boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> params;
  params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(9.8);  // Z-up navigation frame: gravity points along negative Z-axis !!!
  
  params->setAccelerometerCovariance(gtsam::I_3x3 * ACC_N);  // acc white noise in continuous
  params->setGyroscopeCovariance(gtsam::I_3x3 * GYR_N);  // gyro white noise in continuous
  params->biasAccCovariance = ACC_W * gtsam::Matrix33::Identity(3,3);  // acc bias in continuous
  params->biasOmegaCovariance = GYR_W * gtsam::Matrix33::Identity(3,3);  // gyro bias in continuous
  params->setIntegrationCovariance(gtsam::I_3x3 * 0.1);  // error committed in integrating position from velocities
  params->biasAccOmegaInt = 1e-5*gtsam::Matrix66::Identity(6,6); // error in the bias used for preintegration
  
  // Actually create the GTSAM preintegration
  preint_gtsam = new gtsam::PreintegratedCombinedMeasurements(params, init_state.b());
  return true;
}

gtsam::CombinedImuFactor graph_solver::createIMUFactor(double update_time)
{
    imu_lock_.lock();
    while(acc_vec_.size() > 1 && acc_vec_.front().first <= update_time)
    {
        double dt = acc_vec_.at(1).first - acc_vec_.at(0).first;
        if(dt > 0)
        {
            Eigen::Vector3d meas_acc;
            Eigen::Vector3d meas_ang_vel;
            meas_acc        = acc_vec_.at(0).second;
            meas_ang_vel    = ang_vel_vec_.at(0).second;
            preint_gtsam->integrateMeasurement(meas_acc, meas_ang_vel, dt);
        }

        acc_vec_.erase(acc_vec_.begin());
        ang_vel_vec_.erase(ang_vel_vec_.begin());
    }

    double dt_f = update_time - acc_vec_.at(0).first;
    if(dt_f > 0)
    {
        Eigen::Vector3d meas_acc;
        Eigen::Vector3d meas_ang_vel;
        meas_acc        = acc_vec_.at(0).second;
        meas_ang_vel    = ang_vel_vec_.at(0).second;
        preint_gtsam->integrateMeasurement(meas_acc, meas_ang_vel, dt_f);
        acc_vec_.at(0).first = update_time;
        ang_vel_vec_.at(0).first = update_time;
    }
    imu_lock_.unlock();

    return gtsam::CombinedImuFactor(X(cur_sc_),  V(cur_sc_),
                                    X(cur_sc_+1), V(cur_sc_+1),
                                    B(cur_sc_), B(cur_sc_ +1), *preint_gtsam);
}

gtsam::State graph_solver::getPredictedState(gtsam::Values prev_values)
{
    // Get the current state (t=k)
    gtsam::State stateK = gtsam::State(prev_values.at<gtsam::Pose3>(X(cur_sc_)),
                                       prev_values.at<gtsam::Vector3>(V(cur_sc_)),
                                       prev_values.at<gtsam::Bias>(B(cur_sc_)));

    // From this we should predict where we will be at the next time (t=K+1)
    gtsam::NavState stateK1 = preint_gtsam->predict(gtsam::NavState(stateK.pose(), stateK.v()), stateK.b());
    return gtsam::State(stateK1.pose(), stateK1.v(), stateK.b());
}

