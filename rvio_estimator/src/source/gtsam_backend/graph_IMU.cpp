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

