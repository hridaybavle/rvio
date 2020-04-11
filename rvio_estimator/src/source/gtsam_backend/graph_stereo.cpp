#include "gtsam_backend/graph.h"
#include "estimator/parameters.h"

void graph_solver::addStereoMeas(int f_id, Eigen::Vector2d point0, Eigen::Vector2d point1, Eigen::Vector3d point3d)
{

    if(!system_initializied_ || cur_sc_ == 0)
        return;

    const gtsam::noiseModel::Isotropic::shared_ptr model = gtsam::noiseModel::Isotropic::Sigma(3,1);

    //if landmark doesnt exist add the node
    if(!values_prev_.exists(L(f_id)))
    {
        gtsam::Pose3 cam_pose     = values_curr_.at<gtsam::Pose3>(X(cur_sc_));
        gtsam::Point3 world_point = cam_pose.transform_from(gtsam::Point3(point3d(0),point3d(1),point3d(2)));
        values_curr_.insert(L(f_id), world_point);
        values_prev_.insert(L(f_id), world_point);
    }

    double uL, uR, v;
    uL = point0(0);
    uR = point1(0);
    v  = point0(1);

    //    auto gaussian = gtsam::noiseModel::Isotropic::Sigma(3, 1.0);
    //    gtsam::SmartProjectionParams params(gtsam::HESSIAN, gtsam::ZERO_ON_DEGENERACY);

    //    gtsam::SmartStereoProjectionPoseFactor::shared_ptr smart_factor = gtsam::SmartStereoProjectionPoseFactor::shared_ptr(
    //                new gtsam::SmartStereoProjectionPoseFactor(gaussian, params));

    //    smart_factor->add(gtsam::StereoPoint2(uL, uR, v), X(cur_sc_), K);
    //    graph->add(smart_factor);

    graph->push_back(gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>(gtsam::StereoPoint2(uL, uR, v),
                                                                             model, X(cur_sc_), L(f_id), K));
}
