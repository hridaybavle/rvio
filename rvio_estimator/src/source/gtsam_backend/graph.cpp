#include "gtsam_backend/graph.h"


graph_solver::graph_solver()
{
    this->graph = new gtsam::NonlinearFactorGraph();
    // ISAM2 solver
    gtsam::ISAM2Params isam_params;
    isam_params.relinearizeThreshold = 0.01;
    isam_params.relinearizeSkip = 1;
    isam_params.cacheLinearizedFactors = false;
    isam_params.enableDetailedResults = true;
    isam_params.print();
    this->isam2 = new gtsam::ISAM2(isam_params);
}


graph_solver::~graph_solver()
{

}

void graph_solver::initVariables()
{
    cur_sc_ = 0;
}

void graph_solver::initialize(Eigen::Vector3d Ps, Eigen::Matrix3d Rs, Eigen::Vector3d Vs,
                              Eigen::Vector3d Bas, Eigen::Vector3d Bgs)
{

    if(system_initializied_)
        return;

    initVariables();

    Eigen::Quaterniond q(Rs);
    gtsam::State init_state = gtsam::State(gtsam::Pose3(gtsam::Quaternion(q.w(), q.x(), q.y(),q.z()), gtsam::Vector3(Ps)),
                                           gtsam::Vector3(Vs), gtsam::Bias(gtsam::Vector3(Bas), gtsam::Vector3(Bgs)));


    std::cout << "initial rotation for gtsam:" << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << std::endl;

    auto pose_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(1e-4),
                                                           gtsam::Vector3::Constant(1e-4)).finished());

    auto v_noise = gtsam::noiseModel::Isotropic::Sigma(3, 1e-4);
    auto b_noise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-4);

    //create graph
    graph->add(gtsam::PriorFactor<gtsam::Pose3>(X(cur_sc_), init_state.pose(), pose_noise));
    graph->add(gtsam::PriorFactor<gtsam::Vector3>(V(cur_sc_), init_state.v(),  v_noise));
    graph->add(gtsam::PriorFactor<gtsam::Bias>(   B(cur_sc_), init_state.b(), b_noise));


    //add graph values
    // Add initial state to the graph
    values_curr_.insert(    X(cur_sc_), init_state.pose());
    values_curr_.insert(    V(cur_sc_), init_state.v());
    values_curr_.insert(    B(cur_sc_), init_state.b());
    values_prev_.insert(X(cur_sc_), init_state.pose());
    values_prev_.insert(V(cur_sc_), init_state.v());
    values_prev_.insert(B(cur_sc_), init_state.b());

    //    cur_sc_lookup_[t] = cur_sc_;
    //    timestamp_lookup_[cur_sc_] = t;

    if(set_imu_preintegration(init_state))
        system_initializied_ = true;

    return;
}
