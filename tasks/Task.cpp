/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

#ifndef D2R
#define D2R M_PI/180.00 /** Convert degree to radian **/
#endif
#ifndef R2D
#define R2D 180.00/M_PI /** Convert radian to degree **/
#endif

#define DEBUG_PRINTS 1

using namespace shark_slam;

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}

void Task::gps_pose_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &gps_pose_samples_sample)
{
    #ifdef DEBUG_PRINTS
    RTT::log(RTT::Warning)<<"[SHARK_SLAM GPS_POSE_SAMPLES] Received time-stamp: "<<gps_pose_samples_sample.time.toMicroseconds()<<RTT::endlog();
    #endif

    Eigen::Affine3d tf_body_gps; /** Transformer transformation **/
    /** Get the transformation Tbody_sensor **/
    if (_gps_frame.value().compare(_body_frame.value()) == 0)
    {
        tf_body_gps.setIdentity();
    }
    else if (!_gps2body.get(ts, tf_body_gps, false))
    {
        RTT::log(RTT::Fatal)<<"[SHARK_SLAM FATAL ERROR] No transformation provided."<<RTT::endlog();
        return;
    }


    /** New GPS sample: increase index **/
    this->idx++;

    /** Add ImuFactor **/
    gtsam::PreintegratedImuMeasurements *preint_imu = dynamic_cast<gtsam::PreintegratedImuMeasurements*>(this->imu_preintegrated.get());

    gtsam::ImuFactor imu_factor(X(this->idx-1), V(this->idx-1),
                           X(this->idx), V(this->idx),
                           B(this->idx-1),
                           *preint_imu);
    this->factor_graph->add(imu_factor);

    gtsam::imuBias::ConstantBias zero_bias(gtsam::Vector3(0, 0, 0), gtsam::Vector3(0, 0, 0));
    this->factor_graph->add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(this->idx-1),
                                                                            B(this->idx),
                                                                            zero_bias, this->bias_noise_model));

    /** Add GPS factor **/
    gtsam::noiseModel::Diagonal::shared_ptr correction_noise = gtsam::noiseModel::Isotropic::Sigma(3,1.0);
    //gtsam::GPSFactor gps_factor(X(this->idx),gtsam::Point3(gps_pose_samples_sample.position), correction_noise);
    gtsam::GPSFactor gps_factor(X(this->idx),gtsam::Point3(Eigen::Vector3d::Zero()), correction_noise);
    this->factor_graph->add(gps_factor);

    /** Optimize **/
    this->optimize();

    /** Output the result **/
    base::samples::RigidBodyState output_pose;
    output_pose.time = gps_pose_samples_sample.time;
    output_pose.position = this->prev_state.pose().translation();
    output_pose.orientation = this->prev_state.pose().rotation().toQuaternion();
    output_pose.velocity = this->prev_state.velocity();
    _pose_samples_out.write(output_pose);

}

void Task::imu_samplesTransformerCallback(const base::Time &ts, const ::base::samples::IMUSensors &imu_samples_sample)
{
    #ifdef DEBUG_PRINTS
    RTT::log(RTT::Warning)<<"[SHARK_SLAM IMU_SAMPLES] Received time-stamp: "<<imu_samples_sample.time.toMicroseconds()<<RTT::endlog();
    #endif

    Eigen::Affine3d tf_body_imu; /** Transformer transformation **/
    /** Get the transformation Tbody_sensor **/
    if (_imu_frame.value().compare(_body_frame.value()) == 0)
    {
        tf_body_imu.setIdentity();
    }
    else if (!_imu2body.get(ts, tf_body_imu, false))
    {
        RTT::log(RTT::Fatal)<<"[SHARK_SLAM FATAL ERROR] No transformation provided."<<RTT::endlog();
        return;
    }


    /** Integrate the IMU samples in the preintegration **/
    this->imu_preintegrated->integrateMeasurement(imu_samples_sample.acc, imu_samples_sample.gyro, _imu_samples_period.value());
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    /***************************/
    /**    IMU Noise Init     **/
    /***************************/

    /** TODO: change from config values **/
    double accel_noise_sigma = 0.0003924;
    double gyro_noise_sigma = 0.000205689024915;
    double accel_bias_rw_sigma = 0.004905;
    double gyro_bias_rw_sigma = 0.000001454441043;

    /** Covariance matrices **/
    gtsam::Matrix33 measured_acc_cov = gtsam::Matrix33::Identity(3,3) * pow(accel_noise_sigma,2);
    gtsam::Matrix33 measured_omega_cov = gtsam::Matrix33::Identity(3,3) * pow(gyro_noise_sigma,2);
    gtsam::Matrix33 integration_error_cov = gtsam::Matrix33::Identity(3,3) * 1e-8; // error committed in integrating position from velocities
    gtsam::Matrix33 bias_acc_cov = gtsam::Matrix33::Identity(3,3) * pow(accel_bias_rw_sigma,2);
    gtsam::Matrix33 bias_omega_cov = gtsam::Matrix33::Identity(3,3) * pow(gyro_bias_rw_sigma,2);
    gtsam::Matrix66 bias_acc_omega_int = gtsam::Matrix::Identity(6,6) * 1e-5; // error in the bias used for preintegration


    /***************************/
    /**  GTSAM Initialization **/
    /***************************/

    /** Initial idx **/
    this->idx = 0;

    /** Initial prior pose **/
    Eigen::Affine3d tf;
    tf.setIdentity();
    gtsam::Pose3 prior_pose(gtsam::Rot3(tf.rotation()), gtsam::Point3(tf.translation()));

    /** Initial prior velocity (assume zero initial velocity)**/
    gtsam::Vector3 prior_velocity = Eigen::Vector3d::Zero();

    /** Initial prior bias (assume zero initial bias)**/
    gtsam::imuBias::ConstantBias prior_imu_bias;

    /** Create the estimated values **/
    this->initial_values.reset(new gtsam::Values());

    /** Add the initial values (pose, velocity and bias) **/
    this->initial_values->insert(X(this->idx), prior_pose);
    this->initial_values->insert(V(this->idx), prior_velocity);
    this->initial_values->insert(B(this->idx), prior_imu_bias);

    /** Assemble prior noise model and add it to the graph. **/
    this->pose_noise_model = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished()); // rad,rad,rad,m, m, m
    this->velocity_noise_model = gtsam::noiseModel::Isotropic::Sigma(3,0.1); // m/s
    this->bias_noise_model = gtsam::noiseModel::Isotropic::Sigma(6,1e-3);


    /** Create the factor graph **/
    this->factor_graph.reset(new gtsam::NonlinearFactorGraph());

    /** Add all prior factors (pose, velocity and bias) to the graph **/
    this->factor_graph->add(gtsam::PriorFactor<gtsam::Pose3>(X(this->idx), prior_pose, pose_noise_model));
    this->factor_graph->add(gtsam::PriorFactor<gtsam::Vector3>(V(this->idx), prior_velocity, velocity_noise_model));
    this->factor_graph->add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(this->idx), prior_imu_bias, this->bias_noise_model));

    /** Initialize the measurement noise **/
    boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> p = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0);

    /** PreintegrationBase params: **/
    p->accelerometerCovariance = measured_acc_cov; // acc white noise in continuous
    p->integrationCovariance = integration_error_cov; // integration uncertainty continuous

    /** should be using 2nd order integration
    PreintegratedRotation params: **/
    p->gyroscopeCovariance = measured_omega_cov; // gyro white noise in continuous

    /** PreintegrationCombinedMeasurements params: **/
    p->biasAccCovariance = bias_acc_cov; // acc bias in continuous
    p->biasOmegaCovariance = bias_omega_cov; // gyro bias in continuous
    p->biasAccOmegaInt = bias_acc_omega_int;

    this->imu_preintegrated.reset(new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias));

    /** Store previous state for the imu integration and the latest predicted outcome. **/
    this->prev_state = gtsam::NavState(prior_pose, prior_velocity);
    this->prop_state = this->prev_state;
    this->prev_bias = prior_imu_bias;

    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();

    /** Reset GTSAM **/
    this->factor_graph.reset();
    this->initial_values.reset();
    this->imu_preintegrated.reset();

    /** Reset estimation **/
    this->idx = 0;
}

void Task::optimize()
{
    /** Propagate the state and store in the values **/
    this->prop_state = this->imu_preintegrated->predict(prev_state, prev_bias);//TODO perhaps move to imu method
    this->initial_values->insert(X(this->idx), this->prop_state.pose());
    this->initial_values->insert(V(this->idx), this->prop_state.v());
    this->initial_values->insert(B(this->idx), this->prev_bias);

    /** Optimize **/
    gtsam::LevenbergMarquardtOptimizer optimizer (*(this->factor_graph), *(this->initial_values));
    RTT::log(RTT::Warning)<<"[SHARK_SLAM OPTIMIZE] INITIAL_VALUES WITH: "<<this->initial_values->size()<<"\n";

    /** Store in the values **/
    gtsam::Values result = optimizer.optimize();

    /** Overwrite the beginning of the preintegration for the next step. **/
    this->prev_state = gtsam::NavState(result.at<gtsam::Pose3>(X(this->idx)),
                                       result.at<gtsam::Vector3>(V(this->idx)));
    this->prev_bias = result.at<gtsam::imuBias::ConstantBias>(B(this->idx));

    // Reset the preintegration object.
    this->imu_preintegrated->resetIntegrationAndSetBias(prev_bias);


    return;
}
