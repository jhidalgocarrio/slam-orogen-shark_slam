/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef SHARK_SLAM_TASK_TASK_HPP
#define SHARK_SLAM_TASK_TASK_HPP

#include "shark_slam/TaskBase.hpp"


/** Boost **/
#include <boost/shared_ptr.hpp> /** shared pointers **/

/** GTSAM factor types **/
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

/** GTSAM optimizers **/
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/inference/Symbol.h>

namespace shark_slam{

    /*! \class Task
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * Bias Noise: rate random walk for accelerometers (rad/s/sqrt(s))
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','shark_slam::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument.
     */
    class Task : public TaskBase
    {
    friend class TaskBase;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW //Structures having Eigen members

    protected:

        /******************************/
        /*** Control Flow Variables ***/
        /******************************/

        /** Initialization flag **/
        bool inititialize;

        /** Orientation is available from IMU **/
        bool orientation_available;

        /** Needs optimization **/
        bool needs_optimization;

        /** Indices to identify estimates **/
        unsigned long int idx;

        /**************************/
        /***    Input Ports     ***/
        /**************************/
        base::samples::RigidBodyState gps_pose_samples;
        base::samples::IMUSensors imu_samples;
        base::samples::RigidBodyState orientation_samples;

        /**************************/
        /*** Property Variables ***/
        /**************************/


        /** IMU covariance matrices **/
        gtsam::Matrix33 measured_acc_cov;
        gtsam::Matrix33 measured_omega_cov; 
        gtsam::Matrix33 integration_error_cov;
        gtsam::Matrix33 bias_acc_cov;
        gtsam::Matrix33 bias_omega_cov;
        gtsam::Matrix66 bias_acc_omega_int;

        /** Initial pose uncertainties **/
        gtsam::noiseModel::Diagonal::shared_ptr pose_noise_model;
        gtsam::noiseModel::Diagonal::shared_ptr velocity_noise_model;
        gtsam::noiseModel::Diagonal::shared_ptr bias_noise_model;

        /******************************************/
        /*** General Internal Storage Variables ***/
        /******************************************/

        Eigen::Affine3d tf_init, tf_init_inverse; /** initial pose **/

        gtsam::NavState prev_state, prop_state;
        gtsam::imuBias::ConstantBias prev_bias;

        /** GTSAM Factor graph **/
        boost::shared_ptr< gtsam::NonlinearFactorGraph > factor_graph;

        /** iSAM2 variable **/
        boost::shared_ptr< gtsam::ISAM2 > isam;

        /** Values of the estimated quantities **/
        boost::shared_ptr< gtsam::Values > initial_values;

        /** IMU preintegrated measurements (Imufactor or CombinedImufactor) **/
        boost::shared_ptr< gtsam::PreintegrationType > imu_preintegrated;

        /**************************/
        /***   Output Ports     ***/
        /**************************/
        base::samples::RigidBodyState output_pose;

    protected:

        virtual void gps_pose_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &gps_pose_samples_sample);

        virtual void imu_samplesTransformerCallback(const base::Time &ts, const ::base::samples::IMUSensors &imu_samples_sample);

        virtual void orientation_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &orientation_samples_sample);

    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "shark_slam::Task");

        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices.
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task.
         * 
         */
        Task(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Task
         */
        ~Task();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states.
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();

        /**@brief initialization
         */
        void initialization(Eigen::Affine3d &tf);

        /** Optimize 
         * @brief optimize the factor graph and get the results
         * **/
        void optimize();
    };
}

#endif

