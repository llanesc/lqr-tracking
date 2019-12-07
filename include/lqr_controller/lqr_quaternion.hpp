#pragma once

#include <cmath>
#include <ros/ros.h>
#include <ct/optcon/optcon.h>
#include <lqr_controller/declarations_quaternion.hpp>
#include <nav_msgs/Odometry.h>
#include <Eigen/Geometry>
#include <mavros/frame_tf.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
//#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/package.h>

namespace LQR {
class LQR_Quaternion {
  public:
    /*!
     * Constructor.
     * @param nodeHandle the ROS node handle.
     */
    LQR_Quaternion(ros::NodeHandle& nodeHandle);

    /*!
     * Destructor.
     */
    virtual ~LQR_Quaternion();

    control_vector_t getTrajectoryControl();
    state_vector_t getError();
    ct::core::FeedbackMatrix<nStates, nControls> getGain();
    void setOutput(double output, int j);
    void setOutput(control_vector_t output);
    control_vector_t getOutput();
    state_vector_t getRefStates();

   private:

    /*!
     * ROS topic callback method.
     * @param message the received message.
     */

    void topicCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void setStates(const nav_msgs::Odometry::ConstPtr& msg, state_vector_t& x);
    void setError(const state_vector_t& xref, const state_vector_t& x, state_vector_t& xerror);
    bool setTrajectoryReference(state_vector_t& xref,control_vector_t& uref);
    bool setStaticReference(state_vector_t& xref,control_vector_t& uref, Eigen::Vector4d& flat_states);
    Eigen::Vector3d quaternion_to_rpy_wrap(const Eigen::Quaterniond &q);
    void generateTrajectory(mav_msgs::EigenTrajectoryPoint::Vector& states);

    //! ROS node handle.
    ros::NodeHandle& nodeHandle_;

    //! ROS topic subscriber.
    ros::Subscriber odom_sub_;

    //Marker publisher
    ros::Publisher marker_pub_;

    //! State and control matrix dimensions
    const size_t state_dim = nStates;
    const size_t control_dim = nControls;

    //Trajectory
    double sampling_interval = 0.1;
    const double v_max = 2.0;
    const double a_max = 5.0;
    const int dimension = 3;
    int traj_index;
    bool initiated;
    mav_msgs::EigenTrajectoryPoint::Vector states_;
    visualization_msgs::MarkerArray markers_;

    ros::Time init_time_;
    Eigen::Vector3d position_enu_;
    Eigen::Vector3d velocity_enu_;
    Eigen::Quaterniond q_enu_;
    state_matrix_t A_;
    control_gain_matrix_t B_;
    ct::core::FeedbackMatrix<nStates, nControls> Kold_;
    ct::core::FeedbackMatrix<nStates, nControls> Knew_;
    ros::Time callBack_;
    state_vector_t x_;
    control_vector_t u_;
    state_vector_t xref_;
    control_vector_t uref_;
    state_vector_t xerror_;
    control_vector_t output_;

    ct::optcon::TermQuadratic<nStates, nControls> quadraticCost_;
    ct::optcon::TermQuadratic<nStates, nControls>::state_matrix_t Q_;
    ct::optcon::TermQuadratic<nStates, nControls>::control_matrix_t R_;
    ct::optcon::LQR<nStates, nControls> lqrSolver_;
    //states
    state_matrix_t A_quadrotor(const state_vector_t& x, const control_vector_t& u);
    control_gain_matrix_t B_quadrotor(const state_vector_t& x, const control_vector_t& u);
  };

} /* namespace */
