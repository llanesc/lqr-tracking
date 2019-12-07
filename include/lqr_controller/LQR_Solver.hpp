#pragma once

#include <cmath>

#include <ros/ros.h>
#include <ct/optcon/optcon.h>
#include <lqr_controller/declarations_euler.hpp>
#include <nav_msgs/Odometry.h>
#include <Eigen/Geometry>

namespace LQR {
class LQR_Solver {
  public:
    /*!
     * Constructor.
     * @param nodeHandle the ROS node handle.
     */
    LQR_Solver(ros::NodeHandle& nodeHandle);

    /*!
     * Destructor.
     */
    virtual ~LQR_Solver();

    control_vector_t output;


    ct::core::FeedbackMatrix<nStates, nControls> K_;
    control_vector_t uref_;
    state_vector_t xerror;

   private:

    /*!
     * ROS topic callback method.
     * @param message the received message.
     */
    void topicCallback(const nav_msgs::Odometry::ConstPtr& msg);

    //! ROS node handle.
    ros::NodeHandle& nodeHandle_;

    //! ROS topic subscriber.
    ros::Subscriber subscriber_;

    //! ROS topic name to subscribe to.
    std::string subscriberTopic_;

    //! State and control matrix dimensions
    const size_t state_dim = nStates;
    const size_t control_dim = nControls;



    control_vector_t u_;


    state_matrix_t A_;
    control_gain_matrix_t B_;
    ros::Time callBack_;
    state_vector_t x_;
    state_vector_t xref_;

    ct::optcon::TermQuadratic<nStates, nControls> quadraticCost_;
    ct::optcon::TermQuadratic<nStates, nControls>::state_matrix_t Q_;
    ct::optcon::TermQuadratic<nStates, nControls>::control_matrix_t R_;
    ct::optcon::LQR<nStates, nControls> lqrSolver_;


    state_matrix_t A_quadrotor(const state_vector_t& x, const control_vector_t& u);
    control_gain_matrix_t B_quadrotor(const state_vector_t& x, const control_vector_t& u);
  };

} /* namespace */
