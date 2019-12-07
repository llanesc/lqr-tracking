#include "lqr_controller/lqr_quaternion.hpp"

namespace LQR {

LQR_Quaternion::LQR_Quaternion(ros::NodeHandle& nodeHandle)
  : nodeHandle_(nodeHandle)
{
  quadraticCost_.loadConfigFile(ros::package::getPath("lqr_controller") + "/lqrCostEuler.info", "termLQR");
  Q_ = quadraticCost_.getStateWeight();
  R_ = quadraticCost_.getControlWeight();
  odom_sub_ = nodeHandle_.subscribe("/mavros/local_position/odom", 1, &LQR_Quaternion::topicCallback, this);
  marker_pub_ = nodeHandle_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 100);

  initiated = false;
  traj_index = 1;
  callBack_ = ros::Time::now();
  init_time_ = ros::Time::now();

}

LQR_Quaternion::~LQR_Quaternion()
{
}

void LQR_Quaternion::topicCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  setStates(msg,x_);
  setTrajectoryReference(xref_,uref_);
  //Eigen::Vector4d static_ref(0,0,1,0);
  //setStaticReference(xref_,uref_,static_ref);
  setError(xref_,x_,xerror_);
  marker_pub_.publish(markers_);

  if ((ros::Time::now().toSec() - callBack_.toSec()) > 0.1)
  {
    A_ = A_quadrotor(xref_,uref_);
    B_ = B_quadrotor(xref_,uref_);

    if (lqrSolver_.compute(Q_, R_, A_, B_, Knew_))
    {
      if (!Knew_.hasNaN())
      {
        Kold_ = Knew_;
      }
    }

    std::cout << "A matrix:" << std::endl << A_ << std::endl;
    std::cout << "B matrix:" << std::endl << B_ << std::endl;
    std::cout << "LQR gain matrix:" << std::endl << Kold_ << std::endl;
    std::cout << "Reference:" << std::endl << xref_ << std::endl;
    callBack_ = ros::Time::now();
  }
}



state_matrix_t LQR_Quaternion::A_quadrotor(const state_vector_t& x, const control_vector_t& u)
{
    double wx = u(0);
    double wy = u(1);
    double wz = u(2);
    double norm_thrust  = u(3);
    Eigen::Quaternion<double> q(x(3),x(4),x(5),x(6));
    Eigen::Matrix<double,4,4> q_partial_correction;
    Eigen::Matrix<double,4,4> dqdot_dq;
    Eigen::Matrix<double,3,4> dvdot_dq;
    Eigen::Matrix<double,4,1> q_vec;

    q_vec(0) = q.w();
    q_vec(1) = q.x();
    q_vec(2) = q.y();
    q_vec(3) = q.z();

    state_matrix_t A;
    A.setZero();

    //Position
    A(0,7) = 1;
    A(1,8)= 1;
    A(2,9)= 1;
    Eigen::Matrix<double,4,4> Identity;

    //Orientation
    q_partial_correction = pow(q.norm(),-1.0)*(Identity.Identity() - pow(q.norm(),-2.0)*(q_vec * q_vec.transpose()));

    dqdot_dq << 0, -wx, -wy, -wz,
                wx, 0, wz, -wy,
                wy, -wz, 0, wx,
                wz, wy, -wx, 0;
    dqdot_dq = 0.5*dqdot_dq*q_partial_correction;

    A(3,3) = dqdot_dq(0,0);
    A(3,4) = dqdot_dq(0,1);
    A(3,5) = dqdot_dq(0,2);
    A(3,6) = dqdot_dq(0,3);

    A(4,3) = dqdot_dq(1,0);
    A(4,4) = dqdot_dq(1,1);
    A(4,5) = dqdot_dq(1,2);
    A(4,6) = dqdot_dq(1,3);

    A(5,3) = dqdot_dq(2,0);
    A(5,4) = dqdot_dq(2,1);
    A(5,5) = dqdot_dq(2,2);
    A(5,6) = dqdot_dq(2,3);

    A(6,3) = dqdot_dq(3,0);
    A(6,4) = dqdot_dq(3,1);
    A(6,5) = dqdot_dq(3,2);
    A(6,6) = dqdot_dq(3,3);


    //Velocity
    dvdot_dq << q.y(),  q.z(),  q.w(), q.x(),
              -q.x(), -q.w(),  q.z(), q.y(),
               q.w(), -q.x(), -q.y(), q.z();

    dvdot_dq = 2*norm_thrust*dvdot_dq*q_partial_correction;

    A(7,3) = dvdot_dq(0,0);
    A(7,4) = dvdot_dq(0,1);
    A(7,5) = dvdot_dq(0,2);
    A(7,6) = dvdot_dq(0,3);

    A(8,3) = dvdot_dq(1,0);
    A(8,4) = dvdot_dq(1,1);
    A(8,5) = dvdot_dq(1,2);
    A(8,6) = dvdot_dq(1,3);

    A(9,3) = dvdot_dq(2,0);
    A(9,4) = dvdot_dq(2,1);
    A(9,5) = dvdot_dq(2,2);
    A(9,6) = dvdot_dq(2,3);

    return A;
}

control_gain_matrix_t LQR_Quaternion::B_quadrotor(const state_vector_t& x, const control_vector_t& u)
{
   double wx = u(0);
   double wy = u(1);
   double wz = u(2);
   double norm_thrust  = u(3);
   Eigen::Quaternion<double> q(x(3),x(4),x(5),x(6));
   Eigen::Matrix<double,3,1> dvdot_dc;
   Eigen::Matrix<double,4,3> dqdot_dw;

   control_gain_matrix_t B;
   B.setZero();

   dvdot_dc << 2*(q.w()*q.y() + q.x()*q.z()),
               2*(q.y()*q.z() - q.w()*q.x()),
               pow(q.w(),2) - pow(q.x(),2) - pow(q.y(),2) + pow(q.z(),2);

   B(7,3) = dvdot_dc(0);
   B(8,3) = dvdot_dc(1);
   B(9,3) = dvdot_dc(2);

   dqdot_dw << -q.x(), -q.y(), -q.z(),
                q.w(), -q.z(),  q.y(),
                q.z(),  q.w(), -q.x(),
               -q.y(),  q.x(),  q.w();

   dqdot_dw = 0.5*dqdot_dw;

   B(3,0) = dqdot_dw(0,0);
   B(3,1) = dqdot_dw(0,1);
   B(3,2) = dqdot_dw(0,2);

   B(4,0) = dqdot_dw(1,0);
   B(4,1) = dqdot_dw(1,1);
   B(4,2) = dqdot_dw(1,2);

   B(5,0) = dqdot_dw(2,0);
   B(5,1) = dqdot_dw(2,1);
   B(5,2) = dqdot_dw(2,2);

   B(6,0) = dqdot_dw(3,0);
   B(6,1) = dqdot_dw(3,1);
   B(6,2) = dqdot_dw(3,2);

   return B;
}

void LQR_Quaternion::setStates(const nav_msgs::Odometry::ConstPtr& msg, state_vector_t& x)
{
  position_enu_ << msg->pose.pose.position.x,
                   msg->pose.pose.position.y,
                   msg->pose.pose.position.z;

  q_enu_.w() = msg->pose.pose.orientation.w;
  q_enu_.x() = msg->pose.pose.orientation.x;
  q_enu_.y() = msg->pose.pose.orientation.y;
  q_enu_.z() = msg->pose.pose.orientation.z;

  velocity_enu_ << msg->twist.twist.linear.x,
                   msg->twist.twist.linear.y,
                   msg->twist.twist.linear.z;
  velocity_enu_ = mavros::ftf::transform_frame_baselink_enu(velocity_enu_,q_enu_);

  /*Position*/
  x(0) = position_enu_(0);
  x(1) = position_enu_(1);
  x(2) = position_enu_(2);

  /*Orientation*/
  x(3) = q_enu_.w();
  x(4) = q_enu_.x();
  x(5) = q_enu_.y();
  x(6) = q_enu_.z();

  /*Linear Velocities*/
  x(7) = velocity_enu_(0);
  x(8) = velocity_enu_(1);
  x(9) = velocity_enu_(2);
}

void LQR_Quaternion::setError(const state_vector_t& xref, const state_vector_t& x, state_vector_t& xerror)
{
  /*Position error*/
  xerror(0) = (x(0) - xref(0));
  xerror(1) = (x(1) - xref(1));
  xerror(2) = (x(2) - xref(2));

  /*Orientation error*/
  Eigen::Quaterniond q(x(3),x(4),x(5),x(6));
  Eigen::Quaterniond qref(xref(3),xref(4),xref(5),xref(6));
  Eigen::Quaterniond qerror = q.inverse() * qref;
  xerror(3) = 0;
  xerror(4) = qerror.x();
  xerror(5) = qerror.y();
  xerror(6) = qerror.z();

  /*Velocity error*/
  xerror(7) = (x(7) - xref(7));
  xerror(8) = (x(8) - xref(8));
  xerror(9) = (x(9) - xref(9));
}

bool LQR_Quaternion::setTrajectoryReference(state_vector_t& xref,control_vector_t& uref)
{
  if (!initiated)
  {
    generateTrajectory(states_);
  }

  initiated = true;
  Eigen::Vector3d position_ref_enu;
  Eigen::Vector3d accel;
  Eigen::Vector3d orient_yaw;
  Eigen::Vector3d thrust;
  Eigen::Vector3d thrust_dir;
  Eigen::Quaterniond q_yaw;
  Eigen::Vector3d rpy_ref_enu;
  Eigen::Quaterniond qref_enu;
  Eigen::Vector3d velocity_enu;
  std::vector<double> gamma;
  gamma.resize(states_.size()-1);
  std::vector<double> dist_traj;
  dist_traj.resize(states_.size()-1);

  int i = traj_index - 1;

  do {
    i++;
    Eigen::Vector3d distance_vec(states_[i].position_W - states_[i-1].position_W);
    gamma[i-1] = (distance_vec.dot(position_enu_ - states_[i-1].position_W)/pow(distance_vec.norm(),2));
    dist_traj[i-1] = ((states_[i].position_W - position_enu_).norm());

  } while (i < states_.size() - 1);

  if (traj_index > (states_.size() - 5))
  {
    traj_index = states_.size() - 1;

    position_ref_enu = states_[traj_index].position_W;

    /*Position*/
    xref(0) = position_ref_enu.x();
    xref(1) = position_ref_enu.y();
    xref(2) = position_ref_enu.z();

    /*Orientation*/
    xref(3) = states_[traj_index].orientation_W_B.w();
    xref(4) = states_[traj_index].orientation_W_B.x();
    xref(5) = states_[traj_index].orientation_W_B.y();
    xref(6) = states_[traj_index].orientation_W_B.z();

    /*Velocity*/
    xref(7) = 0;
    xref(8) = 0;
    xref(9) = 0;

    /*Body rates*/
    uref(0) = 0;
    uref(1) = 0;
    uref(2) = 0;
    uref(3) = 9.81;
    std::cout<< "xref" << xref <<std::endl;
    std::cout<< "uref" << uref <<std::endl;
    return true;

  } else {

    std::vector<double>::iterator result = std::min_element(dist_traj.begin() + traj_index,dist_traj.end());
    traj_index = std::distance(dist_traj.begin(),result);
    std::cout<< "traj_index" << traj_index <<std::endl;

    /*Position*/
    position_ref_enu = states_[traj_index-1].position_W + gamma[traj_index]*(states_[traj_index].position_W - states_[traj_index-1].position_W);

    /*Orientation*/
    accel = (states_[traj_index-1].acceleration_W + gamma[traj_index]*(states_[traj_index].acceleration_W - states_[traj_index-1].acceleration_W));
    orient_yaw = (quaternion_to_rpy_wrap(states_[traj_index-1].orientation_W_B) + gamma[traj_index]*(quaternion_to_rpy_wrap(states_[traj_index].orientation_W_B) - quaternion_to_rpy_wrap(states_[traj_index-1].orientation_W_B)));
    thrust = (Eigen::Vector3d(0,0,9.81) + accel);
    thrust_dir = (thrust.normalized());
    q_yaw = (Eigen::AngleAxisd(orient_yaw.z(),Eigen::Vector3d(0,0,1)));
    Eigen::Vector3d interm_thrust_frame(mavros::ftf::detail::transform_frame(thrust_dir,q_yaw));
    //rpy_ref_enu << atan2(-1*interm_thrust_frame.y(),sqrt(pow(interm_thrust_frame.x(),2)+pow(interm_thrust_frame.z(),2))),
    //               atan2(interm_thrust_frame.x(),interm_thrust_frame.z()),
    //               orient_yaw.z();
    rpy_ref_enu = (quaternion_to_rpy_wrap(states_.at(traj_index-1).orientation_W_B) + gamma[traj_index]*(quaternion_to_rpy_wrap(states_.at(traj_index).orientation_W_B) - quaternion_to_rpy_wrap(states_.at(traj_index-1).orientation_W_B)));

    qref_enu = (mavros::ftf::quaternion_from_rpy(rpy_ref_enu));

    /*Velocity*/
    velocity_enu << states_.at(traj_index-1).velocity_W.x() + gamma[traj_index]*(states_.at(traj_index).velocity_W.x() - states_.at(traj_index-1).velocity_W.x()),
                    states_.at(traj_index-1).velocity_W.y() + gamma[traj_index]*(states_.at(traj_index).velocity_W.y() - states_.at(traj_index-1).velocity_W.y()),
                    states_.at(traj_index-1).velocity_W.z() + gamma[traj_index]*(states_.at(traj_index).velocity_W.z() - states_.at(traj_index-1).velocity_W.z());

    /*Body rates*/
    Eigen::Vector3d angular_velocity_des_W;
    angular_velocity_des_W << states_.at(traj_index-1).angular_velocity_W + gamma[traj_index]*(states_.at(traj_index).angular_velocity_W - states_.at(traj_index-1).angular_velocity_W);
    Eigen::Vector3d angular_velocity_des_B(mavros::ftf::transform_frame_enu_baselink(angular_velocity_des_W,qref_enu));

    xref(0) = position_ref_enu.x();
    xref(1) = position_ref_enu.y();
    xref(2) = position_ref_enu.z();

    xref(3) = qref_enu.w();
    xref(4) = qref_enu.x();
    xref(5) = qref_enu.y();
    xref(6) = qref_enu.z();

    xref(6) = velocity_enu.x();
    xref(7) = velocity_enu.y();
    xref(8) = velocity_enu.z();

    uref(0) = angular_velocity_des_B.x();
    uref(1) = angular_velocity_des_B.y();
    uref(2) = angular_velocity_des_B.z();
    uref(3) = thrust.norm();
    return false;
  }
}

bool LQR_Quaternion::setStaticReference(state_vector_t& xref,control_vector_t& uref, Eigen::Vector4d& flat_states)
{
  Eigen::Vector3d position_ref_enu;
  position_ref_enu << flat_states(0),
                      flat_states(1),
                      flat_states(2);

  Eigen::Quaterniond qref_enu(mavros::ftf::quaternion_from_rpy(0,0,flat_states(3)));

  /*Position*/
  xref(0) = position_ref_enu.x();
  xref(1) = position_ref_enu.y();
  xref(2) = position_ref_enu.z();

  /*Orientation*/
  xref(3) = qref_enu.w();
  xref(4) = qref_enu.x();
  xref(5) = qref_enu.y();
  xref(6) = qref_enu.z();

  /*Velocity*/
  xref(7) = 0;
  xref(8) = 0;
  xref(9) = 0;

  /*Body rates*/
  uref(0) = 0;
  uref(1) = 0;
  uref(2) = 0;
  uref(3) = 9.81;

  double ref_pos_diff = sqrt(pow(x_(0)-xref(0),2)+pow(x_(1)-xref(1),2)+pow(x_(2)-xref(2),2));
  double ref_yaw_diff = abs(x_(6)-xref(6));

  if (ref_pos_diff < 0.05 && ref_yaw_diff < 0.05)
  {
    return true;
  } else {
    return false;
  }
}

Eigen::Vector3d LQR_Quaternion::quaternion_to_rpy_wrap(const Eigen::Quaterniond& q)
{
  Eigen::Vector3d rpy;
  double roll = atan2(2*(q.w()*q.x()+q.y()*q.z()),1-2*(pow(q.x(),2)+pow(q.y(),2)));
  double pitch = asin(2*(q.w()*q.y()-q.z()*q.x()));
  double yaw = atan2(2*(q.w()*q.z()+q.x()*q.y()),1-2*(pow(q.y(),2)+pow(q.z(),2)));

  rpy << roll,
         pitch,
         yaw;

  return rpy;
}

void LQR_Quaternion::generateTrajectory(mav_msgs::EigenTrajectoryPoint::Vector& states)
{
  std::vector<double> segment_times;
  mav_trajectory_generation::Vertex::Vector vertices;
  const int deriv_to_opt = mav_trajectory_generation::derivative_order::SNAP;
  mav_trajectory_generation::Trajectory trajectory;
  Eigen::Vector3d rpy_enu = (quaternion_to_rpy_wrap(q_enu_));

  mav_trajectory_generation::Vertex start(dimension), interm1(dimension), interm2(dimension), interm3(dimension), interm4(dimension),end(dimension);

  start.makeStartOrEnd(Eigen::Vector3d(0,0,0-0.1), deriv_to_opt);
  vertices.push_back(start);

  interm1.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(0,0,1));
  vertices.push_back(interm1);

  interm2.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(4,4,1));
  interm2.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d(0,1,0));
  vertices.push_back(interm2);

  interm3.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(0,8,1));
  interm3.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d(-1,0,0));
  vertices.push_back(interm3);

  interm4.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-4,4,1));
  interm4.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d(0,-1,0));
  vertices.push_back(interm4);

  end.makeStartOrEnd(Eigen::Vector3d(-4,0,1), deriv_to_opt);
  vertices.push_back(end);

  segment_times = estimateSegmentTimes(vertices, v_max, a_max);

  //  mav_trajectory_generation::NonlinearOptimizationParameters parameters;
  //  parameters.max_iterations = 1000;
  //  parameters.f_rel = 0.05;
  //  parameters.x_rel = 0.1;
  //  parameters.time_penalty = 500.0;
  //  parameters.initial_stepsize_rel = 0.1;
  //  parameters.inequality_constraint_tolerance = 0.1;

  //  const int N = 10;
  //  mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
  //  opt.setupFromVertices(vertices, segment_times, deriv_to_opt);
  //  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, v_max);
  //  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, a_max);
  //  opt.optimize();

  const int N = 10;
  mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
  opt.setupFromVertices(vertices, segment_times, deriv_to_opt);
  opt.solveLinear();
  mav_trajectory_generation::Segment::Vector segments;
  opt.getSegments(&segments);

  opt.getTrajectory(&trajectory);

  bool success = mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_interval, &states);

  double distance = 1.0;
  std::string frame_id = "local_origin";
  mav_trajectory_generation::drawMavSampledTrajectory(states, distance, frame_id, &markers_);
}

state_vector_t LQR_Quaternion::getError()
{
  return this->xerror_;
}

ct::core::FeedbackMatrix<nStates, nControls> LQR_Quaternion::getGain()
{
  return this->Kold_;
}

void LQR_Quaternion::setOutput(double output,int j)
{
  this->output_(j) = output;
}

void LQR_Quaternion::setOutput(control_vector_t output)
{
  this->output_ = output;
}

control_vector_t LQR_Quaternion::getOutput()
{
  return this->output_;
}

state_vector_t LQR_Quaternion::getRefStates()
{
  return this->xref_;
}

control_vector_t LQR_Quaternion::getTrajectoryControl()
{
  return this->uref_;
}

}/* namespace LQR */
