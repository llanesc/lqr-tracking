#include <lqr_controller/trajectory.hpp>
namespace lqr {

  void CirTrajectory::evaluateRange(double dt, int derivative_order, std::vector<Eigen::VectorXd>* result) {
    const double t_end = 2.0 * PI_ / w_;
    const size_t number_of_samples = (t_end) / dt + 1;
    result->clear();
    result->reserve(number_of_samples);
    double accumulated_time = 0.0;

    while (accumulated_time < t_end) {

      result->push_back(evaluate(accumulated_time, derivative_order));

      accumulated_time += dt;
    }

  }
  Eigen::VectorXd CirTrajectory::evaluate(double t, int derivative_order){
    Eigen::VectorXd result(3);
    result.setZero();

      switch(derivative_order) {
        case POSITION:
        {
          result << R_*cos(w_*t)
                   ,R_*sin(w_*t)
                   ,h_;
        }
          break;
        case VELOCITY:
        {
          result << -R_*w_*sin(w_*t)
                    ,R_*w_*cos(w_*t)
                    ,0;
        }
          break;
        case ACCELERATION:
        {
          result << -R_*pow(w_,2)*cos(w_*t)
                   ,-R_*pow(w_,2)*sin(w_*t)
                   ,0;
        }
          break;
        default:
        {
        LOG(ERROR) << "Incorrect derivative input to CirTrajectory class evaluate method.";
        }
     }
      return result;

  }

  void CirTrajectory::SampleTrajectory(double dt, mav_msgs::EigenTrajectoryPointVector* states) {
    double t_max = 2.0 * PI_ / w_;
    std::vector<Eigen::VectorXd> position, velocity, acceleration;

    evaluateRange(dt, POSITION, &position);
    evaluateRange(dt, VELOCITY, &velocity);
    evaluateRange(dt, ACCELERATION, &acceleration);

    size_t n_samples = position.size();

    states->resize(n_samples);

    for (int i = 0; i < n_samples; i++) {
      mav_msgs::EigenTrajectoryPoint& state = (*states)[i];

      Eigen::Quaterniond q(1.0,0.0,0.0,0.0);

      state.position_W = position[i];
      state.velocity_W = velocity[i];
      state.acceleration_W = acceleration[i];
      state.orientation_W_B = q;
      state.time_from_start_ns = static_cast<int64_t>((dt * i) * kNumNanosecondsPerSecond);

    }
  }

  void setTrajectoryStates(mav_msgs::EigenTrajectoryPointVector* states) {
    size_t n_samples = (*states).size();


    std::vector<Eigen::Vector3d> accel;

    std::vector<Eigen::Vector3d> orient_yaw;

    std::vector<Eigen::Vector3d> thrust;

    std::vector<Eigen::Vector3d> thrust_dir;

    std::vector<Eigen::Quaterniond> q_yaw;

    std::vector<Eigen::Vector3d> interm_thrust_frame;

    std::vector<Eigen::Vector3d> rpy_ref_enu;

    std::vector<Eigen::Quaterniond> qref_enu;


    for (int i = 0; i < n_samples; i++) {
     mav_msgs::EigenTrajectoryPoint& state = (*states)[i];

     accel.push_back(state.acceleration_W);

     orient_yaw.push_back(quaternion_to_rpy_wrap(state.orientation_W_B));
     thrust.push_back(Eigen::Vector3d(0,0,9.81) + accel[i]);
     thrust_dir.push_back(thrust[i].normalized());
     std::cout << "thrust: " << thrust[i].norm() << std::endl;
     Eigen::Quaterniond q_yaw_buffer(Eigen::AngleAxisd(orient_yaw[i].z(), Eigen::Vector3d(0,0,1)));
     q_yaw.push_back(q_yaw_buffer);
     Eigen::Vector3d rpy_ref_enu_buffer;
     interm_thrust_frame.push_back(mavros::ftf::detail::transform_frame(thrust_dir[i],q_yaw[i]));
     rpy_ref_enu_buffer << atan2(-1*interm_thrust_frame[i].y(),sqrt(pow(interm_thrust_frame[i].x(),2)+pow(interm_thrust_frame[i].z(),2))),
                    atan2(interm_thrust_frame[i].x(),interm_thrust_frame[i].z()),
                    orient_yaw[i].z();
     rpy_ref_enu.push_back(rpy_ref_enu_buffer);
     qref_enu.push_back(mavros::ftf::quaternion_from_rpy(rpy_ref_enu[i]));

     state.orientation_W_B = qref_enu[i];
    }

    for (int j = 0; j < n_samples-1; j++) {

    double dt = ((*states)[j+1].time_from_start_ns - (*states)[j].time_from_start_ns)/kNumNanosecondsPerSecond;

    Eigen::Vector3d euler_rates = (rpy_ref_enu[j+1] - rpy_ref_enu[j])/dt;
    (*states)[j].angular_velocity_W = euler_rates;
    }
  }

  Eigen::Vector3d quaternion_to_rpy_wrap(const Eigen::Quaterniond& q)
  {
    Eigen::Vector3d rpy;
    double roll = atan2(2 * (q.w() * q.x() + q.y() * q.z()),1 - 2 * (pow(q.x(),2) + pow(q.y(),2)));
    double pitch = asin(2 * (q.w() * q.y() - q.z() * q.x()));
    double yaw = atan2(2 * (q.w() * q.z() + q.x() * q.y()),1 - 2 *(pow(q.y(),2) + pow(q.z(),2)));

    rpy << roll,
           pitch,
           yaw;

    return rpy;
  }
}
