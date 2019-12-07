/*
 * Copyright (c) 2019, Christian Llanes, ADCL, Embry-Riddle Aeronautical University
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mavros/frame_tf.h>

namespace lqr {
  class CirTrajectory {
   public:
    CirTrajectory(double w, double R, double h) : w_(w), h_(h), R_(R) {};
    virtual ~CirTrajectory() {}

    void evaluateRange(double dt, int derivative_order, std::vector<Eigen::VectorXd>* result);
    Eigen::VectorXd evaluate(double t, int derivative_order);
    void SampleTrajectory(double dt, mav_msgs::EigenTrajectoryPointVector* states);

    enum derivative_order {POSITION, VELOCITY, ACCELERATION};

   private:
    double w_;              //Angular rate
    double R_;              //Circle Radius
    double h_;              //Altitude
    double PI_ = 3.1415926;
    mav_msgs::EigenTrajectoryPointVector states_;
  };
  const double kNumNanosecondsPerSecond = 1.e9;
  void setTrajectoryStates(mav_msgs::EigenTrajectoryPointVector* states);
  Eigen::Vector3d quaternion_to_rpy_wrap(const Eigen::Quaterniond& q);
}

