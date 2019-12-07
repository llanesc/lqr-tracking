#include "ros/ros.h"
#include <lqr_controller/lqr_euler.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/State.h>
#include <mavros/frame_tf.h>

mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state = *msg;
}

int main(int argc, char** argv)
{

  ros::init(argc,argv,"LQR_Controller_node");
  ros::NodeHandle nh;

  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::Publisher cmd_raw = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  ros::Rate rate(100);

  while(ros::ok() && !current_state.connected) {
     ros::spinOnce();
     rate.sleep();
  }

  lqr::LQR_Euler lqr(nh);


  //main while loop
  while (ros::ok()) {

	  mavros_msgs::AttitudeTarget bodyrate_msg;

	  lqr.setOutput(lqr.getTrajectoryControl() - lqr.getGain()*lqr.getError());

	  std::cout << "motor cmd:" << lqr.getMotorCmd() << std::endl;
//	  for (int i = 0; i < 3; i++) {
//		if (lqr.getOutput()(i) >= 2.0 || lqr.getOutput()(i) <= -2.0) {
//		  if (lqr.getOutput()(i) >= 2.0 ) {
//			lqr.setOutput(2.0,i);
//		  } else {
//			lqr.setOutput(-2.0,i);
//		  }
//		}
//	  }


//	  lqr.setOutput(-1*lqr.getOutput()(3), 3);
//
//	  if (lqr.getOutput()(3) >= 18.0) {
//		lqr.setOutput(18.0,3);
//	  } else if (lqr.getOutput()(3) <= 2.0) {
//		lqr.setOutput(2.0,3);
//	  }


	  Eigen::Vector3d cmd_body_rate_aircraft;
	  Eigen::Vector3d cmd_body_rate_baselink;
	  cmd_body_rate_aircraft << lqr.getOutput()(0),
								lqr.getOutput()(1),
								lqr.getOutput()(2);

	  cmd_body_rate_baselink = mavros::ftf::transform_frame_aircraft_baselink<Eigen::Vector3d>(cmd_body_rate_aircraft);

	  bodyrate_msg.body_rate.x = cmd_body_rate_baselink(0);
	  bodyrate_msg.body_rate.y = cmd_body_rate_baselink(1);
	  bodyrate_msg.body_rate.z = cmd_body_rate_baselink(2);
	  bodyrate_msg.thrust = lqr.getMotorCmd();
	  bodyrate_msg.type_mask = 128;

	  cmd_raw.publish(bodyrate_msg);

	  ros::spinOnce();
	  rate.sleep();

  }


  return 0;

}
