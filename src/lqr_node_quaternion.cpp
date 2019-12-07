#include "ros/ros.h"
#include <lqr_controller/lqr_quaternion.hpp>
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
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  ros::ServiceClient arm_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

  ros::Publisher cmd_raw = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

  ros::Rate rate(100);

  while(ros::ok() && !current_state.connected) {
     ros::spinOnce();
     rate.sleep();
  }


  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";
  offb_set_mode.request.base_mode = 0;

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  if(set_mode_client.call(offb_set_mode)) {
     ROS_INFO("Offboard enabled");
  }

  LQR::LQR_Quaternion lqr(nh);



  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = 2;


  for(int i = 100; ros::ok() && i > 0; --i){
      local_pos_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();
  }

  ros::Time lastob_request = ros::Time::now();
  ros::Time lastarm_request = ros::Time::now();

  //main while loop
  while (ros::ok()) {

    if(current_state.mode != "OFFBOARD" && (ros::Time::now() - lastob_request > ros::Duration(2.0))){
         if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
             ROS_INFO("Offboard enabled");
         }
    lastob_request = ros::Time::now();
    }
    if( !current_state.armed && (ros::Time::now() - lastarm_request > ros::Duration(2.0))){
        if(arm_client.call(arm_cmd) && arm_cmd.response.success){
            ROS_INFO("Vehicle armed");
         }
    lastarm_request = ros::Time::now();
    }

  mavros_msgs::AttitudeTarget bodyrate_msg;

  lqr.setOutput(lqr.getTrajectoryControl() - lqr.getGain()*lqr.getError());


  for (int i = 0; i < 3; i++) {
    if (lqr.getOutput()(i) >= 2.0 || lqr.getOutput()(i) <= -2.0) {
      if (lqr.getOutput()(i) >= 2.0 ) {
        lqr.setOutput(2.0,i);
      } else {
        lqr.setOutput(-2.0,i);
      }
    }
  }


  if (lqr.getOutput()(3) >= 18.0) {
    lqr.setOutput(18.0,3);
  } else if (lqr.getOutput()(3) <= 2.0) {
    lqr.setOutput(2.0,3);
  }

  std::cout << "X Error:" << lqr.getError() << std::endl;
  std::cout << "Output:" << lqr.getOutput() << std::endl;


  Eigen::Vector3d cmd_body_rate_baselink;
  cmd_body_rate_baselink << lqr.getOutput()(0),
                            lqr.getOutput()(1),
                            lqr.getOutput()(2);

  bodyrate_msg.body_rate.x = cmd_body_rate_baselink(0);
  bodyrate_msg.body_rate.y = cmd_body_rate_baselink(1);
  bodyrate_msg.body_rate.z = cmd_body_rate_baselink(2);
  bodyrate_msg.thrust = lqr.getOutput()(3)/17;
  bodyrate_msg.type_mask = 128;

  cmd_raw.publish(bodyrate_msg);

  ros::spinOnce();
  rate.sleep();

  }


  return 0;

}
