// Software License Agreement (BSD License)
// Copyright (c) 2015, Tarsbot
// All rights reserved.

// Author: Bo Tian

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <kobuki_dock_drive/dock_drive.hpp>
#include <tf/transform_listener.h>
#include "kobuki_auto_docking/auto_docking_ros.hpp"

typedef actionlib::SimpleActionClient<kobuki_msgs::AutoDockingAction> AutoDockingClient;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void doneCb(const actionlib::SimpleClientGoalState& status,
            const kobuki_msgs::AutoDockingResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", status.toString().c_str());
  ROS_INFO("Answer: %s", result->text.c_str());
  //ros::shutdown();
}

void activeCb()
{
  //ROS_INFO("Action server went active.");
}

void feedbackCb(const kobuki_msgs::AutoDockingFeedbackConstPtr& feedback)
{
  //ROS_INFO("Feedback: [DockDrive: %s]: %s",feedback->state.c_str(), feedback->text.c_str());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo_702_simple");
  //tell the action client that we want to spin a thread by default
  ros::NodeHandle nh;
  ros::Publisher velocity_command_publisher = nh.advertise< geometry_msgs::Twist >("/mobile_base/commands/velocity", 1);
  MoveBaseClient ac("move_base", true);
  AutoDockingClient bc("dock_drive_action", true);
  move_base_msgs::MoveBaseGoal goal;
  kobuki_msgs::AutoDockingGoal goala; // define a message is different from python
  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";  //目标姿态，头属性，坐标系的ID
  goal.target_pose.header.stamp = ros::Time::now();  //时间戳
  ros::Rate loop_rate(10);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(2.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  } 

  /*弹出*/
  ROS_INFO("backward beginning");
  for(int j = 0; j < 64; j++)
  {
    geometry_msgs::Twist backward_vel;
    backward_vel.linear.x = -0.1;
    velocity_command_publisher.publish(backward_vel);
    ros::spinOnce();
    loop_rate.sleep();
  }
  ROS_INFO("backward over");

  /*qian tai*/
  do
  {
    goal.target_pose.pose.position.x = -2.502;
    goal.target_pose.pose.position.y = 8.232;
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(2.861);
    ROS_INFO("Sending goal reception");
    ac.sendGoal(goal);
    ac.waitForResult();
  }while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);

  ros::Duration(3.0).sleep();


  /*hui yi shi*/
  do
  {
    goal.target_pose.pose.position.x = -2.167;
    goal.target_pose.pose.position.y = 3.901;
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(1.568);
    ROS_INFO("Sending goal laboratory");
    ac.sendGoal(goal);
    ac.waitForResult();
  }while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);

  ros::Duration(3.0).sleep();

  /*chong dian qian*/
  do
  {
    goal.target_pose.pose.position.x = -0.758;
    goal.target_pose.pose.position.y = 0.0;
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    ROS_INFO("Go to charge");
    ac.sendGoal(goal);
    ac.waitForResult();
  }while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);

  /*chong dian*/
  bc.sendGoal(goala, doneCb, activeCb, feedbackCb);
  bc.waitForResult();

  return 0;
}
