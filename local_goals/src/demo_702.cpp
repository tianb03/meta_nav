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
  ROS_INFO("Action server went active.");
}

void feedbackCb(const kobuki_msgs::AutoDockingFeedbackConstPtr& feedback)
{
  ROS_INFO("Feedback: [DockDrive: %s]: %s",feedback->state.c_str(), feedback->text.c_str());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "auto_dock_test");
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

  while (ros::ok())
  {
    /*弹出*/
    ROS_INFO("backward beginning");
    for(int j = 0; j < 48; j++)
    {
      geometry_msgs::Twist backward_vel;
      backward_vel.linear.x = -0.1;
      ROS_INFO("backward");
      velocity_command_publisher.publish(backward_vel);
      ros::spinOnce();
      loop_rate.sleep();
    }
    ROS_INFO("backward over");

	  /*软件部窄路点*/
    do
    {
      goal.target_pose.pose.position.x = 4.558;
      goal.target_pose.pose.position.y = 0.013;
      goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.00);
      ROS_INFO("Sending goal software department");
      ac.sendGoal(goal);
      ac.waitForResult();
    }while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);

    /*行政部窄路点*/
    do
    {
      goal.target_pose.pose.position.x = 11.707;
      goal.target_pose.pose.position.y = -0.297;
      goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.00);
      ROS_INFO("Sending goal administration department");
      ac.sendGoal(goal);
      ac.waitForResult();
    }while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);

	  /*咖啡机点*/
    do
    {
      goal.target_pose.pose.position.x = 15.389;
      goal.target_pose.pose.position.y = 1.067;
      goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(-1.946);
      ROS_INFO("Sending goal coffee machine");
      ac.sendGoal(goal);
      ac.waitForResult();
      ros::Duration(3.0).sleep();
    }while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);

    /*行政部窄路点*/
    do
    {
      goal.target_pose.pose.position.x = 11.707;
      goal.target_pose.pose.position.y = -0.297;
      goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(3.10);
      ROS_INFO("Sending goal administration department");
      ac.sendGoal(goal);
      ac.waitForResult();
    }while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);

	  /*软件部窄路点*/
    do
    {
      goal.target_pose.pose.position.x = 4.558;
      goal.target_pose.pose.position.y = 0.013;
      goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(3.10);
      ROS_INFO("Sending goal software department");
      ac.sendGoal(goal);
      ac.waitForResult();
    }while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);

	  /*总经理室点*/
    do
    {
      goal.target_pose.pose.position.x = -2.321;
      goal.target_pose.pose.position.y = 0.345;
      goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(3.10);
      ROS_INFO("Sending goal general manager office");
      ac.sendGoal(goal);
      ac.waitForResult();
      ros::Duration(3.0).sleep();
    }while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);

	  /*实验室点
    do
    {
      goal.target_pose.pose.position.x = -8.821;
      goal.target_pose.pose.position.y = 2.988;
      goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(-1.657);
      ROS_INFO("Sending goal laboratory");
      ac.sendGoal(goal);
      ac.waitForResult();
	    ros::Duration(3.0).sleep();
    }while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);*/

	  /*充电准备点*/
    do
    {
      goal.target_pose.pose.position.x = 1.861;
      goal.target_pose.pose.position.y = -1.055;
      goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(-1.744);
      ROS_INFO("Go to charge");
      ac.sendGoal(goal);
      ac.waitForResult();
    }while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);

    /*自动充电三秒*/
    do
    {
      bc.sendGoal(goala, doneCb, activeCb, feedbackCb);
      bc.waitForResult();
      ros::Duration(3.0).sleep();
    }while(bc.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);
  }

  return 0;
}
