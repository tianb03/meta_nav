/*********************************************************************************
 *Description:Tutobot first rotate 180 degrees after the base charge, charging 
  complete a distance from the bottom of the base.
 
**********************************************************************************/
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <kobuki_dock_drive/dock_drive.hpp>
#include "kobuki_auto_docking/auto_docking_ros.hpp"
#include<geometry_msgs/Twist.h>
#include<kobuki_msgs/SensorState.h>   
geometry_msgs::Twist backward_vel;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionClient<kobuki_msgs::AutoDockingAction> AutoDockingClient;
void doneCb(const actionlib::SimpleClientGoalState& status,
            const kobuki_msgs::AutoDockingResultConstPtr& result)
 {
     ROS_INFO("Finished in state [%s]", status.toString().c_str());
	   ROS_INFO("Answer: %s", result->text.c_str());
	
 }
void activeCb()
 {
	   ROS_INFO("Action server went active.");
 }
void feedbackCb(const kobuki_msgs::AutoDockingFeedbackConstPtr& feedback)
 {
	   ROS_INFO("Feedback: [DockDrive: %s]: %s",feedback->state.c_str(),feedback->text.c_str());
 }

void Callback(const kobuki_msgs::SensorState power)
{
if (power.battery>=163)
     ROS_INFO("Charging has been completed");
  { 
     backward_vel.linear.x = -0.1;
     backward_vel.linear.y = 0.0;
     backward_vel.linear.z = 0.0;
     backward_vel.angular.x = 0.0;
     backward_vel.angular.y = 0.0;
     backward_vel.angular.z = 0.0;
     ROS_INFO("back");
     ROS_INFO("Power is %d", power.battery);
  }
}

int main(int argc, char** argv)
{
     ros::init(argc, argv, "turn180_autodock_back");
     MoveBaseClient ac("move_base", true);
 while(!ac.waitForServer(ros::Duration(2.0)))
  {
     ROS_INFO("Waiting for the move_base action server to come up");
  }
     AutoDockingClient bc ("dock_drive_action",true);
 while(!bc.waitForServer(ros::Duration(2.0)))
  {
     ROS_INFO("Waiting for the move_base action server to come up");
  }
     kobuki_msgs::AutoDockingGoal goalx; 
     move_base_msgs::MoveBaseGoal goal;
     goal.target_pose.header.frame_id = "base_link";
     goal.target_pose.header.stamp = ros::Time::now();
     goal.target_pose.pose.position.x =0;
     goal.target_pose.pose.position.y = 0;
     goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(3.141592654);
     ROS_INFO("turn");
     ac.sendGoal(goal);
     ac.waitForResult();
 if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
 {
     ROS_INFO("Hooray, the base turn 180 degrees");
     bc.sendGoal(goalx, doneCb, activeCb, feedbackCb);
	   bc.waitForResult();
     ros::NodeHandle n;
     ros::Subscriber sub = n.subscribe("mobile_base/sensors/core/", 10, Callback);
     ros::Publisher velocity_command_publisher = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
     ros::Rate loop_rate(10);
 while(ros::ok())
  { 
     ros::spinOnce();
     velocity_command_publisher.publish(backward_vel);
     loop_rate.sleep();
  }
 }
else
     ROS_INFO("The base failed to turn 180 degrees");
  return 0;
}


