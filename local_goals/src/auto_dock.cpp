// Software License Agreement (BSD License)
// Copyright (c) 2015, Tarsbot
// All rights reserved.

// Author: Bo Tian

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <kobuki_dock_drive/dock_drive.hpp>
#include <kobuki_auto_docking/auto_docking_ros.hpp>
typedef actionlib::SimpleActionClient<kobuki_msgs::AutoDockingAction> AutoDockingClient;

void doneCb(const actionlib::SimpleClientGoalState& status,
            const kobuki_msgs::AutoDockingResultConstPtr& result)
{

    ROS_INFO("Finished in state [%s]", status.toString().c_str());
    ROS_INFO("Answer: %s", result->text.c_str());
    ros::shutdown();
}

void activeCb()
{
    ROS_INFO("Action server went active.");
}

void feedbackCb(const kobuki_msgs::AutoDockingFeedbackConstPtr& feedback)
{
    ROS_INFO("Feedback: [DockDrive: %s]: %s",feedback->state.c_str(),feedback->text.c_str());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "auto_dock");
    AutoDockingClient ac("dock_drive_action",true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(2.0)))
    {
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    kobuki_msgs::AutoDockingGoal goal; // define a message is different from python

    ac.sendGoal(goal, doneCb, activeCb, feedbackCb);
    ac.waitForResult();

//    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//      ROS_INFO("Hooray, the base moved 1 meter forward");
//    else
//      ROS_INFO("The base failed to move forward 1 meter for some reason");
//    ros::spin();

    return 0;
}


/*
def doneCb(status, result):
  if 0: print ''
  elif status == GoalStatus.PENDING   : state='PENDING'
  elif status == GoalStatus.ACTIVE    : state='ACTIVE'
  elif status == GoalStatus.PREEMPTED : state='PREEMPTED'
  elif status == GoalStatus.SUCCEEDED : state='SUCCEEDED'
  elif status == GoalStatus.ABORTED   : state='ABORTED'
  elif status == GoalStatus.REJECTED  : state='REJECTED'
  elif status == GoalStatus.PREEMPTING: state='PREEMPTING'
  elif status == GoalStatus.RECALLING : state='RECALLING'
  elif status == GoalStatus.RECALLED  : state='RECALLED'
  elif status == GoalStatus.LOST      : state='LOST'
  # Print state of action server
  print 'Result - [ActionServer: ' + state + ']: ' + result.text

def activeCb():
  if 0: print 'Action server went active.'

def feedbackCb(feedback):
  # Print state of dock_drive module (or node.)
  print 'Feedback: [DockDrive: ' + feedback.state + ']: ' + feedback.text

def dock_drive_client():
  # add timeout setting
  client = actionlib.SimpleActionClient('dock_drive_action', AutoDockingAction)
  while not client.wait_for_server(rospy.Duration(5.0)):
    if rospy.is_shutdown(): return
    print 'Action server is not connected yet. still waiting...'

  goal = AutoDockingGoal();
  client.send_goal(goal, doneCb, activeCb, feedbackCb)
  print 'Goal: Sent.'
  rospy.on_shutdown(client.cancel_goal)
  client.wait_for_result()

  #print '    - status:', client.get_goal_status_text()
  return client.get_result()

if __name__ == '__main__':
  try:
    rospy.init_node('dock_drive_client_py', anonymous=True)
    dock_drive_client()
    #print ''
    #print "Result: ", result
  except rospy.ROSInterruptException:
    print "program interrupted before completion"
*/
