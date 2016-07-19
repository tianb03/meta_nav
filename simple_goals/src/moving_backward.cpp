#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <kobuki_dock_drive/dock_drive.hpp>
#include "kobuki_auto_docking/auto_docking_ros.hpp"
/*void moving_backward()
{
      //  ros::NodeHandle nh;
            ros::Publisher velocity_command_publisher = nh.advertise< geometry_msgs::Twist >("/mobile_base/commands/velocity", 10);
                ros::Rate loop_rate(10);
                    for(int j=0;j<8;j++)
                            {
                                        geometry_msgs::Twist backward_vel;
                                                backward_vel.linear.x = -0.1;
                                                        backward_vel.linear.y = 0.0;
                                                                backward_vel.linear.z = 0.0;
                                                                        backward_vel.angular.x = 0.0;
                                                                                backward_vel.angular.y = 0.0;
                                                                                        backward_vel.angular.z = 0.0;
                                                                                                ROS_INFO("backward");
                                                                                                        velocity_command_publisher.publish(backward_vel);
                                                                                                                ros::spinOnce();
                                                                                                                        loop_rate.sleep();
                                                                                                                            }
}
*/
int main(int argc, char** argv)
{
        ros::init(argc, argv, "auto_dock_test");
          ROS_INFO("test");
                
                    ros::NodeHandle nh;
                        ros::Publisher velocity_command_publisher = nh.advertise< geometry_msgs::Twist >("/mobile_base/commands/velocity", 10);
                            ros::Rate loop_rate(10);
                                for(int j=0;j<8;j++)
                                        {
                                                    geometry_msgs::Twist backward_vel;
                                                            backward_vel.linear.x = -0.1;
                                                                    backward_vel.linear.y = 0.0;
                                                                            backward_vel.linear.z = 0.0;
                                                                                    backward_vel.angular.x = 0.0;
                                                                                            backward_vel.angular.y = 0.0;
                                                                                                    backward_vel.angular.z = 0.0;
                                                                                                            ROS_INFO("backward");
                                                                                                                    velocity_command_publisher.publish(backward_vel);
                                                                                                                            ros::spinOnce();
                                                                                                                                    loop_rate.sleep();
                                                                                                                                        }

                ROS_INFO("test");
                  return 0;
}
