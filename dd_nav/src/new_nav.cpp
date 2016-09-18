//Navigation with global planner 22 July 2013

#include "ros/ros.h"
#include "stdio.h"
#include "std_msgs/String.h"
#include <sensor_msgs/LaserScan.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value
//#include <LinearMath/btQuaternion.h> // Needed to convert rotation ...
//#include <LinearMath/btMatrix3x3.h>  // ... quaternion into Euler angles
#include <tf/transform_listener.h>
#include "std_msgs/String.h"
#include <sstream>
#include <string>
#include "global.h"
#include "new_nav.h"

//For Coldar Demo 11 Mar 2014
int main(int argc, char **argv)
{
    //ROS_INFO("Hello world!!");
    ros::init(argc, argv, "New_Nav");
    ros::NodeHandle nh;

    NewNav new_navigation(nh);
    new_navigation.roboControl();
    return 0;

}
