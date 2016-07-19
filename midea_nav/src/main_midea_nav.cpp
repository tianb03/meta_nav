#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include <kobuki_msgs/BumperEvent.h>

#include <tf/transform_datatypes.h>

const double PI = 3.14159;
double linear_speed = 0.1;
double angular_speed = 0.4;

class MideaNav
{
  public:
  ros::NodeHandle nh_;

  MideaNav();
  ~MideaNav();

  void process();
  void moveRobot(double x, double y, double z);
  double EuDistance(double pre_x,double pre_y, double curr_x, double curr_y);
  double OrienDistance(double pre_theta, double curr_theta);
  double move_forward(double set_dist);
  double turn_right();//based on accumulated turning angle
  double turn_right2(double desired_orientation);//based on imu theta
  double check_theta(double theta_to_check);
  double stop();
  double big_to_small_pattern();
  double square_pattern();
  double square_pattern2();
  double small_square_pattern();


  geometry_msgs::Twist cmd;
  ros::Publisher vel_pub_;

  ros::Subscriber pose_sub;
  ros::Subscriber imu_sub;
  ros::Subscriber bumper_sub;

  void odomCallback(const nav_msgs::OdometryConstPtr& msg);
  void imuCallback(const sensor_msgs::ImuConstPtr& msg);
  void bumperCallback(const kobuki_msgs::BumperEventConstPtr msg);

  double pose_x, pose_y, theta;
  double imu_theta;
  int bumper_no, bumper_state;

};

MideaNav::MideaNav()
{
    ROS_INFO("midea_nav node is running");
    cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;
    pose_x = pose_y = theta = imu_theta = 0.0;
    bumper_no = bumper_state = 0;
    vel_pub_ = nh_.advertise<geometry_msgs::Twist> ("mobile_base/commands/velocity",1000);
    pose_sub = nh_.subscribe("/midea/odom",1,&MideaNav::odomCallback,this);
    imu_sub = nh_.subscribe("/mobile_base/sensors/imu_data",1,&MideaNav::imuCallback,this);
    bumper_sub = nh_.subscribe("/mobile_base/events/bumper",1,&MideaNav::bumperCallback,this);
}

MideaNav::~MideaNav()
{
    ROS_INFO("Shutting down midea_nav node");
}

void MideaNav::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    pose_x = msg->pose.pose.position.x;
    pose_y = msg->pose.pose.position.y;
    theta = tf::getYaw(msg->pose.pose.orientation);
}

void MideaNav::imuCallback(const sensor_msgs::ImuConstPtr& msg)
{
    imu_theta = tf::getYaw(msg->orientation);
}

void MideaNav::bumperCallback(const kobuki_msgs::BumperEventConstPtr msg)
{
    bumper_no = msg->bumper;
    bumper_state = msg->state;

}

void MideaNav::moveRobot(double x, double y, double z)
{
    cmd.linear.x = x;
    cmd.linear.y = y;
    cmd.angular.z = z;
    vel_pub_.publish(cmd);
}

double MideaNav::EuDistance(double pre_x,double pre_y, double curr_x, double curr_y)
{
    double distance;
    distance = (pre_x-curr_x)*(pre_x-curr_x) + (pre_y-curr_y)*(pre_y-curr_y);
    return distance = sqrt(distance);
}

double MideaNav::OrienDistance(double pre_theta, double curr_theta)
{
    return fabs(pre_theta - curr_theta);
}


double MideaNav::move_forward(double set_dist)
{
    ros::Rate loop_rate(10);
    double distance_travelled=0.0;
    double pre_x=pose_x;
    double pre_y=pose_y;
    double current_x=pose_x;
    double current_y=pose_y;
    double pre_theta=theta;

    //forward set_dist m
    while (distance_travelled<set_dist)
    {
        if (EuDistance(pre_x, pre_y, current_x, current_y)<0.1)
        {
            distance_travelled = distance_travelled + EuDistance(pre_x, pre_y, current_x, current_y);
            if (pre_theta-theta<10*PI/180)
                moveRobot(-linear_speed, 0.0, pre_theta-theta);
            else
                moveRobot(-linear_speed, 0.0, 0.0);

            ros::spinOnce();
            loop_rate.sleep();
        }
        pre_x=current_x;
        pre_y=current_y;
        current_x=pose_x;
        current_y=pose_y;
        //std::cout<<"forward"<<distance_travelled<<" "<<pre_x<<" "<<pre_y<<" "<<current_x<<" "<<current_y<<std::endl;
        std::cout<<"forward"<<" "<<distance_travelled<<std::endl;
    }

}

double MideaNav::check_theta(double theta_to_check)
{
    if (theta_to_check<0.0)
        theta_to_check = PI + PI + theta_to_check;

    return theta_to_check;
}

double MideaNav::turn_right()
{
    ros::Rate loop_rate(10);
    double orientation_travelled=0.0;
    double pre_orien=theta;
    double current_orien=theta;
    double desired_degree = 0.0;

    current_orien = check_theta(current_orien);
    pre_orien = check_theta(pre_orien);
    desired_degree = current_orien - PI/2; //-PI/2 denote turning 90 clockwise, or rightward turning
    if (desired_degree < 0.0)
        desired_degree = PI + PI + desired_degree;
    if (desired_degree > 2*PI)
        desired_degree = desired_degree - 2*PI;

    //while (fabs(current_orien-desired_degree)<0.2)
    while (orientation_travelled<(83.5*PI/180))
    {
        current_orien = check_theta(current_orien);
        pre_orien = check_theta(pre_orien);
        //std::cout<<orientation_travelled<<" "<<theta<<std::endl;

        if (OrienDistance(pre_orien, current_orien)<0.1)
        {
            orientation_travelled = orientation_travelled + OrienDistance(pre_orien, current_orien);
            moveRobot(0.0, 0.0, -angular_speed);
        }
        ros::spinOnce();
        loop_rate.sleep();
        pre_orien=current_orien;
        current_orien=theta;
        std::cout<<"turn right"<<"  "<<orientation_travelled<<std::endl;
    }
}

double MideaNav::turn_right2(double desired_orientation)
{
    ros::Rate loop_rate(10);
    double current_orien=theta;
    current_orien = check_theta(current_orien);
    std::cout<<"angle different = "<<fabs(desired_orientation*PI/180-current_orien)<<std::endl;

    while(fabs(desired_orientation*PI/180-current_orien)>0.05)
    {
        current_orien=theta;
        current_orien = check_theta(current_orien);
        moveRobot(0.0, 0.0, -angular_speed);
        std::cout<<"theta = "<<theta<<std::endl;
        std::cout<<"angle different = "<<fabs(desired_orientation*PI/180-current_orien)<<std::endl;
        ros::spinOnce();
        loop_rate.sleep();
    }
}

double MideaNav::stop()
{
    ros::Rate loop_rate(10);
    int i;
    for (i=0; i<10; i++)
    {
        moveRobot(0.0, 0.0, 0.0);
        ros::spinOnce();
        loop_rate.sleep();
        std::cout<<"stop"<<std::endl;
    }
}

double MideaNav::big_to_small_pattern()
{
    int i;
    for (i=0; i<8; i++)
    {
         move_forward(0.9);
         stop();
         turn_right();
         stop();
    }
    for (i=0; i<8; i++)
    {
         move_forward(0.6);
         stop();
         turn_right();
         stop();
    }
    for (i=0; i<8; i++)
    {
         move_forward(0.3);
         stop();
         turn_right();
         stop();
    }
}

double MideaNav::small_square_pattern()
{
    int i;
    for (i=0; i<8; i++)
    {
         move_forward(0.3);
         stop();
         turn_right();
         stop();
    }
}

double MideaNav::square_pattern()
{
    int i;
    small_square_pattern();
    move_forward(0.3); stop();
    small_square_pattern();
    move_forward(0.3); stop();
    small_square_pattern();
    move_forward(0.3); stop();
}

double MideaNav::square_pattern2()
{
    int i;
    for(i=0; i<3; i++)
    {
        move_forward(0.3); stop();
        turn_right2(270);stop();
        move_forward(0.3); stop();
        turn_right2(180);stop();
        move_forward(0.3); stop();
        turn_right2(90);stop();
        move_forward(0.3); stop();
        turn_right2(0);stop();
        move_forward(0.3); stop();
    }
}

void MideaNav::process()
{
    ROS_INFO("Processing start");
    ros::Rate loop_rate(10);

    int counter = 0;

    while (nh_.ok())
    {
        if (counter==0)
        {
           //big_to_small_pattern();
           square_pattern();
           //square_pattern2();
           counter++;
        }
        else
        {
            std::cout<<"Process completed"<<std::endl;
        }

        ros::spinOnce();
        loop_rate.sleep();
        //std::cout<<"distance=="<<distance_travelled<<std::endl;

    }
}

int main(int argc, char **argv)
{
  ROS_INFO("midea_nav starts here");
  ros::init(argc, argv, "talker");

  MideaNav nav;
  nav.process ();

  return 0;
}
