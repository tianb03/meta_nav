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
#include <iostream>
#include <fstream>
#include <iomanip>
//11 sept 2013 testing
class NewNav
{
public:

   ros::NodeHandle nh_;
   ros::Publisher cmdvel_pub;
   ros::Publisher cmdmstate_pub;
   ros::Publisher blocking_pub;

   ros::Subscriber scan_sub;
   ros::Subscriber pose_sub;
   ros::Subscriber chatter_sub;
   ros::Subscriber sonar_sub;
   double scan_range [672];//total_size
   double scan_filtered [84];//filtered_size
   double scan_left [42];//filtered_size/2
   double scan_right [42];
   double scan_diff [42];
   double scan_weight [42];

   double scan_left2 [42];
   double scan_right2 [42];

   double pose_x, pose_y;
   double roll, pitch, theta;
   float sonar_range;
   ros::Time t; // last received time
   ros::Time timefirst;

   //Constructor
   NewNav(ros::NodeHandle &nh)
   {
      nh_ = nh;
      cmdvel_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel",1);
      //cmdmstate_pub = nh_.advertise<p2os_driver::MotorState>("cmd_motor_state",1);
      blocking_pub = nh_.advertise<std_msgs::String>("blocking",1);
      scan_sub = nh_.subscribe("scan",1, &NewNav::scanCallback,this);
      pose_sub = nh_.subscribe("odom",1, &NewNav::poseCallback,this);
      chatter_sub = nh_.subscribe("chatter",1, &NewNav::chatterCallback,this);
      //sonar_sub = nh_.subscribe("sonar",1, &NewNav::sonarCallback, this);
   }

   void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan)
   {
        sensor_msgs::LaserScan scan_msgs = *scan;

        for (int i=0; i<total_size; i++)
        {
            scan_range [i] = scan_msgs.ranges[i];
        }
        scanFilter ();
   }

   void poseCallback(const nav_msgs::OdometryConstPtr& msg)
   {
       pose_x = msg->pose.pose.position.x;
       pose_y = msg->pose.pose.position.y;
       theta = tf::getYaw(msg->pose.pose.orientation);
       theta = theta*180/3.142;
   }

   void chatterCallback(const std_msgs::String::ConstPtr& msg)
   {
       global_command = msg->data.c_str();
   }

   /*
   void sonarCallback(const p2os_driver::SonarArrayConstPtr& msg)
   {
       p2os_driver::SonarArray msg1 = *msg;
       for (int i=0; i<8; i++)
       {
           sonar_reading [7-i] = msg1.ranges[i];
           if (sonar_reading[7-i]>=0.02 && sonar_reading[7-i]<=5.0)
           {}
           else
               sonar_reading[7-i]=5.0;
       }

       //ROS_INFO("Sonar %f %f %f", sonar_reading[0], sonar_reading[3], sonar_reading[7]);
   }
   */
   void scanFilter ()
   {
       int count=0;
       int i,j;

       //Reduce the size from total_size to filtered_size
       for (i=0; i<filtered_size; i++)
       {   count=0;
           scan_filtered[i]=0.0;
           for (j=0; j<interval; j++)
           {
               if (scan_range [i*interval+j]>5.2) scan_range [i*interval+j]=5.2;
               if (scan_range [i*interval+j]<0.02) scan_range [i*interval+j]=5.0;
               scan_filtered[i] = scan_filtered[i] + scan_range [i*interval+j];
               count++;
           }
           scan_filtered[i]=scan_filtered[i]/double(count);
           if (scan_filtered[i]>0.01 && scan_filtered[i]<10){} else scan_filtered[i]=0.0;
       }

       for (i=0; i<filtered_size/2; i++)
       {
           scan_left [i] = scan_filtered [i];
           scan_right [i] = scan_filtered [filtered_size-1-i];

           if (scan_left[i]>scan_max1) {scan_left[i]=scan_max1;}
           if (scan_left[i]<scan_min1) {scan_left[i]=scan_max1;}
           if (scan_right[i]>scan_max1){scan_right[i]=scan_max1;}
           if (scan_right[i]<scan_min1){scan_right[i]=scan_max1;}
           scan_diff [i] = scan_right [i] - scan_left [i];
       } 
   }

   void scan_weight_superclose ()
   {
       int i;
       double sum_scan_weight=0.0;
       for (i=0; i<filtered_size; i++)
       {
           if (scan_filtered[i]<=0.25)
           {
               if (i>=42)
                   scan_weight[i-(i-41)] = 5.0;
               else
                   scan_weight [i] = 5.0;
           }
           else scan_weight [i] = 1.0;
           sum_scan_weight = sum_scan_weight + scan_weight [i];
       }
       //Determine scan_weight normalized
       for (i=0; i<filtered_size/2; i++){scan_weight [i] = scan_weight [i]/sum_scan_weight;}
   }

   void scan_weight_tooclose ()
   {
       int i;
       double sum_scan_weight=0.0;
       for (i=0; i<filtered_size/2; i++)
       {
           if (scan_filtered[i]<=first_level_distance)
           {
               if (i>=42)
                   scan_weight[i-(i-41)] = 5.0;
               else
                   scan_weight [i] = 5.0;
           }
           else
               scan_weight [i] = 1.0;
           sum_scan_weight = sum_scan_weight + scan_weight [i];
       }
       //Determine scan_weight normalized
       for (i=0; i<filtered_size/2; i++){scan_weight [i] = scan_weight [i]/sum_scan_weight;}
   }

   void scan_weight_constant ()
   {
       int i;
       double sum_scan_weight=0.0;
       for (i=0; i<filtered_size/2; i++)
       {
           scan_weight [i] = 1.0;
           sum_scan_weight = sum_scan_weight + scan_weight [i];
       }
       //Determine scan_weight normalized
       for (i=0; i<filtered_size/2; i++){scan_weight [i] = scan_weight [i]/sum_scan_weight;}
   }

   void scan_weight_triangular ()
   {
       int i;
       double sum_scan_weight=0.0;
       for (i=0; i<filtered_size/2; i++)
       {
           scan_weight [i] = 1.0 + 1.0*double(i);
           sum_scan_weight = sum_scan_weight + scan_weight [i];
       }
       //Determine scan_weight normalized
       for (i=0; i<filtered_size/2; i++){scan_weight [i] = scan_weight [i]/sum_scan_weight;}
   }

   void scan_weight_triangularx2 ()
   {
       int i;
       double sum_scan_weight=0.0;
       for (i=0; i<filtered_size/2; i++)
       {
           scan_weight [i] = 1.0 + 2.0*double(i);
           sum_scan_weight = sum_scan_weight + scan_weight [i];
       }
       //Determine scan_weight normalized
       for (i=0; i<filtered_size/2; i++){scan_weight [i] = scan_weight [i]/sum_scan_weight;}
   }

   void scan_weight_extreme_triangular ()
   {
       int i;
       double sum_scan_weight=0.0;

       for (i=0; i<filtered_size/2; i++)
       {
           scan_left [i] = scan_filtered [i];
           scan_right [i] = scan_filtered [filtered_size-1-i];

           if (i>=31 && i<=53)
           {
               if (scan_left[i]>scan_max2) scan_left[i]=scan_max2;
               if (scan_left[i]<scan_min1) scan_left[i]=scan_max1;
               if (scan_right[i]>scan_max2) scan_right[i]=scan_max2;
               if (scan_right[i]<scan_min1) scan_right[i]=scan_max1;
           }
           else
           {
               if (scan_left[i]>scan_max1) scan_left[i]=scan_max1;
               if (scan_left[i]<scan_min1) scan_left[i]=scan_max1;
               if (scan_right[i]>scan_max1) scan_right[i]=scan_max1;
               if (scan_right[i]<scan_min1) scan_right[i]=scan_max1;
           }
           scan_diff [i] = scan_right [i] - scan_left [i];
       }

       for (i=0; i<filtered_size/2; i++)
       {
           if ( (i>=31 && i<=53) && (scan_left[i]>=scan_max2 || scan_right[i]>=scan_max2))
                scan_weight [i] = 1.0 + double(i)*double(i);
           else
               scan_weight [i] = 1.0;
           sum_scan_weight = sum_scan_weight + scan_weight [i];
       }
       //Determine scan_weight normalized
       for (i=0; i<filtered_size/2; i++)
       {
           scan_weight [i] = scan_weight [i]/sum_scan_weight;
       }
   }

   double turnCalculation ()
   {
       double turn_calc=0.0;
       int i;
       scan_weight_constant ();
       for (i=0; i<filtered_size/2; i++)
       {
           turn_calc = turn_calc + scan_diff[i]*scan_weight[i];
       }
       return (turn_calc);
   }

   void laser_level_check ()
   {
        int i;
        int count1 = 0, count2 = 0, count3 = 0;
        for (i=0; i<filtered_size; i++)
            if (scan_filtered[i]<=0.25) count1++;
        for (i=0; i<filtered_size; i++)
            if (scan_filtered[i]>0.25 && scan_filtered[i]<=first_level_distance) count2++;
        for (i=0; i<filtered_size; i++)
            if (scan_filtered[i]>=second_level_distance) count3++;

        if      (count1 >= 400)                      {laser_status = 1;}
        else if (count2 >= 4)                        {laser_status = 2;}
        else if (count3 >=30 && count3 <=90)         {laser_status = 3;}
        else                                         {laser_status = 0;}
   }

   double dynamic_turnCalculation ()
   {
       double turn_calc=0.0;
       int i;

       //First level - for obstacle avoidance
        if (laser_status == 1) //super close
        {
            scan_weight_superclose ();
            for (i=0; i<filtered_size/2; i++)
            {
                turn_calc = turn_calc + scan_diff[i]*scan_weight[i];
            }
        }
        else if (laser_status == 2) //if obstacle exist in the first laser scan region
        {
            scan_weight_tooclose ();
            for (i=0; i<filtered_size/2; i++)
                turn_calc = turn_calc + scan_diff[i]*scan_weight[i];
        }
        else if (laser_status == 3)
        {
            scan_weight_extreme_triangular ();
            for (i=0; i<filtered_size/2; i++)
                turn_calc = turn_calc + scan_diff[i]*scan_weight[i];

            //scan_weight_triangular ();
            //for (i=10; i<filtered_size/2; i++)
            //    turn_calc = turn_calc + scan_diff[i]*scan_weight[i];
        }
        else //default
        {
            scan_weight_triangular ();
            for (i=0; i<filtered_size/2; i++)
                turn_calc = turn_calc + scan_diff[i]*scan_weight[i];
        }
       return (turn_calc);
   }

   double veloCalculation ()
   {
       double velo_calc=0.0;
       int i;
       for (i=velo_start_beam; i<velo_end_beam; i++)
       {
           velo_calc = velo_calc + scan_filtered[i]*scan_weight[i];
       }
       return (velo_calc);
   }

   double dynamic_veloCalculation ()
   {
       double velo_calc=0.0;
       double side_th = 0.5;
       int sideR_th_count = 0;
       int sideL_th_count = 0;
       int i;
       for (i=5; i<=15; i++)//about 30 degree
       {
           if (scan_filtered[i]<side_th)
               sideR_th_count++;
       }
       for (i=68; i<=78; i++)//about 30 degree
       {
           if (scan_filtered[i]<side_th)
               sideL_th_count++;
       }

       if (laser_status == 1)
       {
            velo_calc = 0.0;
       }
       else if (laser_status == 2) //if obstacle exist in the first laser scan region
       {
            velo_calc = 0.15/velo_rate_coff;
       }
       else
       {
           for (i=velo_start_beam; i<velo_end_beam; i++)
           {
               velo_calc = velo_calc + fabs(scan_filtered[i]*scan_weight[i]);
           }
       }
       return (velo_calc);
   }

   void turn_velo_rate_calculation()
   {
       enableMotor (1);
       laser_level_check ();
       turn_rate = turn_rate_coff*dynamic_turnCalculation ();
       if (turn_rate > turn_rate_max) turn_rate = turn_rate_max;
       if (turn_rate < -1.0*turn_rate_max) turn_rate = -1.0*turn_rate_max;

       velo_rate = velo_rate_coff*dynamic_veloCalculation ();
       if (velo_rate > velo_rate_max) velo_rate = velo_rate_max;
       if (velo_rate < 0.0) velo_rate = 0.0;
   }

   //Start of all subfunctions
   void stop()
   {
       turn_velo_rate_calculation ();
       enableMotor (1);
       moveRobot(0.0, 0.0, 0.0);
       //ROS_INFO("STOP");
       blocking_send (0);
   }

   void full_uTurn_with_break()
   {
       ros::Rate loop_rate(loop_freq_time); //(frequency) use 100 for implementation
       for (int i=0; i<int(9000*0.4/const_turn); i++)
       {
           enableMotor (1);
           moveRobot(0.0, 0.0, const_turn); //const_turn
           ros::spinOnce();
           loop_rate.sleep();
           ROS_INFO("Full U turn with break check");
           blocking_send (0);

           if (scan_filtered[41]>uturn_front_check && scan_filtered[42]>uturn_front_check && scan_filtered[43]>uturn_front_check)
           {
             ROS_INFO("Break Full U turn");
             stop();
             break;
           }
       }
   }

   void uTurn()
   {
       ros::Rate loop_rate(loop_freq_time); //(frequency) use 100 for implementation
       double tmp_theta = theta + 180;
       if (tmp_theta>180)
           tmp_theta = tmp_theta-360;
       //Turn using odomTheta
       while (fabs(tmp_theta - theta)>5.0)
       {
           enableMotor (1);
           moveRobot(0.0, 0.0, -1.0*const_turn); //const_turn
           ros::spinOnce();
           loop_rate.sleep();
           ROS_INFO("temp different is %f", tmp_theta - theta);
           blocking_send (0);
           if (global_command == "STOP") break;
       }

       for (int i=0; i<int(uTurn_stop_time); i++)
       {
           enableMotor (1);
           moveRobot(0.0, 0.0, 0.0);
           ros::spinOnce();
           loop_rate.sleep();
           ROS_INFO("Stop after U-turn");
       }
   }

   void uTurn_block()
   {
       ros::Rate loop_rate(loop_freq_time); //(frequency) use 100 for implementation
       double tmp_theta = theta + 180;
       if (tmp_theta>180)
           tmp_theta = tmp_theta-360;
       //Turn using odomTheta
       while (fabs(tmp_theta - theta)>5.0)
       {
           enableMotor (1);
           moveRobot(0.0, 0.0, -1.0*const_turn); //const_turn
           ros::spinOnce();
           loop_rate.sleep();
           ROS_INFO("temp different is %f", tmp_theta - theta);
           blocking_send (1);
           if (global_command == "STOP") break;
       }

       for (int i=0; i<int(uTurn_stop_time); i++)
       {
           enableMotor (1);
           moveRobot(0.0, 0.0, 0.0);
           ros::spinOnce();
           loop_rate.sleep();
           ROS_INFO("Stop after U-turn");
           blocking_send (1);
       }

       back_distance_first = 1.0;

   }

   void back_distance_calculation()
   {
       if (scan_filtered[73]<scan_filtered[10])
           back_distance_first =scan_filtered [73];
       else
           back_distance_first =scan_filtered [10];

       temp_back_x = pose_x;
       temp_back_y = pose_y;
   }

   void set_back_distance()
   {
       back_distance_first =0.0;
       back_distance = 0.0;
       temp_back_x = pose_x;
       temp_back_y = pose_y;
   }

   void stopTurn()
   {
       ros::Rate loop_rate(loop_freq_time); //(frequency) use 100 for implementation
       turn_velo_rate_calculation ();
       moveRobot(0.0, 0.0, 2.0*turn_rate);
       ROS_INFO("stop turn");
       back_distance_calculation ();
       blocking_send (0);
       ros::spinOnce();
       loop_rate.sleep();
   }

   void turnRight()
   {
       ros::Rate loop_rate(loop_freq_time);
       int path_condition, i;
       for (i=0; i<int(junction_first_forward_time*0.2/const_velo); i++)//move forward a bit after detect right can turn //1500
       {
           turn_velo_rate_calculation();
           path_condition = check_path ();
           if (path_condition==1){moveRobot(0.0, 0.0, 2*turn_rate);}
           else {moveRobot(velo_rate, 0.0, turn_rate);}
           ros::spinOnce();
           loop_rate.sleep();
           ROS_INFO("Right Junction - Turn Right - 1st forward");
           blocking_send (0);
           if (global_command == "STOP") return;
      }
      for (i=0; i<int(junction_turning_time*0.4/const_turn); i++)//Turn right 90 degree
      {
           enableMotor (1);
           moveRobot(0.0, 0.0, -1.0*const_turn);
           ros::spinOnce();
           loop_rate.sleep();
           ROS_INFO("Right Junction - Turn right");
           blocking_send (0);
           if (global_command == "STOP") return;
       }
      waiting_time ();
      for (i=0; i<int(junction_second_forward_time*0.2/const_velo); i++)//move forward further after
      {
          turn_velo_rate_calculation();
          path_condition = check_path ();
          if (path_condition==1){moveRobot(0.0, 0.0, 2*turn_rate);}
          else {moveRobot(velo_rate, 0.0, turn_rate);}
          ros::spinOnce();
          loop_rate.sleep();
          ROS_INFO("Right Junction - Turn Right - 2nd forward");
          blocking_send (0);
          if (global_command == "STOP") return;
      }
      set_back_distance ();
   }

   void turnLeft()
   {
       ros::Rate loop_rate(loop_freq_time);
       int path_condition, i;
       for (i=0; i<int(junction_first_forward_time*0.2/const_velo); i++)//move forward a bit after detect left can turn //1500
       {
           turn_velo_rate_calculation();
           path_condition = check_path ();
           if (path_condition==1){moveRobot(0.0, 0.0, 2.0*turn_rate);}
           else {moveRobot(velo_rate, 0.0, turn_rate);}
           ros::spinOnce();
           loop_rate.sleep();
           ROS_INFO("Left Junction - Turn Left - 1st forward");
           blocking_send (0);
           if (global_command == "STOP") return;
      }
      for (i=0; i<int(junction_turning_time*0.4/const_turn); i++)//Turn Left 90 degree
      {
           enableMotor (1);
           moveRobot(0.0, 0.0, const_turn);
           ros::spinOnce();
           loop_rate.sleep();
           ROS_INFO("Left Junction - Turn Left");
           blocking_send (0);
           if (global_command == "STOP") return;
       }
      waiting_time ();
      for (i=0; i<int(junction_second_forward_time*0.2/const_velo); i++)//move forward further after turn left
      {
          turn_velo_rate_calculation();
          path_condition = check_path ();
          if (path_condition==1){moveRobot(0.0, 0.0, 2.0*turn_rate);}
          else {moveRobot(velo_rate, 0.0, turn_rate);}
          ros::spinOnce();
          loop_rate.sleep();
          ROS_INFO("Left Junction - Turn Left - 2nd forward");
          blocking_send (0);
          if (global_command == "STOP") return;
       }
       set_back_distance ();
   }

   void forwardR()
   {
       ros::Rate loop_rate(loop_freq_time);
       int path_condition, i;
       for (i=0; i<int(junction_forward_time*0.2/const_velo); i++)//forward
       {
           turn_velo_rate_calculation();
           path_condition = check_path ();
           if (path_condition==1){moveRobot(0.0, 0.0, 2.0*turn_rate);}
           else {moveRobot(velo_rate, 0.0, turn_rate);}
           ROS_INFO("Right can turn - Forward");
           blocking_send (0);
           ros::spinOnce();
           loop_rate.sleep();
           if (global_command == "STOP") return;
      }
      set_back_distance ();
   }

   void forwardL()
   {
       ros::Rate loop_rate(loop_freq_time);
       int path_condition, i;
       for (i=0; i<int(junction_forward_time*0.2/const_velo); i++)//forward
       {
           turn_velo_rate_calculation();
           path_condition = check_path ();
           if (path_condition==1){moveRobot(0.0, 0.0, 2.0*turn_rate);}
           else {moveRobot(velo_rate, 0.0, turn_rate);}
           ROS_INFO("Left can turn - Forward");
           blocking_send (0);
           ros::spinOnce();
           loop_rate.sleep();
           if (global_command == "STOP") return;
      }
      set_back_distance ();
   }

   void forwardPlus()
   {
       ros::Rate loop_rate(loop_freq_time);
       int path_condition, i;
       for (i=0; i<int(junction_forward_time*0.2/const_velo); i++)//forward
       {
           turn_velo_rate_calculation();
           path_condition = check_path ();
           if (path_condition==1){moveRobot(0.0, 0.0, 2.0*turn_rate);}
           else {moveRobot(velo_rate, 0.0, turn_rate);}
           ROS_INFO("Plus junction - Forward");
           blocking_send (0);
           ros::spinOnce();
           loop_rate.sleep();
           if (global_command == "STOP") return;
      }
      set_back_distance ();
   }

   void tJunction()
   {
       ros::Rate loop_rate(loop_freq_time);
       int path_condition, i;
       //int rand_value = rand()%2;//return 0-1
       for (i=0; i<int(tjunction_first_forward_time*0.2/const_velo); i++)//move forward a bit after detect T junction
       {
           turn_velo_rate_calculation();
           path_condition = check_path ();
           if (path_condition==1){moveRobot(0.0, 0.0, 2.0*turn_rate);}
           else{moveRobot(velo_rate, 0.0, turn_rate);}
           ROS_INFO("T-junction - 1st forward");
           blocking_send (0);
           ros::spinOnce();
           loop_rate.sleep();
           if (global_command == "STOP") return;
       }
       if (global_command == "RIGH")
       {
           for (i=0; i<int(junction_turning_time*0.4/const_turn); i++)
           {
                enableMotor (1);
                moveRobot(0.0, 0.0, -1.0*const_turn);
                ros::spinOnce();
                loop_rate.sleep();
                ROS_INFO("T-junction - Turn Right");
                blocking_send (0);
                if (global_command == "STOP") return;
            }
       }
       else
       {
           for (i=0; i<int(junction_turning_time*0.4/const_turn); i++)
           {
                enableMotor (1);
                moveRobot(0.0, 0.0, const_turn);
                ros::spinOnce();
                loop_rate.sleep();
                ROS_INFO("T-junction - Turn Left");
                blocking_send (0);
                if (global_command == "STOP") return;
            }
       }
       waiting_time ();
       for (i=0; i<int(junction_second_forward_time*0.2/const_velo); i++)
       {
           turn_velo_rate_calculation();
           path_condition = check_path ();
           if (path_condition==1){moveRobot(0.0, 0.0, 2.0*turn_rate);}
           else{moveRobot(velo_rate, 0.0, turn_rate);}
           ROS_INFO("T-junction - 2nd forward");
           blocking_send (0);
           ros::spinOnce();
           loop_rate.sleep();
           if (global_command == "STOP") return;
        }
        set_back_distance ();
   }

   void PlusJunction()
   {
       ros::Rate loop_rate(loop_freq_time);
       int path_condition, i;
       //int rand_value = rand()%2;//return 0-1
       for (i=0; i<int(junction_first_forward_time*0.2/const_velo); i++)//move forward a bit after detect T junction
       {
           turn_velo_rate_calculation();
           path_condition = check_path ();
           if (path_condition==1){moveRobot(0.0, 0.0, 2.0*turn_rate);}
           else{moveRobot(velo_rate, 0.0, turn_rate);}
           ROS_INFO("Plus-junction - 1st forward");
           blocking_send (0);
           ros::spinOnce();
           loop_rate.sleep();
           if (global_command == "STOP") return;
       }
       if (global_command == "RIGH")
       {
           for (i=0; i<int(junction_turning_time*0.4/const_turn); i++)
           {
                enableMotor (1);
                moveRobot(0.0, 0.0, -1.0*const_turn);
                ros::spinOnce();
                loop_rate.sleep();
                ROS_INFO("Plus-junction - Turn Right");
                blocking_send (0);
                if (global_command == "STOP") return;
            }
       }
       else if (global_command == "LEFT"){
           for (i=0; i<int(junction_turning_time*0.4/const_turn); i++){
                enableMotor (1);
                moveRobot(0.0, 0.0, const_turn);
                ros::spinOnce();
                loop_rate.sleep();
                ROS_INFO("Plus-junction - Turn Left");
                blocking_send (0);
                if (global_command == "STOP") return;}}
       else {forwardPlus ();}

       waiting_time ();
       for (i=0; i<int(junction_second_forward_time*0.2/const_velo); i++)
       {
           turn_velo_rate_calculation();
           path_condition = check_path ();
           if (path_condition==1){moveRobot(0.0, 0.0, 2.0*turn_rate);}
           else{moveRobot(velo_rate, 0.0, turn_rate);}
           ROS_INFO("Plus-junction - 2nd forward");
           blocking_send (0);
           ros::spinOnce();
           loop_rate.sleep();
           if (global_command == "STOP") return;
        }
       back_distance_first =0.0;back_distance = 0.0;temp_back_x = 0.0;temp_back_y = 0.0;
   }

   void goStraight()
   {
       ros::Rate loop_rate(loop_freq_time);
       turn_velo_rate_calculation();
       moveRobot(velo_rate, 0.0, turn_rate);
       ROS_INFO("Go Straight fast");
       back_distance = back_distance_first + sqrt((temp_back_x -pose_x)*(temp_back_x -pose_x) + (temp_back_y -pose_y)*(temp_back_y -pose_y));
       blocking_send (0);
       ros::spinOnce();
       loop_rate.sleep();
   }

   void rotateL()
   {
       moveRobot(0.0, 0.0, const_turn_lr);
       ROS_INFO("Rotate in left");
       blocking_send (0);
   }

   void rotateR()
   {
       moveRobot(0.0, 0.0, -1.0*const_turn_lr);
       ROS_INFO("Rotate in Right");
       blocking_send (0);
   }

   void waiting_time ()
   {
       ros::Rate loop_rate(loop_freq_time);
       int i;
       for (i=0; i<int(junction_instruction_waiting_time); i++)
       {
            enableMotor (1);
            moveRobot(0.0, 0.0, 0.0);
            ros::spinOnce();
            loop_rate.sleep();
            ROS_INFO("Junction-wait for instruction");
            blocking_send (0);
        }
   }

   void controlLRF()
   {
       int path_condition;
       ros::Rate loop_rate(loop_freq_time);
       enableMotor (1);
       path_condition = check_path ();
      // if (path_condition==0) full_uTurn_with_break ();   //Dead end in three sides - U-turn in left

       if (path_condition==0) uTurn_block();
       else blocking_send (0);

       if (path_condition==1) stopTurn(); //Close obstacle in front - stop turn
       else if (path_condition==2)//Right can turn
       {
           if (global_command == "RIGH") {waiting_time (); turnRight();}
           else {waiting_time ();forwardR();}
       }
       else if (path_condition==3)
       {
           if (global_command == "LEFT") {waiting_time (); turnLeft();}
           else {waiting_time (); forwardL();}
       }
       else if (path_condition==4)//T-Junction, choose to turn left or right
       {
           waiting_time ();
           tJunction();
       }
       else if (path_condition==5)//Plus junction
       {
           waiting_time ();
           PlusJunction();
       }
       else //straight
           goStraight();

       ros::spinOnce();
       loop_rate.sleep();
   }

   void goStraightslow()
   {
       ros::Rate loop_rate(loop_freq_time);
       turn_velo_rate_calculation();
       moveRobot(const_velo_slow, 0.0, turn_rate);
       ROS_INFO("Go Straight slow");
       back_distance = back_distance_first + sqrt((temp_back_x -pose_x)*(temp_back_x -pose_x) + (temp_back_y -pose_y)*(temp_back_y -pose_y));
       blocking_send (0);
       ros::spinOnce();
       loop_rate.sleep();
   }

   void forwardslow()
   {
       int path_condition;
       ros::Rate loop_rate(loop_freq_time); //(frequency) use 100 for implementation
       enableMotor (1);
       path_condition = check_path ();
       if (path_condition==0) uTurn_block();//full_uTurn_with_break ();   //Dead end in three sides - U-turn in left
       else blocking_send (0);

       if (path_condition==1) stopTurn(); //Close obstacle in front - stop turn
       else if (path_condition==2) goStraightslow();//Right can turn
       else if (path_condition==3) goStraightslow();//Left can turn
       else if (path_condition==4) goStraightslow();//T-Junction, choose to turn left or right
       else goStraightslow();//straight
       ros::spinOnce();
       loop_rate.sleep();
   }

   void roboControl()
   {
       ROS_INFO("RoboControl Start!!");
       ros::Rate loop_rate(loop_freq_time); //(frequency) use 100 for implementation
       while (nh_.ok())
       {
           //param declaration
           nh_.param("const_velo", const_velo, const_velo);
           nh_.param("const_turn", const_turn, const_turn);
           nh_.param("const_turn_lr", const_turn_lr, const_turn_lr);
           nh_.param("loop_freq_time", loop_freq_time, loop_freq_time);
           nh_.param("junction_first_forward_time", junction_first_forward_time, junction_first_forward_time);
           nh_.param("junction_turning_time", junction_turning_time, junction_turning_time);
           nh_.param("junction_second_forward_time", junction_second_forward_time, junction_second_forward_time);
           nh_.param("junction_forward_time", junction_forward_time, junction_forward_time);
           nh_.param("junction_instruction_waiting_time", junction_instruction_waiting_time, junction_instruction_waiting_time);
           nh_.param("uTurn_stop_time", uTurn_stop_time, uTurn_stop_time);

           enableMotor (1);
           if (global_command.empty()) stop ();
           else if (global_command == "STOP") stop ();
           else if (global_command == "UTUR") uTurn();
           else if (global_command == "ROTL") rotateL ();
           else if (global_command == "ROTR") rotateR ();
           else if (global_command == "FORS") forwardslow ();
           else controlLRF ();
           blocking_send (0);
           ros::spinOnce();
           loop_rate.sleep();
       }//end of while

   }//end of roboControl

   int check_path ()
   {
       int path_condition, i, j;
       int front_count_beam = 0;
       int front_count_beam_block = 0;
       int left_count_beam = 0;
       int right_count_beam = 0;

       int front_count_junction1 = 0;
       int front_count_junction2 = 0;

       int right_count_junction = 0;
       int left_count_junction = 0;

       int t_right_count_junction = 0;
       int t_left_count_junction = 0;

       double sonar_averaged[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

       for (i=0; i<8; i++)
       {    sonar_averaged[i]=0.0;
           for (j=0; j<10; j++)
           {
               sonar_averaged[i] = sonar_averaged[i] + sonar_reading[i];
           }
           sonar_averaged[i] = sonar_averaged[i]/10.0;
       }
       ROS_INFO("sonar averaged %f", sonar_averaged[0]);

       //Calculate number of beams that less than wall_dist in front
       for (i=front_start_beam; i<=front_end_beam; i++)
           if (scan_filtered[i]<front_wall_dist) front_count_beam++;
       for (i=front_start_beam; i<=front_end_beam; i++)
           if (scan_filtered[i]<front_wall_dist_block) front_count_beam_block++;
       for (i=left_start_beam; i<=left_end_beam; i++)
           if (scan_filtered[i]<left_wall_dist) left_count_beam++;
       for (i=right_start_beam; i<=right_end_beam; i++)
           if (scan_filtered[i]<right_wall_dist) right_count_beam++;

       //Used for left or right junctions detection
       for (i=30; i<=55; i++)
           if (scan_filtered[i]>2.0) front_count_junction1++; //To make sure junction is not deteched when facing a corner
       for (i=35; i<=50; i++)
           if (scan_filtered[i]>5.5) front_count_junction2++; //To make sure junction is not detected in a long distance
       for (i=7; i<=14; i++)
           if (scan_filtered[i]>j_right_distance) right_count_junction++;
       for (i=69; i<=76; i++)
           if (scan_filtered[i]>j_left_distance) left_count_junction++;
       for (i=8; i<=13; i++)
           if (scan_filtered[i]>jt_right_distance) t_right_count_junction++;
       for (i=70; i<=75; i++)
           if (scan_filtered[i]>jt_left_distance) t_left_count_junction++;

       //Local view condition
       if (front_count_beam_block>uturn_front_beam_num && left_count_beam>left_beam_num && right_count_beam>right_beam_num) //u turn
           path_condition=0;
       else if (front_count_beam>front_beam_num) //stop turn
           path_condition=1;
       else if (scan_filtered[71]<1.5 && scan_filtered[72]<1.5 && scan_filtered[73]<1.5 &&
                scan_filtered[68]<1.5 && scan_filtered[69]<1.5 && scan_filtered[70]<1.5 &&
                right_count_junction>=2 && back_distance > j_back_distance && sonar_averaged[0]>j_right_distance)//right junction
           path_condition=2;
       else if (front_count_junction1>=3 &&
                scan_filtered[10]<1.5 && scan_filtered[11]<1.5 && scan_filtered[12]<1.5 &&
                scan_filtered[13]<1.5 && scan_filtered[14]<1.5 && scan_filtered[15]<1.5 &&
                left_count_junction>=2 && back_distance > j_back_distance && (sonar_reading[7]>1.5 || sonar_reading[6]>1.5))//left junction
           path_condition=3;
       else if (((scan_filtered[42]<jt_front_distance && scan_filtered[41]<jt_front_distance && scan_filtered[43]<jt_front_distance)
                 ||(sonar_reading [3]<jt_front_distance && sonar_reading[4]<jt_front_distance))
                && t_right_count_junction>=2 && t_left_count_junction>=2)//T-junction
           path_condition=4;
       else if (scan_filtered[42]>=jt_front_distance && scan_filtered[41]>=jt_front_distance && scan_filtered[43]>=jt_front_distance
                && right_count_junction>=2 && left_count_junction>=2 && back_distance > j_back_distance)//Plus-junction
           path_condition=5;    //Plus junction
       else
           path_condition=6;    //straight

       //ROS_INFO("left turn %d %f %f", left_count_junction, sonar_reading[6], sonar_reading[7]);
       path_condition_print = path_condition;
       return (path_condition);
   }

   void checkScan ()
   {
       for (int i=250; i<270; i++)
           ROS_INFO("%d %f",i, scan_range[i]);
   }

   void blocking_send (int block)
   {
       std_msgs::String msg;
       std::stringstream ss;
       if (block == 1) ss << "YES"; //ROS_INFO("Block - Sending YESsssssssssssssss");
       else ss << "NON"; //ROS_INFO("Block - Sending NONnnnnnnnnnnnnnnnn");
       msg.data = ss.str();
       blocking_pub.publish(msg);
   }//end of enableMotor

   void enableMotor (int enable)
   {
      //p2os_driver::MotorState cmdmstate;
      //cmdmstate.state = enable;
      //cmdmstate_pub.publish(cmdmstate);
      //ROS_INFO("cmdstate=%d",enable);
   }//end of enableMotor


   void moveRobot (double x, double y, double z)
   {
       int i;
       geometry_msgs::Twist cmd_vel;
       cmd_vel.linear.x = x;
       cmd_vel.linear.y = y;
       cmd_vel.angular.z = z;
       cmdvel_pub.publish(cmd_vel);

       if (counting_velo == 0 && path_condition_print!=7)
       {
           if(counting_first_timer==0)
           {
               timefirst = ros::Time::now();
               counting_first_timer=1;
           }
           ros::Time timer = ros::Time::now();
           ofstream outputFile;
           outputFile.open("velocity.txt", ios_base::out|ios_base::app);
           outputFile <<(timer - timefirst)<<setprecision(5)<<setw(12)<< x <<setw(12)<< z <<setw(12)<< laser_status <<setw(12)<< pose_x <<setw(12)<<pose_y<<setw(12)<<theta<<setw(12)<<path_condition_print<<endl;
           outputFile.close ();

           outputFile.open("instruction.txt", ios_base::out|ios_base::app);
           outputFile <<setprecision(5)<<setw(12)<< global_command <<endl;
           outputFile.close ();

           outputFile.open("scan_filtered.txt", ios_base::out|ios_base::app);
           outputFile <<(timer - timefirst);
           for (i=0; i<filtered_size; i++)
           {
               outputFile<<setprecision(5)<<setw(15)<<scan_filtered[i];
           }
           outputFile <<endl;
           outputFile.close ();

           outputFile.open("scan_range.txt", ios_base::out|ios_base::app);
           outputFile <<(timer - timefirst);
           for (i=0; i<total_size; i++)
           {
               outputFile<<setprecision(5)<<setw(15)<<scan_range[i];
           }
           outputFile <<endl;
           outputFile.close ();

       }
       counting_velo++;
       if (counting_velo>=100) counting_velo=0;
   }//end of moveRobot

};

