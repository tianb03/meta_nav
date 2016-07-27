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

#include <alsa/asoundlib.h>

char *wav_file;
FILE *fp;
int set_pcm_play(FILE *fp);

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

  while (ros::ok())
  {
    /*弹出*/
    ROS_INFO("backward beginning");
    //说话:充完电了
    wav_file = argv[1];
    fp = fopen(wav_file, "rb");
    set_pcm_play(fp);
    for(int j = 0; j < 48; j++)
    {
      geometry_msgs::Twist backward_vel;
      backward_vel.linear.x = -0.1;
      velocity_command_publisher.publish(backward_vel);
      ros::spinOnce();
      loop_rate.sleep();
    }
    ROS_INFO("backward over");

    //说话:出发
    wav_file = argv[2];
    fp = fopen(wav_file, "rb");
    set_pcm_play(fp);
	  /*yinxiang*/
    do
    {
      goal.target_pose.pose.position.x = 6.474;
      goal.target_pose.pose.position.y = 8.569;
      goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(1.647);
      ROS_INFO("Sending goal software department");
      ac.sendGoal(goal);
      ac.waitForResult();
    }while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);
    //说话:到位
    wav_file = argv[4];
    fp = fopen(wav_file, "rb");
    set_pcm_play(fp);

    ros::Duration(3.0).sleep();

    /*A
    do
    {
      goal.target_pose.pose.position.x = 11.707;
      goal.target_pose.pose.position.y = -0.297;
      goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.00);
      ROS_INFO("Sending goal administration department");
      ac.sendGoal(goal);
      ac.waitForResult();
    }while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);*/

    //说话:出发
    wav_file = argv[2];
    fp = fopen(wav_file, "rb");
    set_pcm_play(fp);
	  /*qiang*/
    do
    {
      goal.target_pose.pose.position.x = 0.045;
      goal.target_pose.pose.position.y = 7.569;
      goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(1.644);
      ROS_INFO("Sending goal coffee machine");
      ac.sendGoal(goal);
      ac.waitForResult();
    }while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);
    //说话:到位
    wav_file = argv[4];
    fp = fopen(wav_file, "rb");
    set_pcm_play(fp);

    ros::Duration(3.0).sleep();
    
    /*
    A
    do
    {
      goal.target_pose.pose.position.x = 11.707;
      goal.target_pose.pose.position.y = -0.297;
      goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(3.10);
      ROS_INFO("Sending goal administration department");
      ac.sendGoal(goal);
      ac.waitForResult();
    }while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);*/

	  /*A
    do
    {
      goal.target_pose.pose.position.x = 4.558;
      goal.target_pose.pose.position.y = 0.013;
      goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(3.10);
      ROS_INFO("Sending goal software department");
      ac.sendGoal(goal);
      ac.waitForResult();
    }while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);*/

	  /*总经理室点
    do
    {
      goal.target_pose.pose.position.x = -2.321;
      goal.target_pose.pose.position.y = 0.345;
      goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(3.10);
      ROS_INFO("Sending goal general manager office");
      ac.sendGoal(goal);
      ac.waitForResult();
    }while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);
    //说话:到位
    wav_file = argv[4];
    fp = fopen(wav_file, "rb");
    set_pcm_play(fp);

    ros::Duration(3.0).sleep();*/
    
    /*//说话:出发
    wav_file = argv[2];
    fp = fopen(wav_file, "rb");
    set_pcm_play(fp);
	  实验室点
    do
    {
      goal.target_pose.pose.position.x = -8.821;
      goal.target_pose.pose.position.y = 2.988;
      goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(-1.657);
      ROS_INFO("Sending goal laboratory");
      ac.sendGoal(goal);
      ac.waitForResult();
	    ros::Duration(3.0).sleep();
    }while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);
    //说话:到位
    wav_file = argv[4];
    fp = fopen(wav_file, "rb");
    set_pcm_play(fp);*/

    //说话:出发
    wav_file = argv[2];
    fp = fopen(wav_file, "rb");
    set_pcm_play(fp);
	  /*chongdianqian*/
    do
    {
      goal.target_pose.pose.position.x = 1.030;
      goal.target_pose.pose.position.y = 7.470;
      goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(-1.057);
      ROS_INFO("Go to charge");
      ac.sendGoal(goal);
      ac.waitForResult();
    }while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);
    //说话:到位
    wav_file = argv[4];
    fp = fopen(wav_file, "rb");
    set_pcm_play(fp);

    ros::Duration(3.0).sleep();
    /*自动充电三秒*/
    do
    {
      bc.sendGoal(goala, doneCb, activeCb, feedbackCb);
      //说话:开始充电
      wav_file = argv[5];
      fp = fopen(wav_file, "rb");
      set_pcm_play(fp);
      bc.waitForResult();
      ros::Duration(3.0).sleep();
    }while(bc.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);
  }

  return 0;
}

int set_pcm_play(FILE *fp)
{
  int rc;
  int ret;
  int size;
  snd_pcm_t* handle; //PCI设备句柄
  snd_pcm_hw_params_t* params;//硬件信息和PCM流配置
  snd_pcm_uframes_t frames;

  unsigned int val;
  int dir=0;
  char *buffer;
  int channels=1;
  int frequency=16000;
  int bit=16;
  int datablock=2;
  unsigned char ch[100]; //用来存储wav文件的头信息
    
  rc=snd_pcm_open(&handle, "default", SND_PCM_STREAM_PLAYBACK, 0);
  if(rc<0)
  {
    perror("\nopen PCM device failed:");
    exit(1);
  }

  //snd_pcm_hw_params_malloc(&params)
  snd_pcm_hw_params_alloca(&params); //分配params结构体
  if(rc<0)
  {
    perror("\nsnd_pcm_hw_params_alloca:");
    exit(1);
  }

  rc=snd_pcm_hw_params_any(handle, params);//初始化params
  if(rc<0)
  {
    perror("\nsnd_pcm_hw_params_any:");
    exit(1);
  }
  rc=snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED); //初始化访问权限
  if(rc<0)
  {
    perror("\nsnd_pcm_hw_set_access:");
    exit(1);
  }

  //采样位数
  switch(bit/8)
  {
    case 1:rc=snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_U8);
           if(rc<0)
           {
             perror("\nsnd_pcm_hw_set_format:");
             exit(1);
           }
           break ;
    case 2:snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_S16_LE);
           if(rc<0)
           {
             perror("\nsnd_pcm_hw_set_format:");
             exit(1);
           }
           break ;
    case 3:snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_S24_LE);
           if(rc<0)
           {
              perror("\nsnd_pcm_hw_set_format:");
              exit(1);
           }
           break ;
  }

  rc=snd_pcm_hw_params_set_channels(handle, params, channels); //设置声道,1表示单声道，2表示立体声
  if(rc<0)
  {
    perror("\nsnd_pcm_hw_params_set_channels:");
    exit(1);
  }
       
  val = frequency;
  rc=snd_pcm_hw_params_set_rate_near(handle, params, &val, &dir); //设置>频率
  if(rc<0)
  {
    perror("\nsnd_pcm_hw_params_set_rate_near:");
    exit(1);
  }

  rc = snd_pcm_hw_params(handle, params);
  if(rc<0)
  {
    perror("\nsnd_pcm_hw_params: ");
    exit(1);
  }

  rc=snd_pcm_hw_params_get_period_size(params, &frames, &dir); /*获取周期长度*/
  if(rc<0)
  {
    perror("\nsnd_pcm_hw_params_get_period_size:");
    exit(1);
  }

  size = frames * datablock; /*4 代表数据快长度*/

  buffer =(char*)malloc(size);
  fseek(fp,58,SEEK_SET); //定位歌曲到数据区,偏移量“58”如何确定的？

  while (1)
  {
    memset(buffer,0,sizeof(buffer));
    ret = fread(buffer, 1, size, fp);
    if(ret == 0)
    {
      printf("wav写入结束\n");
      break;
    }
    else if (ret != size)
    {
    }
    // 写音频数据到PCM设备 
    while(ret = snd_pcm_writei(handle, buffer, frames)<0)
    {
      usleep(2000); 
      if (ret == -EPIPE)
      {
        /* EPIPE means underrun */
        fprintf(stderr, "underrun occurred\n");
        //完成硬件参数设置，使设备准备好 
        snd_pcm_prepare(handle);
      }
      else if (ret < 0)
      {
        fprintf(stderr,"error from writei: %s\n",snd_strerror(ret));
      }
    }
  }
  snd_pcm_drain(handle);
  snd_pcm_close(handle);
  free(buffer);
  return 0;
}
