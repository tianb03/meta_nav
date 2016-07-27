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

typedef actionlib::SimpleActionClient<kobuki_msgs::AutoDockingAction> AutoDockingClient;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

char *wav_file, *wav_once;
FILE *fp;
int set_pcm_play(FILE *fp);

bool once = 0;

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

void onceTarget(MoveBaseClient& ac, move_base_msgs::MoveBaseGoal& goal, double position_x, double position_y, double yaw, std::string target_name)
{
  static bool into = 0, out = 0;
  if (once == 1)
    into = 1;
  else
    out = 0, into = 0;
  if (out == 0 && into == 1)
  {
    out = 1;
    goal.target_pose.pose.position.x = position_x;
    goal.target_pose.pose.position.y = position_y;
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    ROS_INFO("%s", target_name.c_str());
    ac.sendGoal(goal);
    once = 1;
    ros::Duration(0.5).sleep();
    ROS_INFO("Enter once");
    //说话:出发
    fp = fopen(wav_once, "rb");
    set_pcm_play(fp);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo_702_new");
  //tell the action client that we want to spin a thread by default
  ros::NodeHandle nh;
  MoveBaseClient ac("move_base", true);
  AutoDockingClient bc("dock_drive_action", true);
  move_base_msgs::MoveBaseGoal goal;
  kobuki_msgs::AutoDockingGoal goala; // define a message is different from python
  //we'll send a goal to the robot to move 1 meter forward
  ros::Rate loop_rate(10);
  ros::Publisher velocity_command_publisher = nh.advertise< geometry_msgs::Twist >("/mobile_base/commands/velocity", 1);
  goal.target_pose.header.frame_id = "map";  //目标姿态，头属性，坐标系的ID
  goal.target_pose.header.stamp = ros::Time::now();  //时间戳
  //wait for the action server to come up

  wav_once = argv[2];

  while(!ac.waitForServer(ros::Duration(2.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

/*Enumerator:
PENDING  待定
ACTIVE  活跃的
RECALLED  回忆起
REJECTED  拒绝
PREEMPTED  抢占
ABORTED  中止
SUCCEEDED  成功
LOST  失去
*/

//argv[1]:充完电了;argv[2]:出发;argv[3]:请你避让;argv[4]:到位;argv[5]:开始充电

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

	  /*软件部窄路点:4.558, 0.013, 0.00, "Sending goal software department"*/
    do
    {
      onceTarget(ac, goal, 4.558, 0.013, 0.00, "Sending goal software department");

      /*if (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
      {
        //说话:出发
        wav_file = argv[2];
        fp = fopen(wav_file, "rb");
        set_pcm_play(fp);
      }*/
      if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
      {
        //说话:请你避让
        wav_file = argv[3];
        fp = fopen(wav_file, "rb");
        set_pcm_play(fp);
        ac.sendGoal(goal);
        once = 1;
      }
    }while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      once = 0;
    }

    /*行政部窄路点:11.707, -0.297, 0.00, "Sending goal administration department"*/
    do
    {
      onceTarget(ac, goal, 11.707, -0.297, 0.00, "Sending goal administration department");

      if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
      {
        //说话:请你避让
        wav_file = argv[3];
        fp = fopen(wav_file, "rb");
        set_pcm_play(fp);
        ac.sendGoal(goal);
        once = 1;
      }
    }while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      once = 0;
    }

	  /*咖啡机点:15.389, 1.067, -1.946, "Sending goal coffee machine"*/
    do
    {
      onceTarget(ac, goal, 15.389, 1.067, -1.946, "Sending goal coffee machine");

      if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
      {
        //说话:请你避让
        wav_file = argv[3];
        fp = fopen(wav_file, "rb");
        set_pcm_play(fp);
        ac.sendGoal(goal);
        once = 1;
      }
      ros::Duration(3.0).sleep();
    }while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      //说话:到位
      wav_file = argv[4];
      fp = fopen(wav_file, "rb");
      set_pcm_play(fp);
      once = 0;
    }

    /*行政部窄路点:11.707, -0.297, 3.10, "Sending goal administration department"*/
    do
    {
      onceTarget(ac, goal, 11.707, -0.297, 3.10, "Sending goal administration department");

      /*if (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
      {
        //说话:出发
        wav_file = argv[2];
        fp = fopen(wav_file, "rb");
        set_pcm_play(fp);
      }*/
      if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
      {
        //说话:请你避让
        wav_file = argv[3];
        fp = fopen(wav_file, "rb");
        set_pcm_play(fp);
        ac.sendGoal(goal);
        once = 1;
      }
    }while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      once = 0;
    }

	  /*软件部窄路点:4.558, 0.013, 3.10, "Sending goal software department"*/
    do
    {
      onceTarget(ac, goal, 4.558, 0.013, 3.10, "Sending goal software department");

      if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
      {
        //说话:请你避让
        wav_file = argv[3];
        fp = fopen(wav_file, "rb");
        set_pcm_play(fp);
        ac.sendGoal(goal);
        once = 1;
      }
    }while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      once = 0;
    }

	  /*总经理室点:-2.321, 0.345, 3.10, "Sending goal general manager office"*/
    do
    {
      onceTarget(ac, goal, -2.321, 0.345, 3.10, "Sending goal general manager office");

      if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
      {
        //说话:请你避让
        wav_file = argv[3];
        fp = fopen(wav_file, "rb");
        set_pcm_play(fp);
        ac.sendGoal(goal);
        once = 1;
      }
      ros::Duration(3.0).sleep();
    }while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      //说话:到位
      wav_file = argv[4];
      fp = fopen(wav_file, "rb");
      set_pcm_play(fp);
      once = 0;
    }

	  /*实验室点:-8.821, 2.988, -1.657, "Sending goal laboratory"
    do
    {
      onceTarget(ac, goal, -8.821, 2.988, -1.657, "Sending goal laboratory");

      //if (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
      //{
        //说话:出发
        //wav_file = argv[2];
        //fp = fopen(wav_file, "rb");
        //set_pcm_play(fp);
      //}
      if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
      {
        //说话:请你避让
        wav_file = argv[3];
        fp = fopen(wav_file, "rb");
        set_pcm_play(fp);
        ac.sendGoal(goal);
        once = 1;
      }
	    ros::Duration(3.0).sleep();
    }while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      //说话:到位
      wav_file = argv[4];
      fp = fopen(wav_file, "rb");
      set_pcm_play(fp);
      once = 0;
    }*/

	  /*充电准备点:1.861, -1.055, -1.744, "Go to charge"*/
    do
    {
      onceTarget(ac, goal, 1.861, -1.055, -1.744, "Go to charge");

      /*if (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
      {
        //说话:出发
        wav_file = argv[2];
        fp = fopen(wav_file, "rb");
        set_pcm_play(fp);
      }*/
      if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
      {
        //说话:请你避让
        wav_file = argv[3];
        fp = fopen(wav_file, "rb");
        set_pcm_play(fp);
        ac.sendGoal(goal);
        once = 1;
      }
    }while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      //说话:到位
      wav_file = argv[4];
      fp = fopen(wav_file, "rb");
      set_pcm_play(fp);
      once = 0;
    }

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
