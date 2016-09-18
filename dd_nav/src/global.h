#include <string>

using namespace std;

//laser beam filtering
int interval = 8;       //Fix
int filtered_size = 84; //Fix //interval * filtered_size = 512
int total_size = 672;   //Fix

//control straight line movement
double velo_rate;
double velo_rate_max = 0.5;
double turn_rate;            //The degree of turn from calculation
double turn_rate_max = 0.2;  //To be tuned - this is the best setting so far regardless of the maze
double scan_max1 = 1.2;      //To be tuned - depend on maze size - better slightly smaller than 1/2 of the path size
double scan_max2 = 5.5;      //5.5
double scan_min1 = 0.1;      //Fixed
double turn_rate_coff = 2.0; //Fixed
double velo_rate_coff = 0.4; //0.4

//constrant velo and turn rate
double const_velo_slow = 0.08;//0.1
double const_velo = 0.3;     //Fix                          //nh_.param
double const_turn = 0.4;     //0.4 Fix                          //nh_.param
double const_turn_lr = 0.2;  //Fix                          //nh_.param

//waiting time - 3000 = 3s with loop_freq_time = 1000
double loop_freq_time = 1000;                    //Fix      //nh_.param
double junction_first_forward_time = 0;          //Fix      //nh_.param //1500
double tjunction_first_forward_time = 2000;          //Fix      //nh_.param //1500
double junction_turning_time = 3500;             //Fix      //nh_.param
double junction_second_forward_time = 3800;      //Fix      //nh_.param
double junction_forward_time = 5000;             //Fix      //nh_.param
double junction_instruction_waiting_time = 3000;    //Fix      //nh_.param //3000 original
double uTurn_stop_time = 3000;                   //Fix      //nh_.param

//check front
double front_wall_dist = 0.8;  //To be tuned //distance to wall to be considered as wall exists
double front_wall_dist_block = 0.8;  //To be tuned //distance to wall to be considered as wall exists
int front_start_beam = 31;     //To be tuned  //16
int front_end_beam = 53;       //To be tuned  //48
int front_beam_num = 10;        //To be tuned  //8//number of beams when a wall exists
int uturn_front_beam_num = 20; //50 never block

//check right
double right_wall_dist = 1.5; //To be tuned //distance to wall to be considered as wall exists
int right_start_beam = 10;     //To be tuned
int right_end_beam = 25;      //To be tuned
int right_beam_num = 12;      //12//To be tuned //number of beams when a wall exists

//check left
double left_wall_dist = 1.5;  //To be tuned //distance to wall to be considered as wall exists
int left_start_beam = 59;     //To be tuned
int left_end_beam = 73;       //To be tuned
int left_beam_num = 12;       //12//To be tuned //number of beams when a wall exists

//General parameters
double back_distance = 0.0;   //Fix
double temp_back_x = 0.0;     //Fix
double temp_back_y = 0.0;     //Fix
double back_distance_first = 0.0; //Fix

//Velocity front
int velo_start_beam = 34;
int velo_end_beam = 50;

//Junction detection
double j_left_distance = 4.5;
double j_right_distance = 4.5;
double jt_left_distance = 1.5;
double jt_right_distance = 1.5;
double jt_front_distance = 1.2; //must higher than the front_wall_dist, else it will consider as wall instead of t-junction
double j_back_distance = 1.2;

//laser lavel check
double first_level_distance = 0.4; //too close to the robot
double second_level_distance = 2.0;
int laser_status = 0;

double uturn_front_check = 2.0;   //To be tuned - Not in used

string global_command;
int counting_velo = 0;
int counting_first_timer = 0;

double sonar_reading [8]={5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0};
double scan_filtered_sonar [8]={5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0};

int path_condition_print = 7;
