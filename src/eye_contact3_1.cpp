#include <ros/ros.h>
#include <iostream>

#include "humans_msgs/Humans.h"
#include "std_msgs/Bool.h"
#include <fstream>
#include <functional>
#include <algorithm>
#include <numeric>

#define STATE1 1
#define STATE2 2
#define STATE3 3

using namespace std;

class EyeContact
{
private:
  ros::NodeHandle nh;
  ros::Subscriber eye_sub;
  ros::Publisher eye_pub;

  vector<int> point_buff;
  int queue_size;
  int tolerance;
  int conf_th;
  int contact_count;

  int co_time[2];
  int th_time[2];

  stringstream file_name;
  int state;
  //ofstream ofs;

public:
  EyeContact()
  {   
    state = STATE1; 
    contact_count = 0;

    queue_size = 10;


    co_time[0] = 0;
    co_time[1] = 0;
    th_time[0] = 10;
    th_time[1] = 5;

    for(int i = 0; i < queue_size; ++i)
      point_buff.push_back(0);
    
    tolerance = 7;
    conf_th = 100;
    eye_sub = nh.subscribe("/humans/okao_server", 1, 
			   &EyeContact::Callback, this);

    eye_pub = nh.advertise<std_msgs::Bool>("/humans/eye_contact", 1);
  }
  ~EyeContact()
  {

  }

  void Callback(const humans_msgs::HumansConstPtr& msg)
  {
    std_msgs::Bool torf;
    for(int i = 0; i < msg->human.size(); ++i)
      {
	int dir_horizon = msg->human[i].face.direction.x;
	int gaze_horizon = msg->human[i].face.gaze_direction.x;
	int dir_conf = msg->human[i].face.direction.conf;
	int gaze_conf = msg->human[i].face.gaze_direction.conf;

	cout <<"face:" << dir_horizon <<", gaze:" << gaze_horizon 
	     << ", face_conf:"<< dir_conf << ", gaze_conf:"<< gaze_conf << endl;

	point_buff.push_back(PointEyeContactDirAndGaze(dir_horizon, gaze_horizon, dir_conf, gaze_conf));
	point_buff.erase(point_buff.begin());

	//現在目が合っているかどうか判定
	bool contact_state;
	int point_sum = accumulate(point_buff.begin(), point_buff.end(), 0);
	cout << "point_sum:" << point_sum << endl;
	if( point_sum > queue_size/2)
	  {
	    contact_state = true;
	  }
	else
	  {
	    contact_state = false;
	  }


	//状態遷移
	bool mode;
	if(state == STATE1)
	  {
	    if(contact_state)
	      {
		state = STATE2;
		mode = true;
	      }
	    else
	      {
		state = STATE1;
		mode = false;
	      }  
	  }
	else if(state == STATE2)
	  {
	    co_time[1] = 0;
	    ++co_time[0];
	    if( !contact_state )
	      state = STATE1;
	    else
	      {
		if( co_time[0] > th_time[0] )
		  state = STATE3;
		else
		  state = STATE2;
	      }
	    mode = false;
	  }
	else if(state == STATE3)
	  {
	    co_time[0] = 0;
	    ++co_time[1];
	   
	    if( !contact_state )
	      state = STATE1;
	    else
	      {
		if( co_time[1] > th_time[1] )
		  state = STATE2;
		else
		  state = STATE3;
	      }
	    mode = true;
	  }
	cout << "now state: "<<state<<endl;
	torf.data= mode;	
       
	eye_pub.publish( torf );
      }
  }


  int PointEyeContactDirAndGaze(int f_horizon, int g_horizon, int f_conf, int g_conf)
  {
    //cout <<"horizon:" <<horizon << ", conf:"<< conf<< endl;
    int diagonally_point = queue_size/3;
    int straight_point = 1;
    int zero_point = 0;
    if(f_conf>100 && g_conf>100)
      {
	if( f_horizon < 0 && g_horizon > 0 )
	  {
	    return diagonally_point;
	  }    
	else if( f_horizon > 0 && g_horizon < 0 )
	  {
	    return diagonally_point;
	  }
	else if(abs(f_horizon) < tolerance && tolerance>abs(abs(abs(f_horizon) - abs(g_horizon))))
	  {
	    return straight_point;
	  }
	else
	  return zero_point;
      }
    else 
      return zero_point;
  }


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "eye_contact");
  
  EyeContact ec;
  
  ros::spin();
  return 0;
}
