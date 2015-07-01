#include <ros/ros.h>
#include <iostream>
#include <queue>
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
  vector<bool> buff;
  vector<int> point_buff;
  int queue_size;
  int tolerance;
  int conf_th;
  int contact_count;
  int period;
  int co_time[2];
  int th_time[2];
  bool inv;
  stringstream file_name;
  int state;
  //ofstream ofs;

public:
  EyeContact()
  {   
    state = STATE1; 
    contact_count = 0;
    period = 10;
    queue_size = 10;
    inv = false;

    co_time[0] = 0;
    co_time[1] = 0;
    th_time[0] = 10;
    th_time[1] = 5;

    for(int i = 0; i < queue_size; ++i)
      buff.push_back(false);

    for(int i = 0; i < queue_size; ++i)
      point_buff.push_back(0);
    
    tolerance = 7;
    conf_th = 100;
    eye_sub = nh.subscribe("/humans/okao_server", 1, 
			   &EyeContact::Callback, this);

    eye_pub = nh.advertise<std_msgs::Bool>("/humans/eye_contact", 1);

   //ファイル名
    time_t now = time(NULL);
    struct tm *pnow = localtime(&now);
    file_name <<"/home/uema/histdata/" 
	      << pnow->tm_year+1900<< "-"<< pnow->tm_mon<< "-" 
	      << pnow->tm_mday<< "_" <<pnow->tm_hour<<"-"
	      << pnow->tm_min<< "-" << pnow->tm_sec<<".txt"; 


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

	//ofstream ofs( file_name.str().c_str(), std::ios::out | std::ios::app );
	//ofs << dir_horizon << ", " << dir_conf 
	//    << ", " << gaze_horizon << ", " << gaze_conf << endl;

	/*
	ofstream ofs( file_name.str().c_str(), std::ios::out | std::ios::app );
	ofs << gaze_horizon << ", " << gaze_conf << endl;
	*/

	cout <<"face:" << dir_horizon <<", gaze:" << gaze_horizon 
	     << ", face_conf:"<< dir_conf << ", gaze_conf:"<< gaze_conf << endl;
	buff.push_back(CheckEyeContactDirAndGaze(dir_horizon, gaze_horizon, dir_conf, gaze_conf));
	point_buff.push_back(PointEyeContactDirAndGaze(dir_horizon, gaze_horizon, dir_conf, gaze_conf));
	//buff.push_back(CheckEyeContactDir(dir_horizon, dir_conf));
	buff.erase(buff.begin());
	point_buff.erase(point_buff.begin());

	/*
	int eye_state[2];
	eye_state[0] = 0;
	eye_state[1] = 0;

	for(vector<bool>::iterator it = buff.begin(); it != buff.end(); ++it)
	  {
	    if(*it == true)
	      {
		++eye_state[0];
	      }
	    else
	      {
		++eye_state[1];
	      }
	  }
	

	//現在目が合っているかどうか判定
	bool contact_state;
	if(eye_state[0]>eye_state[1])
	  {
	    contact_state = true;
	  }
	else
	  {
	    contact_state = false;
	  }
	*/
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
	      }
	    else
	      state = STATE1;

	    mode = false;  
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
	  //cout << "contact count: "<<contact_count<<endl;

	torf.data= mode;	
       
	eye_pub.publish( torf );
      }
  }

  bool CheckEyeContactDir(int horizon, int conf)
  {

    
    if(tolerance>abs(horizon))
      if(conf > conf_th)
	return true;
      else
	return false;
    else
      return false;
  }

  bool CheckEyeContactDirAndGaze(int f_horizon, int g_horizon, int f_conf, int g_conf)
  {
    //cout <<"horizon:" <<horizon << ", conf:"<< conf<< endl;

    if(f_conf && g_conf)
      {
	if( f_horizon < 0 && g_horizon > 0 )
	  {
	    return true;
	  }    
	else if( f_horizon > 0 && g_horizon < 0 )
	  {
	    return true;
	  }
	else if(abs(f_horizon) < tolerance && tolerance>abs(abs(abs(f_horizon) - abs(g_horizon))))
	  {
	/*
	if(conf > conf_th)
	  return true;
	else
	  return false;
	*/
	    return true;
	  }
	else
	  return false;
      }
    else 
      return false;
  }

  int PointEyeContactDirAndGaze(int f_horizon, int g_horizon, int f_conf, int g_conf)
  {
    //cout <<"horizon:" <<horizon << ", conf:"<< conf<< endl;
    int diagonally_point = queue_size;
    int straight_point = 1;
    int zero_point = 0;
    if(f_conf && g_conf)
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
	/*
	if(conf > conf_th)
	  return true;
	else
	  return false;
	*/
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
