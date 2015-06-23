#include <ros/ros.h>
#include <iostream>
#include <queue>
#include "humans_msgs/Humans.h"
#include "std_msgs/Bool.h"
#include <fstream>
#include <functional>
#include <algorithm>

using namespace std;

class EyeContact
{
private:
  ros::NodeHandle nh;
  ros::Subscriber eye_sub;
  ros::Publisher eye_pub;
  vector<bool> buff;
  int queue_size;
  int tolerance;
  int conf_th;

  stringstream file_name;
  //ofstream ofs;

public:
  EyeContact()
  {    
    queue_size = 15;
    
    for(int i = 0; i < queue_size; ++i)
      buff.push_back(false);
    
    tolerance = 10;
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

	ofstream ofs( file_name.str().c_str(), std::ios::out | std::ios::app );
	ofs << dir_horizon << ", " << dir_conf 
	    << ", " << gaze_horizon << ", " << gaze_conf << endl;

	cout <<"face:" << dir_horizon <<", gaze:" << gaze_horizon 
	     << ", face_conf:"<< dir_conf << ", gaze_conf:"<< gaze_conf << endl;
	//buff.push_back(CheckEyeContactDirGaze(dir_horizon, gaze_horizon, dir_conf));
	buff.push_back(CheckEyeContactDir(dir_horizon, dir_conf));
	buff.erase(buff.begin());


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

	if(eye_state[0]>eye_state[1])
	  {
	    ROS_INFO("eye contact!! %d, %d", eye_state[0],eye_state[1]);
	    torf.data= true;
	    eye_pub.publish( torf );
	  }
	else
	  {
	    ROS_WARN("eye not contact!! %d, %d", eye_state[0],eye_state[1]);
	    torf.data= false;
	    eye_pub.publish( torf );
	  }
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

  bool CheckEyeContactDirGaze(int horizon, int g_horizon, int conf)
  {
    //cout <<"horizon:" <<horizon << ", conf:"<< conf<< endl;
    
    if(tolerance>abs(abs(horizon)-abs(g_horizon)))
      if(conf > conf_th)
	return true;
      else
	return false;
    else
      return false;
  }



};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "eye_contact");
  
  EyeContact ec;
  
  ros::spin();
  return 0;
}
