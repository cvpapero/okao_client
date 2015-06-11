
#include <ros/ros.h>

#include <turtlesim/Spawn.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <sstream>
#include <string>
#include <iostream>
#include <vector>
#include <map>

//#include <math.h>
#include "MsgToMsg.hpp"

//オリジナルのメッセージ
#include <humans_msgs/Humans.h>
#include "okao_client/OkaoStack.h"

using namespace std;
template<class T> inline std::string toString(T x) {std::ostringstream sout;sout<<x;return sout.str();}

class FaceMapping
{
private:
  ros::NodeHandle n;
  //ros::Publisher markerArray_pub_;
  ros::Subscriber info_sub_;

public:
  FaceMapping()
  {
    info_sub_ = n.subscribe("/humans/KinectV2", 1, &FaceMapping::callback, this);
  }

  void callback(const humans_msgs::HumansConstPtr& info)
  {
    for(int i = 0; i<info->num; ++i)
      {
	cout << "head[ "<<i << "], POS3D = ( " 
	     << info->human[i].body.joints[3].position.x << " , " 
	     << info->human[i].body.joints[3].position.y << " , " 
	     << info->human[i].body.joints[3].position.z << " ) , POS2D = ( "
	     << info->human[i].body.joints[3].position_color_space.x - 480<< " , "
	     << info->human[i].body.joints[3].position_color_space.y << " )" <<endl;
      }
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pos_sub");
  FaceMapping FMObject;
  ros::spin();
  return 0;
}



