/*
2014.8.29-----------------
pub/sub方式で表示する
point background

2014.6.13-----------------
srv

2014.6.6-------------------
搭載する仕様の予定

1.顔の位置に点をポイント
2.名前表示
　今見ているヒト→みずいろ
　過去に見たヒト→おれんじ
3.人らしき物体（NiTEトラッキングok,OkAOVision is No!）のマッピング




*/


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

#include <time.h>

//#include <math.h>
#include "MsgToMsg.hpp"

//オリジナルのメッセージ
#include <humans_msgs/Humans.h>
#include "okao_client/OkaoStack.h"
#include <fstream>

#define MEMBER 20
#define ALLCOUNT 100

using namespace std;
template<class T> inline std::string toString(T x) {std::ostringstream sout;sout<<x;return sout.str();}

class FaceMapping
{
private:
  ros::NodeHandle n;
  //ros::Publisher markerArray_pub_;
  ros::Subscriber recogInfo_sub_;
  //int all_count = 1000;
  int count = 0;
  int member[ MEMBER ] = {0};

public:
  FaceMapping()
  {
 
    recogInfo_sub_ = n.subscribe("/humans/OkaoServer", 1, &FaceMapping::callback, this);
  }

  void callback(const humans_msgs::HumansConstPtr& rein)
  {

    //visualization_msgs::MarkerArray points, lines, names;
    //認識人数の取得
    int p_num = rein->num;
    
    //ros::Time map_time = ros::Time::now();      
    //float pr,pg,pb,pa,br,bg,bb,ba; 
    //float ts,ps,ls,bs;
    
    for(int i = 0; i < p_num ; ++i)
      {


	//if( rein->human[i].d_id )
	//  { 
	//  }
	int okao = rein->human[i].face.persons[0].okao_id;
	if( okao!= 0 )
	  {
	    cout << "count:" << count << endl;
	    ++member[okao];
	    ++count;
	  }
      }

    if( count >= ALLCOUNT )
      {
	cout << "end" << endl;
	
	stringstream name;
	time_t now = time(NULL);
	struct tm *pnow = localtime(&now);
	
	name <<"/home/yhirai/okaodata/" 
	     <<toString( pnow->tm_year+1900 ) << toString(pnow->tm_mon) 
	     << toString(pnow->tm_mday)<< toString(pnow->tm_hour)
	     << toString(pnow->tm_min) << toString(pnow->tm_sec); 

	for(int i = 0; i < MEMBER ; ++i)
	  {
	    if( member[ i ] )
	      {
		stringstream ss;

		ss << "okao_id: " << toString(  i  ) 
		   <<", point: " << toString( member[ i ] ) 
		   <<", per: "<< toString( (double)member[ i ]/ALLCOUNT ) << endl;
		//name << "okao_cost_" << toString( ros::Time::now() ) <<".txt" ;
		//string namestr = name.str();
		std::ofstream ofs( name.str().c_str(), std::ios::out | std::ios::app );
		ofs << ss.str() << endl;
	      }
	  }
	ros::shutdown();
      }

 
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "face_mapping");
  FaceMapping FMObject;
  ros::spin();
  return 0;
}



