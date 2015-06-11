/*
2015.4.19-----------------
顔の信頼度チェック
登録されていない人はどのように表示されるか

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
  //int count = 0;
  //int member[ MEMBER ] = {0};

public:
  FaceMapping()
  {
 
    recogInfo_sub_ = n.subscribe("/humans/okao_server", 1, &FaceMapping::callback, this);
  }

  void callback(const humans_msgs::HumansConstPtr& rein)
  {
    //cout << "callback" <<endl;
    //認識人数の取得
    //int p_num = ;
    if(rein->human.size())
      {
	stringstream ss;
	for(int i = 0; i < rein->human.size() ; ++i)
	  {
	    stringstream okaoconf;
	    for(int j = 0; j < rein->human[i].face.persons.size(); ++j)
	      {
		okaoconf << "okao_id: " << rein->human[i].face.persons[j].okao_id
			 << ", conf: " << rein->human[i].face.persons[j].conf << ", ";
	      }
	    //okaoconf << endl;
	    ss << okaoconf.str();
	  }
	
	cout << ss.str() << endl;
      }
    /*
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
    */
 
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "okao_conf");
  FaceMapping FMObject;
  ros::spin();
  return 0;
}



