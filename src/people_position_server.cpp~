/*
2014.6.3------------------
認識した人数分の人物位置を一時的に記録しておくモジュール

map<int,point3D>で、okao_idをキーにする

 
*/

#include <ros/ros.h>

#include <map>
#include <string>
#include <sstream>
#include <iostream>

//オリジナルメッセージ
#include "humans_msgs/Humans.h"

#include "okao_client/OkaoPosSrv.h"

using namespace std;

class point2D{
public: 
  int id;
  float x;
  float y;
  point2D(){
  }
};

map<int, point2D> dicts;


class Server
{
private:
  ros::NodeHandle n;
  ros::Subscriber sub_;
  ros::ServiceServer srv_;

public:
  Server()
  {
    sub_ = n.subscribe("/humans/RecogInfo", 1, &Server::callback, this);
    srv_ = n.advertiseService("OkaoClient_srv", &Server::sendData, this);
  }

  void callback(const humans_msgs::HumansConstPtr& rein)
  {
    for(int i = 0; i < rein->num; ++i)
      {
	point2D position;
	//position.id = rein->human[i].max_okao_id;
	position.x = rein->human[i].p.x;
	position.y = rein->human[i].p.y;
	dicts[ rein->human[i].max_okao_id ] = position;
      }  
  }

  bool sendData(okao_client::OkaoPosSrv::Request &req,
		okao_client::OkaoPosSrv::Response &res)
  {
    point2D position;
    if(req.rule == "request")
      {
	std::cout<< "requast!"<<std::endl;
	res.res_x = dicts[ req.okao_id ].x;
	res.res_y = dicts[ req.okao_id ].y;
	return true;
      }
    else
      {
	std::cout<< "false"<<std::endl;
	return false;
      }
  }
};
  
int main(int argc, char** argv){
  ros::init(argc, argv, "people_position_server");
  Server SRV;
  ros::spin();
  return 0;
}
