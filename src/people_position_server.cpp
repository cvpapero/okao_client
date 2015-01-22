/*
2015.1.9----------------

今は、ruleに入ってるキーワードを判定した条件分岐で、検索の処理をしている
つまり、キーワードの数だけ条件分岐がいる

 
*/

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

#include <map>
#include <string>
#include <sstream>
#include <iostream>
//#include <time.h>

//オリジナルメッセージ
#include "humans_msgs/Humans.h"
#include "humans_msgs/HumanSrv.h"
#include "humans_msgs/HumansSrv.h"
//#include "okao_client/OkaoStack.h"
#include "okao_client/OkaoStack.h"
#include "MsgToMsg.hpp"

using namespace std;

//map<int, humans_msgs::Human> n_DBHuman;
//map<int, humans_msgs::Human> p_DBHuman;
map<int, humans_msgs::Human> o_DBHuman;

class PeoplePositionServer
{
private:
  ros::NodeHandle n;
  ros::Subscriber rein_sub_;
  ros::ServiceServer track_srv_;
  ros::ServiceServer okao_srv_;
  ros::ServiceClient okaoStack_;
  ros::ServiceServer array_srv_;

  //人物の初期化をする
  //humans_msgs::Human init;
  //o_DBHuman[1] = 


public:
  PeoplePositionServer()
  {
    rein_sub_ 
      = n.subscribe("/humans/RecogInfo", 1, 
		    &PeoplePositionServer::callback, this);
    track_srv_ 
      = n.advertiseService("track_srv", 
			   &PeoplePositionServer::resTrackingId, this);
    okao_srv_ 
      = n.advertiseService("okao_srv", 
			   &PeoplePositionServer::resOkaoId, this);
    okaoStack_
      = n.serviceClient<okao_client::OkaoStack>("stack_send");

    array_srv_ 
      = n.advertiseService("array_srv", 
			   &PeoplePositionServer::resHumans, this);
  }
  
  ~PeoplePositionServer()
  {
    //n_DBHuman.clear();
    //p_DBHuman.clear();
    o_DBHuman.clear();
  }

  void callback(const humans_msgs::HumansConstPtr& rein)
  {

    if( rein->num == 0 )
      {
	//ここで、見えなくなった人物のd_idをパブリッシュしてもいいかも
	//もし、N_DBHumanの中にデータが入っていたら、d_idをpeople_recog_infoにreq/resして、.clear()する
	//n_DBHuman.size();
      }
    else
      {

	for(int i = 0; i < rein->num; ++i)
	  {
	    humans_msgs::Human ah;
	    ah.header.stamp = ros::Time::now();
	    ah = rein->human[ i ];
	    //n_DBHuman[ i ] = ah;
	    //p_DBHuman[ rein->human[ i ].d_id ] = ah;
	    o_DBHuman[ rein->human[ i ].max_okao_id ] = ah;
	    ROS_INFO("people data update! okao_id: %d", rein->human[ i ].max_okao_id);
	  } 
      } 
  }

  void transformPosition(humans_msgs::Human hsrc, humans_msgs::Human *hdst)
  {
    //cout << "now looking: " << it_o->second.max_okao_id << endl;
    ros::Time t = ros::Time::now();
    geometry_msgs::PointStamped src;
    src.point = hsrc.p; 
    src.header.stamp = t;
    src.header.frame_id = hsrc.header.frame_id;
    geometry_msgs::PointStamped dst;
    dst.header.stamp = t;
    dst.header.frame_id = hdst->header.frame_id;
    MsgToMsg::transformHead(src, &dst);
    hdst->p = dst.point;
    hdst->header.stamp = t;
    hdst->header.frame_id = dst.header.frame_id;
    hdst->d_id = hsrc.d_id;
    hdst->max_okao_id = hsrc.max_okao_id;
    hdst->max_hist = hsrc.max_hist;
  }


  void getPerson(humans_msgs::Human hsrc, humans_msgs::Human *hdst)
  {
    okao_client::OkaoStack stack;
    stack.request.person.okao_id = hsrc.max_okao_id;
    okaoStack_.call( stack );

    humans_msgs::Person person;
    person = stack.response.person;
    hdst->face.persons.push_back( person );
  }

  bool resTrackingId(humans_msgs::HumanSrv::Request &req,
		     humans_msgs::HumanSrv::Response &res)
  {
    cout<<"tracking_id"<<endl;

    //o_DBHuman内から、tracking_idをキーにして検索
    map<int, humans_msgs::Human>::iterator it_o = o_DBHuman.begin();
    while( it_o != o_DBHuman.end() )
      {
	if( it_o->second.body.tracking_id == req.src.body.tracking_id )
	  {
	    humans_msgs::Human h_res;
	    h_res.header.frame_id = req.src.header.frame_id;
	    cout <<"frame_id: "<< it_o->second.header.frame_id 
		 <<" to frame_id: " << h_res.header.frame_id <<endl;
	    transformPosition(it_o->second, &h_res);
	    getPerson(it_o->second, &h_res);

	    //h_res;
	    res.dst = h_res;//it_o->second;
	    return true;
	  }
	++it_o;
      }
    return false;
  }

  bool resOkaoId(humans_msgs::HumanSrv::Request &req,
		 humans_msgs::HumanSrv::Response &res)
    
  {
    //n_DBHumanについての検索
    //もしそれでもみつからなかったらp_DBHumanから検索する
    //どのデータベースから見つかったかをラベルづけして返す(.srvに記述しておく)
    cout<<"okao_id"<<endl;

    map<int, humans_msgs::Human>::iterator it_o = o_DBHuman.begin();
    while( it_o != o_DBHuman.end() )
      {
	if( it_o->second.max_okao_id == req.src.max_okao_id )
	  {
	    humans_msgs::Human h_res;
	    h_res.header.frame_id = req.src.header.frame_id;
	    cout <<"frame_id: "<< it_o->second.header.frame_id 
		 <<" to frame_id: " << h_res.header.frame_id <<endl;
	    transformPosition(it_o->second, &h_res);
	    getPerson(it_o->second, &h_res);

	    res.dst = h_res;//it_o->second;
	    return true;
	  }
	++it_o;
      }
    return false;
  }

  
  bool resHumans(humans_msgs::HumansSrv::Request &req,
		 humans_msgs::HumansSrv::Response &res)
  {
    ROS_INFO("all people request!");
    humans_msgs::Humans hums;
    map<int, humans_msgs::Human>::iterator it = o_DBHuman.begin();
    int num = 0;
    while( it != o_DBHuman.end() )
      { 
	//ここで、mapが持っている値を次の値につめて返す
	humans_msgs::Human hum;
	hum = it->second;  
	hums.human.push_back( hum );
	++num;
      }
    //hums.header
    hums.num = num;
    res.dst = hums;
    return true;
  }
  
 
};
  
int main(int argc, char** argv)
{
  ros::init(argc, argv, "people_position_server");
  PeoplePositionServer PPS;
  ros::spin();
  return 0;
}
