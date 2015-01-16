/*
2014.12.23------------------
とりあえず閾値を設定して、それを越えたらその人物の位置を更新する

1.OKAO_IDとヒストグラムとtracking_idのサブスクライブ
2.mapに過去にその人物の要素が保持されていたかどうかを調べる
もし保持されていたら？


登録された人物の現在位置を出力するモジュール（信頼度を利用する）.
かつ
見えなくなった人をresponseする

このモジュールで主役になるのはOKAO_IDである
名前にクエスチョンマークをつける？
 */

#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <map>

#include <humans_msgs/Humans.h>
#include <okao_client/LostTracking.h>

using namespace std;

class PeoplePosition
{
private:
  ros::NodeHandle n_;
  ros::Subscriber p_sub_;
  ros::Publisher p_pub_;
  ros::ServiceServer t_srv_;


public:
  PeoplePosition()
  {
    p_sub_ = n_.subscribe("/humans/RecogInfo", 1, 
			  &PeoplePosition::callback, this);
    p_pub_ = n_.advertise<humans_msgs::Humans>("/humans/PeoplePos", 1);
    t_srv_ = n_.subscribe("lost_tracking", 
			  &PeoplePosition::sendData, this);
  }

  void callback(const humans_msgs::HumansConstPtr& pepos)
  {
    //とりあえず    


  } 

  bool sendData(okao_client::LostTracking::Request &req,
		okao_client::LostTracking::Response &res)
  {

    //サブスクライブされなくなったtracking_idを返す
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "people_position");
  PeoplePosition PP;
  ros::spin();
  return 0;
}
