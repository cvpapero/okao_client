/*
2015.1.15-----------------
やはり、service方式でデータを受け取る方がよい
なぜなら、パブサブだと、データが来たときにしか更新されない
そうじゃなくて、今、この瞬間の状態がどうなっているかを知りたい

 
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
#include <sstream>
#include <string>
#include <iostream>
#include <vector>
#include <map>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>

//#include <math.h>
#include "MsgToMsg.hpp"

//オリジナルのメッセージ
#include <humans_msgs/PersonPoseImgArray.h>
#include <humans_msgs/Humans.h>
#include "okao_client/OkaoStack.h"
#include "humans_msgs/HumansSrv.h"

using namespace std;

class FaceMappingPub
{
private:
  ros::NodeHandle n;
  ros::Publisher facemap_pub_ ;
  ros::ServiceClient okaoStack_;
  ros::Subscriber recogInfo_sub_;

public:
  FaceMappingPub()
  {
    facemap_pub_
      = n.advertise<humans_msgs::PersonPoseImgArray
		    >("human_and_image", 10);
    okaoStack_
      = n.serviceClient<okao_client::OkaoStack>("stack_send");
    recogInfo_sub_ 
      = n.subscribe("/humans/RecogInfo", 1, &FaceMappingPub::callback, this);
  }

  ~FaceMappingPub()
  {

  }

  void callback(const humans_msgs::HumansConstPtr& rein)
  {

    humans_msgs::PersonPoseImgArray ppia;
    //cout << rein->num << endl;
    //ROS_INFO("people found");
    for(int i = 0; i < rein->num; ++i)
      {
	humans_msgs::PersonPoseImg ppi;
	okao_client::OkaoStack stack;
	
	stack.request.person.okao_id = rein->human[i].max_okao_id;
	okaoStack_.call( stack );
	
	//haia.human.push_back( dbhumans.response.dst.human[i] );
	
	//sensor_msgs::Image output;
	ppi.person.hist = rein->human[i].max_hist;
	ppi.person.okao_id = rein->human[i].max_okao_id;
	ppi.person.name = stack.response.name;
	ppi.pose.position = rein->human[i].p;
	ppi.pose.orientation.w = 1;//dbhuman.response.dst.human[i].body.p;
	
	stringstream ss;
	ss <<"/home/uema/catkin_ws/src/okao_client/src/images/" 
	   << rein->human[i].max_okao_id <<".jpg";
	/*
	cv::Mat faceImage = cv::imread(ss.str(), 1);
	sensor_msgs::Image output;
	output.height = faceImage.rows; 
	output.width = faceImage.cols;
	output.encoding = "bgr8";
	output.step = faceImage.cols * faceImage.elemSize();
	output.data.assign(faceImage.data, 
			   faceImage.data + size_t(faceImage.rows*output.step));
	ppi.image = output;
	*/
	ppi.image = stack.response.image;
	
	ppia.ppis.push_back( ppi );
	
      }
    //cout<< ppia <<endl;
    ppia.header.stamp = ros::Time::now();
    ppia.header.frame_id = "map";
    //.num = dbhumans.response.dst.num;
    facemap_pub_.publish( ppia );
  }
  
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "face_mapping_publisher");
  FaceMappingPub pmp;
  ros::spin();
  return 0;
}



