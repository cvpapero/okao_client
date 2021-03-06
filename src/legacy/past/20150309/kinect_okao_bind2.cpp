/*
2014.11.9---------------------------------
no callback


2014.10.31---------------------------------
どうするか？


2014.10.30---------------------------------
対応漏れの処理
対応漏れしたデータのパブリッシュ
1.顔がとれていない場合
→正規のトピックにパブリッシュ
2.頭がとれていない場合
→特別にトピックが作るか？


2014.10.27---------------------------------
body_idにゼロが入る


2014.10.25---------------------------------
kinect v2とokao serverをバインドするモジュール

okaoがなくても人物位置をパブリッシュ可能

callbackのなかでpublishしたい
http://answers.ros.org/question/59725/publishing-to-a-topic-via-subscriber-callback-function/

*/

#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <humans_msgs/Humans.h>
#include <okao_client/OkaoStack.h>
#include <tf/transform_listener.h>
#include "MsgToMsg.hpp"

#define OKAO_NUM 3

using namespace std;

int main(int argc, char** argv)
{

  ros::init(argc, argv, "kinect_okao_bind");
  ros::NodeHandle nh;
  ros::Publisher bind_pub_ = nh.advertise<humans_msgs::Humans>("/humans/BindData",10);

  while(ros::ok())
    {
      humans_msgs::HumansConstPtr okao   = ros::topic::waitForMessage<humans_msgs::Humans>("/humans/OkaoServer");
      humans_msgs::HumansConstPtr kinect = ros::topic::waitForMessage<humans_msgs::Humans>("/humans/KinectV2");  
                                                                                          
      //認識人数の取得
      int b_num = kinect->num;
      int f_num = okao->num;
      
      //int no_face = 0;   
 
      humans_msgs::Humans bind;
      int test=0;
      bind.num = b_num;
      cout<< "body detect: "<<b_num <<endl;
      cout<< "face detect: "<<f_num <<endl;
      //リサイズ
      bind.human.resize(b_num);
      for(int i = 0; i < b_num; ++i)
	{ 
	  //一位から三位までの配列を確保
	  bind.human[i].face.persons.resize(OKAO_NUM);
	}
      //std::cout<< "faces ok" << std::endl;
      for(int b_index = 0; b_index < b_num; ++b_index)
	{
	  double head_x = kinect->human[b_index].body.joints[3].position_color_space.x;      
	  double head_y = kinect->human[b_index].body.joints[3].position_color_space.y;      
	  
	  if( f_num )
	    {
	      //ROS_INFO("face exist");
	      for(int f_index = 0; f_index < f_num; ++f_index) 
		{
		  double face_lt_x = okao->human[f_index].face.position.lt.x;
		  double face_lt_y = okao->human[f_index].face.position.lt.y;
		  double face_rb_x = okao->human[f_index].face.position.rb.x;
		  double face_rb_y = okao->human[f_index].face.position.rb.y;
		  std::cout << "x: " << face_lt_x  << " < "  << head_x << " < " << face_rb_x << std::endl;
		  std::cout << "y: " << face_lt_y  << " < "  << head_y << " < " << face_rb_y << std::endl;
		  

		  if((face_lt_x <= head_x) && (face_lt_y <= head_y) && (head_x <= face_rb_x) && (head_y <= face_rb_y))
		    {

		      std::cout << "head in face" << std::endl;
		      MsgToMsg::bodyAndFaceToMsg( kinect->human[b_index].body, okao->human[f_index].face, &bind.human[b_index] );


		    }
		}
	    }       
	  else
	    {
	      //対応漏れ
	      //++no_face;
	    }	
	}	
      std::cout << "-----------------------------" << std::endl;

      for (int i = 0; i < b_num ; ++i)
	{
	  std::cout << "tracking_id[ " << i << " ]:  " << bind.human[i].body.tracking_id << std::endl;
	}

      bind.header.stamp = ros::Time::now();
      bind.header.frame_id = "bind";
      bind_pub_.publish(bind);    
      std::cout << "*****************************" << std::endl;
    }
  
  return 0;
}  

