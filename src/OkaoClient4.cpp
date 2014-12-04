



//basic
#include <iostream>
#include <sstream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
//ROS
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//オリジナルのメッセージ
#include <humans_msgs/Humans.h>
#include "okao_client/OkaoStack.h"
//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//通信用
#include <msgpack.hpp>
#include "zmq.hpp"
#include "zmq.h"
#include "picojson.h"
#include "Message.h"
//message
#include "JsonToMsg.hpp"
#include "MsgToMsg.hpp"

#define OKAO 3 
#define SRVTEMPO 10
#define NECK 2
#define HEAD 3
#define SHOULDER_L 4
#define SHOULDER_R 8
#define SPINE_S 20

#define HORIZON 70
#define VERTICAL 60
#define FACE_W 0.15
#define FACE_H 0.20
#define MAX_W 1920
#define MAX_H 1080


static const std::string OPENCV_WINDOW = "OKAO Client Window";
using namespace cv;
using namespace std;

class POS
{
public:
  double x;
  double y;
  double depth;
};


class MyClass 
{
public:
  MyClass() :
    it_(nh_),  
    image_sub_( it_, "/camera/image/color", 1000 ),
    humans_sub_( nh_, "/humans/KinectV2", 1000 ),

    sync( MySyncPolicy( 100 ), image_sub_, humans_sub_ )
  {
    sync.registerCallback( boost::bind( &MyClass::callback, this, _1, _2 ) );

    //ソケットの作成
    responder = new zmq::socket_t(context, ZMQ_REQ);
    assert( responder );
    try
      {
	responder->connect("tcp://133.19.23.33:50001");
      }
    catch(const zmq::error_t& e)
      {
	cout <<"Server Conect Error: " << e.what() << endl;
	return;
      } 

    image_pub_ = it_.advertise("/image_converter/output_video",1);
    okaoData_pub_ = nh_.advertise<humans_msgs::Humans>("/humans/OkaoServer",10);

    //ウィンドウ
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~MyClass()
  {
    cv::destroyWindow(OPENCV_WINDOW);
    if( responder )
      {
    	delete responder;
      } 
    //zmq_close( responder );
  } 
 
  void callback(
		const sensor_msgs::ImageConstPtr& imgmsg,
		const humans_msgs::HumansConstPtr& kinect
		)
  {  
    int p_i = 0;
    humans_msgs::Humans okao_human;   
    cv_bridge::CvImagePtr cv_ptr;
    try
      {
	cv_ptr = cv_bridge::toCvCopy(imgmsg, sensor_msgs::image_encodings::BGR8);
	for(int i = 0; i < kinect->num; i++)
	  {
	    POS head2d, head3d, cut;
	    head2d.x = kinect->human[i].body.joints[HEAD].position_color_space.x;	   
	    head2d.y = kinect->human[i].body.joints[HEAD].position_color_space.y;
	    head3d.depth = kinect->human[i].body.joints[HEAD].position.x;

	    double f_horizon = HORIZON * M_PI / 180.0;
	    double f_vertical = VERTICAL * M_PI / 180.0;
	    double diff_w =  MAX_W * atan2( FACE_W, head3d.depth ) / f_horizon;
	    double diff_h =  MAX_H * atan2( FACE_H, head3d.depth ) / f_vertical;

	    cut.x = head2d.x - diff_w;
	    cut.y = head2d.y - diff_h;

	    if( cut.x < 0 )
	      cut.x = 0;
	    else if( cut.x > cv_ptr->image.cols-diff_w*2 )
	      cut.x = cv_ptr->image.cols-diff_w*2;

	    if( cut.y < 0 )
	      cut.y = 0;
	    else if( cut.y > cv_ptr->image.rows-diff_h*2 )
	      cut.y = cv_ptr->image.rows-diff_h*2;

	    Mat cutRgbImage(cv_ptr->image, cv::Rect(cut.x, cut.y, diff_w*2, diff_h*2));

	    Mat rgbImage;
	    if( cutRgbImage.cols > 1280 || cutRgbImage.rows > 1024 )
	      {
		cv::resize( rgbImage, cutRgbImage, cv::Size(1280, 1024) );
	      }	
	    else
	      {
		rgbImage = cutRgbImage;
	      }   

	    Mat grayImage;	   
	    cv::cvtColor(rgbImage,grayImage,CV_BGR2GRAY);

	    //test
	    cv::rectangle( cv_ptr->image, cv::Point(cut.x, cut.y),
			   cv::Point(cut.x+diff_w*2, cut.y+diff_h*2),
			   cv::Scalar(0,200,0), 5, 8);
	    
	    try
	      {
		cv::Mat img = grayImage;
		std::vector<unsigned char> 
		  buf(img.data, img.data + img.cols * img.rows * img.channels());

		std::vector<int> encodeParam(2);
		encodeParam[0] = CV_IMWRITE_PNG_COMPRESSION;
		encodeParam[1] = 3;
		cv::imencode(".png", img, buf, encodeParam);
		
		picojson::object p;	
		p.insert(std::make_pair("mode",
					picojson::value(std::string("FaceRecognition"))));
		p.insert(std::make_pair("format",picojson::value(std::string("PNG"))));	
		p.insert(std::make_pair("width",picojson::value((double)img.cols)));
		p.insert(std::make_pair("height",picojson::value((double)img.rows)));
		p.insert(std::make_pair("depth",picojson::value((double)1)));
		
		picojson::value para = picojson::value(p); 
		std::string param = para.serialize().c_str();
		// リクエストメッセージの作成
		OkaoServer::RequestMessage reqMsg;
		reqMsg.img = buf;
		reqMsg.param = param;	
		// 送信
		OkaoServer::sendRequestMessage(*responder, reqMsg);
		// 受信
		OkaoServer::ReplyMessage repMsg;
		OkaoServer::recvReplyMessage(*responder, &repMsg);
		//std::cout << "repMsg.okao: " << repMsg.okao << std::endl;
	    	
		const char* json = repMsg.okao.c_str();
		picojson::value v;
		std::string err;
		picojson::parse(v,json,json + strlen(json),&err);
		if(err.empty())
		  {
		    humans_msgs::Face face_msg;
		    humans_msgs::Body body_msg;
		    bool p_ok = 0;
		    JsonToMsg::face(v, &face_msg, cut.x, cut.y, &p_ok);		    
		    MsgToMsg::bodyToBody(kinect->human[i].body, &body_msg);
		    
		    if( p_ok )
		      {
			humans_msgs::Human h;
			h.body = body_msg;
			h.face.persons.resize( OKAO );
			h.face = face_msg;
			okao_human.human.push_back( h );
			++p_i;
			cv::Point lt(face_msg.position.lt.x, face_msg.position.lt.y);
			cv::Point rb(face_msg.position.rb.x, face_msg.position.rb.y);
			cv::Point rb_out(cut.x+diff_w*2, cut.y+diff_h*2);
			cv::Scalar red(0,0,200);
			cv::Scalar green(0,200,0);
			cv::rectangle( cv_ptr->image, lt, rb, red, 5, 8);
		
			cv::putText( cv_ptr->image, face_msg.persons[0].name, 
				     rb_out, FONT_HERSHEY_SIMPLEX, 2.5, 
				     green, 2, CV_AA);

			ros::ServiceClient client = nh_.serviceClient<
			  okao_client::OkaoStack>("okao_stack");
			okao_client::OkaoStack stack;
			stack.request.rule = "add";
			stack.request.okao_id = face_msg.persons[0].okao_id;
			stack.request.name = face_msg.persons[0].name;
			stack.request.laboratory = face_msg.persons[0].laboratory;
			stack.request.grade = face_msg.persons[0].grade;
			if ( client.call(stack) )
			  cout << "service success!" << endl;
			else
			  cout << "service missing!" << endl;	
		      }
		  }	       
	      }    
	    catch(const zmq::error_t& e)
	      {
		std::cout << e.what() << std::endl;
		return ;
	      }	    
	  }
	//パブリッシュ
	okao_human.num = p_i;
	okao_human.header.stamp = ros::Time::now();
	okao_human.header.frame_id = "okao";
	okaoData_pub_.publish(okao_human);
	image_pub_.publish(cv_ptr->toImageMsg());
	//cv::imshow(OPENCV_WINDOW, cv_ptr->image);
	//cv::waitKey(1);
      }
    catch(cv_bridge::Exception& e)
      {
	ROS_ERROR("cv_bridge exception: %s",e.what());
	return;
      } 
  }
  
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;

  ros::Publisher okaoData_pub_;
  image_transport::Publisher image_pub_;
 
  zmq::context_t context;
  zmq::socket_t* responder; 

  typedef image_transport::SubscriberFilter ImageSubscriber;
  typedef message_filters::Subscriber< humans_msgs::Humans > HumansSubscriber;
  
  ImageSubscriber image_sub_;
  HumansSubscriber humans_sub_;
  
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image, humans_msgs::Humans
    > MySyncPolicy;
  
  message_filters::Synchronizer< MySyncPolicy > sync;
};

int main(int argc, char** argv) 
{
  ros::init( argc, argv, "my_node" );
  MyClass mc;
 
  ros::spin();
  
  return EXIT_SUCCESS;
}


