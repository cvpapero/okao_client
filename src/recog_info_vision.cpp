/*
2014.12.8---------------
同期

2014.6.2-----------------
okao_recog_vision
openNiTE2_8の画像とpeople_recognize3_1のヒストグラムを噛ました人物データを
結びつけて可視化する

mapで名前も出せるようにする


疑問
なんでUnknownが出てしまうのか？
おそらく、データが空のときに反応しているのではないか
つまり、顔が取れなかったときも、0つまりUnknownとして出力しているのではないか

だったら、データの受け渡しをしない方が良いのか？？？

*/

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>
#include <iostream>
#include <vector>
#include <math.h>
#include <cassert>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <map>

#include <humans_msgs/Humans.h>
#include "okao_client/OkaoStack.h"

using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "recog Image window";


class RIVision
{  
public:
  RIVision()
    : it_(nh_),
    image_sub_( it_, "/camera/image/color", 100 ),
    humans_sub_( nh_, "/humans/RecogInfo", 100 ),
    sync( MySyncPolicy( 10 ), image_sub_, humans_sub_ )
  {
    sync.registerCallback( boost::bind( &RIVision::imageCb, this, _1, _2 ) );
    image_pub_ = it_.advertise("/camera/image/recog_info",1);
   
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~RIVision()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(
	       const sensor_msgs::ImageConstPtr& msg,
	       const humans_msgs::HumansConstPtr& okao_data)
  {	
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    ros::ServiceClient okaoStack = nh_.serviceClient<okao_client::OkaoStack>("okao_stack");
    okao_client::OkaoStack stack;


    // Draw an example circle on the video stream
    for(int i= 0; i<okao_data->num; i++)
      {
	cv::Scalar red(0,0,200);
	cv::Point lt(okao_data->human[i].face.position.lt.x,
	       okao_data->human[i].face.position.lt.y);
	cv::Point rb(okao_data->human[i].face.position.rb.x,
		      okao_data->human[i].face.position.rb.y);
	cv::rectangle(cv_ptr->image,
		      lt, 
		      rb,
		      red, 5, 8);

	stack.request.rule = "req";
	stack.request.okao_id = okao_data->human[i].max_okao_id;
	okaoStack.call(stack);	

	std::stringstream idAndNameStream;
	std::string idAndName;
	idAndNameStream << okao_data->human[i].d_id <<","
			<< stack.response.name << ","
			<< okao_data->human[i].max_hist;
	idAndName = idAndNameStream.str();	    
	cout <<"recog[ "<<i<<" ]: " << idAndName << endl;
	cv::putText(cv_ptr->image, idAndName,
		    rb,
		    FONT_HERSHEY_SIMPLEX, 2.5, red, 2, CV_AA);
	
      }
    image_pub_.publish(cv_ptr->toImageMsg());
  }
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Publisher image_pub_;

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
  ros::init(argc, argv, "recog_info_vision");
  RIVision ic;
  ros::spin();
  return 0;
}
