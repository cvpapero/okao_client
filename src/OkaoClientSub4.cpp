/*
2014.11.19------------------
人物の頭の部分をくりぬいてOKAO Serverに送信



2014.10.21------------------
humans_msgsによってパブリッシュ
個人属性データが、一人一人についてパブリッシュされてないのか？


2014.8.22-------------------
信頼度によるUNKNOWNの設定

2014.7.16-------------------
パブリッシュ機能を追加

2014.7.15-------------------
安達くんのOKAOサーバにコネクトする
json


*/


#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
//ROS
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//オリジナルのメッセージ
#include <humans_msgs/Humans.h>
#include "okao_client/OkaoStack.h"
//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//通信用ライブラリ
#include <msgpack.hpp>
#include "zmq.hpp"
#include "zmq.h"

#include "picojson.h"
#include "Message.h"

#define OKAO 3 
#define SRVTEMPO 10

static const std::string OPENCV_WINDOW = "OKAO Client Sub4 Window";

using namespace cv;
using namespace std;

int callbackCount = 0;
bool stackSrv = false;

class ImageConverter
{
private:
  ros::NodeHandle nh_;

  //パブリッシャとサブスクライバの定義
  ros::Publisher okaoData_pub_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber depth_sub_;
  image_transport::Publisher image_pub_;

  zmq::context_t context;
  zmq::socket_t* responder; 



public:
  ImageConverter()
    : it_(nh_), context(3)
  {
    //画像データのサブスクライブとパブリッシュ
    image_sub_ = it_.subscribe("/camera/image/color",1, 
			       &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video",1);

    //検出した顔データのパブリッシュ
    okaoData_pub_ = nh_.advertise<humans_msgs::Humans>("/humans/OkaoServer",10);

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
    //ウィンドウ
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
    if( responder )
      {
	delete responder;
      } 
    //zmq_close( responder );
  }
  
  //コールバック関数
  void imageCb(const sensor_msgs::ImageConstPtr& imgmsg)
  {
    //Subscribe  
    humans_msgs::HumansConstPtr kinect = ros::topic::waitForMessage<humans_msgs::Humans>("/humans/KinectV2", nh_);

    Mat rgbImage,grayImage;//画像イメージ格納用
    cv_bridge::CvImagePtr cv_ptr;
    try
      {
	//画像メッセージ(ROS)をMat型(OpenCV)に代入し、グレイスケール変換する
	cv_ptr = cv_bridge::toCvCopy(imgmsg, sensor_msgs::image_encodings::BGR8);
	Mat tmpRgbImage = cv_ptr->image;
	Mat reRgbImage(tmpRgbImage, cv::Rect(960, 540, 960, 540));
	rgbImage = reRgbImage;
	cv::cvtColor(rgbImage,grayImage,CV_BGR2GRAY);
      }
    catch(cv_bridge::Exception& e)
      {
	ROS_ERROR("cv_bridge exception: %s",e.what());
	return;
      }

    try
      {
	//画像の読み込み
	cv::Mat img = grayImage;//cv_ptr->image;
	//cout <<"cols:" << img.cols << ",rows:"<<img.rows << endl;
	std::vector<unsigned char> buf(img.data, img.data + img.cols * img.rows * img.channels());
	double width =  rgbImage.cols;
	double height = rgbImage.rows;
	double resize_width =  width / img.cols; 
	double resize_height = height / img.rows;
	//cout<<"img.cols: "<< img.cols << endl;
	//cout<<"resize_width: "<< resize_width << endl;
	// 画像をエンコードする必要があれば
	std::vector<int> encodeParam(2);
	encodeParam[0] = CV_IMWRITE_PNG_COMPRESSION;
	encodeParam[1] = 3;
	cv::imencode(".png", img, buf, encodeParam);
	//std::cout << "encoded size: " << buf.size() << std::endl;
	// 画像のパラメータ
	// エンコードした際はformatをRAWからPNG等に変えること
	// また、width, heightも適切に設定すること	
	picojson::object p;	
	p.insert(std::make_pair("mode",picojson::value(std::string("FaceRecognition"))));
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
	//	cout << "send to OKAOServer" << endl;
	// 送信
	OkaoServer::sendRequestMessage(*responder, reqMsg);
	// 受信
	//cout << "receive from OKAOServer" << endl;
	OkaoServer::ReplyMessage repMsg;
	OkaoServer::recvReplyMessage(*responder, &repMsg);
	std::cout << "repMsg.okao: " << repMsg.okao << std::endl;	
	const char* json = repMsg.okao.c_str();
	picojson::value v;
	std::string err;
	picojson::parse(v,json,json + strlen(json),&err);
	if(err.empty())
	  {
	    int pos[4];
	    int people_num = 0;
	    //一番外側のオブジェクトを取得
	    picojson::object& obj = v.get<picojson::object>();
	    //facesの中身があるならば
	    if( obj.find("error") != obj.end() && obj["error"].get<std::string>().empty())
	      {
		if( obj.find("faces") != obj.end() )
		  {
		    picojson::array array = obj["faces"].get<picojson::array>();
		    int people_num = array.size();//人数	    		
		    humans_msgs::Humans okao_msg;	   
		    okao_msg.human.resize(people_num);

		    for( int n = 0; n<people_num; ++n)
		      okao_msg.human[n].face.persons.resize(3);

		    okao_msg.num = people_num;//OkaoサーバとKinectサーバで認識した人数の違いをどうするか？
		    //cout << "people_num:" <<people_num << endl;
		    //検出した人数分ループ
		    int num = 0;
		    for(picojson::array::iterator it = array.begin(); it != array.end(); ++it,++num)
		      {
			picojson::object& person_obj = it->get<picojson::object>();
			
			//顔の位置のとりだし
			picojson::array pos_array = person_obj["position"].get<picojson::array>();
			for(int i = 0; i < 4; ++i)
			  {
			    pos[i] = (int)pos_array[i].get<double>();
			  }

			//人物ID,信頼度の取り出し
			picojson::array id_array = person_obj["id"].get<picojson::array>();
			picojson::array db_info_array = person_obj["db_info"].get<picojson::array>();
			//picojson::array id1 = id_array[0].get<picojson::array>();
			//double conf = (int)id1[1].get<double>();
			double tmp_id[3], tmp_conf[3]; 
			std::string tmp_name[3], tmp_grade[3], tmp_laboratory[3];
			
			++callbackCount; 
			if( !(callbackCount % SRVTEMPO) )
			  {
			    stackSrv = true;
			    callbackCount = 0;
			  }
			for(int n = 0; n < OKAO; ++n)
			  {
			    picojson::array id_array_array = id_array[n].get<picojson::array>();
			    tmp_id[n] = (int)id_array_array[0].get<double>();
			    tmp_conf[n] = (int)id_array_array[1].get<double>();
			    picojson::object& db_info_obj = db_info_array[n].get<picojson::object>();
			    tmp_name[n] = db_info_obj["name"].get<std::string>();
			    tmp_grade[n] = db_info_obj["grade"].get<std::string>();
			    tmp_laboratory[n] = db_info_obj["laboratory"].get<std::string>();

			    //if(stackSrv){
			      //srv
			      //stackSrv = false;
			      ros::ServiceClient client = nh_.serviceClient<okao_client::OkaoStack>("okao_stack");
			      okao_client::OkaoStack stack;
			      stack.request.rule = "add";
			      stack.request.okao_id = tmp_id[n];
			      stack.request.name = tmp_name[n];
			      stack.request.laboratory = tmp_laboratory[n];
			      stack.request.grade = tmp_grade[n];
			      if ( client.call(stack) )
				cout << "service success!" << endl;
			      else
				cout << "service missing!" << endl;
			      //}		 
			  }
			
			//一人目の信頼度を利用する
			if(tmp_conf[0] < 200)
			  {
			    for(int n = 0; n < OKAO; ++n)
			      {
				tmp_id[n] = 0;
				tmp_conf[n] = 0;
				tmp_name[n] = "Unknown";
				tmp_laboratory[n] = "Unknown";
				tmp_grade[n] = "Unknown";
			      }			    

			  }

			  
			for(int n = 0; n < OKAO; ++n)
			  {
			    okao_msg.human[num].face.persons[n].okao_id = tmp_id[n];
			    okao_msg.human[num].face.persons[n].conf = tmp_conf[n];
			    okao_msg.human[num].face.persons[n].name = tmp_name[n];
			    okao_msg.human[num].face.persons[n].laboratory = tmp_laboratory[n];
			    okao_msg.human[num].face.persons[n].grade = tmp_grade[n];
			  }
			
			//顔の部分を四角で囲む
			cv::Point lt(pos[0],pos[1]);
			cv::Point rt(pos[2],pos[1]);
			cv::Point lb(pos[0],pos[3]);
			cv::Point rb(pos[2],pos[3]);
			cv::Scalar color(0, 0, 255);
			cv::line(rgbImage, lt , rt , color, 2);
			cv::line(rgbImage, rt , rb , color, 2);
			cv::line(rgbImage, rb , lb , color, 2);
			cv::line(rgbImage, lb , lt , color, 2);
			//名前の表示
			cv::putText(rgbImage,tmp_name[0],rb,FONT_HERSHEY_SIMPLEX,2.5,color,2,CV_AA);
			

			okao_msg.human[num].face.position.lt.x = pos[0]*resize_width;
			okao_msg.human[num].face.position.lt.y = pos[1]*resize_height;
			okao_msg.human[num].face.position.rt.x = pos[2]*resize_width;
			okao_msg.human[num].face.position.rt.y = pos[1]*resize_height;
			okao_msg.human[num].face.position.lb.x = pos[0]*resize_width;
			okao_msg.human[num].face.position.lb.y = pos[3]*resize_height;
			okao_msg.human[num].face.position.rb.x = pos[2]*resize_width;
			okao_msg.human[num].face.position.rb.y = pos[3]*resize_height;

		      }
		    //Mat tmpRgbImage(rgbImage, cv::Rect(640, 360, 640, 360));
		    cv::imshow(OPENCV_WINDOW, rgbImage);
		    
		    cv::waitKey(1);
		    //パブリッシュ
		    okao_msg.header.stamp = ros::Time::now();
		    okao_msg.header.frame_id = "okao";
		    okaoData_pub_.publish(okao_msg);
		    
		  }	
	      }
	  }
      }   
    catch(const zmq::error_t& e)
      {
	std::cout << e.what() << std::endl;
	return ;
      }   
  }
  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "OkaoClientSub4");
  
  ImageConverter ic;
  
  ros::spin();
  return 0;
}
