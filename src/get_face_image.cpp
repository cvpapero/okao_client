//okaoサーバ用の顔画像を取得するモジュール

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sys/stat.h>
#include <sys/types.h>

static const std::string OPENCV_WINDOW = "Image window";
using namespace std;

int i;
int okao_id;
//stirng file_path;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, 
      &ImageConverter::imageCb, this);
    //image_pub_ = it_.advertise("/image_converter/output_video", 1);
    i = 1;
    cv::namedWindow(OPENCV_WINDOW);

    inputId();

    //ファイル名
    time_t now = time(NULL);
    struct tm *pnow = localtime(&now);
    stringstream file_path;
    file_path << "/home/uema/outimage/id_"
	      << okao_id << "_"
	      << pnow->tm_year+1900 
	      << pnow->tm_mon  
	      << pnow->tm_mday
	      << pnow->tm_hour 
	      << pnow->tm_min 
	      << pnow->tm_sec; 
    mkdir(file_path.str().c_str(), 0755);
    chdir(file_path.str().c_str());

    countdown();
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void inputId()
  {
    cout << "please input okao_id: ";
    cin >> okao_id;
  }

  void countdown()
  {

    int count = 5;
    for(int i = 0; i <= count; ++i)
      {
	cv::Mat img = cv::Mat::zeros(240, 320, CV_8UC3);
	stringstream ss;
	ss << "count: " << count - i ;
	cv::putText(img, ss.str(), 
		    cv::Point(0,100), cv::FONT_HERSHEY_SIMPLEX,
		    1.2, cv::Scalar(0,0,200), 2, CV_AA);
	cv::imshow(OPENCV_WINDOW, img);
	cv::waitKey(500);
      }
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
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

    stringstream num, file;
    num <<"n: "<< i;

    if( okao_id < 10 )
      file << "00" << okao_id << "-"; 
    else
      file << "0" << okao_id << "-";

    if( i < 10 )
      file << "000" << i << ".jpg"; 
    else
      file << "00" << i << ".jpg";

    //int param = CV_IMWRITE_JPEG_QUALITY;
    cv::Mat outImg;
    cv::resize(cv_ptr->image, outImg, cv::Size(320,240));
    cv::imwrite(file.str(), outImg);
    cv::putText(outImg, num.str(), 
		cv::Point(0,30), cv::FONT_HERSHEY_SIMPLEX,
		1.2, cv::Scalar(0,200,0), 2, CV_AA);
    cv::imshow(OPENCV_WINDOW, outImg);
    cv::waitKey(200);
    
    ++i;

    if( i > 50 )
      ros::shutdown();
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
