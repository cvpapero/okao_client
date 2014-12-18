#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <okao_client/OkaoPosSrv.h>

typedef actionlib::SimpleActionClient<
  move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace std;

class Client
{
private:
  ros::NodeHandle n;
  ros::Subscriber sub;

public:
  Client()
  {
    sub = n.subscribe("/okao_id", 1, &Client::callback, this);
  }

  bool recvData(int okao_id, move_base_msgs::MoveBaseGoal* goal)
  {
    ros::ServiceClient okaoPosSrv = 
      n.serviceClient<okao_client::OkaoPosSrv>("okao_pos_srv");
    okao_client::OkaoPosSrv ops;

    ops.request.rule = "req";
    ops.request.okao_id = okao_id;

    if( okaoPosSrv.call(ops) )
      {
	cout <<"ROS Time [" <<ops.response.res_t <<"] : ( x , y ) = ( "
	     << ops.response.res_x << " , " << ops.response.res_y <<" )" 
	     << endl;	
	goal->target_pose.pose.position.x = ops.response.res_x;
	goal->target_pose.pose.position.y = ops.response.res_y;
	goal->target_pose.pose.orientation.w = 1;
	return true;
      }
    else
      {
	return false;
      }
  }

  void callback(const std_msgs::Int32ConstPtr& okao_id)
  { 
    move_base_msgs::MoveBaseGoal goal;
    recvData(okao_id->data, &goal);

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    MoveBaseClient ac("move_base", true);
    while(!ac.waitForServer(ros::Duration(5.0)))
      {
	ROS_INFO("Waiting for the move_base action server to come up!");
      }

    ac.sendGoal( goal );
    ROS_INFO("sending goal");

    //このとき中断できるか？割り込みできるか？
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Hooray, the base moved 1 meter forward");
    else
      ROS_INFO("The base failed to move forward 1 meter for some reason");
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "people_position_client");
  Client CRT;
  ros::spin();
  return 0;
}
