/*
2014.11.12-----------------
やること
発見した人物をstackしていく
service形式
*/


#include <ros/ros.h>
#include <string>
#include <iostream>
#include "okao_client/OkaoStack.h"

using namespace std;

class property{
public:
  string name;
  string laboratory;
  string grade;
};

map<int, property> stack;

bool sendData(okao_client::OkaoStack::Request &req,
	      okao_client::OkaoStack::Response &res)
{

  if(req.rule == "add")
    {
      if(stack.count(req.okao_id) == 0)
	{
	  property prop;
	  prop.name = req.name;
	  prop.laboratory = req.laboratory;
	  prop.grade = req.grade;
	  stack[ req.okao_id ] = prop;
	  cout <<"add---> okao_id: "<< req.okao_id <<", name: "<< req.name << endl;
	}
      else
	{
	  //cout << "already exists!" << endl;
	}
      return true;
    }
  else if(req.rule == "req")
    {
      if(stack.count(req.okao_id) != 0)
	{
	  res.name = stack[ req.okao_id ].name;
	  res.laboratory = stack[ req.okao_id ].laboratory;
	  res.grade = stack[ req.okao_id ].grade;
	  cout <<"req---> okao_id: "<< req.okao_id <<", name: "<< stack[ req.okao_id ].name << endl;
	  return true;
	}
      else
	{
	  //cout << "do not have!" << endl;
	  return false;
	}
    }
  else
    {
      cout << "request false" << endl;
      return false;
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "okao_stack");
  ros::NodeHandle n;
  ros::ServiceServer okaostack = n.advertiseService("okao_stack", sendData);

  ros::spin();
  return 0;
}
