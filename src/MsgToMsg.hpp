/*
  2014.10.29------------------------
  このヘッダがあるだけで ROS Message ---> ROS Messageできるようにする
*/


#pragma
#include <string>
#include <iostream>
#include <humans_msgs/Humans.h>
#include <humans_msgs/Body.h>
#include <humans_msgs/Face.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

//#include <cstdlib>

#define JOINTS 25
#define OKAO 3

//using namespase std;

double waittime = 1.0;

namespace MsgToMsg{

  void bodyToBody(humans_msgs::Body src, humans_msgs::Body *dst)
  {
    dst->tracking_id = src.tracking_id;
    dst->is_tracked = src.is_tracked;
    dst->left_hand_state = src.left_hand_state;
    dst->right_hand_state = src.right_hand_state;
    dst->joints.resize( JOINTS );
    
    for( int i = 0; i < JOINTS; ++i )
      {
	dst->joints[i] = src.joints[i];
	/*
	dst->joints[i].joint_name 
	  = src.joints[i].joint_name;
	dst->joints[i].tracking_state 
	  = src.joints[i].tracking_state;
	dst->joints[i].position_color_space.x 
	  = src.joints[i].position_color_space.x;
	dst->joints[i].position_color_space.y 
	  = src.joints[i].position_color_space.y;
	dst->joints[i].position.x 
	  = src.joints[i].position.x;
	dst->joints[i].position.y 
	  = src.joints[i].position.y;
	dst->joints[i].position.z 
	  = src.joints[i].position.z;
	dst->joints[i].orientation.x 
	  = src.joints[i].orientation.x;
	dst->joints[i].orientation.y 
	  = src.joints[i].orientation.y;
	dst->joints[i].orientation.z 
	  = src.joints[i].orientation.z;
	dst->joints[i].orientation.w 
	  = src.joints[i].orientation.w;
	*/
      } 
  }

  void bodyToMsg( humans_msgs::Body src, humans_msgs::Human *dst )
  {
    dst->body.tracking_id = src.tracking_id;
    dst->body.is_tracked = src.is_tracked;
    dst->body.left_hand_state = src.left_hand_state;
    dst->body.right_hand_state = src.right_hand_state;
    dst->body.joints.resize( JOINTS );
    
    for( int i = 0; i < JOINTS; ++i )
      {
	dst->body.joints[i].joint_name 
	  = src.joints[i].joint_name;
	dst->body.joints[i].tracking_state 
	  = src.joints[i].tracking_state;
	dst->body.joints[i].position_color_space.x 
	  = src.joints[i].position_color_space.x;
	dst->body.joints[i].position_color_space.y 
	  = src.joints[i].position_color_space.y;
	dst->body.joints[i].position.x 
	  = src.joints[i].position.x;
	dst->body.joints[i].position.y 
	  = src.joints[i].position.y;
	dst->body.joints[i].position.z 
	  = src.joints[i].position.z;
	dst->body.joints[i].orientation.x 
	  = src.joints[i].orientation.x;
	dst->body.joints[i].orientation.y 
	  = src.joints[i].orientation.y;
	dst->body.joints[i].orientation.z 
	  = src.joints[i].orientation.z;
	dst->body.joints[i].orientation.w 
	  = src.joints[i].orientation.w;
      } 

  }

  void faceToMsg(humans_msgs::Face src, humans_msgs::Human *dst )
  {
    //face
    dst->face.position.lt.x = src.position.lt.x;
    dst->face.position.lt.y = src.position.lt.y;
    dst->face.position.rb.x = src.position.rb.x;
    dst->face.position.rb.y = src.position.rb.y;
	  
    dst->face.persons.resize( OKAO );  
    for(int c = 0; c < OKAO; ++c)
      {
	dst->face.persons[c].okao_id = src.persons[c].okao_id;
        dst->face.persons[c].conf = src.persons[c].conf;
        dst->face.persons[c].name = src.persons[c].name;
        dst->face.persons[c].laboratory = src.persons[c].laboratory;
        dst->face.persons[c].grade = src.persons[c].grade;
      }
  }


  void bodyAndFaceToMsg( humans_msgs::Body src_body, humans_msgs::Face src_face, humans_msgs::Human *dst_human)
  {

    MsgToMsg::bodyToMsg( src_body, dst_human );
    MsgToMsg::faceToMsg( src_face, dst_human );

  } 

  void transformHead(geometry_msgs::PointStamped src, 
		     geometry_msgs::PointStamped *dst)
  {
    tf::TransformListener tl;
    try
      {
	//notice ros::time!!
	
	tl.waitForTransform(dst->header.frame_id, src.header.frame_id, 
			    src.header.stamp, ros::Duration(3.0));
	
	tl.transformPoint(dst->header.frame_id, ros::Time(), 
			  src, src.header.frame_id, *dst);
      }
    catch(tf::TransformException& ex)
      {
	ROS_ERROR("Received an exception trying to transform a point to /map: %s", 
		  ex.what());
	//++waittime;
      }

  }

  /*
  void transformJoint(humans_msgs::Joints src,  humans_msgs::Joints *dst)
  {
    tf::TransformListener tflistener;

    geometry_msgs::PoseStamped srcJoint;
    geometry_msgs::PoseStamped dstJoint;

    srcJoint.pose.position.x = src.position.x;
    srcJoint.pose.position.y = src.position.y;
    srcJoint.pose.position.z = src.position.z;
    srcJoint.pose.orientation.x = src.orientation.x;
    srcJoint.pose.orientation.y = src.orientation.y;
    srcJoint.pose.orientation.z = src.orientation.z;
    srcJoint.pose.orientation.w = src.orientation.w;

    try
      {
	tflistener.waitForTransform("map", src.header.frame_id, 
				    ros::Time(), ros::Duration(5.0));
	tflistener.transformPose("map",ros::Time::now(), 
				 srcJoint, src.header.frame_id, dstJoint); 
	
	dst->position.x = dstJoint.pose.position.x;
	dst->position.y = dstJoint.pose.position.y;
	dst->position.z = dstJoint.pose.position.z;
	dst->orientation.x = dstJoint.pose.orientation.x;
	dst->orientation.y = dstJoint.pose.orientation.y;
	dst->orientation.z = dstJoint.pose.orientation.z;
	dst->orientation.w = dstJoint.pose.orientation.w;
      }
    catch(tf::TransformException& ex)
      {
	ROS_ERROR("Received an exception trying to transform a point (no!) to /map: %s", ex.what());
      }
  }
  */
  /*
  void transformJoints(humans_msgs::Body src, humans_msgs::Body *dst)
  {

    for( int i = 0; i < JOINTS; ++i )
      {
	humans_msgs::Joints src_joint = src.joints[i];
	humans_msgs::Joints dst_joint;
	MsgToMsg::transformJoint( src_joint, &dst_joint );
	dst->joints[i] = dst_joint;
      }
  }
  */

}
