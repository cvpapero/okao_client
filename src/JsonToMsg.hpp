/*
2015.6.12------------------------
コアダンプが生じやすいので配列指定を使わないようにしたい（途中）
なるべくイテレータを使いたい

2014.10.29------------------------
このヘッダがあるだけでJSON ---> ROS Messageできるようにする
*/


#pragma
#include "picojson.h"
#include <humans_msgs/Humans.h>
#include <cstdlib>

#define OKAO 3

/*
class POSE{
public:
  double x;
  double y;
  double z;
};
*/
namespace JsonToMsg{

  void face(picojson::value v, humans_msgs::Face *okao, 
	    double cx, double cy, bool *p_ok, int threshold)
  {  
    std::vector<int> pos;
    std::vector<int> direction;
    std::vector<int> gaze_direction;

    picojson::object& obj = v.get<picojson::object>();
    //facesの中身があるならば
    if( obj.find("error") != obj.end() && obj["error"].get<std::string>().empty())
      {
	if( obj.find("faces") != obj.end() )
	  {
	    picojson::array array = obj["faces"].get<picojson::array>();
	    //int num = 0;
	    for(picojson::array::iterator it = array.begin(); it != array.end(); ++it)
	      {
		*p_ok = true;
		picojson::object& person_obj = it->get<picojson::object>();
		
		//顔の位置のとりだし
		picojson::array pos_array = 
		  person_obj["position"].get<picojson::array>();
		for(picojson::array::iterator pos_it = pos_array.begin(); 
		    pos_it != pos_array.end(); ++pos_it)
		  {
		    pos.push_back((int)pos_it->get<double>());
		  }
	       
		//顔の方向
		picojson::array direction_array = 
		  person_obj["direction"].get<picojson::array>();
		for(picojson::array::iterator d_it = direction_array.begin(); 
		    d_it != direction_array.end(); ++d_it)
		  {
		    direction.push_back((int)d_it->get<double>());
		  }

		//視線の方向
		picojson::array gaze_direction_array = 
		  person_obj["gaze_direction"].get<picojson::array>();
		for(picojson::array::iterator g_d_it = gaze_direction_array.begin(); 
		    g_d_it != gaze_direction_array.end(); ++g_d_it)
		  {
		    gaze_direction.push_back((int)g_d_it->get<double>());
		  }
		
		picojson::array points_array = 
		  person_obj["points"].get<picojson::array>();
		for(picojson::array::iterator ps_it = points_array.begin();
		    ps_it != points_array.end(); ++ps_it) 
		  {
		    humans_msgs::XYConf face_point;
		    picojson::array fp_array = ps_it->get<picojson::array>();
		    face_point.x = (int)fp_array[0].get<double>();
		    face_point.y = (int)fp_array[1].get<double>();
		    face_point.conf = (int)fp_array[2].get<double>();
		    okao->points.push_back(face_point);
		  }


		//人物ID,信頼度,顔機関開閉度の取り出し
		picojson::array id_array = 
		  person_obj["id"].get<picojson::array>();
		picojson::array db_info_array = 
		  person_obj["db_info"].get<picojson::array>();
		picojson::array open_level_array =
		  person_obj["open_level"].get<picojson::array>();

		double tmp_id[3], tmp_conf[3]; 
		std::string tmp_name[3], tmp_grade[3], tmp_laboratory[3];
		
		for(int n = 0; n < OKAO; ++n)
		  {
		    picojson::array id_array_array = 
		      id_array[n].get<picojson::array>();
		    tmp_id[n] = (int)id_array_array[0].get<double>();
		    tmp_conf[n] = (int)id_array_array[1].get<double>();
		    picojson::object& db_info_obj = 
		      db_info_array[n].get<picojson::object>();
		    tmp_name[n] = 
		      db_info_obj["name"].get<std::string>();
		    tmp_grade[n] = 
		      db_info_obj["grade"].get<std::string>();
		    tmp_laboratory[n] = 
		      db_info_obj["laboratory"].get<std::string>();		 
		  }	
		
		for(picojson::array::iterator ol_it = open_level_array.begin();
		    ol_it != open_level_array.end(); ++ol_it)
		  {
		    picojson::array ol_array = ol_it->get<picojson::array>();
		    humans_msgs::DegConf dc;
		    dc.deg = (int)ol_array[0].get<double>();//開閉度
		    dc.conf = (int)ol_array[1].get<double>();//信頼度
		    okao->open_level.push_back(dc);
		  }

		//一人目の信頼度を利用する
		if(tmp_conf[0] < threshold)
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
		
		//		okao->persons.resize( OKAO );
		
		for(int n = 0; n < OKAO; ++n)
		  {
		    /*
		    okao->persons[n].okao_id = tmp_id[n];
		    okao->persons[n].conf = tmp_conf[n];
		    okao->persons[n].name = tmp_name[n];
		    okao->persons[n].laboratory = tmp_laboratory[n];
		    okao->persons[n].grade = tmp_grade[n];
		    */
		    humans_msgs::Person person;
		    person.okao_id = tmp_id[n];
		    person.conf = tmp_conf[n];
		    person.name = tmp_name[n];
		    person.laboratory = tmp_laboratory[n];
		    person.grade = tmp_grade[n];
		    okao->persons.push_back(person);
		  }
		okao->position.lt.x = pos[0]+cx;
		okao->position.lt.y = pos[1]+cy;
		okao->position.rt.x = pos[2]+cx;
		okao->position.rt.y = pos[1]+cy;
		okao->position.lb.x = pos[0]+cx;
		okao->position.lb.y = pos[3]+cy;
		okao->position.rb.x = pos[2]+cx;
		okao->position.rb.y = pos[3]+cy;
		okao->position.conf = pos[5];

		okao->direction.x = direction[0];
		okao->direction.y = direction[1];
		okao->direction.r = direction[2];
		okao->direction.conf = direction[3];
		
		okao->gaze_direction.x = gaze_direction[0];
		okao->gaze_direction.y = gaze_direction[1];
		okao->gaze_direction.conf = gaze_direction[2];
		
	      }	
	  }    
      }
    
  }
}
