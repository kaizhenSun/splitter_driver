#include <iostream>
#include <string>
#include <sstream>


#include "ros/ros.h"
#include <string>
#include "std_msgs/String.h"
#include <std_msgs/Header.h>
#include <std_msgs/Time.h>
#include <std_msgs/Float64.h>


#include "serial/serial.h"
#include "stm2ros/speed_distance.h"
#include "vision_msgs/ttc_message.h"
#include "strttc_msgs/ttc_message.h"




class Sub_sync_Pub_ttc{
	public:
	Sub_sync_Pub_ttc(std::string sub_topic_name,std::string ttc_pub_topic_name,std::string ttc_sub_topic_name):sub_topic_name_(sub_topic_name),\
	ttc_pub_topic_name_(ttc_pub_topic_name),ttc_sub_topic_name_(ttc_sub_topic_name){
		ttc_message_sub_=n_.subscribe<strttc_msgs::ttc_message>(ttc_sub_topic_name_,5,&Sub_sync_Pub_ttc::ttc_sub_callback,this);
		sync_sub_=n_.subscribe(sub_topic_name_,5,&Sub_sync_Pub_ttc::ttc_callback,this);
		ttc_message_pub_=n_.advertise<strttc_msgs::ttc_message>(ttc_pub_topic_name_, 10);
	}
	void ttc_callback(const std_msgs::Time::ConstPtr& msg){
		ttc_msg_pub_.stamp.data=msg->data;
		ttc_msg_pub_.ttc=ttc_msg_sub_.ttc;
		ttc_message_pub_.publish(ttc_msg_pub_);
		//printf("subscribe successefully!!!");
	}
	void ttc_sub_callback(const strttc_msgs::ttc_message::ConstPtr& ttc_msg){
		ttc_msg_sub_.ttc=ttc_msg->ttc;
		//printf("subscribe ttc successefully!!!");

	}
	private:
	ros::NodeHandle n_;
	ros::Publisher ttc_message_pub_;
	ros::Subscriber sync_sub_;
	ros::Subscriber ttc_message_sub_;

	strttc_msgs::ttc_message ttc_msg_sub_;
	strttc_msgs::ttc_message ttc_msg_pub_;

	std::string sub_topic_name_;
	std::string ttc_pub_topic_name_;
	std::string ttc_sub_topic_name_;

	std_msgs::Header syns_time;

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ttc_sync_pub_sub_node");
	ros::NodeHandle nh_private("~");

	std::string sub_topic_name;
	std::string ttc_pub_topic_name;
	std::string ttc_sub_topic_name;


  	nh_private.getParam("sub_topic_name", sub_topic_name);
  	nh_private.getParam("ttc_pub_topic_name", ttc_pub_topic_name);
  	nh_private.getParam("ttc_sub_topic_name", ttc_sub_topic_name);


	Sub_sync_Pub_ttc ttcObject(sub_topic_name,ttc_pub_topic_name,ttc_sub_topic_name);
  	ros::spin();
	return 0;
}


