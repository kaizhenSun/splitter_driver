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
#include "vision_msgs/boundingbox_message.h"
#include "strttc_msgs/boundingbox_message.h"





class Sub_sync_Pub_ttc{
	public:
	Sub_sync_Pub_ttc(std::string sub_topic_name,std::string bbx_pub_topic_name,std::string bbx_sub_topic_name):sub_topic_name_(sub_topic_name),\
	bbx_pub_topic_name_(bbx_pub_topic_name),bbx_sub_topic_name_(bbx_sub_topic_name){
		bbx_message_sub_=n_.subscribe<strttc_msgs::boundingbox_message>(bbx_sub_topic_name_,5,&Sub_sync_Pub_ttc::bbx_sub_callback,this);
		sync_sub_=n_.subscribe(sub_topic_name_,5,&Sub_sync_Pub_ttc::bbx_callback,this);
		bbx_message_pub_=n_.advertise<strttc_msgs::boundingbox_message>(bbx_pub_topic_name_, 10);
	}
	void bbx_callback(const std_msgs::Time::ConstPtr& msg){
		bbx_msg_pub_.stamp.data=msg->data;
		bbx_msg_pub_.xmin=bbx_msg_sub_.xmin;
		bbx_msg_pub_.ymin=bbx_msg_sub_.ymin;
		bbx_msg_pub_.xmax=bbx_msg_sub_.xmax;
		bbx_msg_pub_.ymax=bbx_msg_sub_.ymax;
		bbx_message_pub_.publish(bbx_msg_pub_);
		printf("subscribe sync bbx successefully!!!");
	}
	void bbx_sub_callback(const strttc_msgs::boundingbox_message::ConstPtr& bbx_msg){
		bbx_msg_sub_=*bbx_msg;
		//printf("subscribe ttc successefully!!!");

	}
	private:
	ros::NodeHandle n_;
	ros::Publisher bbx_message_pub_;
	ros::Subscriber sync_sub_;
	ros::Subscriber bbx_message_sub_;

	strttc_msgs::boundingbox_message bbx_msg_sub_;
	strttc_msgs::boundingbox_message bbx_msg_pub_;

	std::string sub_topic_name_;
	std::string bbx_pub_topic_name_;
	std::string bbx_sub_topic_name_;

	std_msgs::Header syns_time;

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ttc_sync_pub_sub_node");
	ros::NodeHandle nh_private("~");

	std::string sub_topic_name;
	std::string bbx_pub_topic_name;
	std::string bbx_sub_topic_name;


  	nh_private.getParam("sub_topic_name", sub_topic_name);
  	nh_private.getParam("bbx_pub_topic_name", bbx_pub_topic_name);
  	nh_private.getParam("bbx_sub_topic_name", bbx_sub_topic_name);


	Sub_sync_Pub_ttc ttcObject(sub_topic_name,bbx_pub_topic_name,bbx_sub_topic_name);
  	ros::spin();
	return 0;
}


