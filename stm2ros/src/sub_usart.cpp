#include <iostream>
#include <string>
#include <sstream>
using namespace std;

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Header.h>
#include <std_msgs/Time.h>
#include <std_msgs/Float64.h>


#include "serial/serial.h"
#include "stm2ros/speed_distance.h"
#include "vision_msgs/ttc_message.h"
#include "strttc_msgs/ttc_message.h"
#define sBUFFER_SIZE 1024
#define rBUFFER_SIZE 1024
unsigned char s_buffer[sBUFFER_SIZE];
unsigned char r_buffer[rBUFFER_SIZE];

serial::Serial ser;
std_msgs::Header syns_time;

typedef union
{
	float data;
	unsigned char data8[4];
} data_u;
data_u speed;
data_u distance_ttc;



int main(int argc, char** argv)
{
	ros::init(argc, argv, "serial_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	double real_distance;
	ros::param::get("/stm2ros_node/real_distance",real_distance);


  	nh_private.getParam("sub_topic_name", real_distance);


	// 打开串口
	try
	{
		ser.setPort("/dev/ttyUSB0");
		ser.setBaudrate(115200);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		ser.setTimeout(to);
		ser.open();
	}
	catch(serial::IOException &e)
	{
		ROS_INFO_STREAM("Failed to open port ");
		return -1;
	}
	ROS_INFO_STREAM("Succeed to open port" );

	int cnt = 0;
	ros::Rate loop_rate(30);
	ros::Publisher speed_dis_pub = nh.advertise<stm2ros::speed_distance>("stm2ros", 5);
	ros::Publisher ttc_message_pub = nh.advertise<strttc_msgs::ttc_message>("g_ttc", 5);
	while(ros::ok())
	{
		stm2ros::speed_distance msg;
		strttc_msgs::ttc_message ttc_msg;
		if(ser.available())
		{
			// ROS_INFO("%f, %f,%d", speed.data,distance_ttc.data ,cnt);
			// 读取串口数据
			size_t bytes_read = ser.read(r_buffer, ser.available());
			// 使用状态机处理读取到的数据，通信协议与STM32端一致
			int state = 0;
			unsigned char buffer[8] = {0};
			for(int i = 0; i < bytes_read && i < rBUFFER_SIZE; i++)
			{
				if(state == 0 && r_buffer[i] == 0xAA)
				{
					state++;
				}
				else if(state == 1 && r_buffer[i] == 0xBB)
				{
					state++;
				}
				else if(state >= 2 && state < 10)
				{
					buffer[state-2]=r_buffer[i];
					state ++;
					if(state == 10)
					{
						for(int k = 0; k < 4; k++)
						{
							speed.data8[k] = buffer[k];
							distance_ttc.data8[k]=buffer[k+4];
						}						
						ROS_INFO("%f, %f,%d", speed.data,distance_ttc.data ,cnt);
						state = 0;
					}
				}
				else state = 0;
			}
		}
		std::cout<<"real_distance:"<<real_distance<<std::endl;
		// 发布接收到的stm32串口数据
		msg.header.stamp=ros::Time::now();
		msg.speed = 0.001*speed.data;
		msg.distance=real_distance-distance_ttc.data;
		speed_dis_pub.publish(msg);
		ttc_msg.stamp.data=syns_time.stamp;

		if(msg.speed!=0)
		ttc_msg.ttc.data=msg.distance/msg.speed;
		else
		ttc_msg.ttc.data=0;

		ttc_message_pub.publish(ttc_msg);
		// ros::spinOnce();
		loop_rate.sleep();
		cnt++;
	}
	// 关闭串口
	ser.close();
	return 0;
}
