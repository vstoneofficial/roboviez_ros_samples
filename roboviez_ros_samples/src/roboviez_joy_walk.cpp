#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "roboviez_ros_msgs/Walking.h"
#include "sensor_msgs/Joy.h"

#define	WALK_BTN	7
#define	SELECT_BTN	8
#define	START_BTN	9

#define	VAXIS_STK	1
#define	HAXIS_STK	2
#define	ZAXIS_STK	3


roboviez_ros_msgs::Walking walk_vel;
std_msgs::Bool power;
bool istoggle=false,send_pow=false,send_stk=false;

void joy_callback(const sensor_msgs::Joy& joy_msg)
{
	ROS_INFO("%+f,%+f,%f",joy_msg.axes[VAXIS_STK],joy_msg.axes[HAXIS_STK],joy_msg.axes[ZAXIS_STK]);
	if(joy_msg.buttons[SELECT_BTN] && joy_msg.buttons[START_BTN]){
		if(!istoggle){
			send_pow=true;
			power.data=!power.data;
			istoggle=true;
		}
	}
	else istoggle=false;

	walk_vel.walking=joy_msg.buttons[WALK_BTN]>0;
	walk_vel.v_axis = joy_msg.axes[VAXIS_STK];
	walk_vel.h_axis = joy_msg.axes[HAXIS_STK];
	walk_vel.z_axis = joy_msg.axes[ZAXIS_STK];
	send_stk=true;

}

int main(int argc, char **argv)
{
	ROS_INFO("control roboviez with joystick.");
	ros::init(argc, argv, "roboviez_joy_controll");

	ros::NodeHandle n;

	ros::Publisher walk_pub = n.advertise<roboviez_ros_msgs::Walking>("walk", 1000);
	ros::Publisher pow_pub = n.advertise<std_msgs::Bool>("pow", 1000);
	ros::Subscriber joy_sub = n.subscribe("joy", 10, joy_callback);

	walk_vel.v_axis=walk_vel.h_axis=0.0f;
	walk_vel.walking=false;
	power.data=false;

	ros::Rate loop_rate(100);
	while (ros::ok())
	{

		if(send_pow){
			pow_pub.publish(power);
			ROS_INFO("send pow");
			send_pow=false;
		}
		if(send_stk){
			walk_pub.publish(walk_vel);
			ROS_INFO("send stick");
			send_stk=false;
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}


