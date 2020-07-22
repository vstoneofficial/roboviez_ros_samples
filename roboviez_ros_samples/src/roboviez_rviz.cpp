#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "roboviez_ros_msgs/IMU.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"

#include	<math.h>
#include	<string.h>

tf::Vector3 orgvec;
sensor_msgs::JointState js0;
bool isPositionUpdate = false;

void subscribe_acc(const roboviez_ros_msgs::IMU::ConstPtr& sensor)
{
	static tf::TransformBroadcaster br;
	tf::Vector3 nowvec,axis;
	float theta=0;

	nowvec.setValue(-sensor->acc.z,sensor->acc.y,sensor->acc.x);
	nowvec.normalize();

	axis = nowvec.cross(orgvec);
	theta = (float) acos(nowvec.dot(orgvec));

	tf::Transform transform;
	transform.setOrigin(tf::Vector3(0, 0, 0.0));

	tf::Quaternion q;
	q.setRotation(axis,theta);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));

}

void subscribe_position(const std_msgs::Float64MultiArray::ConstPtr& positions)
{
	int len = positions->data.size();
	js0.header.stamp = ros::Time::now();

	js0.name.resize(len);
	js0.position.resize(len);

	for(int i=0;i<len;i++){
		js0.position[i]=positions->data[i];
		js0.name[i]="id" + std::to_string(i+1) + "_joint";
	}

	isPositionUpdate = true;

}

int main(int argc, char** argv)
{

	ros::init(argc, argv, "roboviez_rviz");
	ros::NodeHandle n;

	orgvec.setValue(0,0,1);

	ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 100);

	ros::Subscriber tf_sub = n.subscribe("/roboviez_ros_controller/imu", 10, subscribe_acc);
	ros::Subscriber positions_sub = n.subscribe("/roboviez_ros_controller/positions", 10, subscribe_position);

	ros::Rate loop_rate(100);
	while (ros::ok())
	{
		if(isPositionUpdate){
			joint_pub.publish(js0);
			isPositionUpdate=false;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}