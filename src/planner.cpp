/** Authors
 * Anush S Kumar (anushkumar27@gmail.com)
 * Sushrith Arkal (sushrith.arkal@gmail.com)
**/

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>

// CB for RPLidar data
std_msgs::Float32MultiArray oa_data;
void oa_data_cb(const std_msgs::Float32MultiArray::ConstPtr& msg) {
	oa_data = *msg;
}

// CB for Commander dest pose
geometry_msgs::PoseStamped dest_pose;
void dest_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
	dest_pose = *msg;
}

// The money function
bool path_planner(const std::vector<float> &distances, const geometry_msgs::PoseStamped &msg, geometry_msgs::PoseStamped &result) {
	result.pose.position.x = msg.pose.position.x;
	result.pose.position.y = msg.pose.position.y;
	result.pose.position.z = msg.pose.position.z;
	return true;
}

int main(int argc, char **argv) {   
	// Initialise ros node and advertise to roscore
	ros::init(argc, argv, "planner");

	// NodeHandle is the main access point to communications with the ROS system.
	ros::NodeHandle nh;

	// SUBSCRIBER
	// To fetch the current state of the FCU
	ros::Subscriber oa_data_sub = nh.subscribe<std_msgs::Float32MultiArray>
			("oa/data", 10, oa_data_cb);
	// To fetch the dest from the commander node
	ros::Subscriber dest_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
			("auto_nav/planner/dest/pose", 10, dest_pose_cb);

	// PUBLISHER
	// To publish the raw destination pose to the FCU
	ros::Publisher pos_pub = nh.advertise<geometry_msgs::PoseStamped>
			("auto_nav/mavros_bridge/pose", 10);

	// The setpoint publishing rate MUST be faster than 2Hz
	ros::Rate rate(20.0);

	geometry_msgs::PoseStamped temp_dest_pose;
	temp_dest_pose.pose.position.x = 0;
	temp_dest_pose.pose.position.y = 0;
	temp_dest_pose.pose.position.z = 0;

	std::vector<float> distances(360);
	std::fill(distances.begin(), distances.end(), std::numeric_limits<float>::infinity());
	
	while(ros::ok()) {
		// Get temp destination to go to
		path_planner(distances, dest_pose, temp_dest_pose);
		// Send command to bridge
		pos_pub.publish(temp_dest_pose);

		if(!oa_data.data.empty()) distances = std::move(oa_data.data);
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}

