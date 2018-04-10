/** Authors
 * Anush S Kumar (anushkumar27@gmail.com)
 * Sushrith Arkal (sushrith.arkal@gmail.com)
**/

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <cmath>
#include <algorithm>

// Constants
float ob_safety_buffer;
float size_of_bot;
float max_range;
float interval_size;

// bools to indicate the data has been received from callback
bool oa_data_check = false;
bool dest_pose_check = false;

// CB for RPLidar data
std_msgs::Float32MultiArray oa_data;
void oa_data_cb(const std_msgs::Float32MultiArray::ConstPtr& msg) {
	oa_data = *msg;
	oa_data_check = true;
}

// CB for Commander dest pose
geometry_msgs::PoseStamped dest_pose;
void dest_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
	dest_pose = *msg;
	dest_pose_check = true;
}

// CB for current position of bot
geometry_msgs::PoseStamped curr_pose;
void curr_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	curr_pose = *msg;
	float temp = curr_pose.pose.position.x;
	// curr_pose.pose.position.x = curr_pose.pose.position.y;
	// curr_pose.pose.position.y = -1.0f * temp;
	curr_pose.pose.position.x = -1.0f * curr_pose.pose.position.y;	
	curr_pose.pose.position.y = temp;
}

// Utility functions
float dist_bw_points(const geometry_msgs::PoseStamped &a, const geometry_msgs::PoseStamped &b) {
	return std::sqrt(std::pow(a.pose.position.x - b.pose.position.x, 2) + std::pow(a.pose.position.y - b.pose.position.y ,2));
}

float angle_bw_points(const geometry_msgs::PoseStamped &a, const geometry_msgs::PoseStamped &b) {
	return std::asin((b.pose.position.y - a.pose.position.y) / dist_bw_points(a, b)) * 
		(180.0f/M_PI);
}

int get_rplidar_index(const geometry_msgs::PoseStamped &a, const geometry_msgs::PoseStamped &b) {
	int deg = (int) angle_bw_points(a, b);
	return deg + 90;
}

// Angles for left, right, up and down directions
const int up_angle = 180;
const int down_angle = 0;
const int left_angle = 270;
const int right_angle = 90;

// Flags for left, right, up and down directions
bool up_allowed = true;
bool down_allowed = true;
bool left_allowed = true;
bool right_allowed = true;

// The money function
bool path_planner(const std::vector<float> &distances, const geometry_msgs::PoseStamped &final_dest, const geometry_msgs::PoseStamped &curr_pose, geometry_msgs::PoseStamped &result) {
	
	// Not checking for obstacles in z directions, directly going
	result.pose.position.z = final_dest.pose.position.z;

	// Takeoff first
	if(std::fabs(curr_pose.pose.position.z - dest_pose.pose.position.z) > 0.1f) {
		ROS_INFO("Adjust height");
		result.pose.position.x = curr_pose.pose.position.x;
		result.pose.position.y = curr_pose.pose.position.y;
		return true;
	}

	// Check if final_dest == curr_pose
	// Should be within half of size of quad
	if(dist_bw_points(curr_pose, final_dest) <= size_of_bot * 0.5) {
		ROS_INFO("Reached destination [%f %f %f]", curr_pose.pose.position.x, curr_pose.pose.position.y, curr_pose.pose.position.z);
		result.pose.position.x = final_dest.pose.position.x;
		result.pose.position.y = final_dest.pose.position.y;
		return true;
	}

	// 1. Find dist to dest from curr_pose
	float dist_to_dest = dist_bw_points(curr_pose, final_dest);

	// 2. Find index of rplidar reading to final dest direction
	int index = get_rplidar_index(curr_pose, final_dest);

	// 3. Compare
	if(distances[index] > dist_to_dest) {
		// GO TO DEST
		// TODO: Handle infinity readings in rplidar
		result.pose.position.x = final_dest.pose.position.x;
		result.pose.position.y = final_dest.pose.position.y;
	}
	else if(distances[index] + ob_safety_buffer > dist_to_dest) {
		// GO TO JUST BEFORE THE DEST
		// TODO: Review
		// TODO: Add handling for safety buffer
		result.pose.position.x = final_dest.pose.position.x;
		result.pose.position.y = final_dest.pose.position.y;
	}
	else {
		// DO PATH PLANNING
		// TODO: Make efficient, review
		// TODO: Handle the case where drone is very close to obstacle
		std::vector<float> distances_temp(distances);
		for(int i = 0; i < 360; ++i) {
			if(!( (i == up_angle && up_allowed) || 
				(i == down_angle && down_allowed) || 
				(i == left_angle && left_allowed) || 
				(i == right_angle && right_allowed) )) {
				distances_temp[i] = 0;
			}
		}
		auto temp_dest_ray = std::max_element(distances_temp.begin(), distances_temp.end());
		int temp_dest_index = temp_dest_ray - distances_temp.begin();
		if(temp_dest_index == up_angle) {
			ROS_ERROR("Choosing up!");
			up_allowed = true;
			down_allowed = false;
			left_allowed = true;
			right_allowed = true;
		}
		else if(temp_dest_index == down_angle) {
			ROS_ERROR("Choosing down!");
			up_allowed = false;
			down_allowed = true;
			left_allowed = true;
			right_allowed = true;
		}
		else if(temp_dest_index == left_angle) {
			ROS_ERROR("Choosing left!");
			up_allowed = true;
			down_allowed = true;
			left_allowed = true;
			right_allowed = false;
		}
		else if(temp_dest_index == right_angle) {
			ROS_ERROR("Choosing right!");
			up_allowed = true;
			down_allowed = true;
			left_allowed = false;
			right_allowed = true;
		}
		else {
			ROS_ERROR("temp_dest_index not one of allowed values");
			exit(1);
		}

		// Complex ternary is to protect against multiplying by inf
		// TODO: Clean up
		// if(distances[temp_dest_index] > max_range) infinity_case_index = temp_dest_index;
		// result.pose.position.x = ((distances[temp_dest_index] > max_range) ? max_range * 2 : distances[temp_dest_index]) * std::sin(temp_dest_index * (M_PI/180.0f)) + curr_pose.pose.position.x;
		// result.pose.position.y = ((distances[temp_dest_index] > max_range) ? max_range * 2 : distances[temp_dest_index]) * std::cos(temp_dest_index * (M_PI/180.0f)) + curr_pose.pose.position.y;

		result.pose.position.x = ((distances[temp_dest_index] > interval_size) ? interval_size : distances[temp_dest_index]) * std::sin(temp_dest_index * (M_PI/180.0f)) + curr_pose.pose.position.x;
		result.pose.position.y = -1.0f * ((distances[temp_dest_index] > interval_size) ? interval_size : distances[temp_dest_index]) * std::cos(temp_dest_index * (M_PI/180.0f)) + curr_pose.pose.position.y;
	}
	
	return false;
}

int main(int argc, char **argv) {   
	// Initialise ros node and advertise to roscore
	ros::init(argc, argv, "planner");

	// NodeHandle is the main access point to communications with the ROS system.
	ros::NodeHandle nh;

	nh.param<float>("max_obstacle_range", max_range, 5.0f);
	nh.param<float>("obstacle_safety_buffer", ob_safety_buffer, 0.3f);
	nh.param<float>("size_of_bot", size_of_bot, 1.0f);
	nh.param<float>("interval_size", interval_size, 1.0f);

	// SUBSCRIBER
	// To fetch the RPLidar data
	ros::Subscriber oa_data_sub = nh.subscribe<std_msgs::Float32MultiArray>
			("oa/data", 10, oa_data_cb);
	// To fetch the dest from the commander node
	ros::Subscriber dest_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
			("auto_nav/planner/dest/pose", 10, dest_pose_cb);
	// To fetch the current position of the bot
	ros::Subscriber curr_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
			("mavros/local_position/pose", 10, curr_pose_cb);

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
	
	// Launch Order:
	// 1. Gazebo
	//		1.1 Quad IMU
	//		1.2 Optical Flow
	// 		1.3 RPLidar
	// 2. OA
	//		2.1 Publish filtered lidar data
	// 3. Planner
	// 4. Bridge
	//		4.1 Wait for dest - wil not arm or mode switch
	// 		4.2 Arm and Switch Mode -> After receiving dest
	
	// TODO: 
	// 1. Add loop to buffer the distance(rplidar) for 5 seconds
	// 2. Add a global param to store the size of the quad, to decide reached or not
	// 3. Add a global param to safety boundary around the obstacle (OA)

	// Pre-Checks:
	// 1. RPLidar Data
	//		1.1. Take-off only if there is a free path
	// 2. Commander Data (Bridge)
	// 		wait for data

	// Check for critical data
	while(ros::ok()) {
		if(oa_data_check && dest_pose_check) {
			ROS_INFO("OA DATA and DESTINATION Received");
			break;
		}
		else {
			ROS_WARN("OA DATA or DESTINATION Not Received");
		}
		ros::spinOnce();
		rate.sleep();
	}

	// Get initial point to go to
	path_planner(distances, dest_pose, curr_pose, temp_dest_pose);
	while(ros::ok()) {

		// Get temp destination to go to (path planning)
		ROS_INFO("Dist to temp dest: %f", dist_bw_points(curr_pose, temp_dest_pose));
		ROS_INFO("Target pos: [%f %f %f]", dest_pose.pose.position.x, dest_pose.pose.position.y, dest_pose.pose.position.z);
		ROS_INFO("Curr pos: [%f %f %f]", curr_pose.pose.position.x, curr_pose.pose.position.y, curr_pose.pose.position.z);
		ROS_INFO("Publishing [%f %f %f]", temp_dest_pose.pose.position.x, temp_dest_pose.pose.position.y, temp_dest_pose.pose.position.z);

		if(dist_bw_points(curr_pose, temp_dest_pose) > size_of_bot * 0.5) {
			// Send command to bridge
			ROS_INFO("Publishing [%f %f %f]", temp_dest_pose.pose.position.x, temp_dest_pose.pose.position.y, temp_dest_pose.pose.position.z);
			pos_pub.publish(temp_dest_pose);
		}
		else {
			ROS_INFO("Taking new point");
			path_planner(distances, dest_pose, curr_pose, temp_dest_pose);
			pos_pub.publish(temp_dest_pose);
		}

		ROS_INFO(" ");

		if(!oa_data.data.empty()) distances = std::move(oa_data.data);
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}