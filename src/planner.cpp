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
	return std::acos((b.pose.position.x - a.pose.position.x) / dist_bw_points(a, b)) * 
		(180.0f/M_PI);
}

int get_rplidar_index(const geometry_msgs::PoseStamped &a, const geometry_msgs::PoseStamped &b) {
	int deg = (int) angle_bw_points(a, b);
	return deg + 90;
}

// Graph functions
int l;
int b;
std::vector<std::vector<int>> graph;

void enable_connections(std::vector<std::vector<int> > &graph, int l, int b, int node) {
	// left
	if(node % l != 1) {
		graph[node - 1][node - 2] = 1;
		graph[node - 2][node - 1] = 1;
	}
	// right
	if(node % l != 0) {
		graph[node - 1][node] = 1;
		graph[node][node - 1] = 1;
	}
	// top
	if(node <= l * (b-1)) {
		graph[node - 1][node + l - 1] = 1;
		graph[node + l - 1][node - 1] = 1;
	}
	// down
	if(node > l) {
		graph[node - 1][node - l - 1] = 1;
		graph[node - l - 1][node - 1] = 1;
	}
}

void disable_connections(std::vector<std::vector<int> > &graph, int l, int b, int node) {
	for(int i = 0; i < graph[node - 1].size(); i++) {
		graph[node - 1][i] = 0;
		graph[i][node - 1] = 0;
	}
}

void build_graph(const geometry_msgs::PoseStamped &dest, int &l, int &b) {
	l = std::ceil(dest.pose.position.x);
	b = std::ceil(dest.pose.position.y);
	
	int n = l * b;

	for(int i = 0; i < n; i++) {
		graph.push_back(std::vector<int>(n));
	}

	for(int i = 1; i <= n; i++) {
		enable_connections(graph, l, b, i);
	}
}

bool _is_visited(const std::map<int, int> &visited, int node) {
	return visited.find(node) != visited.end();
}

void find_adjacent_nodes(const std::vector<std::vector<int> > &graph, const int node, const std::map<int, int> &visited, std::vector<int> &result) {
	int n = graph[node - 1].size();
	for(int i = 0; i < n; i++) {
		if(graph[node - 1][i] == 1 && !_is_visited(visited, i + 1)) {
			result.push_back(i + 1);
		}
	}
}

void find_path(const std::vector<std::vector<int> > &graph, const int start_node, const int end_node, std::vector<int> &path) {
	std::map<int, int> visited;

	visited[start_node] = -1;

	std::vector<int> temp;

	find_adjacent_nodes(graph, start_node, visited, temp);

	std::queue<int> queue;
	
	for(auto e : temp) {
		visited[e] = start_node;
		queue.push(e);
	}
	temp.clear();

	int prev_node = start_node;

	while(queue.size() != 0) {
		int curr_node = queue.front();
		queue.pop();

		find_adjacent_nodes(graph, curr_node, visited, temp);

		for(int e : temp) {
			visited[e] = curr_node;
			queue.push(e);
		}
		temp.clear();
		
		if(curr_node == end_node) {
			break;
		}

		prev_node = curr_node;
	}

	if(visited.find(end_node) != visited.end()) {
		std::cout << "Wohhoo\n";
		int temp_node = end_node;
		path.push_back(temp_node);
		while(temp_node != start_node) {
			path.push_back(visited[temp_node]);
			temp_node = visited[temp_node];
		}
		std::reverse(path.begin(), path.end());
	}
	else {
		std::cout << "Shaata\n";
	}
}

// The money function
bool path_planner(const std::vector<float> &distances, const geometry_msgs::PoseStamped &final_dest, const geometry_msgs::PoseStamped &curr_pose, geometry_msgs::PoseStamped &result) {

}

int main(int argc, char **argv) {   
	// Initialise ros node and advertise to roscore
	ros::init(argc, argv, "planner");

	// NodeHandle is the main access point to communications with the ROS system.
	ros::NodeHandle nh;

	nh.param<float>("max_obstacle_range", max_range, 6.0f);
	nh.param<float>("obstacle_safety_buffer", ob_safety_buffer, 1.0f);
	nh.param<float>("size_of_bot", size_of_bot, 1.0f);
	nh.param<float>("interval_size", interval_size, 0.5f);

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

	// Initialise graph
	build_graph(dest_pose, l, b);

	while(ros::ok()) {

		// Get temp destination to go to (path planning)
		ROS_INFO("Dist to temp dest: %f", dist_bw_points(curr_pose, temp_dest_pose));
		ROS_INFO("Target pos: [%f %f %f]", dest_pose.pose.position.x, dest_pose.pose.position.y, dest_pose.pose.position.z);
		ROS_INFO("Curr pos: [%f %f %f]", curr_pose.pose.position.x, curr_pose.pose.position.y, curr_pose.pose.position.z);
		ROS_INFO("Publishing [%f %f %f]", temp_dest_pose.pose.position.x, temp_dest_pose.pose.position.y, temp_dest_pose.pose.position.z);
		
		path_planner(distances, dest_pose, curr_pose, temp_dest_pose);
		pos_pub.publish(temp_dest_pose);
	
		ROS_INFO(" ");

		if(!oa_data.data.empty()) distances = std::move(oa_data.data);
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}