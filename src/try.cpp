#include <vector>
#include <map>
#include <queue>
#include <cmath>
#include <algorithm>
#include <iostream>

std::vector<std::vector<int> > graph;

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

void disable_connections(std::vector<std::vector<int> > &graph, int node) {
	for(int i = 0; i < graph[node - 1].size(); i++) {
		graph[node - 1][i] = 0;
		graph[i][node - 1] = 0;
	}
}

void build_graph(float x, float y, int &l, int &b) {
	l = std::ceil(x);
	b = std::ceil(y);
	
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

int main() {
	int l;
	int b;
	build_graph(5.6, 5.6, l, b);

	for(auto e1 : graph) {
		for(auto e2 : e1) {
			std::cout << e2 << " ";
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;

	int temp[] = {13,14,15,26,27,28,29,30};//,    4, 5,6};
	for(auto e1 : temp)
		disable_connections(graph, e1);

	for(auto e1 : graph) {
		for(auto e2 : e1) {
			std::cout << e2 << " ";
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;

	std::vector<int> path;
	find_path(graph, 1, 36, path);

	for(auto e : path) {
		std::cout << e << " -> ";
	}
	std::cout << "\n";

	return 0;
}

/*

	1 -> 2 -> 3 -> 9 -> 10 -> 16 -> 22 -> 21 -> 20 -> 19 -> 25 -> 31 -> 32 -> 33 -> 34 -> 35 -> 36 ->



*/