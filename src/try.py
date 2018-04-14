from math import ceil

def enable_connections(graph, l, b, node):
	# left
	if node % l != 1:
		graph[node - 1][node - 2] = 1
		graph[node - 2][node - 1] = 1
	# right
	if node % l != 0:
		graph[node - 1][node] = 1
		graph[node][node - 1] = 1
	# top
	if node <= l * (b-1):
		graph[node - 1][node + l - 1] = 1 
		graph[node + l - 1][node - 1] = 1 
	# down
	if node > l:
		graph[node - 1][node - l - 1] = 1
		graph[node - l - 1][node - 1] = 1

def disable_connections(graph, node):
	for i in range(len(graph[node - 1])):
		graph[node - 1][i] = 0
		graph[i][node - 1] = 0

def build_graph(dest):
	l = ceil(dest[0])
	b = ceil(dest[1])
	n = l * b
	graph = [ [ 0 for i in range(n) ] for i in range(n) ]
	for i in range(1, n+1):
		enable_connections(graph, l, b, i)
	return l, b, graph

def find_adjacent_nodes(graph, node, visited):
	def _is_visited(_v, _n):
		_r = False
		for _tn in _v.keys():
			if _tn == _n:
				_r = True
				break
		return _r
	result = []
	for temp_node, connection in enumerate(graph[node - 1]):
		if connection == 1 and not _is_visited(visited, temp_node + 1):
			result.append(temp_node + 1)
	return result

def find_path(graph, start_node, end_node):
	path = []
	
	visited = {start_node:None}
	temp = find_adjacent_nodes(graph, start_node, visited)
	for t in temp:
		visited[t] = start_node
	
	queue = []
	queue.extend( temp )
	
	prev_node = start_node
	while len(queue) != 0:
		curr_node = queue.pop()
		# visited[curr_node] = prev_node
		temp = find_adjacent_nodes(graph, curr_node, visited)
		for t in temp:
			visited[t] = curr_node
		# print('curr_node:', curr_node)
		# print('adjacent_nodes: ', temp)
		# print('visited: ', visited)
		# print('queue: ', queue)
		# print()
		if curr_node == end_node: break
		queue.extend( temp )
		prev_node = curr_node

	# print('\n\n\npath finding done\n\n\n')

	if end_node not in visited.keys():
		print('Cannot find path from', start_node, 'to', end_node)
	else:
		# print('path possible!')
		#print(visited)
		# for i in sorted(visited.items()):
			# print(i)
		temp = end_node
		path.append(temp)
		while temp != start_node:
			# print(path)
			# print('from', temp, 'to', visited[temp])
			path.append(visited[temp])
			temp = visited[temp]
		path.reverse()
	
	return path

def print_graph(graph):
	print('   ', end='')
	for i in range(len(graph)):
		print(i+1, ' ', end='')
	print()
	for i, r in enumerate(graph):
		print(i + 1, r)

# if __name__ == "__main__":
# 	l, b, graph = build_graph((1.3, 2.4))
# 	print_graph(graph)
# 	disable_connections(graph, 3)
# 	print_graph(graph)

# 	path = find_path(graph, 1, 6)

# 	print('path', path)

# if __name__ == "__main__":
# 	l, b, graph = build_graph((2.3,2.3))
# 	print_graph(graph)
# 	disable_connections(graph, 5)
# 	disable_connections(graph, 7)
# 	# disable_connections(graph, l, b, 1)
# 	print_graph(graph)

# 	path = find_path(graph, 4, 9)

# 	print('path', path)

if __name__ == "__main__":
	l, b, graph = build_graph((5.6,5.6))
	# print_graph(graph)
	for i in [13,14,15,26,27,28,29,30]:
		disable_connections(graph, i)
	# disable_connections(graph, l, b, 1)
	# print_graph(graph)

	path = find_path(graph, 6, 36)

	print('path', path)