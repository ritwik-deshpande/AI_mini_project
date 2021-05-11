import heapq
import os
import sys
from collections import OrderedDict


def MSTEfficient(mst_cities) :
    infi = int(sys.maxsize)
    heap = [[infi, city] for city in mst_cities]
    heap[0][0] = 0
    heapq.heapify(heap)
    print (heap)
    visited = []
    unvisited = mst_cities
    lmst = 0
    while len(heap) != 0:
        min_city = heapq.heappop(heap)
        lmst = lmst + min_city[0]
        visited.append(min_city[1])
        for index in range(len(heap)) :
            if heap[index][0] > cost_matrix[min_city[1]-1][heap[index][1]-1] :
                heap[index][0] = cost_matrix[min_city[1]-1][heap[index][1]-1]
        heapq.heapify(heap)
    return lmst

# def MST(cities):
# 	distance_of_cities = dict()
# 	mst_cities = set()
# 	mst_edges = list()
# 	lmst = int(0)

# 	if(len(cities) == 1 ):
# 		return 0,cities

# 	print('The MST for the Cities',cities)
# 	for i_index,i_city in enumerate(cities):
		
# 		for j_index,j_city in enumerate(cities[i_index+1:]):
# 			if cost_matrix[i_city - 1][j_city -1 ] not in distance_of_cities.keys():
# 				distance_of_cities[cost_matrix[i_city -1 ][j_city -1 ]] = list()
# 			distance_of_cities[cost_matrix[i_city -1 ][j_city -1 ]].append((i_city,j_city))
			

# 	distance_of_cities = OrderedDict(sorted(distance_of_cities.items()))

# 	for distance,cities in distance_of_cities.items():
# 		for city in cities: 
# 			if city[0] in mst_cities and city[1] in mst_cities:
# 				pass
# 				# print('Cycle is getting formed hence reject the edge')
# 			else:
# 				mst_cities.add(city[0])
# 				mst_cities.add(city[1])
# 				mst_edges.append(city)
# 				lmst = lmst + distance
# 	# print(mst_cities)
# 	# print(mst_edges)
# 	return lmst,mst_cities


class Search_Node(object):
	def __init__(self,path,cities,city,start_city):
		self.start_city =start_city
		self.city = city
		self.path = path
		self.cities = cities
		self.f_value = self.heuristic()

	def __lt__(self, other):
		
		if self.f_value < other.f_value:
			return True
		else:
			return False

	def heuristic(self):
		unvisited_cities = self.cities
		print(unvisited_cities,self.path)

		g_value = int(0)
		unvisited_cities.remove(self.path[0])
		for city_index in range(1,len(self.path)):
			g_value = g_value + cost_matrix[self.path[city_index-1]-1][self.path[city_index]-1]
			if self.path[city_index] in unvisited_cities:
				unvisited_cities.remove(self.path[city_index])
		if(len(unvisited_cities)==0):
			h_value = 0
		else:
			h_value = MSTEfficient(unvisited_cities)
			mst_cities = unvisited_cities

			#Calculating length of two shortest connections between start node and mst_nodes and current node and mst_nodes

			l1 = min([ cost_matrix[self.start_city -1 ][mst_city -1] for mst_city in mst_cities ])

			l2 =  min([ cost_matrix[self.city -1 ][mst_city -1] for mst_city in mst_cities ])
			print('The l1: {} , l2 :{} values are:'.format(l1,l2))
			h_value = h_value + l1 + l2

		print('G_value {} ,H_value {} for the Search Node {} '.format(g_value,h_value,self.city))
		return g_value + h_value


def A_star_search(cities,start_city):
	expanded_list = []
	fringe_list = []

	found = False
	start_city = start_city
	goal_city = start_city
	path = []
	path.append(start_city)

	parent = Search_Node(path,cities.copy(),start_city,start_city)
	heapq.heappush(expanded_list, parent)
	
	iterations = int(0)
	while not found :
		children = cities.copy()
		print('......................Expanding the city path :{}.................... '.format(parent.path))

		if len(parent.path) == number_of_cities:
			child = start_city
			child_path = parent.path + [child]
			heapq.heappush(fringe_list, Search_Node(child_path,cities.copy(),child,start_city))
		else:
			for child in children:
				if child not in parent.path:
					child_path = parent.path + [child]
					heapq.heappush(fringe_list, Search_Node(child_path,cities.copy(),child,start_city))
					
		

		
		#heapq.heappush(fringe_list, Search_Node([1,3],cities.copy()))
		print('The States in the Fringe List are :')
		for node in fringe_list:
			print('City: {}  , F-Value {} Node Path :{}'.format(node.city,node.f_value,node.path))

		print('The States in the Expanded List are :')
		for node in expanded_list:
			print('City: {}  , F-Value {} '.format(node.city,node.f_value))

		optimal_search_node = heapq.heappop(fringe_list)
		if optimal_search_node.city == goal_city:
			print('Optimal Goal Node reached !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
			return optimal_search_node.path,optimal_search_node.f_value
		elif optimal_search_node in expanded_list:
			print('Do Nothing as Node has been already expanded')
		else:
			parent = optimal_search_node
			heapq.heappush(expanded_list, parent)

		iterations = iterations + 1
		

if __name__ == '__main__':
	global number_of_cities
	cities = list()
	global cost_matrix
	dirname = os.getcwd()
	filename = os.path.join(dirname,'tsp_matrix')
	with open(filename,'r') as f:
		lines = f.readlines()
		number_of_cities = int(lines[0])
		cost_matrix = [[int(x) for x in line.split()] for line in lines[1:]]

	print(cost_matrix)
	for city in range(number_of_cities):
		cities.append(city+1)

	#print(MSTEfficient(cities))

	optimal_path,optimal_cost = A_star_search(cities,2)
	print('The Optimal Path is: ',optimal_path)
	print('The Optimal Cost is:',optimal_cost)

