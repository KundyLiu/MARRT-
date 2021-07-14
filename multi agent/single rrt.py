'''
Single rrt.
'''

import math
import random
import numpy as np
import matplotlib.pyplot as plt

class node:
	def __init__(self, x, y):
		self.x = x
		self.y = y
		self.parent = None
		self.x_path = None
		self.y_path = None

def check_collision(node):
	''' No obstacles right now'''
	return False

def goal_check(node, goal_node):
	if get_dist(node, goal_node) < 5:
		return True
	return False

def get_random_node():
	x = random.randint(1, 100)
	y = random.randint(1, 100)
	new_node = node(x, y)
	if check_collision(new_node):
		return None
	return node(x, y)

def get_dist(node1, node2):
	return math.sqrt((node1.x -node2.x)**2 + (node1.y - node2.y)**2)

def get_nearest_node(rand_node, nodes):
	nearest_node = None
	min_dist = float('inf')
	for node in nodes:
		dist = get_dist(rand_node, node)
		if dist < min_dist:
			nearest_node = node
			min_dist = dist
	return nearest_node

def step_ahead(parent, dest_node):
	par_x = parent.x
	par_y = parent.y
	dest_x = dest_node.x
	dest_y = dest_node.y
	theta = np.arctan2((dest_y-par_y), (dest_x-par_x))
	count = 0
	x_path = [par_x]
	y_path = [par_y]
	x = par_x
	y = par_y
	while(count < 10): 
		dx = 0.2 * math.cos(theta)
		dy = 0.2 * math.sin(theta)
		x = x + dx
		y = y + dy
		if check_collision(node(x, y)):
			return None
		x_path.append(x)
		y_path.append(y)
		count = count + 1
	new_node = node(x, y)
	new_node.parent = parent
	new_node.x_path = x_path
	new_node.y_path = y_path
	return new_node
def rrt(start_node, goal_node):
	nodes = [start_node]
	fig = plt.figure()
	ax = fig.gca()
	ax.set_xlim([0,100])
	ax.set_ylim([0,100])
	cir_start = plt.Circle((start_node.x, start_node.y), 1, fill=True, color = 'b')
	cir_goal = plt.Circle((goal_node.x, goal_node.y), 1, fill=True, color = 'b')

	ax.add_patch(cir_start)
	ax.add_patch(cir_goal)

	while (True):
		rand_node = get_random_node()
		if rand_node == None:
			continue

		nearest_node = get_nearest_node(rand_node, nodes)
		new_node = step_ahead(nearest_node, rand_node)

		# if collision detected, continue
		if new_node == None:
			continue

		nodes.append(new_node)
		# draw_explored(new_node)

		cir_node = plt.Circle((new_node.x, new_node.y), 0.2, fill=True, color = 'r')
		ax.add_patch(cir_node)
		plt.plot(new_node.x_path, new_node.y_path, color = 'g', linewidth = 1)
		plt.pause(0.01)

		if goal_check(new_node, goal_node):
			print("Goal reached")
			break
	return new_node

def backtrace(cur_node):
	if(cur_node.parent == None):
		return [], []

	path_x, path_y = backtrace(cur_node.parent)

	path_x.append(cur_node.x_path)
	path_y.append(cur_node.y_path)

	return path_x, path_y

def main():
	start = [50, 50]
	start_node = node(start[0], start[1])
	goal = [90, 90]
	goal_node = node(goal[0], goal[1])

	rrt1 = rrt(start_node, goal_node)

	# backtrace path 
	path_x, path_y = backtrace(rrt1)

	plt.plot(path_x, path_y, color = 'r', linewidth = 1.5)
	plt.show()

if __name__ == "__main__":
	main()

