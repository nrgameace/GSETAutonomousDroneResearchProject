# Demonstration and Visualization of the A* Algorithm a 2D Grid World
# Compatible with Python 3.8.1 and pygame 2.1.2

# Importing the Required Libraries
import pygame
import math
import random

from djitellopy import tello

# Information for saving the animation frames


nodeNew = []

HIGHDENSITY1 = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0], [0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0], [0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0], [0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0], [0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0], [0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0], [0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0], [0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0], [0, 1, 0, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]
HIGHDENSITY2 = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0], [0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0], [0, 1, 0, 0, 0, 1, 1, 1, 1, 0, 1, 0, 0], [0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0], [0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0], [0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0], [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0], [0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 1, 0, 1, 1, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]
HIGHDENSITY3 = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0], [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0], [0, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0], [0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0], [0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0], [0, 1, 1, 0, 0, 0, 1, 0, 1, 1, 0, 0, 0], [0, 1, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0], [0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]



# Initializing variables defining the world and algorithm
# Can be varied as per convenience and world/algorithm specifications
WINDOW_LENGTH = 325					# Length of the grid world along the X-axis
WINDOW_BREADTH = 325				# Length of the grid world along the Y-axis
NODE_RADIUS = 1							# Radius of the circle displayed for each node
GOAL_RADIUS = 20						# Radius of goal reachability to ensure the algorithm has finished
EPSILON = 20						# Determines how far to place each node from its parent
REWIRING_RADIUS = 25					# Radius to search for nodes to rewire/compare cost
BUFFER = 5
CONVERSION_FACTOR = 2

BLUE = (0, 0, 255)

OBSTACLES = [[]]
OBSTACLES1 = []
OBSTACLESDRAW = []
MAP_TYPE = 1

LOW = 20
HIGH = 40
global distanceTheoretical
distanceTheoretical = 0

w = 13
h = 13


#Puts in Barriers
for i in range(13):
	OBSTACLESDRAW.append((i * 25, 0, 25, 25))
	OBSTACLES1.append((i * 25, 0, 25 + BUFFER, 25 + BUFFER))

	OBSTACLESDRAW.append((i * 25, 12 * 25, 25, 25))
	OBSTACLES1.append((i * 25, 12 * 25, 25 + BUFFER, 25 + BUFFER))

	OBSTACLESDRAW.append((0, i * 25, 25, 25))
	OBSTACLES1.append((0, i * 25, 25 + BUFFER, 25 + BUFFER))

	OBSTACLESDRAW.append((12 * 25, i * 25, 25, 25))
	OBSTACLES1.append((12 * 25, i * 25, 25 + BUFFER, 25 + BUFFER))
# CHoose random block mazes 
def random_Blocks():
    for i in range(len(HIGHDENSITY1)):
        for j in range(len(HIGHDENSITY1[0])):
            if HIGHDENSITY1[i][j] == 1:
                OBSTACLESDRAW.append((i * 25, j * 25, 25, 25))
                OBSTACLES1.append((i * 25, j * 25, 25 + BUFFER, 25 + BUFFER))



random_Blocks()
#ladder_Map1()
#ladder_Map3()

        


#print(OBSTACLES)
# Defining colour values across the RGB Scale
WHITE = (255,255,255)
BLACK = (0,0,0)
GREEN = (0,255,0)
RED = (255,0,0)
BLUE = (0,0,255)
ORANGE = (255,164.5,0)



# A class to define the characteristics of each grid cell (generalized to each discrete data point in a robotic configuration space)
class Node:

	def __init__(self,coord,start_node=False,target_node=False):
		self.x = coord[0]
		self.y = coord[1]
		self.parent = None
		self.children = []
		self.cost =  1e7
		self.start_node = start_node
		self.target_node = target_node
		if self.start_node:
			self.cost = 0

	# Colouring a node for visualization processes in pygame
	def visualize_node(self,viz_window):
		
		colour = RED
		if self.start_node or self.target_node:
			colour = ORANGE
		pygame.draw.circle(viz_window,colour,(self.x,self.y),NODE_RADIUS,width=0)
		pygame.display.update()
		
obstacleCoordinates = []
# Initializing obstacles on the map
def initialize_obstacles(viz_window):
	
	global obstacleCoordinates
	
	obstacle_list = {"rectangles":[]}
	for i in range(len(OBSTACLESDRAW)):
		obstacleCoordinates.append(OBSTACLES1[i])
		obstacle_list["rectangles"].append(pygame.Rect(OBSTACLESDRAW[i])) 
		pygame.draw.rect(viz_window, BLACK, OBSTACLESDRAW[i])

	
	pygame.display.update()
	return obstacle_list

# Checking if the new point generated collides with any obstacles in the environment
def obstacle_collision(point,obstacle_list):
	global obstacleCoordinates
	#print("start")
	for a, b, c, d in obstacleCoordinates:
		#print(a, b, c, d)
		if point[0] >= (a) and point[0] <= (a + c) and point[1] >= (b) and point[1] <= (b + d):
			return True

	
	return False

# Recursively updating the costs of all the child nodes when a parent node gets rewired
def update_children(node):
	for child_node in node.children:
		child_node.cost = node.cost + math.sqrt((child_node.x-node.x)**2 + (child_node.y-node.y)**2)
		update_children(child_node)

# Finding the proximal parent node (as opposed to the closest node in RRT)
def find_proximal_node(new_node,node_list):
	proximal_node = None
	for node in node_list:
		dist = math.sqrt((new_node.x-node.x)**2 + (new_node.y-node.y)**2)
		if dist < REWIRING_RADIUS:
			if node.cost + dist < new_node.cost:
				proximal_node = node
				new_node.cost = node.cost + dist
				new_node.parent = proximal_node
	if proximal_node is None:
		return new_node, False
	else:
		proximal_node.children.append(new_node)
		return new_node, True

# Rewiring the nodes in the vicinity of the newly added node
def rewire_nodes(new_node,node_list):
	for node in node_list:
		dist = math.sqrt((new_node.x-node.x)**2 + (new_node.y-node.y)**2)
		if dist < REWIRING_RADIUS:
			if new_node.cost + dist < node.cost:
				node.cost = new_node.cost + dist
				if node in node.parent.children:
					pygame.draw.line(viz_window,WHITE,(new_node.x,new_node.y),(new_node.parent.x,new_node.parent.y))
					node.parent.children.remove(node)
				node.parent = new_node
				update_children(node)
				new_node.children.append(node)

# Adding a new node while making sure it doesn't collide with any obstacles
def add_new_node(viz_window,node_list,obstacle_list):
	while True:
		point = (random.random()*WINDOW_LENGTH,random.random()*WINDOW_BREADTH)
		nearest_node_dist = 1e7
		nearest_node = None
		for node in node_list:
			dist = math.sqrt((point[0]-node.x)**2 + (point[1]-node.y)**2)
			if dist < nearest_node_dist:
				nearest_node_dist = dist
				nearest_node = node
		theta = math.atan2(point[1]-nearest_node.y,point[0]-nearest_node.x)
		new_pos = (nearest_node.x+EPSILON*math.cos(theta),nearest_node.y+EPSILON*math.sin(theta))
		if not obstacle_collision(new_pos,obstacle_list):
			new_node = Node(new_pos,False,False)
			new_node, success = find_proximal_node(new_node,node_list)
			if not success:
				new_node.parent = nearest_node
				nearest_node.children.append(new_node)
			node_list.append(new_node)
			rewire_nodes(new_node,node_list)
			new_node.visualize_node(viz_window)
			pygame.draw.line(viz_window,BLUE,(new_node.x,new_node.y),(new_node.parent.x,new_node.parent.y))
			return new_node,node_list

# Checking if the latest node added falls within the goal circle (thus completing the search)
def target_reached(node,goal):
	if math.sqrt((node.x-goal.x)**2 + (node.y-goal.y)**2) < GOAL_RADIUS:
		return True
	return False

# Highliting the final RRT path from starting to target node

oldNodeList = []
nodeList = []
def display_final_path(viz_window,goal_node):
	current_node = goal_node
	global nodeList
	nodeList2 = []
	global oldNodeList
	global nodeNew
	nodeList = []
	nodeNew = []
	
	
	
	if (len(oldNodeList) >= 0):
		for i in range(0, len(oldNodeList) - 2):
			pygame.draw.line(viz_window,WHITE,(oldNodeList[i].x,oldNodeList[i].y),(oldNodeList[i+1].x, oldNodeList[i+1].y),width=5)
	pygame.display.update()
	
	while not current_node.start_node:
		
		adder = [current_node.x, current_node.y]

		
		nodeList.append(adder)
		nodeList2.append(current_node)
		pygame.draw.line(viz_window,GREEN,(current_node.x,current_node.y),(current_node.parent.x,current_node.parent.y),width=5)
		#undraw
		current_node = current_node.parent
	print(nodeList)
	for nodeTemp in nodeList:
		nodeNew[:0]=[nodeTemp]			

	
	
	
	oldNodeList = nodeList2

	pygame.display.update()




# The RRT* Algorithm
def rrt_algorithm(viz_window,start_node,goal_node,obstacle_list):
	distanceTheoretical = 0
	print("New RRT")
	node_list = []
	node_list.append(start_node)
	erase_list = []
	run = True
	while run:
		#print("Restart loop")
		new_node, node_list = add_new_node(viz_window,node_list,obstacle_list)
		if target_reached(new_node,goal_node):
			goal_node.parent = new_node
			pygame.draw.line(viz_window,BLUE,(new_node.x,new_node.y),(new_node.parent.x,new_node.parent.y))
			undraw = pygame.draw.line(viz_window,WHITE,(new_node.x,new_node.y),(new_node.parent.x,new_node.parent.y))
			node_list.append(goal_node)
			display_final_path(viz_window,goal_node)
			run = False

# Start of the simulation code
#Sets pygame up
pygame.display.set_caption('RRT* Path Finding Algorithm Visualization')
viz_window = pygame.display.set_mode((WINDOW_LENGTH,WINDOW_BREADTH))
viz_window.fill(WHITE)
#pygame.draw.rect(viz_window, BLUE, (12 * 25, 12 * 25, 25, 25))
pygame.display.update()

#Runs the algorithm
execute = True
start_pos, target_pos = None, None
start_node_found, target_node_found = False, False
start_node, target_node = None, None
obstacle_list = initialize_obstacles(viz_window)
while execute:
	
	for event in pygame.event.get():
	
		if event.type == pygame.QUIT:
			execute = False
		elif event.type == pygame.KEYDOWN and event.key == pygame.K_x:
			print("Hi")
			execute = False
			break
		
		#Initialize start and end nodes using set coordinates
		start_node = Node([37.5, 37.5], True, False)
		start_node.visualize_node(viz_window)
		start_node_found = True

		target_node = Node([287.5, 287.5],False,True)
		target_node.visualize_node(viz_window)
		target_node_found = True

		# Using the space button runs thirty iterations of the alogrithm

		if event.type == pygame.KEYDOWN:
			if event.key == pygame.K_SPACE and start_node_found and target_node_found:
				for i in range(30):
					print("hi")
					distanceTheoretical = 0
					rrt_algorithm(viz_window,start_node,target_node,obstacle_list)

				print("Ended loop")
			if event.key == pygame.K_c:
				start_node = None
				goal_node_node = None
#Prints all the nodes in the current shortest path
print("\n\n\n\n")
print(nodeNew)

#Calculates the theoretical distance of the path that the algorithm generated
for s in range(len(nodeNew) - 1):
	distanceTheoretical += math.sqrt((nodeNew[s + 1][0] - nodeNew[s][0])**2 + (nodeNew[s + 1][1] - nodeNew[s][1]) ** 2)

print("Total theoretical distance:")
print(distanceTheoretical)

#Create instance of tello drone and connect to it
drone = tello.Tello()
drone.connect()

print("Start Battery: ")
print(drone.get_battery())

drone.takeoff()

#While loop to give drone commands until the goal node is reached
foundGoal = False
i = 1
currentLocationX = 0
currentLocationY = 0
while not(foundGoal):
	
	
	#Sets current location to node that the drone is at
	currentLocationX = nodeNew[i - 1][1]
	currentLocationY = nodeNew[i - 1][0]

	#Sends the drone coordinates relative to the drones position through the difference of the future coordinates to the exsisiting ones
	drone.go_xyz_speed(-1 * round((nodeNew[i][1] - currentLocationX) * CONVERSION_FACTOR), -1 * round(((nodeNew[i][0] - currentLocationY) * CONVERSION_FACTOR)), 0, 35)
	
	#Checks if the next node is the goal node
	if i == len(nodeNew) - 1:
		foundGoal = True

	i = i + 1

drone.land()

print("End Battery: ")
print(drone.get_battery())