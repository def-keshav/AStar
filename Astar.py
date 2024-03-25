import numpy as np
from math import dist
import matplotlib.pyplot as plt
import time
import heapq
import os
import cv2
#Creating a Node class 
class Node:

    def __init__(self, x, y, angle, cost_to_come, ParentNode, cost_to_go = 0):
        self.x = x
        self.y = y
        self.angle = angle
        self.cost_to_come = cost_to_come
        self.ParentNode = ParentNode
        self.cost_to_go = cost_to_go 
            
    def __lt__(self,other):
        return self.cost_to_come + self.cost_to_go < other.cost_to_come + other.cost_to_go
    
#Generating the obstacle space     
def GraphGeneration(W, H, obstacle_clearance, ptrobot_rad):
    obstacleMap = np.full((H,W),0)
    
    for y in range(H) :
        for x in range(W):        
            if x<obstacle_clearance+ptrobot_rad or x>W-(obstacle_clearance+ptrobot_rad) or y<obstacle_clearance+ptrobot_rad or y>H-(obstacle_clearance+ptrobot_rad):
                obstacleMap[y,x] = 1 
            #Defining the obstacle on the right
            if x>900-obstacle_clearance+ptrobot_rad and x<1100+obstacle_clearance+ptrobot_rad and y>50-(obstacle_clearance+ptrobot_rad) and y<125+obstacle_clearance+ptrobot_rad:
                obstacleMap[y,x] = 1
            if x>1020-(obstacle_clearance+ptrobot_rad) and x<1100+obstacle_clearance+ptrobot_rad and y>125-(obstacle_clearance+ptrobot_rad) and y<375+obstacle_clearance+ptrobot_rad:
                obstacleMap[y,x] = 1      
            if x>900-(obstacle_clearance+ptrobot_rad) and x<1100+obstacle_clearance+ptrobot_rad and y>375-(obstacle_clearance+ptrobot_rad) and y<450+obstacle_clearance+ptrobot_rad:
                obstacleMap[y,x] = 1  
            #Defining rectangular obstacle
            if x>100-(obstacle_clearance+ptrobot_rad) and x<175+obstacle_clearance+ptrobot_rad and y>100-(obstacle_clearance+ptrobot_rad):
                obstacleMap[y,x] = 1
            if x>275-(obstacle_clearance+ptrobot_rad) and x<350+obstacle_clearance+ptrobot_rad and y<400+obstacle_clearance+ptrobot_rad:
                obstacleMap[y,x] = 1
            #defining the hexagonal obstacle
            if x<785+(obstacle_clearance+ptrobot_rad) and x>515-(obstacle_clearance+ptrobot_rad):
                if 1.73*y+x-(1342.25+2*(obstacle_clearance+ptrobot_rad))<0:
                    if 1.73*y+x-(823-2*(obstacle_clearance+ptrobot_rad))>0:
                        if 1.73*y-x-(42+2*(obstacle_clearance+ptrobot_rad))<0:
                            if 1.73*y-x+477.25+2*(obstacle_clearance+ptrobot_rad)>0:
                                obstacleMap[y,x] = 1              
    
    return obstacleMap

#defining the action moves
def ActionMove60CCW(x,y,angle,StepSize, cost_to_come):  #funtion to move 60 degrees counter clockwise
    angle = angle + 60
    x = x + (StepSize*np.cos(np.radians(angle)))
    y = y + (StepSize*np.sin(np.radians(angle)))
    x = round(x)
    y = round(y)
    cost_to_come = StepSize + cost_to_come
    return x,y,angle,cost_to_come

def ActionMove30CCW(x,y,angle, StepSize, cost_to_come): #function to move 30 degrees counter clockwise
    angle = angle + 60
    x = x + (StepSize*np.cos(np.radians(angle)))
    y = y + (StepSize*np.sin(np.radians(angle)))
    x = round(x)
    y = round(y)
    cost_to_come = StepSize + cost_to_come
    return x,y,angle, cost_to_come

def ActionMoveforward(x,y,angle, StepSize, cost_to_come): #function to move forward
    angle = angle + 0
    x = x + (StepSize*np.cos(np.radians(angle)))
    y = y + (StepSize*np.sin(np.radians(angle)))
    x = round(x)
    y = round(y)
    cost_to_come = StepSize + cost_to_come
    return x,y,angle, cost_to_come

def ActionMove30CW(x,y,angle, StepSize, cost_to_come): #function to move 30 degrees clockwise
    angle = angle - 30
    x = x + (StepSize*np.cos(np.radians(angle)))
    y = y + (StepSize*np.sin(np.radians(angle)))
    x = round(x)
    y = round(y)
    cost_to_come = StepSize + cost_to_come
    return x,y,angle, cost_to_come

def ActionMove60CW(x,y,angle, StepSize, cost_to_come): #function to move 60 degrees clockwise
    angle = angle - 60
    x = x + (StepSize*np.cos(np.radians(angle)))
    y = y + (StepSize*np.sin(np.radians(angle)))
    x = round(x)
    y = round(y)
    cost_to_come = StepSize + cost_to_come
    return x,y,angle,cost_to_come

#defining the action set with possible moves
def ActionSet(move,x,y,angle,StepSize,cost_to_come):

	if move == 'ActionMove60CCW':
		return ActionMove60CCW(x,y,angle, StepSize,cost_to_come)
	elif move == 'ActionMove30CCW':
		return ActionMove30CCW(x,y,angle, StepSize,cost_to_come)
	elif move == 'ActionMoveForward':
		return ActionMoveforward(x,y,angle,StepSize,cost_to_come)
	elif move == 'ActionMove30CW':
		return ActionMove30CW(x,y,angle,StepSize,cost_to_come)
	elif move == 'ActionMove60CW':
		return ActionMove60CW(x,y,angle,StepSize,cost_to_come)
	else:
		return None

#checking the validity of a move
def ValidMoveCheck(x, y, obstacleMapSpace):
	s = obstacleMapSpace.shape
	if( x > s[1] or x < 0 or y > s[0] or y < 0 ):
		return False	
	else:
		try:
			if(obstacleMapSpace[y][x] == 1  or obstacleMapSpace[y][x]==2):
				return False
		except:
			pass
	return True
#checking if the goal has been reached
def GoalCheck(present, goal):   
    ed = dist((present.x, present.y), (goal.x, goal.y))             
    if ed < 1.5 and (present.angle == goal.angle):
        return True
    else:
        return False
#validating the orientation
def validorientationcheck(angle):
    if((angle%30)==0):  #only multiples of 30 degrees as orientation are allowed
        return angle
    else:
        return False
#generating Unique Ids for the nodes    
def key(node):
    key = 1022*node.x + 111*node.y 
    return key
#defining the A* algorithm
def A_Str(start,goal,obstacleMapSpace,StepSize):                       
#checking if the goal and start noad are the same
    if GoalCheck(start, goal):
        return None,1
    node_g = goal        #intializing goal node
    node_s = start      #initializing start node
    #defining moves available
    moves = ['ActionMove60CCW','ActionMove30CCW', 'ActionMoveForward', 'ActionMove30CW', 'ActionMove60CW']   
    nodes_open = {}     #all open nodes
    #key generation for each node
    key_s = key(node_s)
    nodes_open[(key_s)] = node_s
    nodes_closed = {}       #all nodes_closed noded
    P_queue = [] 
    heapq.heappush(P_queue, [node_s.cost_to_come, node_s])
    total_nodes = [] #for visualisation
    

    while (len(P_queue) != 0):

        node_current = (heapq.heappop(P_queue))[1] #popping the node with lowest cost
        total_nodes.append([node_current.x, node_current.y, node_current.angle])          
        id_current = key(node_current)
        if GoalCheck(node_current, node_g):         #checking if goal node has been found
            node_g.ParentNode = node_current.ParentNode
            node_g.cost_to_come = node_current.cost_to_come
            print("Goal Reached!")
            return total_nodes,1
        #checking if present node is a new node
        if id_current in nodes_closed:  
            continue
        else:
            nodes_closed[id_current] = node_current
        del nodes_open[id_current]
        #finding all the possible moves
        for i in moves:  
            x,y,angle,cost_to_come = ActionSet(i,node_current.x,node_current.y,node_current.angle, StepSize, node_current.cost_to_come) 
            cost_to_go = dist((x, y), (goal.x, goal.y))  
            #creating a new node 
            new_node = Node(x,y,angle, cost_to_come,node_current, cost_to_go)   
            #Generating a key for the new node
            new_node_id = key(new_node) 
            # checking move validity
            if not ValidMoveCheck(new_node.x, new_node.y, obstacleMapSpace):
                continue
            elif new_node_id in nodes_closed:
                continue
            #checking to see if it is a new node
            if new_node_id in nodes_open:
                if new_node.cost_to_come < nodes_open[new_node_id].cost_to_come: 
                    nodes_open[new_node_id].cost_to_come = new_node.cost_to_come
                    nodes_open[new_node_id].ParentNode = new_node.ParentNode
            else:
                nodes_open[new_node_id] = new_node
            heapq.heappush(P_queue, [(new_node.cost_to_come + new_node.cost_to_go), new_node]) 
   
    return  total_nodes,0

#defining the backtracking funtion
def Backtracking(node_g):  
    X_track = []
    Y_track = []
    X_track.append(node_g.x)
    Y_track.append(node_g.y)
    node_parent = node_g.ParentNode
    while node_parent != -1:
        X_track.append(node_parent.x)
        Y_track.append(node_parent.y)
        node_parent = node_parent.ParentNode        
    X_track.reverse()
    Y_track.reverse()   
    x = np.asarray(X_track)
    y = np.asanyarray(Y_track)   
    return x,y
#plotting the path
def plot(node_s, node_g, X_track, Y_track, all_nodes, obstacleMapSpace, f_count, final_path):
    plt.figure(figsize=(10,5))
    plt.plot(node_s.x, node_s.y, "Dw")
    plt.plot(node_g.x, node_g.y, "Dg")
    plt.imshow(obstacleMapSpace, "GnBu")  #plotting the obstacle space
    ax = plt.gca()
    ax.invert_yaxis() 
    for i in range(len(all_nodes)): #plotting the visited nodes
        plt.plot(all_nodes[i][0], all_nodes[i][1], "2g-")
    if final_path: #plotting the path
        plt.plot(X_track[:f_count+1], Y_track[:f_count+1], ':r')        
    plt.savefig(f"f_{f_count}.png")
    plt.close()

#creating an animated video from the frames
def AnimateVideo(frame_prefix, output_video_path, frame_rate):
    f_list = []
    frames = [f for f in os.listdir() if f.startswith(frame_prefix) and f.endswith('.png')]
    frames.sort(key=lambda x: int(x.split('_')[1].split('.')[0]))
    for frame in frames:
        f_list.append(cv2.imread(frame))
        os.remove(frame)
    vid = cv2.VideoWriter_fourcc(*'mp4v')
    video = cv2.VideoWriter(output_video_path, vid, frame_rate, (f_list[0].shape[1], f_list[0].shape[0]))
    for frame in f_list:
        video.write(frame)
    video.release()
    cv2.destroyAllWindows()



#providing the clearance and radius
obstacle_clearance =  int(input("provide the clearance for the obstacles: "))
ptrobot_rad = int(input("provide the radius of the point robot: "))
#providing the step size of the robot
step_size = input("provide step size for the point robot: ")
step_size = int(step_size)
W = 1200
H = 500
obstacleMapSpace = GraphGeneration(W, H, obstacle_clearance, ptrobot_rad)
cost_to_go = 0
#initial coordinates and angle at the start 
initial_coordinates = input("provide initial x and y coordinates of the point robot:")
initial_x, initial_y = initial_coordinates.split()
initial_x = int(initial_x)
initial_y = int(initial_y)    
initial_angle = input("Enter the initial angle/orientation of the point robot: ")
angle_initial = int(initial_angle)
#checking move validity
if not ValidMoveCheck(initial_x, initial_y, obstacleMapSpace):
    print("start node is not valid, kindly enter a different value")
    exit(-1)
#checking orientation validity    
if not validorientationcheck(angle_initial):
    print("kindly enter an orientation which is a multiple of 30")
    exit(-1)
#providing the coordinates and angle for the goal        
final_coordinates = input("Provide the x and y coordinates of the Goal: ")
final_x, final_y = final_coordinates.split()
final_x = int(final_x)
final_y = int(final_y)    
final_angle = input("provide the Orientation for the goal: ")
angle_final = int(final_angle)
#checking validity 
if not ValidMoveCheck(final_x, final_y, obstacleMapSpace):
    print("Goal node can not be reached, kindly enter a different value")
    exit(-1)
#checking validity of the orientation    
if not validorientationcheck(angle_final):
    print("kindly enter an orientation which is a multiple of 30")
    exit(-1)

start_time = time.time()
node_s = Node(initial_x, initial_y,angle_initial, 0.0, -1,cost_to_go)
node_g = Node(final_x, final_y,angle_final, 0.0, -1, cost_to_go)
all_nodes,flag = A_Str(node_s, node_g, obstacleMapSpace, step_size)
#if goal has been reached, backtracking from initial to goal position
if (flag)==1:
    X_track,Y_track = Backtracking(node_g)
else:
    print("Could not find Path, kindly enter a different goal")
f_count = 0  
for i in range(len(all_nodes)):
    plot(node_s, node_g, X_track, Y_track, all_nodes[:i+1], obstacleMapSpace, f_count, final_path=False)
    f_count += 1

for i in range(len(X_track)):
    plot(node_s, node_g, X_track, Y_track, all_nodes, obstacleMapSpace, f_count, final_path=True)
    f_count += 1
AnimateVideo("f", "PathAnimationvideo.mp4", 30) 
end_time = time.time()
time_taken = end_time - start_time
print("Time taken:  ", time_taken)