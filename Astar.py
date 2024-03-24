import numpy as np
from math import dist
import matplotlib.pyplot as plt
import time
import heapq
import os
import cv2

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
    
def GraphGeneration(width, height, obstacle_clearance, ptrobot_rad):

    obstacleMap = np.full((height,width),0)
    
    for y in range(height) :
        for x in range(width):        
            if x<obstacle_clearance+ptrobot_rad or x>width-(obstacle_clearance+ptrobot_rad) or y<obstacle_clearance+ptrobot_rad or y>height-(obstacle_clearance+ptrobot_rad):
                obstacleMap[y,x] = 1 

            if x>900-obstacle_clearance+ptrobot_rad and x<1100+obstacle_clearance+ptrobot_rad and y>50-(obstacle_clearance+ptrobot_rad) and y<125+obstacle_clearance+ptrobot_rad:
                obstacleMap[y,x] = 1
            if x>1020-(obstacle_clearance+ptrobot_rad) and x<1100+obstacle_clearance+ptrobot_rad and y>125-(obstacle_clearance+ptrobot_rad) and y<375+obstacle_clearance+ptrobot_rad:
                obstacleMap[y,x] = 1      
            if x>900-(obstacle_clearance+ptrobot_rad) and x<1100+obstacle_clearance+ptrobot_rad and y>375-(obstacle_clearance+ptrobot_rad) and y<450+obstacle_clearance+ptrobot_rad:
                obstacleMap[y,x] = 1  

            if x>100-(obstacle_clearance+ptrobot_rad) and x<175+obstacle_clearance+ptrobot_rad and y>100-(obstacle_clearance+ptrobot_rad):
                obstacleMap[y,x] = 1
            if x>275-(obstacle_clearance+ptrobot_rad) and x<350+obstacle_clearance+ptrobot_rad and y<400+obstacle_clearance+ptrobot_rad:
                obstacleMap[y,x] = 1

            if x<785+(obstacle_clearance+ptrobot_rad) and x>515-(obstacle_clearance+ptrobot_rad):
                if 1.73*y+x-(1342.25+2*(obstacle_clearance+ptrobot_rad))<0:
                    if 1.73*y+x-(823-2*(obstacle_clearance+ptrobot_rad))>0:
                        if 1.73*y-x-(42+2*(obstacle_clearance+ptrobot_rad))<0:
                            if 1.73*y-x+477.25+2*(obstacle_clearance+ptrobot_rad)>0:
                                obstacleMap[y,x] = 1              
    
    return obstacleMap


def ActionMove60CCW(x,y,angle,StepSize, cost_to_come):
    angle = angle + 60
    x = x + (StepSize*np.cos(np.radians(angle)))
    y = y + (StepSize*np.sin(np.radians(angle)))
    x = round(x)
    y = round(y)
    cost_to_come = 1 + cost_to_come
    return x,y,angle,cost_to_come

def ActionMove30CCW(x,y,angle, StepSize, cost_to_come):
    angle = angle + 60
    x = x + (StepSize*np.cos(np.radians(angle)))
    y = y + (StepSize*np.sin(np.radians(angle)))
    x = round(x)
    y = round(y)
    cost_to_come = 1 + cost_to_come
    return x,y,angle, cost_to_come

def ActionMoveforward(x,y,angle, StepSize, cost_to_come):
    angle = angle + 0
    x = x + (StepSize*np.cos(np.radians(angle)))
    y = y + (StepSize*np.sin(np.radians(angle)))
    x = round(x)
    y = round(y)
    cost_to_come = 1 + cost_to_come
    return x,y,angle, cost_to_come

def ActionMove30CW(x,y,angle, StepSize, cost_to_come):
    angle = angle - 30
    x = x + (StepSize*np.cos(np.radians(angle)))
    y = y + (StepSize*np.sin(np.radians(angle)))
    x = round(x)
    y = round(y)
    cost_to_come = 1 + cost_to_come
    return x,y,angle, cost_to_come

def ActionMove60CW(x,y,angle, StepSize, cost_to_come):
    angle = angle - 60
    x = x + (StepSize*np.cos(np.radians(angle)))
    y = y + (StepSize*np.sin(np.radians(angle)))
    x = round(x)
    y = round(y)
    cost_to_come = 1 + cost_to_come
    return x,y,angle,cost_to_come


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

def GoalCheck(present, goal):
    
    ed = dist((present.x, present.y), (goal.x, goal.y))             

    if ed < 1.5:
        return True
    else:
        return False

def validorientationcheck(angle):
    if((angle%30)==0):
        return angle
    else:
        return False
    
def key(node):
    key = 1022*node.x + 111*node.y 
    return key

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
    nodes_total = [] #for visualisation
    

    while (len(P_queue) != 0):

        node_current = (heapq.heappop(P_queue))[1] #popping the node with lowest cost
        nodes_total.append([node_current.x, node_current.y, node_current.angle])          
        id_current = key(node_current)
        if GoalCheck(node_current, node_g):         #checking if goal node has been found
            node_g.ParentNode = node_current.ParentNode
            node_g.cost_to_come = node_current.cost_to_come
            print("Goal Node found")
            return nodes_total,1
        #checking if present node is a new node
        if id_current in nodes_closed:  
            continue
        else:
            nodes_closed[id_current] = node_current
        del nodes_open[id_current]
        #finding all the possible moves
        for move in moves:  
            x,y,angle,cost_to_come = ActionSet(move,node_current.x,node_current.y,node_current.angle, StepSize, node_current.cost_to_come) 
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
   
            if new_node_id in nodes_open:
                if new_node.cost_to_come < nodes_open[new_node_id].cost_to_come: 
                    nodes_open[new_node_id].cost_to_come = new_node.cost_to_come
                    nodes_open[new_node_id].ParentNode = new_node.ParentNode
            else:
                nodes_open[new_node_id] = new_node
   			
            heapq.heappush(P_queue, [(new_node.cost_to_come + new_node.cost_to_go), new_node]) 
   
    return  nodes_total,0

def Backtracking(node_g):  
    x_path = []
    y_path = []
    x_path.append(node_g.x)
    y_path.append(node_g.y)

    parent_node = node_g.ParentNode
    while parent_node != -1:
        x_path.append(parent_node.x)
        y_path.append(parent_node.y)
        parent_node = parent_node.ParentNode
        
    x_path.reverse()
    y_path.reverse()
    
    x = np.asarray(x_path)
    y = np.asanyarray(y_path)
    
    return x,y

def plot(node_s, node_g, x_path, y_path, all_nodes, obstacleMapSpace, frame_count, final_path):
    plt.figure()

    plt.plot(node_s.x, node_s.y, "Dw")
    plt.plot(node_g.x, node_g.y, "Dg")

    plt.imshow(obstacleMapSpace, "GnBu")
    ax = plt.gca()
    ax.invert_yaxis() 
    
    for i in range(len(all_nodes)):
        plt.plot(all_nodes[i][0], all_nodes[i][1], "2g-")

    if final_path:
        plt.plot(x_path[:frame_count+1], y_path[:frame_count+1], ':r')
        
    plt.savefig(f"frame_{frame_count}.png")
    plt.close()

def create_video(frame_prefix, output_video_path, frame_rate):
    frames = []
    frame_files = [f for f in os.listdir() if f.startswith(frame_prefix) and f.endswith('.png')]
    frame_files.sort(key=lambda x: int(x.split('_')[1].split('.')[0]))
    for frame_file in frame_files:
        frames.append(cv2.imread(frame_file))
        os.remove(frame_file)

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video_writer = cv2.VideoWriter(output_video_path, fourcc, frame_rate, (frames[0].shape[1], frames[0].shape[0]))

    for frame in frames:
        video_writer.write(frame)

    video_writer.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    

    obstacle_clearance = input("Assign Clearance to the Obstacles: ")
    obstacle_clearance = int(obstacle_clearance)
    
    ptrobot_rad = input("Enter the Radius of the Robot: ") 
    ptrobot_rad = int(ptrobot_rad)
    
    robot_step_size = input("Enter Step size of the Robot: ")
    robot_step_size = int(robot_step_size)
    
    width = 1200
    height = 500
    obstacleMapSpace = GraphGeneration(width, height, obstacle_clearance, ptrobot_rad)
    c2g = 0
    
    start_coordinates = input("Enter coordinates for Start Node: ")
    s_x, s_y = start_coordinates.split()
    s_x = int(s_x)
    s_y = int(s_y)
    
    s_theta = input("Enter Orientation of the robot at start node: ")
    s_t = int(s_theta)
    
    if not ValidMoveCheck(s_x, s_y, obstacleMapSpace):
        print("Start node is out of bounds")
        exit(-1)
        
    if not validorientationcheck(s_t):
        print("Orientation has to be a multiple of 30")
        exit(-1)
            
    goal_coordinates = input("Enter coordinates for Goal Node: ")
    g_x, g_y = goal_coordinates.split()
    g_x = int(g_x)
    g_y = int(g_y)
    
    g_theta = input("Enter Orientation of the robot at goal node: ")
    g_t = int(g_theta)
    
    if not ValidMoveCheck(g_x, g_y, obstacleMapSpace):
        print("Goal node is out of bounds")
        exit(-1)
        
    if not validorientationcheck(g_t):
        print("Orientation has to be a multiple of 30")
        exit(-1)

    timer_start = time.time()
    node_s = Node(s_x, s_y,s_t, 0.0, -1,c2g)
    node_g = Node(g_x, g_y,g_t, 0.0, -1, c2g)
    all_nodes,flag = A_Str(node_s, node_g, obstacleMapSpace, robot_step_size)
    

    if (flag)==1:
        x_path,y_path = Backtracking(node_g)
    else:
        print("No path was found")
        
 
    frame_count = 0  
    for i in range(len(all_nodes)):
        plot(node_s, node_g, x_path, y_path, all_nodes[:i+1], obstacleMapSpace, frame_count, final_path=False)
        frame_count += 1

    for i in range(len(x_path)):
        plot(node_s, node_g, x_path, y_path, all_nodes, obstacleMapSpace, frame_count, final_path=True)
        frame_count += 1

    create_video("frame", "output_video.mp4", 30) 


    timer_stop = time.time()
    C_time = timer_stop - timer_start
    print("The Total Runtime is:  ", C_time)