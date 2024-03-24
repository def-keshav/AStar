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
    
def GraphGeneration(width, height, obs_clearance, robot_radius):

    pass

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


def ValidMoveCheck(x, y, obs_space):

	e = obs_space.shape

	if( x > e[1] or x < 0 or y > e[0] or y < 0 ):
		return False
	
	else:
		try:
			if(obs_space[y][x] == 1  or obs_space[y][x]==2):
				return False
		except:
			pass
	return True

def Check_goal(present, goal):
    
    dt = dist((present.x, present.y), (goal.x, goal.y))             

    if dt < 1.5:
        return True
    else:
        return False

def validorient(angle):
    if((angle%30)==0):
        return angle
    else:
        return False
    
def key(node):
    key = 1022*node.x + 111*node.y 
    return key

def A_Str(start,goal,obs_space,StepSize):                       
#checking if the goal and start noad are the same
    if Check_goal(start, goal):
        return None,1
    goal_node = goal        #intializing goal node
    start_node = start      #initializing start node
    #defining moves available
    moves = ['ActionMove60CCW','ActionMove30CCW', 'ActionMoveForward', 'ActionMove30CW', 'ActionMove60CW']   
    OpenNodes = {}     #all open nodes
    #key generation for each node
    start_key = key(start_node)
    OpenNodes[(start_key)] = start_node
    Closed = {}       #all closed noded
    priority_list = [] 
    heapq.heappush(priority_list, [start_node.cost_to_come, start_node])
    
    TotalNodes = [] #for visualisation
    

    while (len(priority_list) != 0):

        present_node = (heapq.heappop(priority_list))[1] #popping the node with lowest cost
        TotalNodes.append([present_node.x, present_node.y, present_node.angle])          
        present_id = key(present_node)
        if Check_goal(present_node, goal_node):         #checking if goal node has been found
            goal_node.ParentNode = present_node.ParentNode
            goal_node.cost_to_come = present_node.cost_to_come
            print("Goal Node found")
            return TotalNodes,1
        #checking if present node is a new node
        if present_id in Closed:  
            continue
        else:
            Closed[present_id] = present_node
        del OpenNodes[present_id]
        #finding all the possible moves
        for move in moves:  
            x,y,angle,cost_to_come = ActionSet(move,present_node.x,present_node.y,present_node.angle, StepSize, present_node.cost_to_come) 
            cost_to_go = dist((x, y), (goal.x, goal.y))  
            #creating a new node 
            new_node = Node(x,y,angle, cost_to_come,present_node, cost_to_go)   
            #Generating a key for the new node
            new_node_id = key(new_node) 
            # checking move validity
            if not ValidMoveCheck(new_node.x, new_node.y, obs_space):
                continue
            elif new_node_id in Closed:
                continue
   
            if new_node_id in OpenNodes:
                if new_node.cost_to_come < OpenNodes[new_node_id].cost_to_come: 
                    OpenNodes[new_node_id].cost_to_come = new_node.cost_to_come
                    OpenNodes[new_node_id].ParentNode = new_node.ParentNode
            else:
                OpenNodes[new_node_id] = new_node
   			
            heapq.heappush(priority_list, [(new_node.cost_to_come + new_node.cost_to_go), new_node]) 
   
    return  TotalNodes,0

def Backtracking(goal_node):  
    x_path = []
    y_path = []
    x_path.append(goal_node.x)
    y_path.append(goal_node.y)

    parent_node = goal_node.ParentNode
    while parent_node != -1:
        x_path.append(parent_node.x)
        y_path.append(parent_node.y)
        parent_node = parent_node.ParentNode
        
    x_path.reverse()
    y_path.reverse()
    
    x = np.asarray(x_path)
    y = np.asanyarray(y_path)
    
    return x,y

def plot(start_node, goal_node, x_path, y_path, all_nodes, obs_space, frame_count, final_path):
    plt.figure()
    ### Start node and Goal node ###
    plt.plot(start_node.x, start_node.y, "Dw")
    plt.plot(goal_node.x, goal_node.y, "Dg")

    ### Configuration Space for Obstacles ####
    plt.imshow(obs_space, "GnBu")
    ax = plt.gca()
    ax.invert_yaxis() #y-axis inversion
    
    ### All visited nodes ###
    for i in range(len(all_nodes)):
        plt.plot(all_nodes[i][0], all_nodes[i][1], "2g-")
    
    # plt.plot(all_nodes[len(all_nodes)-1][0], all_nodes[len(all_nodes)-1][1], "2g-")

    # plt.plot(x_path, y_path, ':r')

    # # Plot final path iteration-wise
    if final_path:
        plt.plot(x_path[:frame_count+1], y_path[:frame_count+1], ':r')
        

    # Plot final path iteration-wise
    # plt.plot(x_path[:frame_count+1], y_path[:frame_count+1], ':r')

    # Save each frame as an image
    plt.savefig(f"frame_{frame_count}.png")
    plt.close()

# Function to create a video from saved frames
def create_video(frame_prefix, output_video_path, frame_rate):
    frames = []
    frame_files = [f for f in os.listdir() if f.startswith(frame_prefix) and f.endswith('.png')]
    frame_files.sort(key=lambda x: int(x.split('_')[1].split('.')[0]))  # Sort files by frame number
    for frame_file in frame_files:
        frames.append(cv2.imread(frame_file))
        os.remove(frame_file)  # Remove the frame image after adding it to the video

    # Determine the video codec and create the video writer
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video_writer = cv2.VideoWriter(output_video_path, fourcc, frame_rate, (frames[0].shape[1], frames[0].shape[0]))

    # Write frames to video
    for frame in frames:
        video_writer.write(frame)

    # Release the video writer and destroy any remaining OpenCV windows
    video_writer.release()
    cv2.destroyAllWindows()


######### CALLING ALL MY FUNCTIONS TO IMPLEMENT A STAR ALGORITHM ON A POINT ROBOT ###########

if __name__ == '__main__':
    
    #### Clearance of the Obstacle ####
    obs_clearance = input("Assign Clearance to the Obstacles: ")
    obs_clearance = int(obs_clearance)
    
    #### Radius of the Robot ####
    robot_radius = input("Enter the Radius of the Robot: ") 
    robot_radius = int(robot_radius)
    
    #### Step Size of the Robot ####
    robot_step_size = input("Enter Step size of the Robot: ")
    robot_step_size = int(robot_step_size)
    
    width = 1200
    height = 500
    obs_space = GraphGeneration(width, height, obs_clearance, robot_radius)
    c2g = 0
    
    #### Taking start node coordinates as input from user #####
    start_coordinates = input("Enter coordinates for Start Node: ")
    s_x, s_y = start_coordinates.split()
    s_x = int(s_x)
    s_y = int(s_y)
    
    #### Taking Orientation for the robot ####
    s_theta = input("Enter Orientation of the robot at start node: ")
    s_t = int(s_theta)
    
    ### Checking if the user input is valid #####
    if not ValidMoveCheck(s_x, s_y, obs_space):
        print("Start node is out of bounds")
        exit(-1)
        
    if not validorient(s_t):
        print("Orientation has to be a multiple of 30")
        exit(-1)
            
    ##### Taking Goal node coordinates as input from user ##### 
    goal_coordinates = input("Enter coordinates for Goal Node: ")
    g_x, g_y = goal_coordinates.split()
    g_x = int(g_x)
    g_y = int(g_y)
    
    #### Taking Orientation for the robot ####
    g_theta = input("Enter Orientation of the robot at goal node: ")
    g_t = int(g_theta)
    
    ### Checking if the user input is valid #####
    if not ValidMoveCheck(g_x, g_y, obs_space):
        print("Goal node is out of bounds")
        exit(-1)
        
    if not validorient(g_t):
        print("Orientation has to be a multiple of 30")
        exit(-1)

    ### Timer to calculate computational  time ###
    timer_start = time.time()
    
    ##### Creating start_node and goal_node objects 
    start_node = Node(s_x, s_y,s_t, 0.0, -1,c2g)
    goal_node = Node(g_x, g_y,g_t, 0.0, -1, c2g)
    all_nodes,flag = A_Str(start_node, goal_node, obs_space, robot_step_size)
    
    ##### Plot shortest path only when goal node is reached #####
    if (flag)==1:
        x_path,y_path = Backtracking(goal_node)
    else:
        print("No path was found")
        
    # Calling create_video function after plotting
    frame_count = 0  # Initialize frame count
    for i in range(len(all_nodes)):
        plot(start_node, goal_node, x_path, y_path, all_nodes[:i+1], obs_space, frame_count, final_path=False)
        frame_count += 1

    for i in range(len(x_path)):
        plot(start_node, goal_node, x_path, y_path, all_nodes, obs_space, frame_count, final_path=True)
        frame_count += 1

    # Create video from saved frames
    create_video("frame", "output_video.mp4", 30)  # Adjust the frame rate as needed


    timer_stop = time.time()
    C_time = timer_stop - timer_start
    print("The Total Runtime is:  ", C_time)