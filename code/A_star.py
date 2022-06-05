#Importing necessary libraries
import numpy as np
import cv2 
from queue import PriorityQueue
from math import cos, radians, sin, dist
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas

#Function that checks if the node is in the obstacle space of any of the three obstacles
def in_obstacle_space(node):
    '''
    Input:
    node    : tuple(x, y, theta)

    Output:
    True if node is in the obstacle space
    False, if node is not in the obstacle space
    '''
    obstacle1 = False
    obstacle2 = False
    obstacle3 = False
    
    x,y, theta = node
    
    if ((y<=(0.32*x+173.2)) and (y>=(-1.23*x+229.28))):
        if (y<=(-3.2*x+436)):
            obstacle1 = True
        elif (y>=(0.86*node[0]+111.1)):
            obstacle1 = True 
      
    if ((x>=165) and (y>=(-0.58*x+175.58)) and (y>=(0.58*x-56.42)) and (x<=235) and (y<=(-0.58*node[0]+256.51)) and (y<=(0.58*node[0]+24.42))):
        obstacle2 = True
    
    if (((x-300)**2 + (y-185)**2) <= (40**2)):
        obstacle3 = True
        
    if obstacle1 or obstacle2 or obstacle3:
        return True
    else:
        return False

#Function that checks if the node is in the clearance space of any of the three obstacles
def in_clearance_space(node):
    '''
    Input:
    node    : tuple(x, y, theta)

    Output:
    True if node is in the clearance space
    False, if node is not in the clearance space
    '''
    clearance1 = False
    clearance2 = False
    clearance3 = False
    clearance4 = False
    
    x,y, theta = node
    
    if ((y<=(0.32*x+178.85)) and (y>=(-1.23*x+221.43))):
        if (y<=(-3.2*x+452.76)):
            clearance1 = True
        elif (y>=(0.86*node[0]+104.87)):
            clearance1 = True 
      
    if ((x>=160) and (y>=(-0.58*x+169.8)) and (y>=(0.58*x-62.29)) and (x<=240) and (y<=(-0.58*node[0]+262.2)) and (y<=(0.58*node[0]+30.2))):
        clearance2 = True
    
    if (((x-300)**2 + (y-185)**2) <= (45**2)):
        clearance3 = True
        
    if node[0]<=5 or node[0]>395 or node[1]<=5 or node[1]>245:
        clearance4 = True
        
    if clearance1 or clearance2 or clearance3 or clearance4:
        return True
    else:
        return False

#Function to move the node straight
def ActionMoveZero(node, node_info, step_size):
    '''
    Input:
    node      : tuple(x, y, theta)
    node_info : list[parent_node, cost_to_come]
    
    Output:
    child_node, child_node_info
    '''
    x, y, theta = node
    child_node = (x+step_size*cos(radians(theta)), y+step_size*sin(radians(theta)), theta)
    #Rounding off the node to nearest 0.5 decimal value
    child_node = (round(child_node[0]*2)/2, round(child_node[1]*2)/2, child_node[2])
    child_node_info = [node, (node_info[1] + 1)]
    return child_node, child_node_info

#Function to move the node 30 degrees in anticlockwise direction
def ActionMoveUpThirty(node, node_info, step_size):
    '''
    Input:
    node      : tuple(x, y, theta)
    node_info : list[parent_node, cost_to_come]
    
    Output:
    child_node, child_node_info
    '''
    x, y, theta = node
    child_node = (x+step_size*cos(radians(theta+30)), y+step_size*sin(radians(theta+30)), (theta+30)%360)
    #Rounding off the node to nearest 0.5 decimal value
    child_node = (round(child_node[0]*2)/2, round(child_node[1]*2)/2, child_node[2])
    child_node_info = [node, (node_info[1] + 1)]
    return child_node, child_node_info

#Function to move the node 60 degrees in anticlockwise direction
def ActionMoveUpSixty(node, node_info, step_size):
    '''
    Input:
    node      : tuple(x, y, theta)
    node_info : list[parent_node, cost_to_come]
    
    Output:
    child_node, child_node_info
    '''
    x, y, theta = node
    child_node = (x+step_size*cos(radians(theta+60)), y+step_size*sin(radians(theta+60)), (theta+60)%360)
    #Rounding off the node to nearest 0.5 decimal value
    child_node = (round(child_node[0]*2)/2, round(child_node[1]*2)/2, child_node[2])
    child_node_info = [node, (node_info[1] + 1)]
    return child_node, child_node_info

#Function to move the node 30 degrees in clockwise direction
def ActionMoveDownThirty(node, node_info, step_size):
    '''
    Input:
    node      : tuple(x, y, theta)
    node_info : list[parent_node, cost_to_come]
    
    Output:
    child_node, child_node_info
    '''
    x, y, theta = node
    child_node = (x+step_size*cos(radians(theta-30)), y+step_size*sin(radians(theta-30)), (theta-30)%360)
    #Rounding off the node to nearest 0.5 decimal value
    child_node = (round(child_node[0]*2)/2, round(child_node[1]*2)/2, child_node[2])
    child_node_info = [node, (node_info[1] + 1)]
    return child_node, child_node_info

#Function to move the node 60 degrees in clockwise direction
def ActionMoveDownSixty(node, node_info, step_size):
    '''
    Input:
    node      : tuple(x, y, theta)
    node_info : list[parent_node, cost_to_come]
    
    Output:
    child_node, child_node_info
    '''
    x, y, theta = node
    child_node = (x+step_size*cos(radians(theta-60)), y+step_size*sin(radians(theta-60)), (theta-60)%360)
    #Rounding off the node to nearest 0.5 decimal value
    child_node = (round(child_node[0]*2)/2, round(child_node[1]*2)/2, child_node[2])
    child_node_info = [node, (node_info[1] + 1)]
    return child_node, child_node_info

def costToGo(node, goal_node):
    return dist(node[:-1], goal_node[:-1])

def totalCost(c2c, c2g):
    return c2c + c2g

def isGoalReached(node, goal_node):
    if abs(node[2]-(goal_node[2])%360)<=30 and dist(node[:-1], goal_node[:-1])<=1.5:
        return True
    else:
        return False

#Function that checks if the node is visited
def isNodeVisited(node, visited_node_matrix):
    if visited_node_matrix[round(node[0]*2)-1][round(node[1]*2)-1][int((node[2]%360)/30)] != 1:
        visited_node_matrix[round(node[0]*2)-1][round(node[1]*2)-1][int((node[2]%360)/30)] = 1
        return False, visited_node_matrix
    else:
        return True, visited_node_matrix

#Function that performs backtracking to find a path from the start node to the goal node
def backtracking(closed_list):
    '''
    Input:
    closed_list : dictionary of all closed nodes
    
    Output:
    path : list[nodes from goal node to start node]
    '''
    path = []
    goal = list(closed_list)[-1]
    path.append(goal)
    
    while True:
        parent = closed_list[goal]
        
        if parent == None:
            break 
            
        path.append(parent)
        goal = parent
    return path

# Function that visualizes the output of A* algorithm
def visualization(visited_node, optimal_path):
    '''
    Input:
    visited_node : list of visited nodes along with their
    '''
    img_map = np.zeros((250, 400, 3), dtype = np.uint8)
    for y in range(img_map.shape[0]):
        for x in range(img_map.shape[1]):
            node = (x, y, 0)
            if in_obstacle_space(node):
                img_map[y-1][x-1] = [255, 0, 0]
    
    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot()
    ax.imshow(img_map)
    plt.gca().invert_yaxis()
    fig.canvas.draw()
    fig.canvas.flush_events()
    
    for i in visited_node:
        parent_node = i[0]
        child_node =i[1]
        ax.plot((parent_node[0],child_node[0]),(parent_node[1],child_node[1]),'w')
        fig.canvas.draw()
        fig.canvas.flush_events()
        
    for i in range(len(optimal_path)-1):
        parent_node = optimal_path[-1-i]
        child_node =optimal_path[-2-i]
        ax.plot((parent_node[0],child_node[0]),(parent_node[1],child_node[1]),'g')
        fig.canvas.draw()
        fig.canvas.flush_events()
        
    fig.savefig('path.png')
# main function
if __name__ == '__main__':
    
    while True:
        initial_node = tuple(map(int, input("Enter initial node's x, y, and theta separated by comma: ").split(",")))
        if in_clearance_space(initial_node):
            print("\nThe start node is in the obstacle space. Please try again.\n")
        else:
            print("\nStart node accepted\n")
            break
    while True:
        goal_node = tuple(map(int, input("Enter goal node's x, y, and theta separated by comma: ").split(",")))
        if in_clearance_space(goal_node):
            print("\nThe goal node is in the obstacle space. Please try again.\n")
        else:
            print("\nGoal node accepted\n")
            break
    while True:
        step_size = int(input("Enter step size between 1 and 10: "))
        if 1<=step_size<=10:
            print("\nStep size accepted\n")
            break
        else:
            print("\nStep size should be between 1 and 10. Please try again.\n")
    print("Searching for goal node...\n")        
    open_list = PriorityQueue()
    c2c = 0
    c2g = costToGo(initial_node, goal_node)
    open_list.put((totalCost(c2c, c2g), [initial_node, None, 0, 0]))
    
    V = np.empty([800,500,12])
    
    closed_list = {}
   
    node_index = 1
    
    visited_node = []
    
    while True:
        nodeWithLowestCost = open_list.get()
        node = nodeWithLowestCost[1][0]
        node_info = nodeWithLowestCost[1][1:]

        if node not in closed_list:
            closed_list[node] = node_info[0]
        
            if isGoalReached(node, goal_node):
                print("Reached goal node\n")
                break
            
            new_node, new_node_info = ActionMoveZero(node, node_info, step_size)
            if not in_clearance_space(new_node):
                c2g = costToGo(new_node, goal_node)
                open_list.put((totalCost(new_node_info[1], c2g), [new_node, node, new_node_info[1]]))
                isFalse, V = isNodeVisited(new_node, V)
                if not isFalse:
                    visited_node.append([node, new_node])

            new_node, new_node_info = ActionMoveUpThirty(node, node_info, step_size)
            if not in_clearance_space(new_node):
                c2g = costToGo(new_node, goal_node)
                open_list.put((totalCost(new_node_info[1], c2g), [new_node, node, new_node_info[1]]))
                isFalse, V = isNodeVisited(new_node, V)
                if not isFalse:
                    visited_node.append([node, new_node])
                    
            new_node, new_node_info = ActionMoveUpSixty(node, node_info, step_size)
            if not in_clearance_space(new_node):
                c2g = costToGo(new_node, goal_node)
                open_list.put((totalCost(new_node_info[1], c2g), [new_node, node, new_node_info[1]]))
                isFalse, V = isNodeVisited(new_node, V)
                if not isFalse:
                    visited_node.append([node, new_node])

            new_node, new_node_info = ActionMoveDownThirty(node, node_info, step_size)
            if not in_clearance_space(new_node):
                c2g = costToGo(new_node, goal_node)
                open_list.put((totalCost(new_node_info[1], c2g), [new_node, node, new_node_info[1]]))
                isFalse, V = isNodeVisited(new_node, V)
                if not isFalse:
                    visited_node.append([node, new_node])
                
            new_node, new_node_info = ActionMoveDownSixty(node, node_info, step_size)
            if not in_clearance_space(new_node):
                c2g = costToGo(new_node, goal_node)
                open_list.put((totalCost(new_node_info[1], c2g), [new_node, node, new_node_info[1]]))
                isFalse, V = isNodeVisited(new_node, V)
                if not isFalse:
                    visited_node.append([node, new_node])
                
path = backtracking(closed_list)

visualization(visited_node, path)




