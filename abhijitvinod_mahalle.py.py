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


