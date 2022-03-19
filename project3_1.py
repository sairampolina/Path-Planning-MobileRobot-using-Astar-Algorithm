
"""
Created on Sat Mar 19 08:31:24 2022

@author: sairam
"""

import cv2
import numpy as np
import copy
import matplotlib.pyplot as plt


def createObstacles(canvas):
     
    height,width,_ = canvas.shape
           
    for i in range(width): 
        for j in range(height):
            if(i-5<=0) or (i-395>=0) or (j-5 <=0) or (j-245>=0):
                canvas[j][i]= [0,255,0]
    
            if ((i-300)**2+(j-65)**2-(45**2))<=0:
                canvas[j][i]=[0,255,0]
            
            if (j+(0.57*i)-218.53)>=0 and (j-(0.57*i)+10.04)>=0 and (i-240)<=0 and (j+(0.57*i)-310.04)<=0 and (j-(0.57*i)-81.465)<=0 and (i-160)>=0:
                canvas[j][i]= [0,255,0]
    
            if ((j+(0.316*i)-71.1483)>=0 and (j+(0.857*i)-145.156)<=0 and (j-(0.114*i)-60.909)<=0) or ((j-(1.23*i)-28.576)<=0 and (j-(3.2*i)+202.763)>=0 and (j-(0.114*i)-60.909)>=0):
                canvas[j][i]=[0,255,0]
 
    return canvas

start_state=[]
goal_state=[]

def getInputs():
    
    # get valid x,y,theta_s coordinate of start point
    while True:
        
        while True:
            
            value=input("Enter the X- coordinate of Start Point: ")
            
            if float(value)<0 or float(value)>a_width:
                print("Invalid input...Re-enter a valid X-coordinate \n")
                continue
            else:
                start_state.append(float(value))
                break
        while True:
            
            value=input("Enter Y-coordinate of Start Point: ")
            
            if float(value)<0 or float(value)>a_height:
                print("Invalid Input...Re-enter a valid Y-coordinate\n")
                continue
            else:
                start_state.append(float(value))
                break
        
        while True:
            
            value=input("Enter Orientation of Robot at Start Point(+ve in counter-clockwise):\n")
            
            start_state.append(float(value))
            break
        
        
        if (canvas[a_height-int(start_state[1])][int(start_state[0])][1]==255):
            print("***Start Point Entered is inside Obstacle Space..Retry***")
            start_state.clear()
            continue
        else:
            break
    
    # get valid x,y,theta_g coordinate of goal node
    while True:
        
        while True:
            
            value=input("Enter the X- coordinate of Goal Point: ")
            
            if float(value)<0 or float(value)>a_width:
                print("Invalid input...Re-enter a valid X-coordinate \n")
                
                continue
            else:
                goal_state.append(float(value))
                break
        
        while True:
            
            value=input("Enter Y-coordinate of Goal Point: ")
            
            if float(value)<0 or float(value)>a_height:
                print("Invalid Input...Re-enter a valid Y-coordinate\n")
                continue
            else:
                goal_state.append(float(value))
                break
        
        while True:
            
            value=input("Enter Orientation of Robot at Goal Point(+ve in counter-clockwise): \n")
            
            goal_state.append(float(value))
            break
        
        if canvas[a_height-int(goal_state[1])][int(goal_state[0])][1]==255:
            print("***Goal Point Entered is inside Obstacle Space..Retry***")
            goal_state.clear()
            continue
        else:
            clearence=input('Enter Clearence of robot to obstacles:')
            robot_radius=input('Enter radius of Robot: ')
            step_size=input('Enter the step-size for robot movement:')
            break

    
    return start_state,goal_state,int(clearence),int(robot_radius),int(step_size)
       


def actionZero(node,canvas,step_size):
    next_node=copy.deepcopy(node)
    
    theta_action=0
    
    x1,y1,theta_p=(next_node[0],next_node[1],next_node[2])
    
    
    theta_now=theta_p+theta_action
    
    x=x1+step_size*(np.cos(np.radians(theta_now)))
    y=y1+step_size*(np.sin(np.radians(theta_now)))
    
    if ((x>=0 and x<=a_width) and (y>=0 and y<=a_height)) and (canvas[int(y)][int(x)][1]<255) and (not isItDuplicatenode(x, y, theta_now,V)):
        
        next_node[0]=int(x)
        next_node[1]=int(y)
        next_node[2]=theta_p+theta_action
        
        return True,next_node
    else:
        return False,next_node

def actionPositiveThirty(node,canvas,step_size):
    next_node=copy.deepcopy(node)
    
    theta_action=30
    
    x1,y1,theta_p=(next_node[0],next_node[1],next_node[2])
    
    
    theta_now=theta_p+theta_action
    
    x=x1+step_size*(np.cos(np.radians(theta_now)))
    y=y1+step_size*(np.sin(np.radians(theta_now)))
    
    if ((x>=0 and x<=a_width) and (y>=0 and y<=a_height)) and (canvas[int(y)][int(x)][1]<255) and (not isItDuplicatenode(x, y, theta_now,V)):
        
        next_node[0]=x
        next_node[1]=y
        next_node[2]=theta_p+theta_action
        
        return True,next_node
    else:
        return False,next_node

def actionPositiveSixty(node,canvas,step_size):
    next_node=copy.deepcopy(node)
    
    theta_action=60
    
    x1,y1,theta_p=(next_node[0],next_node[1],next_node[2])
    
    
    theta_now=theta_p+theta_action
    
    x=x1+step_size*(np.cos(np.radians(theta_now)))
    y=y1+step_size*(np.sin(np.radians(theta_now)))
    
    if ((x>=0 and x<=a_width) and (y>=0 and y<=a_height)) and (canvas[int(y)][int(x)][1]<255) and (not isItDuplicatenode(x, y, theta_now,V)):
        
        next_node[0]=x
        next_node[1]=y
        next_node[2]=theta_p+theta_action
        
        return True,next_node
    else:
        return False,next_node

def actionNegativeThirty(node,canvas,step_size):
    next_node=copy.deepcopy(node)
    
    theta_action=-30
    
    x1,y1,theta_p=(next_node[0],next_node[1],next_node[2])
    
    
    theta_now=theta_p+theta_action
    
    x=x1+step_size*(np.cos(np.radians(theta_now)))
    y=y1+step_size*(np.sin(np.radians(theta_now)))
    
    if ((x>=0 and x<=a_width) and (y>=0 and y<=a_height)) and (canvas[int(y)][int(x)][1]<255) and (not isItDuplicatenode(x, y, theta_now,V)):
        
        next_node[0]=x
        next_node[1]=y
        next_node[2]=theta_p+theta_action
        
        return True,next_node
    else:
        return False,next_node

def actionNegativeSixty(node,canvas,step_size):
    next_node=copy.deepcopy(node)
    
    theta_action=-60
    
    x1,y1,theta_p=(next_node[0],next_node[1],next_node[2])
    
    
    theta_now=theta_p+theta_action
    
    x=x1+step_size*(np.cos(np.radians(theta_now)))
    y=y1+step_size*(np.sin(np.radians(theta_now)))
    
    if ((x>=0 and x<=a_width) and (y>=0 and y<=a_height)) and (canvas[int(y)][int(x)][1]<255) and (not isItDuplicatenode(x, y, theta_now,V)):
        
        next_node[0]=x
        next_node[1]=y
        next_node[2]=theta_p+theta_action
        
        return True,next_node
    else:
        return False,next_node

def isItDuplicatenode(p,q,r,V):
    
    p=round(p*2)/2
    q=round(q*2)/2
    
    if r==360:
        r=0
   
    print(p,q,r)
   
    if V[int(2*p)][int(2*q)][int(r//30)]==0:
        
        V[int(2*p)][int(2*q)][int(r//30)]=1
        return False
    else:
        return True
    
    
    

if __name__=='__main__':
    
    canvas=np.ones((250,400,3),dtype='uint8')
    canvas=createObstacles(canvas)
    
    height,width,_=canvas.shape
    a_height=height-1
    a_width=width-1
    
    start_state,goal_state,clearence,robot_radius,step_size=getInputs()
    
# =============================================================================
#     print(start_state)
#     print(goal_state)
#     
# =============================================================================
    # changing world co-ordinates to map-coordinates
    start_state[1]=a_height-start_state[1]
    goal_state[1]=a_height-goal_state[1]
    
    # creating matrix V
    
    V=np.zeros((500,800,12),dtype='uint8')
    
    # x,y=actionZero([10.0,9.0,30],canvas,step_size)
    
    x,y=actionNegativeThirty([10.0,9.0,30],canvas,step_size)
    
    print(x)
    print(y)
    
    
    # print(goal_state)
    # cv2.imshow('CANVAS',canvas)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()