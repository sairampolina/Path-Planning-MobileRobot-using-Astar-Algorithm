
"""
Created on Sat Mar 19 08:31:24 2022

@author: sairam
"""

import cv2
import numpy as np
import copy
import matplotlib.pyplot as plt
import heapq as hp

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
    

def c2g(coordinates1,goal_state):
    point1=np.array(copy.deepcopy(coordinates1[0:2]))
    goal_point=np.array(copy.deepcopy(goal_state[0:2]))
    
    dis=np.linalg.norm(point1-goal_point)
    
    return int(dis)

def AStar(start_state,goal_state,canvas,step_size):
    open_list = []
    closed_list = {}
    back_track_flag = False
    hp.heapify(open_list)
    c2g_i=c2g(start_state,goal_state)
    hp.heappush(open_list,[c2g_i,0,start_state,start_state])
    # 0: Total-cost,1:cost to come,2:parent node, 3: present node
    
    while(len(open_list)>0):
        
        
        node=hp.heappop(open_list)
        
        # check if popped node is goal node
        # if it is goal start back tracking
        if node[3]==goal_state:
            back_track_flag=True
            print('Started BackTracking')
            break
        
        # adding popped node to closed-list
        closed_list[(node[3][0],node[3][1],node[3][2])]=node[2]
        
        print('before openlist',open_list)
        print('before closed_list',closed_list)
        present_c2c=node[1]
        
        # generate child
        flag,next_node=actionPositiveThirty(node[3], canvas, step_size)
        
        if(flag):
            
            present_c2g=c2g(next_node,goal_state)
            
            # consider child if it is not in closed-list
            
            if tuple(next_node) not in closed_list:
                
                new_c2c=present_c2c+step_size
                Total_cost=new_c2c+present_c2g
                
                
                # if child is not in open_list add child with its  Total cost,c2c, and parent node
                temp_list=[]
                
                for i in range(len(open_list)):
                    temp_list.append(open_list[i][3])
               
                if tuple(next_node) not in temp_list:
                    hp.heappush(open_list,[Total_cost,new_c2c, node[3],next_node])
                    hp.heapify(open_list)
                else:
                    idx=temp_list.index(next_node) 
                    if(Total_cost<open_list[idx][0]): # Updating the cost and parent node
                           open_list[idx][0] = Total_cost
                           open_list[idx][1] = new_c2c
                           open_list[idx][2] = node[3]
                           hp.heapify(open_list)
                temp_list.clear()
           
        print("open_list",open_list)
        print('closed_list',closed_list)
        break         
             


    

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
    
    # x,y=actionNegativeThirty([10.0,9.0,30],canvas,step_size)
    
    # print(x)
    # print(y)
    
    
    AStar(start_state,goal_state,canvas,step_size)
    # print(goal_state)
    # cv2.imshow('CANVAS',canvas)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()