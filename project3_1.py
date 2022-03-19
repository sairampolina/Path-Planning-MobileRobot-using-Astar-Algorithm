
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

    
    return start_state,goal_state,clearence,robot_radius,step_size
       
    
    

canvas=np.ones((250,400,3),dtype='uint8')
canvas=createObstacles(canvas)

height,width,_=canvas.shape
a_height=height-1
a_width=width-1

start_state,goal_state,clearence,robot_radius,step_size=getInputs()


# changing world co-ordinates to map-coordinates
start_state[1]=a_height-start_state[1]
goal_state[1]=a_height-goal_state[1]


# print(goal_state)
# cv2.imshow('CANVAS',canvas)
# cv2.waitKey(0)
# cv2.destroyAllWindows()