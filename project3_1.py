
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

def getInputs():
    while True:
        
        while True:
            
            value=input("Enter the X- coordinate of Start Point: ")
            
            if value<0 or value>a_width:
                print("Invalid input...Re-enter a X-coordinate with\n")
                continue
            else:
                start_state.append(value)
                break
        while True:
            
            value=input("Enter Y-coordinate of Start Point: ")
            
            if value<0 or value>a_height:
                print("Invalid Input...Re-enter a valid Y-coordinate\n")
                continue
            else:
                start_state.append(value)
                break
        
        while True:
            
            value=input("Enter Orientation of Robot at Start Point(+ve in counter-clockwise): ")
            
            start_state.append(value)
            break
        
        if canvas[a_height-start_state[1]][start_state[0]][2]==255:
            print("Start Point Entered is inside Obstacle Space: ")
            continue
        else:
            break
    
   
    
       
    
    
        









canvas=np.ones((250,400,3),dtype='uint8')
canvas=createObstacles(canvas)

height,width,_=canvas.shape
a_height=height-1
a_width=width-1
cv2.imshow('CANVAS',canvas)
cv2.waitKey(0)
cv2.destroyAllWindows()