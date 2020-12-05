# -*- coding: utf-8 -*-
"""
Created on Sun Feb  2 12:30:54 2020
111111
@author: prana
"""
import numpy as np
from random import seed
from random import randint
import matplotlib.pyplot as plt
import time
import cv2
import copy


class Node():
    def __init__(self,co_ord):
        self.co_ord=co_ord
        self.neighbours=[]
        self.obstacle=None
        self.cost=0
        self.cost_togo=0
def findDistance(p1,p2):
    return np.sqrt(np.square(p1.co_ord[0]-p2.co_ord[0])+np.square(p1.co_ord[1]-p2.co_ord[1]))


def MazeMaker(threshold, block_dimensions):
    #r=66
    #L=354
    frame= np.zeros( (int(block_dimensions[0]/threshold)+3,int(block_dimensions[1]/threshold)+3,3), np.uint8 )
    
    c1=np.array([[int(250/threshold),int(4250/threshold)],[int(1750/threshold),int(4250/threshold)],[int(1750/threshold),int(5750/threshold)],[int(250/threshold),int(5750/threshold)]])
    c2=np.array([[int(2250/threshold),int(1250/threshold)],[int(3750/threshold),int(1250/threshold)],[int(3750/threshold),int(2750/threshold)],[int(2250/threshold),int(2750/threshold)]])
    c3=np.array([[int(8250/threshold),int(4250/threshold)],[int(9750/threshold),int(4250/threshold)],[int(9750/threshold),int(5750/threshold)],[int(8250/threshold),int(5750/threshold)]])
    
    #cv2.circle(frame, (int(goal[0]/threshold),int(goal[1]/threshold)), 10 , (255,255,255))
    cv2.circle(frame,(int(3000/threshold),int(8000/threshold)),int(1000/threshold),(255,255,255),-1)
    cv2.circle(frame,(int(5000/threshold),int(5000/threshold)),int(1000/threshold),(255,255,255),-1)
    cv2.circle(frame,(int(7000/threshold),int(2000/threshold)),int(1000/threshold),(255,255,255),-1)
    cv2.circle(frame,(int(7000/threshold),int(8000/threshold)),int(1000/threshold),(255,255,255),-1)
    
    cv2.drawContours(frame,[c1],-1,(255,255,255),-1)
    cv2.drawContours(frame,[c3],-1,(255,255,255),-1)
    cv2.drawContours(frame,[c2],-1,(255,255,255),-1)
    
    return frame
            
            

def closestPoint(point_,Nodes):
    d_smallest=float('inf')
    point_smallest_=Node([1500,1500])
    for node in Nodes:
        #print(point_)
        #print(node)
        #print(" ")
        d=findDistance(point_,node)
        #print(d)
        if d<d_smallest:
            d_smallest=d
            point_smallest_=node
        #print([d_smallest,point_smallest_.co_ord])
    return point_smallest_

def isGoal(node_,goal_,delta):
    if euler_dist(node_,goal_)<=delta:
        return True
    return False


def contourIntersect(original_image, contour1, contour2):
    # Two separate contours trying to check intersection on
    contours = [contour1, contour2]

    # Create image filled with zeros the same size of original image
    blank = np.zeros(original_image.shape[0:2])

    # Copy each contour into its own image and fill it with '1'
    image1 = cv2.drawContours(blank.copy(), contours, 0, 1)
    image2 = cv2.drawContours(blank.copy(), contours, 1, 1)

    # Use the logical AND operation on the two images
    # Since the two images had bitwise and applied to it,
    # there should be a '1' or 'True' where there was intersection
    # and a '0' or 'False' where it didnt intersect
    intersection = np.logical_and(image1, image2)

    # Check if there was a '1' in the intersection
    return intersection.any()


def CheckSurroundingObstacle(node_,R,Maze,threshold,block_dimensions):
    
#    Maze1=copy.deepcopy(Maze)
#    cv2.circle(Maze1,(int(node_.co_ord[0]/threshold),int(node_.co_ord[1]/threshold)),int(R/threshold),(255,255,255),-1)
#    
#    gray1 = cv2.cvtColor(Maze1, cv2.COLOR_BGR2GRAY) 
#    ret1, thresh1 = cv2.threshold(gray1, 240, 255, cv2.THRESH_BINARY)
#    
#    gray = cv2.cvtColor(Maze, cv2.COLOR_BGR2GRAY) 
#    ret, thresh = cv2.threshold(gray, 240, 255, cv2.THRESH_BINARY)
#    
#    Maze_dash=cv2.bitwise_not(thresh)
#    Maze_and=cv2.bitwise_and(Maze_dash,thresh1,mask=None)
#    cv2.imshow("amd",cv2.resize(Maze_and,(300,300),interpolation = cv2.INTER_AREA))
#    if cv2.waitKey(10) & 0xFF == ord('q'):
#            print("  ")
#    
#    
#    
#    contours_circle_only=cv2.findContours(Maze_and, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0]
#    arc_len=0
#    for c in contours_circle_only:
#        arc_len=arc_len+cv2.arcLength(c,True)
#        Maze=cv2.drawContours(Maze,c,-1,(250,0,0),3)
#        cv2.waitKey(0)
#            
#    if abs(arc_len-(329.7))>0.1:
#        print(arc_len,2*3.142*R/threshold)
#        cv2.imshow("amd",cv2.resize(Maze1,(300,300),interpolation = cv2.INTER_AREA))
#        #cv2.waitKey(0)
#        return True
#    
#    return False
#        
    
    
    
    
    
    
    
    
    for i in range(8):
        theta=i*360/8
        x_new=node_.co_ord[0]+(np.cos(np.deg2rad(theta))*R)
        y_new=node_.co_ord[1]+(np.sin(np.deg2rad(theta))*R)
        if x_new>block_dimensions[0]:
            x_new=block_dimensions[0]-R-5
        if y_new>block_dimensions[1]:
            y_new=block_dimensions[1]-R-5
        if x_new<0:
            x_new=R+5
        if y_new<0:
            y_new=R+5
        new_node_=Node([x_new,y_new])
        if Maze[int(y_new/threshold)][int(x_new/threshold)][0]==255   and  Maze[int(y_new/threshold)][int(x_new/threshold)][1]==255   and   Maze[int(y_new/threshold)][int(x_new/threshold)][2]==255 :
            return True
    return False

def RRT(start,goal,counter_,delta,threshold,block_dimensions):  
    R=254+20     #radius
    Maze=MazeMaker(threshold,block_dimensions)
    Nodes=[]
    
    start_=Node([start[0],start[1]])
    #goal=[10000,10000]
    goal_=Node([goal[0],goal[1]])
    w=block_dimensions[0]
    h=block_dimensions[1]
        
    Nodes.append(start_)
    counter=0
    flag=0
    start_.cost=0
    radius=delta+20
    print("algo started")
    
    while 1:#counter<counter_:
        counter=counter+1 
        #Nodes_list=[]
        for i in range(1):
            x=randint(0,w-R)
            y=randint(0,h-R)
            point_=Node([x,y])
            point_neighbour_=closestPoint(point_,Nodes)
            #if Maze[int(x/threshold)][int(y/threshold)][0]==255:         #obst
                #continue
            #write heurestic part here
            
            #interpolate function
            theta=np.arctan2(point_.co_ord[1]-point_neighbour_.co_ord[1],point_.co_ord[0]-point_neighbour_.co_ord[0])
            new_x=x
            new_y=y
            if findDistance(point_,point_neighbour_)>delta:
                
                new_x=point_neighbour_.co_ord[0]+(delta*np.cos(theta))
                new_y=point_neighbour_.co_ord[1]+(delta*np.sin(theta))
                #if new_x<x    or  new_y<y:
                    #continue
                if new_x>w:
                    new_x=w-R
                if new_x<0:
                    new_x=0
                if new_y>h:
                    new_y=h-R
                if new_y<0:
                    new_y=0
                point_=Node([new_x,new_y])
                
            costs=[]
            Nodes_list_nearest_=FindNeighbourswithDistance(Nodes,point_,radius)
            #print(len(Nodes_list_nearest_))
            for n_ in Nodes_list_nearest_:
                costs.append(n_.cost+euler_dist(n_,point_))
            
            min_cost=min(costs)
            index=costs.index(min_cost)
            #point_neighbour_dash_=Nodes_list_nearest_[index]
            
                
            point_neighbour_=Nodes_list_nearest_[index]
            point_.cost=point_neighbour_.cost+euler_dist(point_,point_neighbour_)
            
            
            
            
#            if pathinObstacle(point_,point_neighbour_,Maze,threshold,R,block_dimensions)==True:#Maze_[int(new_x)][int(new_y)].obstacle==1   or   Maze_[int((x+new_x)/2)][int((y+new_y)/2)].obstacle==1:
#                counter=counter-1
#                continue
            
#            if CheckSurroundingObstacle(point_,R,Maze,threshold,block_dimensions)==True:
#                counter=counter-1
#                continue
        #print([new_x,new_y])
        
        if pathinObstacle(point_,point_neighbour_,Maze,threshold,R,block_dimensions)==True:
            counter=counter-1
            #print("hi")
            #print([point_neighbour_.co_ord,point_.co_ord])
            continue
        
        point_neighbour_.neighbours.append(point_)
    
        Nodes.append(point_)
        
        

        if isGoal(point_,goal_,delta)==True:#if point_.co_ord[0]>w   and  point_.co_ord[0]<w  and point_.co_ord[1]>h   and  point_.co_ord[1]<h:
            flag=1
            print("Final cost is ....",point_.cost+euler_dist(point_,goal_))
            print(Nodes[len(Nodes)-1].co_ord)
            break
        
        
    
    if flag==1:
        print("Maze Solved")
        print("Iterations= ")
        print(counter)
    else:
        print("Could not solve")
        
        
    
    
    
    
    
    
    
    print(Nodes[0].co_ord)             #Start is start
    print(Nodes[len(Nodes)-1].co_ord)    #last is goal
    Maze,path_=maze_solver_bfs(Nodes,Nodes[len(Nodes)-1],Maze,threshold)   
    path_=[ele for ele in reversed(path_)] 
    path_optimized=FindOptimizedPath(path_,Maze,threshold,R,block_dimensions)
    path_optimized.append(path_[len(path_)-1])
    path_optimized_co_ord=[]
    for p in path_optimized:
        print(p.co_ord)
        path_optimized_co_ord.append(p.co_ord)

    

    for p in range(len(path_optimized)-1):
        cv2.line(Maze,(int(path_optimized[p].co_ord[0]/threshold),int(path_optimized[p].co_ord[1]/threshold)),(int(path_optimized[p+1].co_ord[0]/threshold),int(path_optimized[p+1].co_ord[1]/threshold)),(250,0,255),10)

    Maze=cv2.resize(Maze,(300,300),interpolation = cv2.INTER_AREA)
    cv2.imshow("maze",Maze)
    cv2.waitKey(0)
    
    while 1:
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
    
    plt.show()
    cv2.destroyAllWindows()
    return path_optimized_co_ord
#    for n in range (1,len(Nodes)):
#        
#        for n1 in Nodes[n].neighbours:
#            #print(n1.co_ord)
            

def FindNeighbourswithDistance(Nodes,point_,radius):
    Nodes_list_=[]
    for n_ in Nodes:
        if euler_dist(point_,n_)<radius:
            Nodes_list_.append(n_)
    return Nodes_list_




def FindOptimizedPath(path_,Maze,threshold,R,block_dimensions):
    print("Finding optimized path")
    first_=path_[0]
    second_=0
    i=0
    path_optimized_=[]
    while(1):
        #first_=path_[0]
        second_=path_[len(path_)-1-i]
        
        i=i+1
        #result=CheckObstruction(first_,second_,Maze,threshold)
        result=pathinObstacleOptimum(first_,second_,Maze, threshold,R,block_dimensions)
        if result==True:
            continue
        else:
            if second_==path_[len(path_)-1]:
                path_optimized_.append(first_)
                return path_optimized_ 
            else:
                
                #print("'''''''''''''''''''''''")
                i=0
                path_optimized_.append(first_)
                first_=second_
                
                
def CheckObstruction(first_,second_,Maze,threshold):
    x1,y1=first_.co_ord
    x2,y2=second_.co_ord
    if x1>x2   and x1-x2!=0:
        [x1,y1,x2,y2]=[x2,y2,x1,y1]
    if x1!=x2:    
        for d in np.arange(x1,x2,100).tolist():
            if x2-x1!=0:
                y=(y2-y1)*(d-x1)/(x2-x1)+y1
    
                if Maze[int(y/threshold)][int(d/threshold)][0]==255   and   Maze[int(y/threshold)][int(d/threshold)][1]==255    and    Maze[int(y/threshold)][int(d/threshold)][2]==255:
                    return True
    y_dash=[y1,y2]    
    if x1==x2:
        #print("hi")
        for d in np.arange(min(y_dash),max(y_dash)):
            if Maze[int(d/threshold)][int(x1/threshold)][0]==255   and   Maze[int(d/threshold)][int(x1/threshold)][1]==255    and    Maze[int(d/threshold)][int(x1/threshold)][2]==255:
                    return True
    return False






#pass the last node and it will go to the first node
def maze_solver_bfs(Nodes,node,frame,threshold):
    print("Sol of BFS")
    
    
    

    path=[]
    
    print("loop started")
    
    start=node
    path=[]
    path.append(start)
    while 1:
        start=SearchinNeighbour(start,Nodes)
        path.append(start)
        #print(start.co_ord)
        if start==Nodes[0]:       #i.e.start point
            break
    print("path is")
    #for c in path:
        #print(c.cost)
    for i in range(len(path)-1):
        cv2.line(frame,(int(path[i].co_ord[0]/threshold),int(path[i].co_ord[1]/threshold)),(int(path[i+1].co_ord[0]/threshold),int(path[i+1].co_ord[1]/threshold)),(0,0,255),10)
    return frame,path
    



       
def SearchinNeighbour(start,Nodes):
    for c in Nodes:
        for d in c.neighbours:
            if d==start:
                return c
    


def euler_dist(node1_,node2_):
    x1,y1=node1_.co_ord
    x2,y2=node2_.co_ord
    return (((y2-y1)**2)+((x2-x1)**2))**(0.5)



        
        
    
    
   
def findIndex(n,Nodes):
    for i in range(len(Nodes)):
        if Nodes[i]==n:
            return i
    return -1
def presentIndex(n,Nodes):
    for node in Nodes:
        if node==n:
            return True
    return False

def plotLine(x1,y1,x2,y2,color):#,Xpath,Ypath):
    if x1>x2   and x1-x2!=0:
        [x1,y1,x2,y2]=[x2,y2,x1,y1]
    if x1!=x2:    
        for d in np.arange(x1,x2,0.25).tolist():
            if x2-x1!=0:
                y=(y2-y1)*(d-x1)/(x2-x1)+y1
    
                plt.plot(d,y,color,markersize=2)
        
    if x1==x2:
        print("hi")
        if y1>y2:
            [x1,y1,x2,y2]=[x2,y2,x1,y1]
        for d in np.arange(y1,y2,0.25).tolist():
            plt.plot(x1,d,color,markersize=2)
        #print("hi")
    #time.sleep(0.05)
        #Xpath.append(d)
        #Ypath.append(y)
        #print(Xpath)
        #print(Ypath)
        #return [[Xpath],[Ypath]]
    
def printPointneighbours(point_neighbour_):
    lists=[]
    for c in point_neighbour_.neighbours:
        lists.append(c.co_ord)
    print([point_neighbour_.co_ord,lists])
    
def pathinObstacle(point_,point_neighbour_,Maze, threshold,R,block_dimensions):
    [x1,y1]=point_.co_ord
    [x2,y2]=point_neighbour_.co_ord
    
    if x1>x2   and x1-x2!=0:
        [x1,y1,x2,y2]=[x2,y2,x1,y1]
        
    x1=x1/threshold
    x2=x2/threshold
    y1=y1/threshold
    y2=y2/threshold
    for d in np.arange(int(x1),int(x2)).tolist():
        if x2-x1>1:
            y=(y2-y1)*(d-x1)/(x2-x1)+y1
        #if y<block_dimensions[1]/threshold
        
        else:
            y=y1
        
        p=Maze[int(y)][int(d)]
        if p[0]==255   and p[1]==255    and   p[2]==255:
            return True
        
        if CheckSurroundingObstacle(Node([d*threshold,y*threshold]),R,Maze,threshold,block_dimensions )==True   :
            return True
            
    return False



def pathinObstacleOptimum(point_,point_neighbour_,Maze, threshold,R,block_dimensions):
    [x1,y1]=point_.co_ord
    [x2,y2]=point_neighbour_.co_ord
    
    if x1>x2   and x1-x2!=0:
        [x1,y1,x2,y2]=[x2,y2,x1,y1]
  
    for d in np.arange(int(x1),int(x2)).tolist():
        if x2-x1>1:
            y=(y2-y1)*(d-x1)/(x2-x1)+y1
        #if y<block_dimensions[1]/threshold
        
        else:
            y=y1
        
        p=Maze[int(y/threshold)][int(d/threshold)]
        if p[0]==255   and p[1]==255    and   p[2]==255:
            return True
        
        if CheckSurroundingObstacle(Node([d,y]),R,Maze,threshold,block_dimensions )==True   :
            return True
            
    return False

#RRT([550,550],[9500,9500],500,1000,5,[10000,10000])


    
    