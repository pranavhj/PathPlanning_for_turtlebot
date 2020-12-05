#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
#import matplotlib.pyplot as plt
import cv2
import math
from halfplane import*
import datetime
from datetime import datetime as dtime
#import imutils
import time

def mazeMaker(mazetype): #mazetype can be either "Trial" or "Final"

	if(mazetype.lower()=="trial"):
		print("Generating trial maze....")
		width=200
		height=100
		trialmaze=np.full((height,width),1)


		# Create the square
		square_x=90
		square_y=40 # Defined from the upper right corner

		for x in range (square_x,square_x+20):
			for y in range(square_y,square_y+20):
				trialmaze[y][x]=2


		radius=15
		circle_boundbox_x=160-radius #Upper left corner of the bounding box around the circle
		circle_boundbox_y=100-50-radius#Upper left corner of the bounding box around the circle

		for x in range (circle_boundbox_x,circle_boundbox_x+2*radius):
			for y in range(circle_boundbox_y,circle_boundbox_y+2*radius):
				if((x-160)**2+(y-50)**2<radius**2):
					trialmaze[y][x]=2


		#print(trialmaze)
        
		print("Trial Maze generated.")
		return trialmaze








	if (mazetype.lower()=="final"):
		print("Generating final maze....")
		width=300
		height=200
		finalmaze=np.zeros((height,width,3),np.uint8)
		# finalmaze=np.full((height,width),1)


		# Generate circle obstacle
		radius=25
		circle_centerx=300-75
		circle_centery=50
		cv2.circle(finalmaze,(circle_centerx,circle_centery),radius,(255,255,255),-1)

		

		# Generate ellipse obstacle

		ellipse_center=(150,100) #x,y
		ellipse_axes=(40,20) #major, minor axis

		
		cv2.ellipse(finalmaze,ellipse_center,ellipse_axes,0,0,360,(255,255,255),-1)

		# Generate diamond obstacle
		diamondpts=np.array([[225,190],[250,175],[225,160],[200,175]])
		cv2.drawContours(finalmaze,[diamondpts],-1,(255,255,255),-1)


		# Generate rectangle obstacle
		x1=95
		y1=170
		x2=x1-int(75*math.cos(math.radians(30)))
		y2=170-int(75*math.sin(math.radians(30)))
		x3=x2+int(10*math.cos(math.radians(60)))
		y3=y2-int(10*math.sin(math.radians(60)))
		x4=x3+int(75*math.cos(math.radians(30)))
		y4=y3+int(75*math.sin(math.radians(30)))
		rectpoints=np.array([[x1,y1], [x2,y2],[x3,y3], [x4,y4]])
		cv2.drawContours(finalmaze,[rectpoints],-1,(255,255,255),-1)
		#print("rect complt")

		# Generate 6-poly obstacle
		polypts=np.array([[25,15], [75,15], [100,50], [75,80], [50,50], [20,80]])
		cv2.drawContours(finalmaze,[polypts],-1,(255,255,255),-1)
		#cv2.imshow("The maze",finalmaze)
		#cv2.waitKey(0)
        
        
		maze=np.empty((height,width),dtype='object')
        
		for row in range(width):
			for col in range(height):
                
                
				if finalmaze[col][row][0]==0:
					maze[col][row]=1
				elif finalmaze[col][row][0]==255:
					maze[col][row]=2
		


		print("Final Maze generated.")
		return maze



def inObstacle(x,y,d):
    #print("in obstacle")
    #if int(Y)==115:
        #return True
    #if int(Y)==95:
        #return True
    
    #if int(X)==155:
        #return True
        
    
    if (d*d)-(x*x)>0:
        #print("x not valid")
        return True
        
    if (d*d)-(y*y)>0:
        #print("y not valid")
        return True
        
    if x+d+1>10000    or   y+d+1>10000:
        #print("x,y out od endzone")
        return True
        
    for i in range(12):
        x1=x+(d*np.cos(2*3.14159/12*(i)))
        y1=y+(d*np.sin(2*3.14159/12*(i)))
        if checkPoints([x1,y1])=="inside":
            #print("yes")
            #print(str([x1,y1])+" out of range")
            return True
    return False
#    if checkPoints([X,Y])=="inside":
#        #print("yes")
#        return True
    
    return False
    
#    if maze[int(Y)][int(X)]==1:
#        return False
#    else:
#        return True
    
    
def dist(current,parent):
    
    dist=np.sqrt(np.square(current[0]-parent[0])+np.square(current[1]-parent[1]))
    return dist
#empty space=0
#visited=3
 
    
def isGoal(current,goal,threshold):
    if current[0]>=goal[0]-threshold   and   current[0]<=goal[0]+threshold  and  current[1]>=goal[1]-threshold   and   current[1]<=goal[1]+threshold:
        return True
    return False

def maze_solver_Astar():
    #checkPoints([100,100])
    #plt.axis([0, 300, 0, 200])
    print("Sol of Astar")
    #maze=createMaze2()
    #maze=mazeMaker("final")
    
    starttime=dtime.now()
                
    
#    
            
    #start=[1500.0,10000.0-500,90]
    #goal=[9500.0,2500.0,0]
    

    #start=[500.0,10000-500,90]
    #goal=[9500.0,500,0]

    startx=input("Enter x of start co-ordinates:")
    starty=input("Enter y of start co-ordinates:")
    startz=input("Enter theta of start co-ordinates:")
    start=[float(startx),float(starty),float(startz)]
    
    #start=[50,200-30,60]
    goalx=input("Enter x of Goal co-ordinates:")
    goaly=input("Enter y of Goal co-ordinates:")
    #goalz=input("Enter z of Goal co-ordinates:")
    goal=[float(goalx),float(goaly),0]
    #goal=[150,200-150,0]


    #goal=[90,200-50,0]
    
    start[0]=-(start[0]*1000-5000)
    start[1]=(start[1]*1000+5000)
    
    
    
    goal[0]=-(goal[0]*1000-5000)
    goal[1]=(goal[1]*1000+5000)

    rpm_1=input("Enter left wheel speed:")
    rpm_2=input("Enter right wheel speed:")
    rpm_1=float(rpm_1)
    rpm_2=float(rpm_2)


    rpms=[abs(rpm_1),abs(rpm_2)]
    rpms.sort()
    if start[2]>45:
        rpm_1=-abs(rpms[0])
        rpm_2=abs(rpms[1])
    else:
        rpm_1=abs(rpms[0])
        rpm_2=-abs(rpms[1])  

    robot_diameter=354.0
    clearance=input("Enter clearance value:")
    
    
    
    
    theta=30.0
    threshold=5.0
    #step_size=1
    r=66.0
    L=364.0
    frame= np.zeros( (int(10000.0/threshold),int(10000.0/threshold),3), np.uint8 )
    c1=np.array([[int(250/threshold),int(4250/threshold)],[int(1750/threshold),int(4250/threshold)],[int(1750/threshold),int(5750/threshold)],[int(250/threshold),int(5750/threshold)]])
    c2=np.array([[int(2250/threshold),int(1250/threshold)],[int(3750/threshold),int(1250/threshold)],[int(3750/threshold),int(2750/threshold)],[int(2250/threshold),int(2750/threshold)]])
    c3=np.array([[int(8250/threshold),int(4250/threshold)],[int(9750/threshold),int(4250/threshold)],[int(9750/threshold),int(5750/threshold)],[int(8250/threshold),int(5750/threshold)]])
    cv2.circle(frame, (int(goal[0]/threshold),int(goal[1]/threshold)), 10 , (255,255,255))
    cv2.circle(frame,(int(3000.0/threshold),int(8000.0/threshold)),int(1000.0/threshold),(0,255,0),-1)
    cv2.circle(frame,(int(5000.0/threshold),int(5000.0/threshold)),int(1000.0/threshold),(0,255,0),-1)
    cv2.circle(frame,(int(7000.0/threshold),int(2000.0/threshold)),int(1000.0/threshold),(0,255,0),-1)
    cv2.circle(frame,(int(7000.0/threshold),int(8000.0/threshold)),int(1000.0/threshold),(0,255,0),-1)
    cv2.drawContours(frame,[c1],-1,(0,255,0),-1)
    cv2.drawContours(frame,[c2],-1,(0,255,0),-1)
    cv2.drawContours(frame,[c3],-1,(0,255,0),-1)



#    for i in range(4000):
#        for j in range(1000):
#            if inObstacle(i,j,0.1)==True:
#                frame[int(j/threshold)][int(i/threshold)]=(0,255,0)
#    cv2.imshow("frame",frame) 
#    cv2.waitKey
    
    if inObstacle(start[0],start[1],robot_diameter/2)==True:  
        print("start in obstacle or outside boundary")
        #return
    if inObstacle(goal[0],goal[1],robot_diameter/2)==True:
        print("goal in obstacle or outside boundary")
        #return
    
    
#    for i in range(len(maze)):
#        for j in range(len(maze[0])):
#            if maze[i][j]=="O" :
#                start=[i,j]
#            if maze[i][j]=="X" :
#                goal=[i,j]
    maze_size=[10000,10000,int(360/theta)]
    
    ms1=int(maze_size[0]/threshold)
    ms2=int(maze_size[1]/threshold)
    ms3=int(360/theta)
    
    my_maze=np.zeros((ms1,ms2,ms3))
    my_maze_visited=np.zeros((ms1,ms2))
    
    
    
    s1=int(start[0]/threshold)
    s2=int(start[1]/threshold)
    s3=int(start[2]/theta)
    g1=int(goal[0]/threshold)
    g2=int(goal[1]/threshold)
    g3=int(goal[2]/theta)
    my_maze[s1][s2][s3]=5
    my_maze[g1][g2][g3]=6
    
    

    
    parent=np.zeros((ms1,ms2,ms3,3))              #if error look here at ms2 it was full range rather than /theta
    
    X=np.zeros((ms1))
    Y=np.zeros((ms2))
    #Z=np.zeros((ms3))
    H=np.zeros((ms1,ms2,ms3))
    d_sin = {angle:np.sin(np.deg2rad(angle)) for angle in (0,30,60,90,120,150,180,210,240,270,300,330,360)}
    d_cos = {angle:np.cos(np.deg2rad(angle)) for angle in (0,30,60,90,120,150,180,210,240,270,300,330,360)}
    #print(d_sin[330])
    action_space=[[0,rpm_1],[rpm_1,0],[rpm_1,rpm_1],[0,rpm_2],[rpm_2,0],[rpm_2,rpm_2],[rpm_1,rpm_2],[rpm_2,rpm_1]]
    
#    cv2.circle(frame, (95,170), 5, (0,0,255))
#    cv2.circle(frame, (36,125), 5, (0,0,255))
#    cv2.circle(frame, (100,162), 5, (0,0,255))
    
#    for i in range(ms1):
#        for j in range(ms2):
#            
#                [X[i],Y[j]]=[i,j]
#                temp=np.sqrt(np.square(X[i]*threshold-goal[0])+np.square(Y[j]*threshold-goal[1]))
#                H[i][j]=temp
    
    
    g=np.full((ms1,ms2,ms3),float('inf'))
    f=np.full((ms1,ms2,ms3),float('inf'))
    
    
    
    
    g[s1][s2][s3]=0
    f[s1][s2][s3]=H[s1][s2][s3]
    numExpanded=0
    expanded=[]
    expanded.append(start)
    
    
    current=start
    goal_current=start
    prev_current=start
    
    print("started to solve")
    while 1:

        
        min_dist=float('inf')

        
        for c in expanded:
            if min_dist>f[int(c[0]/threshold)][int(c[1]/threshold)][int(c[2]/theta)]:
                min_dist=f[int(c[0]/threshold)][int(c[1]/threshold)][int(c[2]/theta)]
                current=c
            
            
        #print("current")
        #print(current)            
        #cv2.circle(frame, (int(current[0]),int(current[1])), robot_diameter+1, (0,0,255))
        #cv2.circle(frame, (int(prev_current[0]),int(prev_current[1])), robot_diameter, (0,0,0))

        c1=int(current[0]/threshold)
        c2=int(current[1]/threshold)
        c3=int(current[2]/theta)
        #print(current)
        if isGoal(current,goal,(robot_diameter/2)+50)==True  or min_dist==float('inf'):
            goal_current=current
            print("goal reached")
            break
        
        
        #my_maze[c1][c2][c3]=3             #visited
        my_maze_visited[c1][c2]=1
        
        #plt.plot(int(current[0]),int(current[1]),'yo',markersize=1)
        frame[int(current[1]/threshold)][int(current[0]/threshold)]=(255,0,0)           #Mark blue
        cv2.circle(frame,(int(current[0]/threshold),int(current[1]/threshold)),5,(255,0,0),1)
        expanded.remove(current)
        f[c1][c2][c3]=float('inf')
        
        
        adjacent=[]
        for i in action_space:
            t=current[2]+(float(r)/L*(i[1]-i[0]))
            if t>359.9:
                t=t-360
            #print(float(r) /L)
            Y=current[1]+(r/2*(i[0]+i[1])*np.sin(np.deg2rad(t)))
            X=current[0]+(r/2*(i[0]+i[1])*np.cos(np.deg2rad(t)))
            
#            if inObstacle(X,Y,robot_diameter)==True:
#                print("inObstacle")
#                cv2.circle(frame,(int(X/threshold),int(Y/threshold)),5,(0,0,255),1)
            if  X<10000   and X>0   and Y>0   and Y<10000   and my_maze_visited[int(X/threshold)][int(Y/threshold)]!=1 and inObstacle(X,Y,(robot_diameter/2)+clearance)==False   and      inObstacle((X+current[0])/2,(Y+current[1])/2,(robot_diameter/2)+clearance)==False    and     inObstacle(((3*X)+current[0])/4,((3*Y)+current[1])/4,(robot_diameter/2)+clearance)==False   and    inObstacle((X+(3*current[0]))/4,(Y+(current[1]*3))/4,(robot_diameter/2)+clearance)==False  :#and     my_maze[int(X/threshold)][int(Y/threshold)][int(t/theta)]!=3  and    my_maze[int(X/threshold)][int(Y/threshold)][int(t/theta)]!=4    :
                adjacent.append([X,Y,t])
            #else:
                #adjacent.append(current)

        temp=0
        
        for n in range(len(adjacent)):
            adj1=int(adjacent[n][0]/threshold)
            adj2=int(adjacent[n][1]/threshold)
            adj3=int(adjacent[n][2]/theta)
            
            if (my_maze [adj1] [adj2][adj3] ==0   or   my_maze [adj1] [adj2][adj3] ==4     or  my_maze[adj1][adj2][adj3]==6)   :
                if  g[adj1][adj2][adj3] > g[c1][c2][c3]+dist(current,adjacent[n]):
                    g[adj1][adj2][adj3]=g[c1][c2][c3]+dist(current,adjacent[n])
                    #f[adj1][adj2][adj3]=g[adj1][adj2][adj3]+H[adj1][adj2][adj3]
                    f[adj1][adj2][adj3]=g[adj1][adj2][adj3]+np.sqrt(np.square(adj1*threshold-goal[0])+np.square(adj2*threshold-goal[1]))
                    f[adj1][adj2][adj3]=g[adj1][adj2][adj3]+np.sqrt(np.square(adj1*threshold-goal[0])+np.square(adj2*threshold-goal[1]))
                    #print("ASTAR")
                    parent[adj1][adj2][adj3] =current
                    temp=n
                    #plt.plot(int(adjacent[n][0]),int(adjacent[n][1]),'bo',markersize=1)
                    frame[int(adjacent[n][1]/threshold)][int(adjacent[n][0]/threshold)]=(255,255,255)
                    cv2.circle(frame,(int(adjacent[n][0]/threshold),int(adjacent[n][1]/threshold)),5,(255,255,255),1)
                    if my_maze_visited[adj1][adj2]!=1  and  my_maze[adj1][adj2][adj3]!=4  and  my_maze[adj1][adj2][adj3]!=3  :
                        expanded.append(adjacent[n])
                
                    my_maze[adj1][adj2][adj3]=4        #explored
                
                
                
                
        numExpanded=numExpanded+1
        #print(adjacent)
        #current=adjacent[temp]
        frame1=cv2.resize(frame,(500,500),interpolation = cv2.INTER_AREA)
        cv2.imshow("frame",frame1)
        prev_current=current
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
        
          
        
        #print(parent)
        
    #print(np.shape(parent))
    route=[]
    if f[int(goal_current[0]/threshold)][int(goal_current[1]/threshold)][int(goal_current[2]/theta)]==float('inf'):
        route=[]
        print("route cannot be found")
    else:
        route=[goal_current]
        #print(([route[len(route)-1][0]][route[len(route)-1][1]][0]))
        #print(int(parent[route[len(route)-1][0]][route[len(route)-1][1]][1]))
        a=int(route[len(route)-1][0]/threshold)
        b=int(route[len(route)-1][1]/threshold)
        c=int(route[len(route)-1][2]/theta)
        #print(parent[int(a/threshold)][int(b/threshold)][int(c/theta)])
        while parent[a][b][c][0]!=0  or  parent[a][b][c][1]!=0   or    parent[a][b][c][2]!=0:
            a=int(route[len(route)-1][0]/threshold)
            b=int(route[len(route)-1][1]/threshold)
            c=int(route[len(route)-1][2]/theta)
            #print(parent[int(a/threshold)][int(b/threshold)][int(c/theta)])
            #print([int(a/threshold),int(b/threshold),int(c/theta)])
            c1=parent[a][b][c][0]
            c2=parent[a][b][c][1]
            c3=parent[a][b][c][2]
            #print([c1,c2,c3])
            #plt.plot(c1,c2,'ro',markersize=1)
            frame[int(c2/threshold)][int(c1/threshold)]=(0,255,0)
            route.append([c1,c2,c3])
           
    #print(route)
    
    #for p in route:
        #print(p)
    route.insert(0, goal)   
    for p in range(len(route)-2):
        cv2.line(frame,(int(route[p][0]/threshold),int(route[p][1]/threshold)),(int(route[p+1][0]/threshold),int(route[p+1][1]/threshold)), (0,0,255), 7)
    print(numExpanded)
    endtime=dtime.now()
    print([starttime,endtime])
    frame1=cv2.resize(frame,(500,500),interpolation = cv2.INTER_AREA)
    print(route)
    cv2.imshow("frame",frame1)
    cv2.waitKey(0)   
    cv2.destroyAllWindows() 

    return route
    
#    for i in route:
#        maze[int(i[0])][int(i[1])]="="
#    maze[start[0]][start[1]]="O"
#    maze[goal[0]][goal[1]]="X"
    #print(maze)
   
#maze_solver_Astar()
