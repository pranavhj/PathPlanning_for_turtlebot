# -*- coding: utf-8 -*-
"""
Created on Wed Mar  4 23:19:04 2020

@author: prana
"""

import numpy as np


def getIntercept(x1,y1,x2,y2):
    m=(y2-y1)/(x2-x1)

    b=y1-m*x1
    return b

def getSlope(x1,y1,x2,y2):
    return (y2-y1/(x2-x1))


def checkHalfPlane(X0, Y0, x, y):
    a = (X0[1] - Y0[1])/(X0[0] - Y0[0])
    b = X0[1] - a*X0[0]
    
    check = y - b - a*x
    
    if check <= 0:
        return 1
    return 0

def checkCurves(point):
#    x_dash=(1-np.square((point[0]-150)/40))
#    if x_dash>0:
#        y1=100-20*np.sqrt(x_dash)
#        y2=100+20*np.sqrt(x_dash)
#        if point[0]>=110   and   point[0]<=190   and point[1]>=y1   and point[1]<=y2:
#            #print()
#            return "inside"
#    
#    
#    #for circle
#    x_dash=225-np.square(point[0]-225)
#    if x_dash>0:
#        y1=50-np.sqrt(x_dash)
#        y2=50+np.sqrt(x_dash)
#        if point[0]>=200  and  point[0]<=250 and   point[1]>=y1   and  point[1]<=y2:
#            return "inside"
        
        
    x_dash=1000000-np.square(point[0]-5000)
    if x_dash>0:
        y1=5000-np.sqrt(x_dash)
        y2=5000+np.sqrt(x_dash)
        if point[0]>=4000  and  point[0]<=6000 and   point[1]>=y1   and  point[1]<=y2:
            return "inside"
        
    x_dash=1000000-np.square(point[0]-7000)
    if x_dash>0:
        y1=2000-np.sqrt(x_dash)
        y2=2000+np.sqrt(x_dash)
        if point[0]>=6000  and  point[0]<=8000 and   point[1]>=y1   and  point[1]<=y2:
            return "inside"
        
    x_dash=1000000-np.square(point[0]-3000)
    if x_dash>0:
        y1=8000-np.sqrt(x_dash)
        y2=8000+np.sqrt(x_dash)
        if point[0]>=2000  and  point[0]<=4000 and   point[1]>=y1   and  point[1]<=y2:
            return "inside"
    
    x_dash=1000000-np.square(point[0]-7000)
    if x_dash>0:
        y1=8000-np.sqrt(x_dash)
        y2=8000+np.sqrt(x_dash)
        if point[0]>=6000  and  point[0]<=8000 and   point[1]>=y1   and  point[1]<=y2:
            return "inside"
    
    return "outside"
def checkPoints(point):
    
    #print("new_halfplane)
    if checkCurves(point)=="inside":
        return "inside"
    triangles=[   [[2250,1250],[2250,2750],[3750,2750]], [[3750,1250],[2250,2750],[3750,2750]],  [[250,4250],[1750,4250],[1750,5750]]   ,  [[250,4250],[250,5750],[1750,5750]],    [[8250,4250],[9750,4250],[9750,5750]],   [[8250,4250],[9750,5750],[8250,5750]] ]#, [[50,50],[75,15],[100,50]], [[50,50],[100,50],[75,80]],  [[200,175],[225,160],[250,175]],  [[200,175],[225,190],[250,175]]  ,[[ 95,170],[ 31,133],[ 36,125]] ]
   
    pi=3.14159
    X=[95,30]
    Y=[95-(np.cos(pi/6)*75), 30+(75*np.sin(pi/6))]
    Z=[95+(10*np.sin(pi/6)), 30+(10*np.cos(pi/6))]
    W=[Z[0]-(np.cos(pi/6)*75), Z[1]+(75*np.sin(pi/6))]
    
    if checkHalfPlane(X, Y, point[0], 200-point[1])==0 and checkHalfPlane(X, Z, point[0], 200-point[1])==0 and checkHalfPlane(Y, W,point[0], 200-point[1])==1 and checkHalfPlane(Z, W, point[0], 200-point[1])==1:
        #print("savle")    
        return "inside"
    
    for tri in triangles:
        #print(tri)
        #print("yes")
        a=tri[0]
        b=tri[1]
        c=tri[2]
        X=[a[0],b[0],c[0]]
        Y=[a[1],b[1],c[1]]
        X.sort()
        Y.sort()
        #print(X)
        #print(Y)
#        if point[0]<X[0]    or    point[0]>X[2]    or   point[1]<Y[0]    or    point[1]>Y[2]:
#            return "outside"
        lists=[[a,b,c],[b,c,a],[a,c,b]]
        flag_counter=0
        for p in lists:
            
            p1=p[0]
            p2=p[1]
            p3=p[2]
            #print([p1,p2,p3])
            if p1[0]==p2[0]:
                if p3[0]>p1[0]:
                    #print( ["x","greater",p1[0],"        ",p1[1],p2[1]])
                    q=[p1[1],p2[1]]
                    q.sort()
                    if point[0]>= p1[0] :#  and    point[1]>= q[0]    and    point[1]<= q[1]:
                        flag_counter=flag_counter+1
                        #print("hi")
                        continue
                        
                else:
                    #print(["x","lesser",p1[0]])
                    q=[p1[1],p2[1]]
                    q=q.sort()
                    if point[0]<=p1[0] :#  and    point[1]>=q[0]    and    point[1]<=q[1]:
                        flag_counter=flag_counter+1
                        #print("hi")
                        continue
            else:
                slope=(p2[1]-p1[1])/(p2[0]-p1[0])
                c1=p1[1]-(slope*p1[0])
                sign="lesser"
                if p3[1]>p1[1]    or   p3[1]>p2[1]:
                    sign="greater"
                
                
                
                
                #print(["y",sign,slope,c1,"     ",p1[0],p2[0]])
                
                if sign=="greater":
                    var=(slope*point[0])+c1
                    q=[p1[0],p2[0]]
                    q.sort()
                    #print([var,q,point])
                    if   point[1]>=var :#  and point[0]>=q[0]    and   point[0]<=q[1]:
                        flag_counter=flag_counter+1
                        #print("hi")
                        continue
                    
                    
                    
                else:
                    var=(slope*point[0])+c1
                    q=[p1[0],p2[0]]
                    q.sort()
                    #print([var,q,point])
                    if   point[1]<=var :#or (var<=p1[1]   and   var<=p2[1])  :#  and point[0]>=q[0]    and   point[0]<=q[1]:
                        flag_counter=flag_counter+1
                        #print("hi")
                        continue
            
        #print(flag_counter)
        
        if flag_counter==3:
            return "inside"
        
        

		# Half Planes for Rectangle
		
      
    return "outside" 
    #for ellipse
    
#print(checkPoints([227,188])
#print(checkPoints([1,6/5+0.1])
