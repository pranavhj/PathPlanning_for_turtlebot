#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
#from withRPMs_new_maze import*
from rrt_1_new_maze import*
import math
import matplotlib.pyplot as plt
velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

odom=Odometry()
angle_list=[]
position_list=[]

def quaternion_to_euler(x, y, z, w):

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [yaw*180/3.142, pitch*180/3.142, roll*180/3.142]



def odom_callback(message):
    #get_caller_id(): Get fully resolved name of local node
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s")#, message.data)
    global odom
    
    odom=message
    
    #print("callback executed")
    #print(odom.pose)








def go_to_point(goal):
    goal_flag=0
    position=odom.pose.pose.position
    angles=odom.pose.pose.orientation
    angles_euler=quaternion_to_euler(angles.x,angles.y,angles.z,angles.w)
    #print(np.sqrt(np.square(goal[0]-position.x)+np.square(goal[1]-position.y)))
    goal_k=0.015
    
    while np.sqrt(np.square(goal[0]-position.x)+np.square(goal[1]-position.y))>0.05:
        
    	position_list.append([np.sqrt(np.square(goal[0]-position.x)+np.square(goal[1]-position.y)),rospy.get_rostime().secs])
        position=odom.pose.pose.position
        angles=odom.pose.pose.orientation
        angles_euler=quaternion_to_euler(angles.x,angles.y,angles.z,angles.w)
        #print("gotowhile")
        #print(angles_euler[0])
        rospy.sleep(0.01)
        angles_euler[0]=angles_euler[0]+180
        goal_angle=180.0/3.142*np.arctan2(goal[1]-position.y,goal[0]-position.x)
        goal_angle=goal_angle+180

        
        if abs(goal_angle-angles_euler[0])>250:
            goal_k=-0.001
        else:
            goal_k=0.015
        velocity_msg_=Twist()#;plt.plot(rospy.get_rostime().secs,goal_angle,'ro');plt.plot(rospy.get_rostime().secs,angles_euler[0],'bo')
        #plt.plot(angle_list[0:len(angle_list),1],'bo')
        print([goal_angle,angles_euler[0]])#,position.x,position.y,angles_euler[0]])
        #angle_list.append([angles_euler,goal_angle])
        #print(np.sqrt(np.square(goal[0]-position.x)+np.square(goal[0]-position.y)))
        #print([velocity_msg_.linear.x,velocity_msg_.angular.z])
        
        if abs(goal_angle-angles_euler[0])>35    and    abs(goal_angle-angles_euler[0])<305 :
            
            while abs(goal_angle-angles_euler[0])>3:
                position=odom.pose.pose.position
                angles=odom.pose.pose.orientation
                angles_euler=quaternion_to_euler(angles.x,angles.y,angles.z,angles.w)
                #print("gotowhile")
                angles_euler[0]=angles_euler[0]+180
                goal_angle=180.0/3.142*np.arctan2(goal[1]-position.y,goal[0]-position.x)
                goal_angle=goal_angle+180
                
                if abs(goal_angle-angles_euler[0])>250:
                    goal_k=-0.001
                else:
                    goal_k=0.015
                velocity_msg_.angular.z=(goal_angle-angles_euler[0])*goal_k

                #if abs(velocity_msg_.angular.z)>0.5:
                    #velocity_msg_.angular.z=0.5*abs(velocity_msg_.angular.z)/velocity_msg_.angular.z
                
                velocity_msg_.linear.x=0
                velocity_publisher.publish(velocity_msg_)
                rospy.sleep(0.01)
                angle_list.append([goal_angle,angles_euler[0],rospy.get_rostime().secs])
            	#plt.plot(rospy.get_rostime().secs,goal_angle,'ro');plt.plot(rospy.get_rostime().secs,angles_euler[0],'bo')
				
                #print("Angle While  ",goal_angle,angles_euler[0])

        else:
            #velocity_msg_.angular.z=0

            speed=np.sqrt(np.square(goal[0]-position.x)+np.square(goal[0]-position.y))
            if speed>0.2:
                speed=0.2

            velocity_msg_.linear.x=speed
            if abs(goal_angle-angles_euler[0])>250:
                goal_k=-0.001
            else:
                goal_k=0.015
            velocity_msg_.angular.z=(goal_angle-angles_euler[0])*(goal_k)

            velocity_publisher.publish(velocity_msg_)
            rospy.sleep(0.01);angle_list.append([goal_angle,angles_euler[0],rospy.get_rostime().secs])#;plt.plot(rospy.get_rostime().secs,goal_angle,'ro');plt.plot(rospy.get_rostime().secs,angles_euler[0],'bo')
      		
        flag=0
        if np.sqrt(np.square(goal[0]-position.x)+np.square(goal[0]-position.y))<0.05:
            flag=1
        
        
        

    #return velocity_msg_




def talker():
    #create a new publisher. we specify the topic name, then type of message then the queue size
    


    #we need to initialize the node
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node 
    goal_list=RRT([550,9550],[9500,500],1500,1500,10,[10000,10000])
    #print(goal_list)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.init_node('moveto', anonymous=True)

    #set the loop rate
    print("Execution Started")
    velocity_msg=Twist()
    velocity_msg.linear.x=-0.0
    velocity_msg.linear.y=-0.0
    velocity_msg.angular.z=0.0

    rate = rospy.Rate(1) # 1hz
    
    #goal_list.reverse()
    #goal_list.pop(0)
    
    for p in range(len(goal_list)):
        goal_list[p][0]=-(goal_list[p][0]-5000.0)/1000.0
        goal_list[p][1]=(goal_list[p][1]-5000.0)/1000.0


    print(goal_list)
    #rospy.sleep(3)
    i=0
    j=0
    while not rospy.is_shutdown():

        if j>0:

            #if goal_list[j][0]<goal_list[j-1][0]:
            print(goal_list[j])
            go_to_point(goal_list[j])
            

        

        
        j=j+1

        velocity_publisher.publish(velocity_msg)
        if j==len(goal_list)-1:
            break
        rospy.sleep(0.1)
        i=i+1

if __name__ == '__main__':
    try:
        talker()
        #for i in range(len(angle_list)-2):
        	#plt.plot([ angle_list[i][2],angle_list[i+1][2] ],[angle_list[i][0],angle_list[i+1][0]],'r-')
        	#plt.plot([ angle_list[i][2],angle_list[i+1][2] ],[angle_list[i][1],angle_list[i+1][1]],'b-')
        for i in range(len(position_list)-2):
        	plt.plot([position_list[i][1],position_list[i+1][1]],[position_list[i][0],position_list[i+1][0]],'r-')
        plt.show()
        #plt.plot(angle_list[0:len(angle_list),0],'ro')
        #plt.plot(angle_list[0:len(angle_list),1],'bo')
    except rospy.ROSInterruptException:
        velocity_msg=Twist()
        velocity_msg.linear.x=0.0
        velocity_publisher.publish(velocity_msg)

        pass
