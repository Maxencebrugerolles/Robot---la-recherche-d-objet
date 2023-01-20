#! /usr/bin/env python

import math
import rospy
import message_filters
from sensor_msgs.msg import LaserScan
#from jaguar.msg import OdometryNative
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf
#import geometry_msgs
from numpy import pi, arange, sin, cos, sqrt, arcsin, arctan2, linspace
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

pub = rospy.Publisher('/auto_cmd_vel', Twist, queue_size = 1)
move_cmd = Twist()

speed_max = 0.5
omega_max = 2.5

    
def callback_laser(laser, odometry):
    print("Inside Callback Laser,Odometry: ")
    global omega_max
    global speed_max
    
    #  code a mettre
    
    # localisation du mur  a partir du laser
    # laser.ranges[0] correspond a la distance suivant un angle de -1.5 radian
    # laser.ranges[1] correspond a la distance suivant un angle de -1.3 radian
    # ...
    # laser.ranges[14] correspond a la distance suivant un angle de 1.3 radian
    # laser.ranges[15] correspond a la distance suivant un angle de 1.5 radian
    
    a=laser.range[0]
    b=laser.range[1]
    c=laser.range[2]
    d=laser.range[3]
    e=laser.range[4]
    f=laser.range[5]
    g=laser.range[6]
    h=laser.range[7]
    i=laser.range[8]
    j=laser.range[9]
    k=laser.range[10]
    l=laser.range[11]
    m=laser.range[12]
    n=laser.range[13]
    o=laser.range[14]
    p=laser.range[15]
    
    y=min(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p)
    return y

    if y==a:
        move_cmd.angular.z=-1.5
    elif y==b:
        move_cmd.angular.z=-1.3
    elif y==c:
        move_cmd.angular.z=-1.1
    elif y==d:
        move_cmd.angular.z=-0.9
    elif y==e:
        move_cmd.angular.z=-0.7
    elif y==f:
        move_cmd.angular.z=-0.5
    elif y==g:
        move_cmd.angular.z=-0.3
    elif y==h:
        move_cmd.angular.z=-0.1
    elif y==i:
        move_cmd.angular.z=0.1
    elif y==j:
        move_cmd.angular.z=0.3
    elif y==k:
        move_cmd.angular.z=0.5
    elif y==l:
        move_cmd.angular.z=0.7
    elif y==m:
        move_cmd.angular.z=0.9
    elif y==n:
        move_cmd.angular.z=1.1
    elif y==o:
        move_cmd.angular.z=1.3
    elif y==p:
        move_cmd.angular.z=1.5  
    pub.publish(move_cmd)
    
    move_cmd.linear.x = y
    pub.publish(move_cmd)
    
    
    
    # distance et orientation par rapport au mur
    theta = 0
    d = 0
    print(theta)
    print(d)    
    
    # vitesse lineaire et angulaire du robot
    move_cmd.linear.x = 0.1
    move_cmd.angular.z = - theta
    
    # on s'assure qu'on ne depasse pas les vitesses max
    if move_cmd.linear.x > speed_max:
        move_cmd.linear.x = speed_max
    if move_cmd.linear.x < -speed_max:
        move_cmd.linear.x = -speed_max

    if move_cmd.angular.z > omega_max:
        move_cmd.angular.z = omega_max
    if move_cmd.angular.z < -omega_max:
        move_cmd.angular.z = -omega_max
        
        
    pub.publish(move_cmd)
    
    

if __name__ == '__main__':
    
    rospy.init_node('SuiviMur', anonymous=True)
    
    speed_max = rospy.get_param('~speed_max', 0.5)
    omega_max = rospy.get_param('~omega_max', 2.5)
    
    laser_sub = message_filters.Subscriber("/simple_scan", LaserScan)
    odometry_sub = message_filters.Subscriber("/odom", Odometry)
    command_sub = message_filters.Subscriber("/manu_cmd_vel",Twist)
    
       
    ts = message_filters.ApproximateTimeSynchronizer([laser_sub, odometry_sub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(callback_laser)
       
    ts = message_filters.ApproximateTimeSynchronizer([command_sub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(callback_command_seul)

    print("C'est parti... ")
    rospy.spin()

