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

pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
move_cmd = Twist()

speed_max = 0.5
omega_max = 2.5

#linear_vel:(float)
#angular_vel:(float)
#lateral_velocity:(float)
#steering_angle:(float)

def callback_laser(laser, odometry):
    print("Inside Callback Laser,Odometry: ")
    global omega_max
    global speed_max
    
    a=180/15                                                   
                                                       
    b=laser.ranges[14] 
    c=laser.ranges[15]
    
    theta=arctan((b*cos(a)-c)/(sin(a)*b))

    move_cmd.angular.z=-theta
    move_cmd.linear.x=0.1
    
    if c>0,2:
        move_cmd.angular.z=a
        move_cmd.linear.x=0.1
        if b=0,2:
            move_cmd.angular.z=-a
            move_cmd.linear.x=0.1
    if c<0,2:
        move_cmd.angular.z=-a
        move_cmd.linear.x=0.1
  
        
    #pub.publish(move_cmd) 
    
    # vitesse lineaire et angulaire du robot
    #move_cmd.linear.x = 0.1  #versionner tout le paquet et pas uniquement le script (c'est a dire les fichiers CMakeLists.txt, package.xml et les repertoire).
    #move_cmd.angular.z = - theta
    
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
    #command_sub = message_filters.Subscriber("/cmd_vel",Twist)
    
       
    ts = message_filters.ApproximateTimeSynchronizer([laser_sub, odometry_sub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(callback_laser)
       
    #ts = message_filters.ApproximateTimeSynchronizer([command_sub], 10, 0.1, allow_headerless=True)
    #ts.registerCallback(callback_command_seul)

    print("C'est parti... ")
    rospy.spin()

