#! /usr/bin/env python

import math
import rospy
import numpy
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

# sens 0 : suivi mur de gauche
sens = 1

#linear_vel:(float)
#angular_vel:(float)
#lateral_velocity:(float)
#steering_angle:(float)

def callback_laser(laser, odometry):
    print("Inside Callback Laser,Odometry: ")
    global omega_max
    global speed_max
    
    a=3.14/15.0                                                   
                       
    if sens:                       
        b=laser.ranges[1] 
        c=laser.ranges[0]        
    else:    
        b=laser.ranges[14] 
        c=laser.ranges[15]
    
    
    print("b ",b)
    print("c " , c)
    
    theta=(numpy.arctan2((b*numpy.cos(a)-c),(numpy.sin(a)*b)))
    
    print ("theta " , theta)
    
    move_cmd.linear.x=0.3
    distance_mur = 0.5
    
    if sens:
        theta_desire = -10.0 * (c - distance_mur)
    else:    
        theta_desire = -10.0 * (c - distance_mur)
    
    print ("theta_desire " , theta_desire)
    
    seuil_angle = 0.5
    if (theta_desire > seuil_angle):
        theta_desire = seuil_angle
    if (theta_desire < -seuil_angle):
        theta_desire = -seuil_angle    
    
    if sens:
        move_cmd.angular.z= -0.6*(theta - theta_desire)
    else:
        move_cmd.angular.z= 0.6*(theta - theta_desire)
    
    print ("laser.ranges[7] " , laser.ranges[7])
    
    # sens 0 : suivi mur de gauche
    sens = 1
    
    for i in range(3,10):
        if (laser.ranges[i] < 0.5 and laser.ranges[i] > 0.01 ):
            move_cmd.linear.x = 0.0
            if sens:
                move_cmd.angular.z = 0.5
            else:
                move_cmd.angular.z = -0.5
                
    if b > 1.0 and c > 1.0:
            move_cmd.linear.x = 0.0
            if sens:
                move_cmd.angular.z = -0.5
            else:
                move_cmd.angular.z = 0.5
    

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
