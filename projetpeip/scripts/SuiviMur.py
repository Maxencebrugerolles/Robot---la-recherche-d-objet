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

    
def callback_laser(laser, odometry):
    print("Inside Callback Laser,Odometry: ")
    global omega_max
    global speed_max
    
    d=5                                                   #distance acceptable pour juger que la voie est libre
    
                                                            #on definit les extremums des lasers du lidar ainsi que le milieu
    a=laser.ranges[0] 
    b=laser.ranges[8]
    c=laser.ranges[15]
    
                                                            #On cree une boucle logique pour que le robot sache ou se deplacer 
    while a<100 and b<100 and c<100:
        if a>=d:                                            #On verifie que le robot a la place de se deplacer a droite 
            move_cmd.angular.z=-1,5708                        #                                  
            move_cmd.angular.z=0                              # si oui
            move_cmd.linear.x=0.1                             # 
            move_cmd.linear.x=0   
            a=laser.ranges[0] 
            b=laser.ranges[8]
            c=laser.ranges[15]
        else:                                                 #si non
            if b>=d:                                            #On verifie que le robot a la place de se deplacer au centre
                move_cmd.linear.x=0.1                              #si oui
                move_cmd.linear.x=0
                a=laser.ranges[0] 
                b=laser.ranges[8]
                c=laser.ranges[15]
            else:                                                  #si non
                if a>=d:                                              #On verifie que le robot a la place de se deplacer a gauche
                    move_cmd.angular.z=1,5708                         #
                    move_cmd.angular.z=0                              #
                    move_cmd.linear.x=0.1                             #si oui
                    move_cmd.linear.x=0
                    a=laser.ranges[0] 
                    b=laser.ranges[8]
                    c=laser.ranges[15]
                else:                                                 #si non on fait un demi tour
                    move_cmd.angular.z=3,141592 
                    move_cmd.angular.z=0
                    a=laser.ranges[0] 
                    b=laser.ranges[8]
                    c=laser.ranges[15]
                    
        a=laser.ranges[0] 
        b=laser.ranges[8]
        c=laser.ranges[15]
        
        
        
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

