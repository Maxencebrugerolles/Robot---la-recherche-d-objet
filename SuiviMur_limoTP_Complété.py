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

    d=0.2                                                   #distance acceptable pour juger que la voie est libre
    
                                                            #on définit les extremums des lasers du lidar ainsi que le milieu
    a=laser.range[0] 
    b=laser.range[8]
    c=laser.range[15]
    
                                                            #On créé une boucle logique pour que le robot sache où se déplacer 
    while a>0 && b>0 && c>0:
        if a>=d:                                            #On vérifie que le robot a la place de se déplacer à droite 
            move_cmd.angular.z=-1,5708                        #
            time.sleep(1000)                                  #
            move_cmd.angular.z=0                              # si oui
            move_cmd.linear.x=0.1                             #
            time.sleep(1000)                                  #
            move_cmd.linear.x=0                               #
        else:                                                 #si non
            if b>=d:                                            #On vérifie que le robot a la place de se déplacer au centre
                move_cmd.linear.x=0.1                              #
                time.sleep(1000)                                   #si oui
                move_cmd.linear.x=0                                #
            else:                                                  #si non
                if a>=d:                                              #On vérifie que le robot a la place de se déplacer à gauche
                    move_cmd.angular.z=1,5708                         #
                    time.sleep(1000)                                  #
                    move_cmd.angular.z=0                              #
                    move_cmd.linear.x=0.1                             #si oui
                    time.sleep(1000)                                  #
                    move_cmd.linear.x=0                               #
                else:                                                 #si non on fait un demi tour
                    move_cmd.angular.z=3,141592 
                    time.sleep(1000)
                    move_cmd.angular.z=0
                end
            end
        end
    end
        
        
        
    pub.publish(move_cmd)
    
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

