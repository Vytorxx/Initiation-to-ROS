#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
#esta ya la habíamos incluido

def pose_callback(msg):
    #rospy.loginfo(msg)
    rospy.loginfo("(" + str(msg.x) + " , " + str(msg.y) + ")")

if __name__ == '__main__':
    rospy.init_node("turtle_pose_subscriber")
    
    sub = rospy.Subscriber("/turtle1/pose", Pose, callback=pose_callback)
    #pongo el callback sin ()
    rospy.loginfo("Node has been started")

    rospy.spin()#para que el nodo continúe trabajando y 
                #recibir y procesar todos los mensajes en el callback