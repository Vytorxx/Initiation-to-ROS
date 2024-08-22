#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

def pose_callback(msg: Pose): 
    cmd = Twist()
    if msg.x > 9.0 or msg.x < 2.0 or msg.y >9.0 or msg.y < 2.0: #si está en la zona cercana
        cmd.linear.x= 1.0
        cmd.angular.z= 0.9
        #gira
    else: 
        cmd.linear.x = 5.0
        cmd.angular.z = 0.0
        #así va recto
    pub.publish(cmd)

if __name__ == '__main__':
    rospy.init_node("turtle_controller")
    #Creamos el publisher antes que el subscriber (tiene sentifo)
    pub = rospy.Publisher("/turtle1/cmd_vel",Twist,queue_size=10)
    sub = rospy.Subscriber("/turtle1/pose", Pose, callback=pose_callback)
    rospy.loginfo("Node has started")

    rospy.spin()