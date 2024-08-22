#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen

previous_x = 0

def call_set_pen_service(r, g, b, width, off):
    #parámetros sacados de rossrv show <Type>
    try:
        set_pen = rospy.ServiceProxy("/turtle1/set_pen", SetPen)
        response = set_pen(r,g,b,width,off)
        #rospy.loginfo(response)
    except rospy.ServiceException as e:
        rospy.logwarn(e)

def pose_callback(msg: Pose): 
    cmd = Twist()
    if msg.x > 9.0 or msg.x < 2.0 or msg.y >9.0 or msg.y < 2.0: #si está en la zona cercana
        cmd.linear.x= 1.0
        cmd.angular.z= 1.4
        #gira
    else: 
        cmd.linear.x = 5.0
        cmd.angular.z = 0.0
        #así va recto

    global previous_x #para que el servicio no se llame continuamente,
    #ya que no están pensados para eso
    if msg.x < 5.5 and previous_x > 5.5:
        call_set_pen_service(0, 255, 0, 3, 0)
        rospy.loginfo("Set color to green")
    elif msg.x > 5.5 and previous_x < 5.5:
        call_set_pen_service(255, 0, 0, 3, 0)
        rospy.loginfo("Set color to red")

    previous_x = msg.x
    pub.publish(cmd)

if __name__ == '__main__':
    rospy.init_node("turtle_controller")

    rospy.wait_for_service("/turtle1/set_pen")#Bloquea programa hasta que servicio esté disponible
    #Creamos el publisher antes que el subscriber (tiene sentifo)
    pub = rospy.Publisher("/turtle1/cmd_vel",Twist,queue_size=10)
    sub = rospy.Subscriber("/turtle1/pose", Pose, callback=pose_callback)
    rospy.loginfo("Node has started")

    rospy.spin()