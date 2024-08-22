#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist #encontrado despues de rostopic list
#con rostopic info y el campo (/turtle1/cmd_vel)
#paquete que no incluí antes y lo hago ahora en package.xml


if __name__ == '__main__':
    rospy.init_node("draw_circle")
    rospy.loginfo("Node has been started")

    pub = rospy.Publisher("/turtle1/cmd_vel",Twist,queue_size=10)

    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        #publish cmd vel
        msg = Twist()#creo un objeto de la clase Twist
        msg.linear.x = 2.0
        #se mueve hacia delante en el eje x
        #lo de linear o angular se ve con rosmsg show geometry_msgs/Twist
        #con ese comando vemos que hay muchos otros campos posibles
        msg.angular.z = 1.0
        #todo lo anterior del bucle era cómo crear un mensaje
        #ya tenemos la carta escrita, falta enviarla:
        pub.publish(msg)
        rate.sleep()