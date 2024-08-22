#!/usr/bin/env python3
import rospy #lo hemos incluido como dependencia para usar ROS funcionalities

if __name__ == '__main__':
    rospy.init_node("test_node") #inicializar el nodo que se encuentra dentro
                                 #del ejecutable my_first_node.py, Este será el nombre del nodo
    rospy.loginfo("Test node has been started") #va a printear algún log en la terminal
    """
    rospy.logwarn("This is a warning") #va a printear warnings
    rospy.logerr("This is an error") #va a a printear errores
    #son como especies de prints. Se utilizan para generar mensajes de log (registro de eventos) 
    #en diferentes niveles de severidad. Estos mensajes se imprimen en la terminal donde se está 
    #ejecutando el nodo y pueden ser redirigidos o almacenados según la configuración de ROS.
    Aparecerá algo así:
    [INFO] [Time]: Hello from test node
    [WARN] [Time]: This is a warning
    [ERROR] [Time]: This is an error
    
    rospy.sleep(1.0)#equivalente a time.sleep(1)
    rospy.loginfo("End of program")
    """
    rate = rospy.Rate(10) #crear un objeto de tipo Rate que controla la frecuencia 
                          #a la que se ejecuta un bucle en el nodo.
                          #en este caso 10 Hz = 10 veces/s = cada 0.1s
    while not rospy.is_shutdown(): #te dirá si el nodo ha recibido un shutdown request
                                   #= mientras no se destruya el nodo
        rospy.loginfo("Hello")
        rate.sleep() #Pausa el bucle el tiempo necesario para mantener la frecuencia establecida de 10 Hz.
