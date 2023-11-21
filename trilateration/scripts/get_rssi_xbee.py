#!/usr/bin/env
# Conexión y obtención de medición RSSI de cada Xbee remoto
# Utiliza previamente "sudo usermod -a -G dialout ${USUARIO}"

from python_package.RssiFromXbee import *

rssi_from_xbee = RssiFromXbee()

if __name__ == '__main__':
    try: 
        rssi_from_xbee.ConexionModulos()
        # Velocidad de publicación
        rate = rospy.Rate(30)

        # Mientras ROS este activo
        while not rospy.is_shutdown():
            rssi_from_xbee.ObtenerDatos()
            rssi_from_xbee.EnviarMsg()

            rate.sleep()
    
    except XBeeException as e:
        rospy.logerr(f"Error en la conexión con los módulos Xbee: {e}")

    except rospy.ROSInterruptException:
        pass

