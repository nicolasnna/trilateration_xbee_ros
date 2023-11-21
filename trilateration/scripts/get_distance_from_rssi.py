#!/usr/bin/env
# Cálculo de la distancia entre los nodos desde mediciones RSSI

from python_package.rssi_to_distance import *


# Estado que indica si se realizó la configuración inicial
estado_config = 0

nodo_ros_distancia = RssiToDistance()

if __name__ == '__main__':
    try:
        # Ajuste parametros
        nodo_ros_distancia.AjustarFiltroKalman()

        # Primera medición para la configuración
        print("Posicione cerca del nodo 1")
        X01_REAL = float(input("Ingrese la coordenada X del nodo coordinador: "))
        Y01_REAL = float(input("Ingrese la coordenada Y del nodo coordinador: "))
        print("------------------------------")
        print("Obteniendo mediciones, espere...")
        print("------------------------------")
        for i in range(0,100):
            nodo_ros_distancia.AlmacenarArreglo(estado_config)
        estado_config = 1
        print("------------------------------")
        print("Mueva el nodo coordinador cerca del nodo 2")
        print("------------------------------")
        # Segunda medición para la configuración
        X02_REAL = float(input("Ingrese la nueva coordenada X del nodo coordinador: "))
        Y02_REAL = float(input("Ingrese la nueva coordenada Y del nodo coordinador: "))
        print("------------------------------")
        print("Obteniendo mediciones, espere...")
        print("------------------------------")
        for i in range(0,100):
            nodo_ros_distancia.AlmacenarArreglo(estado_config)
        estado_config = 2
        print("------------------------------")
        print("Mueva el nodo coordinador cerca del nodo 3")
        print("------------------------------")
        # tercera medición para la configuración
        X03_REAL = float(input("Ingrese la nueva coordenada X del nodo coordinador: "))
        Y03_REAL = float(input("Ingrese la nueva coordenada Y del nodo coordinador: "))
        print("------------------------------")
        print("Obteniendo mediciones, espere...")
        print("------------------------------")
        for i in range(0,100):
            nodo_ros_distancia.AlmacenarArreglo(estado_config)
        estado_config = 3
        print("------------------------------")
        print("Mueva el nodo coordinador lo más lejos posible de los tres nodos")
        print("------------------------------")
        # cuarta medición para la configuración
        X1_REAL = float(input("Ingrese la nueva coordenada X del nodo coordinador: "))
        Y1_REAL = float(input("Ingrese la nueva coordenada Y del nodo coordinador: "))
        print("------------------------------")
        print("Obteniendo mediciones, espere...")
        print("------------------------------")
        for i in range(0,100):
            nodo_ros_distancia.AlmacenarArreglo(estado_config)

        # Ajustes y calculos de coeficientes
        nodo_ros_distancia.AjustarDistanciaInicial(X01_REAL, Y01_REAL,X02_REAL, Y02_REAL,X03_REAL, Y03_REAL, X1_REAL, Y1_REAL)
        
        # Frecuencia de publicación
        rate = rospy.Rate(30)

        # Iniciar la estimación de distancia con los datos rssi que lleguen
        nodo_ros_distancia.ejecutar_estimacion = True

        while not rospy.is_shutdown():
            # Bucle mientras ROS funcione
            rate.sleep()

    except rospy.ROSInterruptException:
        pass