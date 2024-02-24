# Trilateración por Xbee en ROS

Paquete ROS para emplear localización por trilateración 2D a partir del RSSI, utilizando dispositivos Xbee S2C.

El paquete `trilateration` incorpora los nodos encargados de obtener las mediciones RSSI, estimaciones de distancia, filtro de mediciones y posicionamiento por trilateración.

Por otro lado, el paquete `trilateration_msgs` proporciona la forma de los tópicos con los que se comunican los nodos. 

## Requisitos

* ROS Noetic
* Python
* C++
* [Catkin Tools](https://catkin-tools.readthedocs.io/en/latest/)
* 4 Módulos Xbee S2C _(1 Coordinador y 3 Ruteadores)_


## Dependencias

* _[Digi Xbee Library](https://xbplib.readthedocs.io/en/latest/index.html)_

```
pip install digi-xbee 
```
* _Numpy_

```
pip install numpy
```

## Utilización

> [!NOTE]
> Previamente se debe obtener las direcciones MAC de los dispositivos Xbee y especificar el funcionamiento de estos en modo API. 

En el _roslaunch_ `get_rssi_xbee.launch` se especifica las direcciones MAC de cada dispositivo y puerto USB del nodo central.

Por otro lado, los parámetros para el cálculo de la trilateración y estimación de distancias se encuentran en el archivo `params_trilateration.yaml` ubicado en `trilateration/params`.

1. Compilar paquete ROS con Catkin Tools:

```
catkin build trilateration
```

2. Ejecutar el archivo `get_rssi_xbee.launch`:

```
roslaunch trilateration get_rssi_xbee.launch
```

Este lanzador ejecuta el nodo `rssi_from_xbee`, encargado de obtener las mediciones RSSI y publicarlas.

3. Ejecutar el archivo `get_distance_from_rssi`:

```
roslaunch trilateration get_distance_from_rssi.launch
```

El nodo `distance_from_rssi` se encarga de obtener las mediciones del RSSI, filtrarlas con Kalman y estimar las distancias entre cada nodo. Para ello es necesario medir el RSSI en 4 posiciones distintas conocidas: 

- Para los 3 primeros posiciones se debe elegir una ubicación cercana a cada nodo (ej: primer punto, posición cercana al nodo remoto 1).

- El cuarto punto se debe elegir una posición lejana de los tres módulos remotos.


4. Ejecutar el archivo `get_trilateration.launch`:

```
roslaunch trilateration get_trilateration.launch
```

Obtiene las distancias estimadas y calcula la posición del nodo Coordinador a partir de la trilateración. 

La posición se publica en un mensaje formato `geometry_msgs/PoseWithCovarianceStamped`.
