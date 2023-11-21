import rospy
from python_package.kalman_rssi import FiltroKalmanRSSI
import numpy as np
from trilateration_msg.msg import rssi_data, distance_data

class RssiToDistance():
    """
    Subscriptor que recibe los RSSI para calcular las distancias de los nodos y publicarlos
    """
    def __init__(self):
        """
        Configuración inicial el nodo
        """
        rospy.init_node('get_distance_from_rssi',anonymous=True)

        rospy.Subscriber('/trilateration/rssi_remote', rssi_data, self.__RssiCallback)
        # Nodo publicador
        self.__pub_distance = rospy.Publisher('/trilateration/data_remote', distance_data, queue_size=5) 

        self.__AjustarParametros()
        if self._mostrar_sin_kalman == True:
            self.__pub_distance_wfilter = rospy.Publisher('/trilateration/data_remote_no_filtering', distance_data, queue_size=5)

        self.rssi0_remoto1 = ([])
        self.rssi0_remoto2 = ([])
        self.rssi0_remoto3 = ([])

        self.rssi1_remoto1 = ([])
        self.rssi1_remoto2 = ([])
        self.rssi1_remoto3 = ([])

        self.ejecutar_estimacion = False

    def __RssiCallback(self, data):
        """
        Almacenar los datos recibidos
        """
        self.id_remote1 = data.id_remote1
        self.rssi_remote1 = data.rssi_remote1
        self.id_remote2 = data.id_remote2
        self.rssi_remote2 = data.rssi_remote2
        self.id_remote3 = data.id_remote3
        self.rssi_remote3 = data.rssi_remote3

        if self.ejecutar_estimacion == True:
            self.PublicarDatosRemotos()

    def AlmacenarArreglo(self, estado):
        """
        Almacenar en arreglo de acuerdo al estado
        : 0 = rssi0
        : 1 = rssi1
        """
        data = rospy.wait_for_message('/trilateration/rssi_remote', rssi_data,timeout=2)
        if estado == 0:
            self.rssi0_remoto1.append(data.rssi_remote1)
        elif estado == 1:
            self.rssi0_remoto2.append(data.rssi_remote2)
        elif estado == 2:
            self.rssi0_remoto3.append(data.rssi_remote3)
        elif estado == 3:   
            self.rssi1_remoto1.append(data.rssi_remote1)
            self.rssi1_remoto2.append(data.rssi_remote2)
            self.rssi1_remoto3.append(data.rssi_remote3)

    def AjustarDistanciaInicial(self, x01, y01, x02, y02, x03, y03, x1, y1):
        """
        Calcular distancia a cada nodo remoto en referencia a las mediciones de configuración para ajustar parámetros
        """
        self.dis0_remoto1 = np.sqrt((self._x_remoto1 - x01)**2 + (self._y_remoto1 - y01)**2 + (self._z_remoto1 - self._z_robot)**2)
        self.dis0_remoto2 = np.sqrt((self._x_remoto2 - x02)**2 + (self._y_remoto2 - y02)**2 + (self._z_remoto2 - self._z_robot)**2)
        self.dis0_remoto3 = np.sqrt((self._x_remoto3 - x03)**2 + (self._y_remoto3 - y03)**2 + (self._z_remoto3 - self._z_robot)**2)

        self.dis1_remoto1 = np.sqrt((self._x_remoto1 - x1)**2 + (self._y_remoto1 - y1)**2 + (self._z_remoto1 - self._z_robot)**2)
        self.dis1_remoto2 = np.sqrt((self._x_remoto2 - x1)**2 + (self._y_remoto2 - y1)**2 + (self._z_remoto2 - self._z_robot)**2) 
        self.dis1_remoto3 = np.sqrt((self._x_remoto3 - x1)**2 + (self._y_remoto3 - y1)**2 + (self._z_remoto3 - self._z_robot)**2)

        # Obtener parámetros para cada nodo
        self.__ObtenerParametros()

    def __ObtenerParametros(self):
        """
        Calcular coeficiente n y establecer parámetros para estimar distancias
        """
        self.prom_rssi0_remoto1 = np.mean(self.rssi0_remoto1)
        self.prom_rssi0_remoto2 = np.mean(self.rssi0_remoto2)
        self.prom_rssi0_remoto3 = np.mean(self.rssi0_remoto3)

        prom_rssi1_remoto1 = np.mean(self.rssi1_remoto1)
        prom_rssi1_remoto2 = np.mean(self.rssi1_remoto2)
        prom_rssi1_remoto3 = np.mean(self.rssi1_remoto3)

        # Calcular coeficientes n
        self.n_remoto1 = -(prom_rssi1_remoto1 - self.prom_rssi0_remoto1)/(10*np.log10(self.dis1_remoto1/self.dis0_remoto1))
        self.n_remoto2 = -(prom_rssi1_remoto2 - self.prom_rssi0_remoto2)/(10*np.log10(self.dis1_remoto2/self.dis0_remoto2))
        self.n_remoto3 = -(prom_rssi1_remoto3 - self.prom_rssi0_remoto3)/(10*np.log10(self.dis1_remoto3/self.dis0_remoto3))

    def __CalcularDistancias(self):
        """
        Calcular distancia actual considerando filtro de kalman para cada nodo
        """
        rssi_remoto1_filtrado = self._kf_remoto1.filtrar(self.rssi_remote1)
        rssi_remoto2_filtrado = self._kf_remoto2.filtrar(self.rssi_remote2)
        rssi_remoto3_filtrado = self._kf_remoto3.filtrar(self.rssi_remote3)

        self.distancia_remoto1 = self.dis0_remoto1 * np.power(10,-(rssi_remoto1_filtrado - self.prom_rssi0_remoto1)/(10*self.n_remoto1))
        self.distancia_remoto2 = self.dis0_remoto2 * np.power(10,-(rssi_remoto2_filtrado - self.prom_rssi0_remoto2)/(10*self.n_remoto2))
        self.distancia_remoto3 = self.dis0_remoto3 * np.power(10,-(rssi_remoto3_filtrado - self.prom_rssi0_remoto3)/(10*self.n_remoto3))
    
    def __CalcularDistanciasSinFiltrar(self):
        """
        Calcular distancia actual sin considerar filtro de Kalman 
        """

        self.distancia_remoto1_sin_filtro = self.dis0_remoto1 * np.power(10,-(self.rssi_remote1 - self.prom_rssi0_remoto1)/(10*self.n_remoto1))
        self.distancia_remoto2_sin_filtro = self.dis0_remoto2 * np.power(10,-(self.rssi_remote2 - self.prom_rssi0_remoto2)/(10*self.n_remoto2))
        self.distancia_remoto3_sin_filtro = self.dis0_remoto3 * np.power(10,-(self.rssi_remote3 - self.prom_rssi0_remoto3)/(10*self.n_remoto3))

    def PublicarDatosRemotos(self):
        """
        Creación de mensaje y publicación de este
        """
        # Calcular distancias actuales
        self.__CalcularDistancias()
        # Creacion del mensaje
        mensaje = distance_data()
        mensaje.id_remote1 = self.id_remote1
        mensaje.distance_remote1 = self.distancia_remoto1
        mensaje.coef_remote1 = self.n_remoto1
        mensaje.distance0_prom_remote1 = self.dis0_remoto1
        mensaje.distance1_prom_remote1 = self.dis1_remoto1

        mensaje.id_remote2 = self.id_remote2
        mensaje.distance_remote2 = self.distancia_remoto2
        mensaje.coef_remote2 = self.n_remoto2
        mensaje.distance0_prom_remote2 = self.dis0_remoto2
        mensaje.distance1_prom_remote2 = self.dis1_remoto2

        mensaje.id_remote3 = self.id_remote3
        mensaje.distance_remote3 = self.distancia_remoto3
        mensaje.coef_remote3 = self.n_remoto3
        mensaje.distance0_prom_remote3 = self.dis0_remoto3
        mensaje.distance1_prom_remote3 = self.dis1_remoto3

        # En caso que se pida los datos sin filtro
        if self._mostrar_sin_kalman == True:
            self.__CalcularDistanciasSinFiltrar()

            mensaje_sin_filtro = distance_data()
            mensaje_sin_filtro.id_remote1 = self.id_remote1
            mensaje_sin_filtro.distance_remote1 = self.distancia_remoto1_sin_filtro
            mensaje_sin_filtro.coef_remote1 = self.n_remoto1
            mensaje_sin_filtro.distance0_prom_remote1 = self.dis0_remoto1
            mensaje_sin_filtro.distance1_prom_remote1 = self.dis1_remoto1

            mensaje_sin_filtro.id_remote2 = self.id_remote2
            mensaje_sin_filtro.distance_remote2 = self.distancia_remoto2_sin_filtro
            mensaje_sin_filtro.coef_remote2 = self.n_remoto2
            mensaje_sin_filtro.distance0_prom_remote2 = self.dis0_remoto2
            mensaje_sin_filtro.distance1_prom_remote2 = self.dis1_remoto2

            mensaje_sin_filtro.id_remote3 = self.id_remote3
            mensaje_sin_filtro.distance_remote3 = self.distancia_remoto3_sin_filtro
            mensaje_sin_filtro.coef_remote3 = self.n_remoto3
            mensaje_sin_filtro.distance0_prom_remote3 = self.dis0_remoto3
            mensaje_sin_filtro.distance1_prom_remote3 = self.dis1_remoto3

            self.__pub_distance_wfilter.publish(mensaje_sin_filtro)


        rospy.loginfo(mensaje)

        self.__pub_distance.publish(mensaje)

    def __AjustarParametros(self):
        """
        Establecer posiciones fijas de los nodos remotos
        """
        self._x_remoto1 = rospy.get_param('x_remoto1')
        self._y_remoto1 = rospy.get_param('y_remoto1')
        self._z_remoto1 = rospy.get_param('z_remoto1')

        self._x_remoto2 = rospy.get_param('x_remoto2')
        self._y_remoto2 = rospy.get_param('y_remoto2')
        self._z_remoto2 = rospy.get_param('z_remoto3')

        self._x_remoto3 = rospy.get_param('x_remoto3')
        self._y_remoto3 = rospy.get_param('y_remoto3')
        self._z_remoto3 = rospy.get_param('z_remoto3')

        self._z_robot = rospy.get_param("z_modulo_robot")
        self._mostrar_sin_kalman = rospy.get_param("mostrar_sin_kalman")

    def AjustarFiltroKalman(self):
        """
        Parámetros del filtro de Kalman
        """
        self._kf_remoto1 = FiltroKalmanRSSI(rospy.get_param("q_remoto1"), rospy.get_param("r_remoto1"))
        self._kf_remoto2 = FiltroKalmanRSSI(rospy.get_param("q_remoto2"), rospy.get_param("r_remoto2"))
        self._kf_remoto3 = FiltroKalmanRSSI(rospy.get_param("q_remoto3"), rospy.get_param("r_remoto3"))
