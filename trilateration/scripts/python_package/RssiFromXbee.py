import rospy
from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice
from digi.xbee.exception import XBeeException
from digi.xbee.models.address import XBee64BitAddress
from digi.xbee.util.utils import bytes_to_int
from trilateration_msg.msg import rssi_data

class RssiFromXbee():
    """
    Obtener mediciones de rssi de xbee remotos y enviarlos por ros
    """
    def __init__(self):
        """
        Configuración inicial nodo
        """
        rospy.init_node('get_rssi_xbee', anonymous=True)
        # Definir el publicador
        self.pub_remoto_ = rospy.Publisher('/trilateration/rssi_remote', rssi_data, queue_size=5)
        # Crear variable con el formato del mensaje
        self.msg = rssi_data()

    def __ObtenerParametros(self):
        """
        Obtener parametros del launch
        """
        # Parámetros
        self.puerto_ = rospy.get_param('rssi_from_xbee/port',"/dev/ttyUSB0")
        self.baudios_ = rospy.get_param('rssi_from_xbee/baudios','9600')
        self.remoto1_mac_ = rospy.get_param('rssi_from_xbee/remoto1_mac')
        self.remoto2_mac_ = rospy.get_param('rssi_from_xbee/remoto2_mac')
        self.remoto3_mac_ = rospy.get_param('rssi_from_xbee/remoto3_mac')

    def ConexionModulos(self):
        """
        Conexión con el modulo coordinador y remotos
        """
        self.__ObtenerParametros()

        self.coordinador = XBeeDevice(self.puerto_,self.baudios_)
        self.coordinador.set_sync_ops_timeout(10)
        self.coordinador.open()
        self.remoto1 = RemoteXBeeDevice(self.coordinador,XBee64BitAddress.from_hex_string(self.remoto1_mac_))
        self.remoto2 = RemoteXBeeDevice(self.coordinador,XBee64BitAddress.from_hex_string(self.remoto2_mac_))
        self.remoto3 = RemoteXBeeDevice(self.coordinador,XBee64BitAddress.from_hex_string(self.remoto3_mac_))
        # Obtener identificadores
        self.msg.id_remote1 = self.remoto1.get_parameter("NI").decode('utf-8')
        self.msg.id_remote2 = self.remoto2.get_parameter("NI").decode('utf-8')
        self.msg.id_remote3 = self.remoto3.get_parameter("NI").decode('utf-8')
        

    def ObtenerDatos(self):
        """
        Obtención de valores rssi y nombres ID de los módulos
        """
        # Obtener medición RSSI de remotos
        self.rssi_remoto1 = float(-bytes_to_int(self.remoto1.get_parameter("DB")))
        self.rssi_remoto2 = float(-bytes_to_int(self.remoto2.get_parameter("DB")))
        self.rssi_remoto3 = float(-bytes_to_int(self.remoto3.get_parameter("DB")))
        
    def EnviarMsg(self):
        """
        Preparar mensaje y enviarlo
        """
        self.msg.rssi_remote1 = self.rssi_remoto1
        self.msg.rssi_remote2 = self.rssi_remoto2
        self.msg.rssi_remote3 = self.rssi_remoto3

        self.pub_remoto_.publish(self.msg)

