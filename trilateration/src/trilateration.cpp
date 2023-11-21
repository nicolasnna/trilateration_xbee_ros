#include <trilateration/trilateration.h>

Trilateracion::Trilateracion(ros::NodeHandle *_nh){
    // Crear suscriptor
    sub_remoto = _nh->subscribe("/trilateration/data_remote", 1000, &Trilateracion::DistanciaRemotoRecibido, this);
    // Crear publicador
    pub_tri = _nh->advertise<geometry_msgs::PoseWithCovarianceStamped>("/trilateration/position", 1000);

    // Obtener parametros
    _nh->getParam("x_remoto1",x_remoto1);
    _nh->getParam("y_remoto1",y_remoto1);
    _nh->getParam("z_remoto1",z_remoto1);
    
    _nh->getParam("x_remoto2",x_remoto2);
    _nh->getParam("y_remoto2",y_remoto2);
    _nh->getParam("z_remoto2",z_remoto2);
    
    _nh->getParam("x_remoto3",x_remoto3);
    _nh->getParam("y_remoto3",y_remoto3);
    _nh->getParam("z_remoto3",z_remoto3);

    _nh->getParam("z_modulo_robot",z_robot);

    _nh->getParam("frame_id",frame_id);
    _nh->getParam("x_offset",x_offset);
    _nh->getParam("y_offset",y_offset);
    _nh->getParam("z_offset",z_offset);
    _nh->getParam("tolerancia_z_dis",tolerancia_z_dis);

}
Trilateracion::~Trilateracion(){
}


void Trilateracion::DistanciaRemotoRecibido(const trilateration_msg::distance_data &datos){
    // // Calcular posición por trilateración 2D
    // float A = 2*x_remoto2 - 2*x_remoto1;
    // float B = 2*y_remoto2 - 2*y_remoto1;
    // float C = pow(datos.distance_remote1,2) - pow(datos.distance_remote2,2) - pow(x_remoto1,2) + pow(x_remoto2,2) - pow(y_remoto1,2) + pow(y_remoto2,2);
    // float D = 2*x_remoto3 - 2*x_remoto2;
    // float E = 2*y_remoto3 - 2*y_remoto2;
    // float F = pow(datos.distance_remote2,2) - pow(datos.distance_remote3,2) - pow(x_remoto2,2) + pow(x_remoto3,2) - pow(y_remoto2,2) + pow(y_remoto3,2);

    // this->pos_x = (C*E - F*B) / (E*A - B*D);
    // this->pos_y = (C*D - A*F) / (B*D - A*E);
    dis_remoto1 = datos.distance_remote1;
    dis_remoto2 = datos.distance_remote2;
    dis_remoto3 = datos.distance_remote3;

    CalcularTrilateracion();

    if(calculo_correcto){
        PublicarMensaje();
    }
}

void Trilateracion::CalcularTrilateracion(){
    // Calcular por deducción tridimensional

    // Obtener promedio de z de los remotos y restarlo para tener un Z cercano a 0
    float z_prom = (z_remoto1 + z_remoto2 + z_remoto3)/3;

    pos_x = (pow(dis_remoto1,2) - pow(dis_remoto2,2) + pow(x_remoto2,2)) / (2*x_remoto2);
    pos_y = (pow(dis_remoto1,2) - pow(dis_remoto3,2) + pow(x_remoto3,2) + pow(y_remoto3,2))/(2 * y_remoto3) - (x_remoto3/y_remoto3)*pos_x;

    // Comprobar que es una solucion correcta con Z
    float discriminante_z = pow(dis_remoto1,2) - pow(pos_x,2) - pow(pos_y,2);
    if(discriminante_z >= tolerancia_z_dis){
        calculo_correcto = true;

        pos_z = sqrt(discriminante_z);
        if (z_robot < z_prom){
            pos_z = -pos_z;
        }
        // Calculo de la posición Z ajustando signo
        pos_z = pos_z + z_prom;

        ROS_INFO("Calculo correcto con coordenadas: (%.2f, %.2f, %.2f)", pos_x, pos_y, pos_z);
    } else {
        calculo_correcto = false;

        ROS_INFO("Calculo incorrecto obteniendo un discriminante de Z de %.2f", discriminante_z);
        ROS_INFO("Coordenadas incorrectas X e Y de %.2f y %.2f",pos_x,pos_y);
    }
}

void Trilateracion::PublicarMensaje(){
    if (isnan(this->pos_x) || isnan(this->pos_y)){
        pos_x = 0;
        pos_y = 0;
    };
    if (isnan(pos_z)){
        pos_z = 0;
    };
    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id;
    msg.pose.pose.position.x = pos_x-x_offset;
    msg.pose.pose.position.y = pos_y-y_offset;
    msg.pose.pose.position.z = pos_z-z_offset;
    msg.pose.pose.orientation.w = 0;
    msg.pose.pose.orientation.x = 0;
    msg.pose.pose.orientation.y = 0;
    msg.pose.pose.orientation.z = 0;

    msg.pose.covariance = { 0.3,   0,   0,   0,   0, 0,
                            0, 0.3,   0,   0,   0, 0,
                            0,   0, 0.3,   0,   0, 0,
                            0,   0,   0, 1e6,   0, 0,
                            0,   0,   0,   0, 1e6, 0,
                            0,   0,   0,   0,   0, 1e6};
    pub_tri.publish(msg);
}
