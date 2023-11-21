#ifndef TRILATERATION_TRILATERATION_H_
#define TRILATERATION_TRILATERATION_H_

#include <ros/ros.h>
#include <trilateration_msg/distance_data.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <math.h>
#include <ros/console.h>

class Trilateracion{
    private:
        // Subscriptor y publicador
        ros::Publisher pub_tri;
        ros::Subscriber sub_remoto;
        // Callback al recibir datos
        void DistanciaRemotoRecibido(const trilateration_msg::distance_data &);      
        void CalcularTrilateracion();  
        bool calculo_correcto;

    public:
        // Constructor y destructor
        Trilateracion(ros::NodeHandle *nh);
        ~Trilateracion();
        // Publicar mensaje
        void PublicarMensaje();
        // Variables
        float pos_x,pos_y,pos_z;
        float x_remoto1, y_remoto1, z_remoto1;
        float x_remoto2, y_remoto2, z_remoto2;
        float x_remoto3, y_remoto3, z_remoto3;
        float z_robot;
        float dis_remoto1, dis_remoto2, dis_remoto3;
        float x_offset, y_offset, z_offset, tolerancia_z_dis;
        std::string frame_id;
};

#endif