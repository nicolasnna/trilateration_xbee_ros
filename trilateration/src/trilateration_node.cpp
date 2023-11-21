#include "trilateration.cpp"

int main(int argc, char **argv){
    // Inicilizar nodo
    ros::init(argc,argv, "Trilateration");
    ros::NodeHandle nh;
    Trilateracion Trilateracion(&nh);
    // Frecuencia
    ros::Rate freq(30);

    while(ros::ok()){
        
        //Trilateracion.PublicarMensaje();

        ros::spinOnce();
        freq.sleep();   
    }
}