#include "servos_fw.hpp"
#include "ros/ros.h"

/**
 * Nodo de interfaz con los un driver de motores conectado a un puerto de PWM.
 * 
 * El nodo permite interactuar con un servo en un puerto PWM especificado
 * en como argumento del Nodo, que tambien es usado como identificador del nodo.
 * 
 */

std::string nombreNodo_base = "Servos_fw";
// std::string topicoPub_base = "motor_req";


int main(int argc, char *argv[])
{
    /* code */

    Servos sv(nombreNodo_base, "1");

    sv.init(argc, argv);

    // st.prepFrecuenciaHz(1.0);

    sv.correr();

    std::cout << "Finalizado" <<std::endl;
    return 0;
}



