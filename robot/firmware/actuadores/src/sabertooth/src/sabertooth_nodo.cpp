#include "sabertooth_fw.hpp"
#include "ros/ros.h"

/**
 * Nodo de interfaz con los un driver de motores conectado a un puerto de PWM.
 * 
 * El nodo permite interactuar con un Sabertooth en un puerto PWM especificado
 * en como argumento del Nodo, que tambien es usado como identificador del nodo.
 * 
 */

std::string nombreNodo_base = "Motor_fw";
std::string topicoPub_base = "motor_req";


int main(int argc, char *argv[])
{
    /* code */

    SaberTooth st(nombreNodo_base, "1");

    st.init(argc, argv);

    // st.prepFrecuenciaHz(1.0);

    st.correr();

    std::cout << "Finalizado" <<std::endl;
    return 0;
}



