#include "leds_comms_fw.hpp"
#include "ros/ros.h"

/**
 * 
 * 
 */

std::string nombreNodo_base = "LED_Comms";

int main(int argc, char *argv[])
{
    /* code */

    LedsComms nodo(nombreNodo_base, "1");

    nodo.init(argc, argv);

    // st.prepFrecuenciaHz(1.0);

    nodo.correr();

    std::cout << "Finalizado" <<std::endl;
    return 0;
}