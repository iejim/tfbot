#include "leds_control_fw.hpp"
#include "ros/ros.h"

/**
 * 
 * 
 */

std::string nombreNodo_base = "LED_Control";

int main(int argc, char *argv[])
{
    /* code */

    LedsControl nodo(nombreNodo_base, "1");

    nodo.init(argc, argv);

    // st.prepFrecuenciaHz(1.0);

    nodo.correr();

    std::cout << "Finalizado" <<std::endl;
    return 0;
}