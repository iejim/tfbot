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

    LedsComms st(nombreNodo_base, "1");

    st.init(argc, argv);

    // st.prepFrecuenciaHz(1.0);

    st.correr();

    std::cout << "Finalizado" <<std::endl;
    return 0;
}