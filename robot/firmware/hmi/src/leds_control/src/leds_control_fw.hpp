#ifndef LEDS_CONTROL_FW_H
#define LEDS_CONTROL_FW_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tfbot_msgs/LED.h"

#include <string>
#include <sstream>
// #include "nodo_tf.hpp"

#define sstring std::string

extern "C" 
{
  #include "rc/led.h"
}
/**
 * Esta clase implementa un servicio que ofrece control sobre los LED
 * del BBBL a traves de ROS. El servicio permite encender o apagar
 * un LED, o hacerlo parpadear con un periodo determinado.
 * 
 * TODO: Seria buena idea tener un record de quien esta a cargo de
 * cada LED.
 * TODO: Se deberia revisar el estado de los LEDs en /sys y guardarlo
 * para recuperarlo al final; o limitar los LEDS controlables.
 *
 * http://strawsondesign.com/docs/librobotcontrol/group___l_e_d.html
 * 
 */

#define CICLOS_CONTADOR 3

class LedsControl {

  private:
        sstring nombreNodo;
        sstring idNodo;
        sstring ns;
        bool inicializado;
        ros::NodeHandlePtr nh;
        // rc_led_t led;

        //Mapa de LEDS (para traducir strings recibidos)
        std::map<sstring, rc_led_t> nombres = {
            { "GREEN", RC_LED_GREEN},
            { "RED", RC_LED_RED},
            { "USR0", RC_LED_USR0},
            // { "USR1", RC_LED_USR1},
            // { "USR2", RC_LED_USR2},
            // { "USR3", RC_LED_USR3},
            { "BAT25", RC_LED_BAT25},
            { "BAT50", RC_LED_BAT50},
            { "BAT75", RC_LED_BAT75},
            { "BAT100", RC_LED_BAT100},
            { "WIFI", RC_LED_WIFI}
        };

        //Mapa de estados de LEDs (¿para no revisar?)
        std::map<rc_led_t, int> estados;

        sstring topico = "control_leds";
  
  public:
    LedsControl(sstring nombre, sstring id, sstring ns = "");

    /* Se asegura de apagar los LEDS
     * aunque pudiera ser que los lleve a su estado original.
     */
    ~LedsControl();


    void init();

    void inicializar();

    //   
    void correr();
    
    // void ejecutarPeriodico();

    // void emergencyCallback(const ros::WallTimerEvent& evnt);

    bool procesar(tfbot_msgs::LED::Request &req, tfbot_msgs::LED::Response &res);
    
    int encenderLed(rc_led_t l;

    int apagarLed(rc_led_t l);
    
    int cambiarLed(rc_led_t l);

};

#endif //LEDS_CONTROL_FW_H