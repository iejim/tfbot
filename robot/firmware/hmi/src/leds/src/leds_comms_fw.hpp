#ifndef LEDS_COMMS_FW_H
#define LEDS_COMMS_FW_H

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <string>
#include <sstream>
#include "nodo_tf.hpp"

extern "C" 
{
  #include "rc/led.h"
}
/**
 * La clase LedsComms se usa para interactuar
 * con el driver de los motores a través de la 
 * interfaz PWM del BBBl. Estaría usando la 
 * librería Servo de robotcontrol para elegir
 * el pin (puerto), el tiempo de ciclo y
 * el encendido.
 * 
 * Este nodo recibe un comando de velocidad (potencia)
 * para los motores desde el nodo de Teleop
 * y lo pasa al driver, controlando el cambio para
 * evitar inestabilidad.
 * 
 * http://strawsondesign.com/docs/librobotcontrol/group___servo.html
 * 
 */

#define CICLOS_CONTADOR 3

class LedsComms : public NodoTF {

  private:
        // int contador;
        int encendido;
        rc_led_t led;

  
  public:
    LedsComms(sstring nombre, sstring id, sstring ns = "");

    ~LedsComms();


    // Para override
    void inicializar() override;

    // Para override  
    // void correr();
    void ejecutarPeriodico() override;

    void emergencyCallback(const ros::WallTimerEvent& evnt) override;

    int encenderLed();

    int apagarLed();
    
    int cambiarLed();

    /**
     * La idea es:
      ----- en NodoTF
     * Se registra como nodo de ROS
     *    Se suscribe al topico de control global
          Empieza el timer de Emergencia
      ----- Fin NodoTF
     * Se suscribe al topico de control de motores
     * Se anuncia en el topico de estado de motores
     * Se suscribe al topico (parametro?) de configuracion de motores (NO AHORA)
     * Inicializa el puerto
     * Al recibir un mensaje de comando:
     *    Lo convierte a un tiempo
     *    Lo pasa al pin
     *    Lo publica en el estado
     * 
     * En una version fancy: el comando es filtrado para estabilidad.
     */
};

#endif //LEDS_COMMS_FW_H