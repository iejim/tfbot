
#ifndef SERVOS_FW_H
#define SERVOS_FW_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tfbot_msgs/servo_cmd.h"

#include <string>
#include <sstream>
#include "nodo_tf.hpp"

extern "C" 
{
  #include "rc/servo.h"
  #include "rc/led.h"
}
/**
 * La clase Servos se usa para interactuar
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
 * https://www.generationrobots.com/media/Sabertooth2x12-userguide.pdf
 */

class Servos : public NodoTF {

  private:
        int canal_min;
        int canal_max;
        int usMax;
        int usMin;
        int usOff;

        

        sstring topicoComandos;
        // sstring topicoAnuncio; 


  public:
    Servos(sstring nombre, sstring id, sstring ns = "");

    ~Servos();


    // Para override
    void inicializar() override;

    // Para override  
    // void correr();

    // void calibrarMax();
    // void calibrarMin();
    
    /**
     * Prepara la salida usando librería Servo:
     * inicializacion
     * apagar el power del riel
     */
    void prepararServo();

    /**
     * Forzar apagado del power al riel de PWM.
     */
    void apagarRiel();
    
    /**
     * Encender el power al riel de PWM.
     */
    void encenderRiel();

    /**
     * Conviente un comando de porcentaje de power a tiempo.
     */
    
    int convertirComando(float cmd);


    // void anunciarComando(int ch1_cmd, int ch2_cmd);

    /**
     * Envía un comando al servos
     */
    void enviarComando(int canal_s, int us_cmd);

    /**
     * Callback para recibir comandos desde el nodo de Operacion.
     */
    void comandoCallback(tfbot_msgs::servo_cmd msg);

    void emergencyCallback(const ros::WallTimerEvent& evnt) override;


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

#endif //SERVOS_FW_H