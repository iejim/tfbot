
#ifndef SABERTOOTH_FW_H
#define SABERTOOTH_FW_H

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <string>
#include <sstream>
#include "nodo_tf.hpp"

extern "C" 
{
  #include "rc/servo.h"
}
/**
 * La clase SaberTooth se usa para interactuar
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

class SaberTooth : public NodoTF {

  private:
        int canalFwd; //Canal 1 (S1)
        int canalTurn; //Canal 2 (S2)
        int usMax;
        int usMin;
        int usOff;

        bool inicializado;

        sstring topicoComandos;
        sstring topicoAnuncio; 


  public:
    SaberTooth(sstring nombre, sstring id, sstring ns = "");

    ~SaberTooth();


    void inicializar();

  
    void correr();
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
     * Conviente un comando de porcentaje de power a tiempo.
     */
    
    int convertirComando(float cmd);


    void anunciarComando(int ch1_cmd, int ch2_cmd);

    /**
     * Envía un comando al sabertooth
     */
    void enviarComando(int ch1_cmd, int ch2_cmd);

    /**
     * Callback para recibir comandos desde el nodo de Control.
     */
    void comandoCallback(tfbot_msgs::drivetrain msg);



    /**
     * La idea es:
     * Se registra como nodo de ROS
     *    Se suscribe al topico de control global
     * Se suscribe al topico de control de motores
     * Se anuncia en el topico de estado de motores
     * Se suscribe al topico (parametro?) de configuracion de motores
     * Inicializa el puerto
     * Al recibir un mensaje de comando:
     *    Lo convierte a un tiempo
     *    Lo pasa al pin
     *    Lo publica en el estado
     * 
     * En una version fancy: el comando es filtrado para estabilidad.
     */
};

#endif //SABERTOOTH_FW_H