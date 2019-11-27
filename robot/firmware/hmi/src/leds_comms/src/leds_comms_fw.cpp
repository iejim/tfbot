#include "leds_comms_fw.hpp"

LedsComms::LedsComms(sstring nombre, sstring id, sstring ns) : 
NodoTF(nombre, id, ns)
{
    //valores por defecto
    contador = 0;
    encendido = 0;
    led = RC_LED_GREEN; // Pudiera ser un parametro
    

}


LedsComms::~LedsComms()
{
  // Envía un 0% a los motores

  if(inicializado)
  {
    
  }
  rc_led_set(led, 0); // dejar apagado
  rc_led_cleanup();
  inicializado = false;
}


void LedsComms::inicializar()
{
  // Empezar encendido
  if(encenderLed()==-1){
    ROS_ERROR("ERROR: No se pudo setear el LED");
    inicializado = false;
    return;
  }
  numFaltas = 0; 
  encendido = 1;
  prepFrecuenciaHz(ESPERA_HZ);

    inicializado = true;
}

void LedsComms::ejecutarPeriodico()
{
    // Contar para cambiar el estado del LED
    // Solo si hay comunicación
    if (!numFaltas) //Esta llegando info
    {
        contador++;
    } else
    {
      contador =0;
      encenderLed();
    }
    if (contador == CICLOS_CONTADOR)
    {
      cambiarLed();
      contador=0;
    }
    numFaltas++; 

}

void LedsComms::emergencyCallback(const ros::WallTimerEvent& evnt) 
{
  // Si no hay una comunicacion, y han pasado suficientes ciclos, grita.
  if (numFaltas > SEC_NO_CONTROL*ESPERA_HZ)
    ROS_WARN("Se perdio la comunicacion.");
    encenderLed();

}


int LedsComms::apagarLed()
{
  int s = rc_led_set(led, 0);
  if (!s)
    encendido = 0;
  return s; // Para?
}

int LedsComms::encenderLed()
{
  int s = rc_led_set(led, 1);
  if (!s)
    encendido = 1;
  return s; // Para?
}

int LedsComms::cambiarLed()
{
  if (encendido)
    return apagarLed();

  return encenderLed();
}