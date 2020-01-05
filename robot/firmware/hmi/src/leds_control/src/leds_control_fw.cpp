#include "leds_control_fw.hpp"

LedsControl::LedsControl(sstring nombre, sstring id, sstring ns)
{

  nombreNodo = nombre + "_" + id;
  idNodo = id;
  this->ns = ns;
  nh = nullptr;


}


LedsControl::~LedsControl()
{
  // Envía un 0% a los motores

  if(inicializado)
  {
    //Iterar entre los leds y apagar
    std::map<rc_led_t, int>::iterator it;
    for (it = estados.begin(); it != estados.end(); it++)
    {
        if (it.second){
          apagarLed(it.first);
        }
    }
    estados.clear();
  }

  rc_led_cleanup();
  inicializado = false;
}
 

void LedsControl::inicializar()
{
  // Empezar encendido
  ros::ServiceServer service = nh->advertiseService(topico, procesar);

  inicializado = true;
}

bool LedsControl::procesar(req, res)
{
  // Extraer led del mensaje y atender
  try
  {
    led = nombres[req.led]; // que pasa si no esta disponible
  }
  catch(const std::out_of_range& e)
  {
    ROS_ERROR("LED no disponible %s", req.led);
    return false;
  }
  
  int val = req.val;

  if (val<0){
    res.ret = cambiarLed(led);
    return true;
  }
  // TODO: Eso solo prende o apaga, no puede hacer mas nada
  res.ret = val ? encenderLed(led) : apagarLed(led);
  return true;
}


void LedsControl::correr()
{
  ROS_INFO("Corriendo");

  if (!inicializado)
  {
    inicializar();
  }
  

  if (inicializado)
  {
    ros::spin();
  }

  ROS_INFO("Detenido");
  
}
/*
void LedsControl::ejecutarPeriodico()
{


}*/

// void LedsControl::emergencyCallback(const ros::WallTimerEvent& evnt) 
// {
//   // Si no hay una comunicacion, y han pasado suficientes ciclos, grita.
//   if (numFaltas > SEC_NO_CONTROL*ESPERA_HZ)
//   {
//     ROS_WARN("Se perdio la comunicacion.");
//     encenderLed();
//     contador = 0; //Para reiniciar?
//   }
// }


int LedsControl::apagarLed(l)
{
  int s = rc_led_set(l, 0);
  estados[l] = s;
  return s; // Para?
}

int LedsControl::encenderLed(l)
{
  int s = rc_led_set(l, 1);
  estados[l] = s;
  return s; // Para?
}

int LedsControl::cambiarLed(l)
{
  if (estados[l])
    return apagarLed(l);

  return encenderLed(l);
}
