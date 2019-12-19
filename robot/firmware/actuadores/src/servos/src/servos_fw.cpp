#include "servos_fw.hpp"


Servos::Servos(sstring nombre, sstring id, sstring ns) : 
NodoTF(nombre, id, ns)
{
    //valores por defecto
    canal_min = 3;
    canal_max = 8;

    usMax = 2000;
    usMin = 1000;
    usOff = 1500;


    //TODO: No incluyen ningun indicio sobre cual driver controlan
    topicoComandos = "comando_servos";
    // topicoAnuncio = "comando_real_motores"; 
    
}


Servos::~Servos()
{
  // Envía un 0% a los motores

  if(inicializado)
  {
    enviarComando(usOff);
  }
  apagarRiel();
  rc_servo_cleanup();

  inicializado = false;
}

/**
 * Inicializar el nodo conectandose a los topicos a publicar.
 */
void Servos::inicializar()
{

  prepararServo();
  // Empezar apagado
  // if(rc_led_set(RC_LED_RED, 0)==-1){
  //   ROS_ERROR("ERROR: No se pudo setear el LED");
    inicializado = false;
  //   return;
  // }
  
  // Se suscribe a /comando_servo_cmd (Float32MultiArray) (debería venir de más arriba)
  ros::Subscriber sub = nh->subscribe<tfbot_msgs::servo_cmd, Servos>(topicoComandos, (uint32_t)2, &Servos::comandoCallback, this);
  agregarSub(sub);

  // ros::Publisher pub = nh->advertise<tfbot_msgs::servos_us>(topicoAnuncio, 5);
  // agregarPub(pub); //Se muere; deja de existir
  // listaPubs[topicoAnuncio] = nh->advertise<tfbot_msgs::servos_us>(topicoAnuncio, 5); //Deberia quedarse

  // rc_led_set(RC_LED_RED, 1);
  inicializado = true;

}



void Servos::prepararServo()
{
  ROS_INFO("Preparando Servos");

  if(rc_servo_init())
  {
    //error
    
    inicializado = false;
    //apagar nodo?
    if(nh->ok()) //Qué pasa si el nodo no ha sido inicializado?
    {
      apagarNodo();
    }
  }else 
  {
   ROS_INFO("Servo inicializado");
   encenderRiel();
  }
}


/**
 * Forzar apagado del power al riel de PWM.
 */
void  Servos::apagarRiel()
{
   rc_servo_power_rail_en(0);
}

void  Servos::encenderRiel()
{
   rc_servo_power_rail_en(1);
}

void  Servos::comandoCallback(tfbot_msgs::servo_cmd msg)
{
  //Extraer comandos
  // sstring nombre = msg.nombre;
  
  if (cmd.canal < canal_min || cmd.canal > canal_max)
    return;
  
  float cmd;
  
  cmd = msg.cmd;
  
  //Convertir comandos  
  // ROS_INFO("Llego");

  int val = convertirComando(cmd);
  //ROS_INFO("Enviando %d: %d; %d: %d", canalFwd, val1, canalTurn, val2);
  enviarComando(cmd.canal, val);

  

}

int Servos::convertirComando(float cmd)
{
    //Recibir un flotante [-100.0, 100.0]

    //Convertiro a los valores min y max
    static float umax = (float)usMax;
    static float umin = (float)usMin;

    float x = (cmd + 100.0f) * (umax - umin) / (100.0f - (-100.0f)) + umin;
    return (int)floorf(x);

}

void Servos::enviarComando(int canal_s, int us_cmd)
{
    rc_servo_send_pulse_us(canal_s, us_cmd);
    
    // anunciarComando(ch1_cmd, ch2_cmd);

}


/* void Servos::anunciarComando(int ch1_cmd, int ch2_cmd)
{
  tfbot_msgs::servos_us msg;

  msg.nombre = nombreNodo;

  msg.s1_fwd = ch1_cmd;
  msg.s2_turn = ch2_cmd;
  // ROS_INFO("Se va");
  listaPubs[topicoAnuncio].publish(msg);

} */


void Servos::emergencyCallback(const ros::WallTimerEvent& evnt) 
{
  // Si no hay una comunicacion, y han pasado suficientes ciclos, grita.
  if (numFaltas > SEC_NO_CONTROL*ESPERA_HZ)
  {
    ROS_WARN("Se perdio la comunicacion.");
    // rc_led_set(RC_LED_RED, 0);
  }
}
