#include "sabertooth_fw.hpp"


SaberTooth::SaberTooth(sstring nombre, sstring id, sstring ns) : 
NodoTF(nombre, id, ns)
{
    //valores por defecto
    canalFwd = 1;
    canalTurn = 2;

    usMax = 2000;
    usMin = 1000;
    usOff = 1500;


    //TODO: No incluyen ningun indicio sobre cual driver controlan
    topicoComandos = "comando_drivetrain";
    topicoAnuncio = "comando_real_motores"; 
    
}


SaberTooth::~SaberTooth()
{
  // Envía un 0% a los motores

  if(inicializado)
  {
    enviarComando(usOff, usOff);
  }
  rc_led_set(RC_LED_RED, 0); // dejar apagado
  rc_led_cleanup();

  rc_servo_cleanup();

  inicializado = false;
}

/**
 * Inicializar el nodo conectandose a los topicos a publicar.
 */
void SaberTooth::inicializar()
{

  prepararServo();
  // Empezar apagado
  if(rc_led_set(RC_LED_RED, 0)==-1){
    ROS_ERROR("ERROR: No se pudo setear el LED");
    inicializado = false;
    return;
  }

  // Se suscribe a /comando_drivetrain (Float32MultiArray) (debería venir de más arriba)
  ros::Subscriber sub = nh->subscribe<tfbot_msgs::drivetrain, SaberTooth>(topicoComandos, (uint32_t)2, &SaberTooth::comandoCallback, this);
  agregarSub(sub);

  // ros::Publisher pub = nh->advertise<tfbot_msgs::sabertooth_us>(topicoAnuncio, 5);
  // agregarPub(pub); //Se muere; deja de existir
  listaPubs[topicoAnuncio] = nh->advertise<tfbot_msgs::sabertooth_us>(topicoAnuncio, 5); //Deberia quedarse

  rc_led_set(RC_LED_RED, 1);
  inicializado = true;

}



void SaberTooth::prepararServo()
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
   apagarRiel();
  }
}


/**
 * Forzar apagado del power al riel de PWM.
 */
void  SaberTooth::apagarRiel()
{
   rc_servo_power_rail_en(0);
}

void  SaberTooth::comandoCallback(tfbot_msgs::drivetrain msg)
{
  //Extraer comandos
  // sstring nombre = msg.nombre;
  
  float cmd1, cmd2;
  
  cmd1 = msg.cmd1_izq;
  cmd2 = msg.cmd2_der;
  
  //Convertir comandos  
  // ROS_INFO("Llego");

  int val1 = convertirComando(cmd1);
  int val2 = convertirComando(cmd2);
  //ROS_INFO("Enviando %d: %d; %d: %d", canalFwd, val1, canalTurn, val2);
  enviarComando(val1, val2);

  

}

int SaberTooth::convertirComando(float cmd)
{
    //Recibir un flotante [-100.0, 100.0]

    //Convertiro a los valores min y max
    static float umax = (float)usMax;
    static float umin = (float)usMin;

    float x = (cmd + 100.0f) * (umax - umin) / (100.0f - (-100.0f)) + umin;
    return (int)floorf(x);

}

void SaberTooth::enviarComando(int ch1_cmd, int ch2_cmd)
{
    rc_servo_send_pulse_us(canalFwd, ch1_cmd);
    rc_servo_send_pulse_us(canalTurn, ch2_cmd);

    anunciarComando(ch1_cmd, ch2_cmd);

}


void SaberTooth::anunciarComando(int ch1_cmd, int ch2_cmd)
{
  tfbot_msgs::sabertooth_us msg;

  msg.nombre = nombreNodo;

  msg.s1_fwd = ch1_cmd;
  msg.s2_turn = ch2_cmd;
  // ROS_INFO("Se va");
  listaPubs[topicoAnuncio].publish(msg);

}


void SaberTooth::emergencyCallback(const ros::WallTimerEvent& evnt) 
{
  // Si no hay una comunicacion, y han pasado suficientes ciclos, grita.
  if (numFaltas > SEC_NO_CONTROL*ESPERA_HZ)
  {
    ROS_WARN("Se perdio la comunicacion.");
    rc_led_set(RC_LED_RED, 0);
  }
}
