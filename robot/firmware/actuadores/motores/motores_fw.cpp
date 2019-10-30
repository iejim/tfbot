#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <sstream>

/**
 * Nodo de interfaz con los un driver de motores conectado a un puerto de PWM.
 * 
 * El nodo permite interactuar con un Sabertooth en un puerto PWM especificado
 * en como argumento del Nodo, que tambien es usado como identificador del nodo.
 * 
 */

const std::string nombreNodo_base = "Motor_fw_";
const std::string topicoPub_base = "motor_req_";

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */

  // Guarda el ultimo argumento como identificador del nodo
  std::string nodoId = argv[argc];

  // Concatena el idenficador al nombre base para crear el nombre del nodo
  // Ejemplo: "Motor_fw_izq"
  ros::init(argc, argv, nombreNodo_base + nodoId); 

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  // Publicar en un topico especifico para este nodo, guardar solo 2 mensajes
  ros::Publisher pub = n.advertise<std_msgs::String>(topicoPub_base + nodoId, 2);

  /**
    * Con ROS se puede programar un nodo para ejecutarse periodicamente
    * a una frecuencia especifica.
    * 
    * Declarar el periodo de ejecucion (no es forzado) en milisegundos
    */
  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  /**
   * Con ros::ok() podemos simular un "while (true)" para un loop infinito,
   * pero se monitorea el estado de ROS (roscore) para decidir cuando salir.
   */
  while (ros::ok()) 
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    /**
     * Publica en roslog el mensaje a modo de INFO
     */
    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    pub.publish(msg);

    /**
     * Permite que se ejecuten las acciones de ROS solicitadas. De inmediato vuelve aqui (spinOnce).
     */
    ros::spinOnce();
    
    /**
     * Termina de agotar el tiempo para cumplir el periodo establecido con loop_rate.
     */
    loop_rate.sleep();
    
    ++count;
  }


  return 0;
}