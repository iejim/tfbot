/**
 * Clase base para todos los nodos a utilizar.
 *
 * Contiene la inicialización en ROS, la cola de suscriptores y
 * publicadores, incluyendo los comunes a cada nodo, así como los
 * métodos para correr un nodo.
 */
#ifndef NODO_TF_H
#define NODO_TF_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tfbot_msgs/drivetrain.h"
#include "tfbot_msgs/sabertooth_us.h"
#include <string>
#include <sstream>

//pudiera ser un typedef
#define sstring std::string


class NodoTF
{
protected:
    /* data */

    typedef std::map<sstring, ros::Publisher> PublisherDict;
    typedef std::map<sstring, ros::Subscriber> SubscriberDict;

    sstring nombreNodo;
    sstring idNodo;
    sstring ns; // Nombre del "namespace" que referencia al robot (BBB)

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandlePtr nh;

    /**
     * Con ROS se puede programar un nodo para ejecutarse periodicamente
     * a una frecuencia especifica.
     *
     * Declarar el periodo de ejecucion (no es forzado) en milisegundos
     */
    ros::Rate* loopFreq;

    // ¿Se debería tener un timer para el KeepAlive?
    ros::WallTimer controlTimer;

    PublisherDict listaPubs;
    SubscriberDict listaSubs;

public:
    NodoTF(sstring nombre, sstring id, sstring ns = "");

    ~NodoTF();


    /********* Ejecucion ******/

    void init(int argc, char **argv);

    //Necesario aquí???
    //void correr();


    /******** Timing *******/
    /**
     * Fija una frecuencia a usar para correr en bucle.
     */
    void prepFrecuenciaHz(double freq);

    bool esPeriodico();

    double periodoNodo();

    /******** Seguridad *******/

    void activarTimerEmergencia(double tiempo);

    /**
     * Apaga el nodo pasivamente y limpia los diccionarios.
     */
    void apagarNodo();

    /******* Topicos ********/

    /**
     * Agregar al Diccionario de Publicadores
     */
    void agregarPub(ros::Publisher &pub );

    /**
     * Eliminar del Diccionario de Publicadores
     */
    void eliminarPub(sstring topico );

    /**
     * Agregar al Diccionario de Subscriptores
     */
    void agregarSub(ros::Subscriber &pub );

    /**
     * Eliminar del Diccionario de Subscriptores
     */
    void eliminarSub(sstring topico );


    /*********** Callbacks **********/

    /**
     * Callback para suscriptor a topico de Control.
     * Escucha por un mensaje de KeepAlive y actualiza el timer de
     * emergencia para poder decidir si mantener el nodo corriendo.
     */

    void controlCallback(std_msgs::String::ConstPtr msg);

    /**
     * Se ejecuta si dejan de llegar los anuncios de que el robot
     * sigue vivo, y toma las decisiones de lugar.
     */
    void emergencyCallback(const ros::WallTimerEvent& evnt);



};

NodoTF::NodoTF(sstring nombre, sstring id, sstring ns)
{

    //nombreNodo = nombre;
    nombreNodo = nombre + "_" + id;
    idNodo = id;
    this->ns = ns;
    nh = nullptr;
    loopFreq = nullptr;
    // controlTimer = ros::WallTimer();

}

NodoTF::~NodoTF()
{


    apagarNodo();

    // ROS_INFO("Borrando Freq");
    delete loopFreq;
    // ROS_INFO("Borrando Timer");
    // delete controlTimer;
    // controlTimer.reset();
    // Iterar sobre los diccionarios para eliminar cada elemento?
    // Se supone que no sea necesario llamar las funciones de
    // shutdown de cada uno porque se mueren solos.
    // ROS_INFO("Borrando Nodo");

    // controlTimer.stop();
    // nh->shutdown();

    nh.reset();

}


/********* Ejecucion ******/

void NodoTF::init(int argc, char **argv)
{

    // Usar sstring id, sstring ns
    // para setear el nombre del nodo y el namespace
    /*
    nombreNodo = nombreNodo + "_" + id;
    this->ns = ns;
    */

    ros::init(argc, argv, nombreNodo);

    // Al usar un shared_ptr, habria mas control de su existencia
    ros::NodeHandlePtr pn( new ros::NodeHandle(ns) );
    nh = pn; // Se exportará el valor fuera del scope?


    // Suscribir al (los) topico(s) de Control
    activarTimerEmergencia(5.0);


    ROS_INFO("Nodo inicializado: %s", nombreNodo.c_str());


}

/******** Timing *******/

void NodoTF::prepFrecuenciaHz(double freq)
{
    loopFreq = new ros::Rate(freq);
}

bool NodoTF::esPeriodico()
{
    return (loopFreq != nullptr && loopFreq->expectedCycleTime().toNSec());
}

double NodoTF::periodoNodo()
{
    if (!esPeriodico())
        return 0.0;

    return loopFreq->expectedCycleTime().toSec();
}

/******** Seguridad *******/

void NodoTF::activarTimerEmergencia(double tiempo)
{
    controlTimer = nh->createWallTimer<NodoTF>(ros::WallDuration(tiempo),&NodoTF::emergencyCallback, this);
}

/******* Topicos ********/

void NodoTF::agregarPub(ros::Publisher &pub)
{
    //¿Seguirá existiendo fuera del scope?
    ROS_INFO("Registrando publicador a %s", pub.getTopic().c_str());
    listaPubs[pub.getTopic()] = pub; // Se pudiera hacer un shared_ptr

}

void NodoTF::eliminarPub(sstring topico )
{
    listaPubs.erase(topico);
}

void NodoTF::agregarSub(ros::Subscriber &sub)
{
    ROS_INFO("Registrando subscriptor a %s", sub.getTopic().c_str());
    listaSubs[sub.getTopic()] = sub;
}

void NodoTF::eliminarSub(sstring topico )
{
    //Llevar a cabo procedimiento de cierre.

    listaSubs.erase(topico);
}

void NodoTF::apagarNodo()
{
    ROS_INFO("Apagando nodo %s", nombreNodo.c_str());
    controlTimer.stop();
    ROS_INFO("Diccionarios - Pubs: %u", (uint)listaPubs.size());
    // Itera por los diccionarios y termina cada elemento
    PublisherDict::iterator itP;
    for (itP = listaPubs.begin(); itP != listaPubs.end(); itP++)
    {
        ROS_INFO("Desconectando Publicador %s", itP->first.c_str());
        itP->second.shutdown();
        // listaPubs.erase(itP);
    }
    listaPubs.clear(); //mas efectivo??
    ROS_INFO("Diccionarios - Subs: %u", (uint)listaSubs.size());
      
    SubscriberDict::iterator itS; 
    for (itS= listaSubs.begin(); itS !=listaSubs.end(); itS++)
    {
        ROS_INFO("Desconectando Subscriptor %s", itS->first.c_str());
        itS->second.shutdown();
        // listaSubs.erase(itS);
    }
    
    listaSubs.clear();

    //Apaga el nodo
    ROS_INFO("Apagado!");
    nh->shutdown();
    ros::shutdown();
}

/*********** Callbacks **********/

void NodoTF::controlCallback(std_msgs::String::ConstPtr msg)
{

    // si llega un mensaje de continuar, reiniciar el Timer

    if (strcmp(msg->data.c_str(),"UP"))
    {
        ROS_INFO("Recibido UP");
        controlTimer.stop();
        controlTimer.start();

    } else if (strcmp(msg->data.c_str(), "STOP")){
        ROS_INFO("Recibido STOP");
        controlTimer.setPeriod( ros::WallDuration(0.01)); //10ms;
    }
    // si llega un mensaje de parar, forzar el timer con un tiempo

}

void NodoTF::emergencyCallback(const ros::WallTimerEvent& evnt)
{

    

    ROS_WARN("EMERGENCIA: Terminando el nodo %s por falta de comunicación en %.2f",nombreNodo.c_str(), evnt.current_real.toSec());

    // Terminar procesos de control de firmware

    /**
     * Esta funcion no tiene acceso a al firmware que maneja el nodo,
     * por tanto, no puede trajar desde aqui, ya que apagaria el nodo
     * sin asegurar un estado predecible del hardware.
     * 
     * Se puede declarar virtual o algo asi para ser sobreescrita
     * o se puede poner como argumento una funcion de la subclase 
     * para ser ejecutada para interactuar con el hardware.
     */

    // Ya se encarga de desconectar todo lo creado por el nodo (subs, pubs, timers)
    // apagarNodo();
}


#endif // NODO_TF_H