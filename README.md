Repositorio con el código para los diferentes nodos de ROS que corren en el BeagleboneBlue (BBBl) para sevir de interfaz con el Hardware, basándose en las funciones dispuestas por la librería librobotcontrol de <http://strawsondesign.com/docs/librobotcontrol/> y las capacidades de mensajería que ofrece [ROS](https://ros.org). Se usará ROS Melodic porque este puede correr en Windows.

Para facilidad en desarrollo, se mantiene una copia local de los headers de la librerías _librobotcontrol_ y ROS Melodic en _include/_.

Estos paquetes *NO PUEDEN SER COMPILADOS* si no se tiene el espacio preparado con ROS

PENDIENTE:

- Preparar los folders como paquetes de ROS para poder ser compilados bajo el ambiente de ROS.
    - Cada folder sería un paquete por su cuenta.


Topicos

  Mensaje de control de nodo
  /control_nodos (std_msgs/String)

  Drivetrain (Recibido por el driver de Sabertooth)
  /comando_drivetrain (tfbot_msgs/drivetrain)

  Estado de los motores (lo que se le pide a los Sabertooth)
  /comando_real_motores (tfbot_msgs/sabertooth_us)

  Valor de Gamepad en el DriverStation
  /gamepad_control (tfbot_msgs/gamepad)