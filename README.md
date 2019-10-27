Repositorio con el código para los diferentes nodos de ROS que corren en el BeagleboneBlue (BBBl) para sevir de interfaz con el Hardware, basándose en las funciones dispuestas por la librería librobotcontrol de <http://strawsondesign.com/docs/librobotcontrol/> y las capacidades de mensajería que ofrece [ROS](https://ros.org). Se usará ROS Melodic porque este puede correr en Windows.

Para facilidad en desarrollo, se mantiene una copia local de los headers de la librerías de linux 4.14.79 (BBBl), librobotcontrol y ROS Melodic en _include/_.

Estos paquetes *NO PUEDEN SER COMPILADOS* si no se tiene el espacio preparado con ROS

PENDIENTE:
- Preparar los folders como paquetes de ROS para poder ser compilados bajo el ambiente de ROS.