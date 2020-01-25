#!/usr/bin/env python2
# -*- coding: latin-1 -*-
import rospy
from time import sleep
from tfbot_msgs.msg import gamepad
from std_msgs.msg import String
from tfbot_msgs.msg import drivetrain
from tfbot_msgs.msg import servo_cmd

# Miembros Mensaje drivetrain
# drivetrain:
#   nombre = ""
#   cmd1_izq = 0
#   cmd2_der = 0

# Miembros Mensaje Servos
# servo_cmd:
#   canal = [1,8]
#   cmd1 = [-100, 100]


SEC_NO_CONTROL = 2 #Segundos sin control
ESPERA_HZ = 5 # Frecuencia de espera (para no dejarlo solo)
CANAL_SERVO_IZQ = 7
CANAL_SERVO_DER = 8

class TeleOpNode(object):

  _nombre_nodo = "teleop_nodo" #Sacar fuera?

  _topico_gamepad = "gamepad_control"
  _topico_control = "control_nodos"
  _topico_drivetrain = "comando_drivetrain"
  _topico_servos = "comando_servos"

  #TODO: Usar namespace para topicos
  _ns = ""

  _pub_drivetrain = None
  _pub_servos = None
  _sub_control = None
  _sub_gamepad = None

  _cmd_msg = None
  _servos_msgR = None
  _servos_msgL = None

  _no_estado = False
  _timer = None
  _num_faltas = 0

  def __init__(self, nombre = None, ns = None):
    if nombre:
      self._nombre_nodo = nombre
    self._cmd_msg = drivetrain()
    self._servos_msgL = servo_cmd()
    self._servos_msgR = servo_cmd()

    self._servos_msgL.canal = CANAL_SERVO_IZQ
    self._servos_msgR.canal = CANAL_SERVO_DER

    

  def __del__(self):
    print "Saliendo"
    try:
      if self._pub_drivetrain:
        self._pub.unregister()
      if self._sub_control:
        self._sub_control.unregister()
      if self._sub_gamepad:
        self._sub_gamepad.unregister()
    except AttributeError:
      pass

  def correr(self):

    # Crear publicador
    self._pub_drivetrain = rospy.Publisher(self._topico_drivetrain, drivetrain, queue_size=2)
    self._pub_servos = rospy.Publisher(self._topico_servos, servo_cmd, queue_size=2)

    # Crear suscriptores
    self._sub_gamepad = rospy.Subscriber(self._topico_gamepad, gamepad, self.gamepad_callback)
    self._sub_control = rospy.Subscriber(self._topico_control, String, self.control_callback)

    # Iniciar nodo
    try:
      rospy.init_node( self._nombre_nodo)
    except rospy.exceptions.ROSInitException:
      print "Error inicializando."
      return

    self._no_estado = True
    rate = rospy.Rate(ESPERA_HZ)
    self._timer = rospy.Timer(rospy.Duration(SEC_NO_CONTROL), self.timer_callback)

    while not rospy.is_shutdown():
      if self._no_estado:
        #rospy.logwarn("Sin comunicación.")
        pass
      self._num_faltas = self._num_faltas + 1 # Antes de dormir
      rate.sleep()

  def gamepad_callback(self, data):
      msg = self._cmd_msg

      # recoger la informacion, convertirla y enviarla
      # print "L: %d; R: %d" % (data.lY, data.rY)
      msg.cmd1_izq = self.pad_a_drive(data.lY)
      msg.cmd2_der = self.pad_a_drive(data.rY)

      # Interpreta y envia un mensaje a los servos
      self.servos_brazos(data.L, data.R)

      self._pub_drivetrain.publish(msg)
      # rospy.loginfo("Enviando %s" % repr(msg))

  def servos_brazos(self, numL, numR):
    msgL = self._servos_msgL
    msgR = self._servos_msgR

    msgL.cmd = self.pad_a_drive(numL + 127.0)
    msgR.cmd = -self.pad_a_drive(numR + 127.0)

    self._pub_servos.publish(msgL)
    self._pub_servos.publish(msgR)

  def pad_a_drive(self, num):
    out_max = 100.0
    out_min = -100.0
    in_max = 255.0 -1.0 # Para tener un cero
    in_min = 0.0
    val = (num - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    return max(min(val, out_max), out_min) # Limitar al rango

  def control_callback(self,data):
    if data.data == "UP":
      self._no_estado = False
      self._num_faltas = 0 # Todo bien
    else:
      self._no_estado = False # Tal vez se quiere hacer otra cosa
      return

  def timer_callback(self, event):
    # Si no ha llego nada de control: se apaga
    if self._num_faltas >= int(SEC_NO_CONTROL*ESPERA_HZ):
      self._no_estado = True
      rospy.logwarn("Mal estado de control. Deteniendo motores. @ " + str(event.current_real))
      msg = drivetrain()
      msg.cmd1_izq = 0.0
      msg.cmd2_der = 0.0
      self._pub_drivetrain.publish(msg)


if __name__ == '__main__':
  nodo = TeleOpNode()
  nodo.correr()
  del nodo
  print "Terminado"
