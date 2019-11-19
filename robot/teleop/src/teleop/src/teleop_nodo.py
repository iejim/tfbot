# -*- coding: latin-1 -*-
import rospy
from time import sleep
from tfbot_msgs.msg import gamepad
from std_msgs.msg import String
from tfbot_msgs.msg import drivetrain

# class drivetrain:
#   nombre = ""
#   cmd1_izq = 0
#   cmd2_der = 0



class TeleOpNode(object):

  _nombre_nodo = "teleop_nodo"
  _topico_gamepad = "control_pad"
  _topico_control = "control"
  _topico_drivetrain = "drivetrain"

  #TODO: Usar namespace para topicos
  _ns = ""

  _pub_drivetrain = None
  _sub_control = None
  _sub_gamepad = None

  _cmd_msg = None

  _no_estado = False

  def __init__(self, nombre = None, ns = None):
    if nombre:
      self._nombre_nodo = nombre
    self._cmd_msg = drivetrain()

    

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

    # Crear suscriptores
    self._sub_gamepad = rospy.Subscriber(self._topico_gamepad, gamepad, self.gamepad_callback)
    self._sub_control = rospy.Subscriber(self._topico_control, String, self.control_callback)

    # Iniciar nodo
    rospy.init_node( self._nombre_nodo)

    self._no_estado = True
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
      if self._no_estado:
        rospy.logwarn("Sin comunicación.")
      rate.sleep()

  def gamepad_callback(self, data):
      msg = self._cmd_msg

      # recoger la informacion, convertirla y enviarla
      msg.cmd1_izq = self.pad_a_drive(data.lY)
      msg.cmd2_der = self.pad_a_drive(data.rY)

      # Por ahora, no hacerle caso
      # msg.lX = data[1]
      # msg.lY = data[2]
      # msg.rX = data[3]
      # msg.rY = data[4]

      # msg.Arrow = data[5] & 0x0f
      # msg.Letter = data[5] & 0xf0

      # msg.L = data[6] & 0x0f
      # msg.R = data[6] & 0xc0

      # msg.Control = data[6] & 0x30

      self._pub_drivetrain.publish(msg)
      rospy.loginfo("Enviando %s" % repr(msg))

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
    else:
      self._no_estado = True
      rospy.logwarn("No hay nodo de control. Deteniendo motores.")
      msg = drivetrain()
      msg.cmd1_izq = 0.0
      msg.cmd2_der = 0.0
      self._pub_drivetrain.publish(msg)


if __name__ == '__main__':
  nodo = TeleOpNode()
  nodo.correr()
  del nodo
  print "Terminado"