#!/usr/bin/env python2
# -*- coding: latin-1 -*-
import rospy
from time import sleep
import inputs
from tfbot_msgs.msg import gamepad
from std_msgs.msg import String
from _botones3 import *

MID_VAL_ABS = 128

class GamepadNode(object):

  _nombre_nodo = "gamepad_nodo"
  _topico_gamepad = "gamepad_control"
  _topico_control = "control_nodos"
  _ns = "" #Namespace para topicos

  _pub = None
  _pub_control = None

  _dev_usb = None
  _usb_msg = None
  _old_msg = None

  _synced = False
  _px = 0
  _py = 0

  def __init__(self, nombre = None, ns = None):
    if nombre:
      self._nombre_nodo = nombre
    if ns:
      self._ns = "/" + ns

    self._usb_msg = gamepad()
    #valores por defecto
    self._usb_msg.lX = MID_VAL_ABS
    self._usb_msg.lY = MID_VAL_ABS
    self._usb_msg.rX = MID_VAL_ABS
    self._usb_msg.rY = MID_VAL_ABS

    self._old_msg = self._usb_msg

    

  def __del__(self):
    print "Saliendo"
    try:
      if self._pub:
        self._pub.unregister()
      if self._pub_control:
        self._pub_control.unregister()
    except AttributeError:
      pass

  def correr(self):

    # Iniciar nodo
    rospy.init_node( self._nombre_nodo)
    print "Registrado como %s" % rospy.get_node_uri()
    # Crear publicador
    self._pub = rospy.Publisher(self.crear_topico(self._topico_gamepad), gamepad, queue_size=1)
    self._pub_control = rospy.Publisher(self.crear_topico(self._topico_control), String, queue_size=1)
    
    # Preparar gamepad 
    if not self.conectarUSB():
      return


    rate = rospy.Rate(5)
    msg = String()
    msg.data = "UP"

    self._timer = rospy.Timer(rospy.Duration(0.005), self.procesar_eventos_usb)
    rospy.loginfo("Arrancando")
    while not rospy.is_shutdown():
      # rospy.loginfo("Enviando: %s" % msg.data)
      print "Enviando: %s\r" % msg.data,
      self._pub_control.publish(msg)
      rate.sleep()

  def conectarUSB(self):
    #pID = 0xc216
    #pID = 0xc21d # Nuevos
    vID = 0x046d
    
    try:
      dev = inputs.devices.gamepads[0]
    except IndexError:
      dev = False
      rospy.loginfo("No se pudo encontrar un dispositivo. \n Revise la conexion.")
    
    if dev:
      rospy.loginfo("Trabajando con el dispositivo: %s." % (dev))
    else:
      return False

    rospy.loginfo("Dispositivo abierto. Listo para usar.")
    self._dev_usb = dev # TODO: confirmar que sigue existiendo
    return True
    
  def crear_topico(self, nombre):
    return self._ns + nombre 

  def procesar_eventos_usb(self, e):
    """Process available events."""
    try:
      events = self._dev_usb.read()
    except EOFError:
      events = []
    for event in events:
      self.procesar_evento(event)
    # print('n')

  def procesar_evento(self, evt):
    # Procesar evento
    msg = self._usb_msg

    t = evt.ev_type
    cod = evt.code
    try:
      c = TIPOS_EVT[cod]
    except KeyError: # en caso de SYNC o algo asi
      c = ""
    val = evt.state
    if t == 'Key':
      self._synced = False
      if c == LETRAS: # actualizar letras
        msg.Letter = self.leer_letras(cod, val)
      elif c == BOT_L: # actualizar L
        msg.L = self.leer_LR(cod, val)
      elif c == BOT_R: # actualizar R
        msg.R = self.leer_LR(cod, val)
      elif c == BOT_CONTROL:
        msg.Control = self.leer_control(cod, val)
      else: # wtf
        pass
      
    elif t == 'Absolute':
      self._synced = False
      if c == BOT_L:
        # Procesar L2
        msg.L = val>>1
      elif c == BOT_R:
        # R2
        msg.R = val>>1
      elif c == A_LX:
        msg.lX = self.normalizar_eje(val)
      elif c == A_LY:
        msg.lY = self.normalizar_eje(val)
      elif c == A_RX:
        msg.rX = self.normalizar_eje(val)
      elif c == A_RY:
        msg.rY = self.normalizar_eje(val)
      elif c == FLECHAS:
        msg.Arrow = self.leer_flechas(cod,val)
      else:
        pass

    elif t == 'Sync':    
      ## Ejecutar si ya llego un SYN_REPORT
      self._synced = True
      print(msg)
      try:
        self._pub.publish(msg)
        # rospy.loginfo("Enviando %s" % repr(msg))
      except rospy.ROSSerializationException as e:
        rospy.loginfo("Error enviando mensaje: %s" % e)

    else:
      return

  def leer_LR(self,cod, val):
    #Devuelve 0,1,3
    if val:
      bot = NOMBRES_EVT[cod]
      return int(bot[1]) # numero en L1 o R2
    return 0

  def leer_letras(self,cod, val):
    if val:
      return ord(NOMBRES_EVT[cod])
    return ord("N")
    
  def leer_flechas(self,cod, val):
    # Leer y completar una lectura
    if self._synced:
      n = self._px*2 + self._py*3
      if n>0:
        v = flechas[n]
        return v-1 # Mantener compatibilidad control anterior
      return 8
    else:
      # Actualizar el que toca
      if cod[-1] == 'X':
        self._px = val
      else:
        self._py = val
    return 8
      
  def leer_control(self, cod, val):
    # Opciones
    if val:
      return ord(NOMBRES_EVT[cod])
    return 0

  def normalizar_eje(self, val):
    # devuelve [0,255]
    # para obtener [-128,127], remover la suma
    return (val>>8)+128
  
if __name__ == '__main__':

  nodo = GamepadNode("DSGamepad")
  nodo.correr()
  del nodo
  print "Terminado"