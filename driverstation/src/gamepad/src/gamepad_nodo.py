#!/usr/bin/env python2
# -*- coding: latin-1 -*-
import rospy
from time import sleep
import pywinusb.hid as hid
from tfbot_msgs.msg import gamepad
from std_msgs.msg import String
from _botones2 import *

class GamepadNode(object):

  _nombre_nodo = "gamepad_nodo"
  _topico_gamepad = "gamepad_control"
  _topico_control = "control_nodos"
  _ns = "" #Namespace para topicos

  _pub = None
  _pub_control = None

  _dev_usb = None
  _usb_msg = None

  def __init__(self, nombre = None, ns = None):
    if nombre:
      self._nombre_nodo = nombre
    if ns:
      self._ns = "/" + ns

    self._usb_msg = gamepad()

    

  def __del__(self):
    if self._dev_usb and self._dev_usb.is_opened():
      self._dev_usb.close()
      print "Dispostivito cerrado"

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

    # Crear publicador
    self._pub = rospy.Publisher(self.crear_topico(self._topico_gamepad), gamepad, queue_size=1)
    self._pub_control = rospy.Publisher(self.crear_topico(self._topico_control), String, queue_size=1)

    # Preparar gamepad 
    if not self.conectarUSB():
      return


    rate = rospy.Rate(5)
    msg = String()
    msg.data = "UP"
    while not rospy.is_shutdown():
      # rospy.loginfo("Enviando: %s" % msg.data)
      self._pub_control.publish(msg)
      rate.sleep()

  def conectarUSB(self):
    #pID = 0xc216
    #pID = 0xc21d # Nuevos
    vID = 0x046d
    filter = hid.HidDeviceFilter(vendor_id = vID)#, product_id = pID)

    devices = filter.get_devices()

    try: 
      dev = devices[0]
    except KeyError:
      dev = False
      rospy.loginfo("No se pudo encontrar un dispositivo. \n Revise la conexion.")
    
    if dev:
      rospy.loginfo("Trabajando con el dispositivo: %s. 0x%4x:0x%4x" % (dev.product_name, dev.vendor_id, dev.product_id)
    else:
      return False

    rospy.loginfo("Abriendo dispositvo.")
    dev.open()
    
    if dev.is_opened():
      #if dev.product_id == 0xc216:
      #  dev.set_raw_data_handler(self.usb_data_handler)
      #else:
      #  dev.set_raw_data_handler(self.usb_data_handler2)
      dev.set_raw_data_handler(self.usb_data_handler2)
      rospy.loginfo("Dispositivo abierto. Listo para usar.")
      self._dev_usb = dev
      return True
    
    rospy.loginfo("No se pudo abrir el dispositivo.")
    return False
    
  def crear_topico(self, nombre):
    return self._ns + nombre 

  def usb_data_handler(self, data):
    # Procesar data
    #TODO: Usar MODE para deshabilitar todo
    #TODO: Habilitar la lectura de botones similares al mismo tiempo
    msg = self._usb_msg
    
    msg.lX = data[indices[LX]]
    msg.lY = data[indices[LY]]
    msg.rX = data[indices[RX]]
    msg.rY = data[indices[RY]]
    
    # Botones
    bot = data[indices["botones"]] 
    msg.Arrow = self.leer_flechas(bot)
    msg.Letter = self.leer_letras(bot)
    
    bot = data[indices["extra"]]
    msg.L = self.leer_L(bot)
    msg.R = self.leer_R(bot)

    msg.Control = self.leer_control(bot)
    try:
     self._pub.publish(msg)
     rospy.loginfo("Enviando %s" % repr(msg))
    except rospy.ROSSerializationException:
     rospy.loginfo("Error leyendo USB")
      

  def leer_flechas(self, val):
    # Primer nibble
    nib = val & 0x0f
    # Flechas
    # Desestimar si no esta presionado (0x8)
    if nib < NO_ARROWS:
      flechas = nib
    else:
      flechas = 8
    return flechas
  
  def leer_letras(self, val):
    # Letras
    letra = 0
    # Segundo nibble
    nib = val & 0xf0
    try: 
      letra = letras[nib] # Asumiendo una a la vez
    except KeyError:
      pass
    return letra

  def leer_L(self, val):
    atras = 0
    if val & masks[L1]:
      atras = L1
    elif val & masks[L2]:
      atras = L2
    elif val & masks[L3]:
      set
      atras = L3
    return atras

  def leer_R(self, val):
    val = val >> 1
    return self.leer_L(val)

  def leer_control(self, val):
    # Opciones
    op = 0
    if val & masks[BACK]:
      op = BACK
    elif val & masks[START]:
      op = START
    return op
    
  def usb_data_handler2(self, data):
    # Procesar data
    #TODO: Usar MODE para deshabilitar todo
    #TODO: Habilitar la lectura de botones similares al mismo tiempo
    msg = self._usb_msg
    msg.lX = data[indices[LX]]
    msg.lY = data[indices[LY]]
    msg.rX = data[indices[RX]]
    msg.rY = data[indices[RY]]
    
    # Botones
    bot = data[indices["botones"]] 
    msg.Letter = self.leer_letras2(bot)
    bot = data[indices["flechas"]] 
    msg.Arrow = self.leer_flechas2(bot)
    
    msg.L = self.leer_L2(data)
    msg.R = self.leer_R2(data)
    
    bot = data[indices["extra"]]
    msg.Control = self.leer_control2(bot)
    try:
     self._pub.publish(msg)
     rospy.loginfo("Enviando %s" % repr(msg))
    except rospy.ROSSerializationException:
     rospy.loginfo("Error leyendo USB")

  def leer_L2(self,data):
    #Devuelve 1,2,3
    
    # Leer 1
    n = 0
    if data[indices["detras"]] & masks[L1]:
      n = 1
    # No tiene 2 por el momento
    if data[indices["push"]] & masks[L3]:
      n = 3
    
    return n

  def leer_R2(self,data):
    #Devuelve 1,2,3
    
    # Leer 1
    n = 0
    if data[indices["detras"]] & masks[R1]:
      n = 1
    # No tiene 2 por el momento
    if data[indices["push"]] & masks[R3]:
      n = 3
    
    return n

  def leer_letras2(self,byte):
    # No aguanta mas de una letra
    if byte & masks[A]:
        return A
    if byte & masks[B]:
        return B
    if byte & masks[X]:
        return X
    if byte & masks[Y]:
        return Y
    return "N"
    
  def leer_flechas2(self,byte):
    n = (byte-128)>>2
    if n>0:
      return n-1 # Mantener compatibilidad control anterior
    return 8
    
  def leer_control2(self, val):
    # Opciones
    op = 0
    if val & masks[BACK]:
      op = BACK
    elif val & masks[START]:
      op = START
    return op
    
if __name__ == '__main__':

  nodo = GamepadNode("DS_Gamepad")
  nodo.correr()
  del nodo
  print "Terminado"