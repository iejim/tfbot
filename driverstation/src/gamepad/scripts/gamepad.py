#!/usr/bin/env python2
# -*- coding: latin-1 -*-
from time import sleep
import sys
import pywinusb.hid as hid
from _botones2 import *

class gamepad(object):
  rX = None
  rY = None
  lX = None
  lY = None
  Arrow = None
  Letter = None
  L = None
  R = None
  Control = None

  def __str__(self):
    return "rX: %d, rY: %d\nlX: %d, lY: %d\nArrow: %d, Letter: %s\nL: %d, R: %d\nControl: %s" %\
      (self.rX, self.rY, self.lX, self.lY, self.Arrow, self.Letter, self.L, self.R, self.Control.__str__())

class GamepadNode(object):

  _dev_usb = None
  _usb_msg = None
  _datos = []
  _mem = 0
  _L2_val = 0
  _R2_val = 0

  def __init__(self, nombre = None):
    if nombre:
      self._nombre_nodo = nombre
    self._usb_msg = gamepad()

    

  def __del__(self):
    if self._dev_usb and self._dev_usb.is_opened():
      self._dev_usb.close()
      print "Dispostivito cerrado"

    print "Saliendo"

  def correr(self):

    # Preparar gamepad 
    if not self.conectarUSB():
      return


    rate = 5
    try:
      while True:
        sleep(1.0/rate)
    except KeyboardInterrupt as e:
      print "\r\n",
    print "Terminando."

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
      print "No se pudo encontrar un dispositivo. \n Revise la conexion."
    
    if dev:
      print "Trabajando con el dispositivo: %s. 0x%04x:0x%04x" % (dev.product_name, dev.vendor_id, dev.product_id)
    else:
      return False

    print "Abriendo dispositvo."
    dev.open()
    
    if dev.is_opened():
      #if dev.product_id == 0xc216:
      #  dev.set_raw_data_handler(self.usb_data_handler)
      #else:
      #  dev.set_raw_data_handler(self.usb_data_handler2)
      dev.set_raw_data_handler(self.usb_data_handler2)
      print "Dispositivo abierto. Listo para usar."
      self._dev_usb = dev
      return True
    
    print "No se pudo abrir el dispositivo."
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
    print msg

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
    msg.Letter = ord(self.leer_letras2(bot))
    
    bot = data[indices["flechas"]] 
    msg.Arrow = self.leer_flechas2(bot)
    
    msg.L = self.leer_L2(data)
    msg.R = self.leer_R2(data)
    
    bot = data[indices["extra"]]
    msg.Control = self.leer_control2(bot)
    
    # print "\x1b7",
    print "\n%s" % msg
    print data, "\x1b[7d\r"

    # if self._datos.__len__() < 100:
    #   self._datos.append(data[9])
    # else:
    #   print self._datos
    #   exit()
    
    
  def leer_L2(self,data):
    #Devuelve 1,2,3
    
    # Leer 1
    n = 0
    if data[indices["detras"]] & masks[L1]:
      n = 1

    if data[indices["cambio"]] > 0:
      n = self.leer_gatillo2(data[indices["palancas"]], L2)

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
    if data[indices["cambio"]] > 0:
      n = self.leer_gatillo2(data[indices["palancas"]], R2)

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
    if byte > 128: 
      n = (byte-128)>>2
    else:
      n = byte>>2

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
    
  def leer_gatillo2(self, val, bot):
    # Lee cuanto se ha presionado el gatilllo
    # Solo se llama si se leyo un valor en data[10]

    #para L2 sumaer
    #Para R2, quitar
    # detectar en que direccion fue el cambio y traducir acorde
    v = self._mem - val
    self._mem = val
    if v>0:
      #bajando
      self._L2_val = self._L2_val + v
    else:
      #subiendo
      self._R2_val = self._R2_val + (-1)*v
    
    if bot == R2:
      return self._R2_val
    elif bot == L2:
      return self._L2_val
    else:
       return 0

if __name__ == '__main__':
  nodo = GamepadNode("DSGamepad")
  nodo.correr()
  del nodo