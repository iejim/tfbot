from time import sleep

import pywinusb.hid as hid

# flechas : 5, # [0-7], 8 en descanso desde arriba, cw
# lX : 1, #[0,255]
# lY : 2,
# rX : 3,
# rY : 4,
# botones : 5, #[24, 40, 72, 136]
# atras : 6, #[1,2,4,8] izq-der
# centro : 6, #[16, 32]
# joy_botones : 6, #[64, 128]
# modo : 7, #[0, 8] 0:normal

# Valor reportado si no se presiona una flecha
NO_ARROWS = 0x8

UP =          0x0 #"UP"
UP_RIGHT =    0x1 #"UR" 
RIGHT =       0x2 #"RT"
RIGHT_DOWN =  0x3 #"RD"
DOWN =        0x4 #"DN"
DOWN_LEFT =   0x5 #"DDL"
LEFT =        0x6 #"LT"
LEFT_UP =     0x7 #"LP"

X = "X"
Y = "Y"
B = "B"
A = "A"

L1 = "1"
R1 = "1"
L2 = "2"
R2 = "2"
L3 = "3"
R3 = "3"

BACK = "K"
START = "S"

MODE = "M"

LX = 1
LY = 2
RX = 3
RY = 4

class Gamepad(object):

  _pID = 0xc216
  _vID = 0x046d

  _indices = {
    LX : 1, #[0,255]
    LY : 2,
    RX : 3,
    RY : 4,
    "botones" : 5, #[24, 40, 72, 136]
    "atras" : 6, #[1,2,4,8] izq-der
    "centro" : 6, #[16, 32]
    "modo" : 7 #[0, 8] 0:normal
  }

  _masks = {
    UP :          0x00 ,
    UP_RIGHT :    0x01 ,
    RIGHT :       0x02 ,
    RIGHT_DOWN :  0x03 ,
    DOWN :        0x04 ,
    DOWN_LEFT :   0x05 ,
    LEFT :        0x06 ,
    LEFT_UP :     0x07 ,
    
    X : 0x10 ,
    Y : 0x20 ,
    B : 0x40 ,
    A : 0x80 ,
    
    L1 : 0x1 ,
    R1 : 0x2 ,
    L2 : 0x4 ,
    R2 : 0x8 ,

    BACK :   0x10 ,
    START : 0x20 ,
    L3 :    0x40 ,
    R3 :    0x80 ,

    MODE : 0x8
  }

  # Inicializados en 0
  # _botones ={
  #   UP :          False ,
  #   UP_RIGHT :    False
  #   RIGHT :       False
  #   RIGHT_DOWN :  False
  #   DOWN :        False
  #   DOWN_LEFT :   False
  #   LEFT :        False
  #   LEFT_UP :     False
  #
  #   X : False
  #   Y : False
  #   B : False
  #   A : False
  #
  #   L1 : False
  #   R1 : False
  #   L2 : False
  #   R2 : False
  #
  #   BACK :  False
  #   START : False
  #   L3 :    False
  #   R3 :    False
  #
  #   MODE : False
  # }
  #
  _letras = {
    _masks[X] : X ,
    _masks[Y] : Y ,
    _masks[B] : B ,
    _masks[A] : A
  }
  

  _disp_abierto = False
  _disp = None

  _actualizando = False


  def __init__(self, dispositivo):
    
    filter = hid.HidDeviceFilter(vendor_id = self._vID, product_id = self._pID)
    devices = filter.get_devices()
    # TODO: Asignar diferentes dispositivos
    self._disp = devices[dispositivo]
    self._disp_abierto = self._disp.open()

  def __del__(self):
    if self._disp_abierto:
      self._disp.close()

  def leer_esta_abierto(self):
    return self._disp_abierto

  def cerrar(self):
    self._disp_abierto = self._disp.close()

  def iniciar(self):

    self._disp.set_raw_data_handler(self._data_handler)

    

  def _data_handler(self,data):
    """Procesar data enviada por el control"""
    # Será esto tan rapido como necesitamos?
    # Los resultados deben ser guardados para la posteridad.
    # Como este método funciona en otro thread,
    # hay que bloquear la lectura de los valores
    _actualizando = True

    # Reiniciar el mensaje
    ## msg = [] # o equivalente
    
    # Procesar data
    #TODO: Usar MODE para deshabilitar todo
    #TODO: Habilitar la lectura de botones similares al mismo tiempo

    # Botones
    bot = data[5] # _indices["botones"]
    # Primer nibble
    nib = bot & 0x0f

    # # Reiniciar valores
    # for i in range(0,NO_ARROWS):
    #     _botones[i] = 0x0 # Desactivado

    # Flechas
    # Desestimar si no esta presionado (0x8)
    if not nib < NO_ARROWS:
      flechas = nib
    else:
      flechas = 0
    
    # Letras
    letra = 0
    # Segundo nibble
    nib = bot & 0xf0
    try: 
      letra = self._letras[nib] # Asumiendo una a la vez
    except KeyError:
      pass
      
    # Traseros
    atras_R = 0
    atras_L = 0 

    bot = data[6] # _indices["atras"]

    if bot & self._masks[L1]:
      atras_L = L1
    elif bot & self._masks[L2]:
      atras_L  = L2
    elif bot & self._masks[L3]:
      atras_L = L3
    # El else se cubre con el default arriba

    if bot & self._masks[R1]:
      atras_R = R1
    elif bot & self._masks[R2]:
      atras_R  = R2
    elif bot & self._masks[R3]:
      atras_R = R3

    # Opciones
    op = 0
    
    if bot & self._masks[BACK]:
      op = BACK
    elif bot & self._masks[START]:
      op = START


    # Actualizar
    self._actualizando = False

    # Preparar mensaje
    msg.lX = data[LX]
    msg.lY = data[LY]
    msg.rX = data[RX]
    msg.rT = data[RY]
    msg.arrow = flechas
    msg.letter = letra
    msg.L = atras_L
    msg.R = atras_R
    msg.op = op

    self._pub.publish(msg) # No se si se pueda llamar desde aqui

  # Todos los accesores deben leer si se está actualizando
