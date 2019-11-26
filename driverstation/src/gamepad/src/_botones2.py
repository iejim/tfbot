#!/usr/bin/env python2
# -*- coding: latin-1 -*-

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
NO_ARROWS = 0x0

UP =          0x0 #"UP"
UP_RIGHT =    0x1 #"UR" 
RIGHT =       0x2 #"RT"
RIGHT_DOWN =  0x3 #"RD"
DOWN =        0x4 #"DN"
DOWN_LEFT =   0x5 #"DL"
LEFT =        0x6 #"LT"
LEFT_UP =     0x7 #"LU"

X = "X"
Y = "Y"
B = "B"
A = "A"

L1 = "L1"
R1 = "R1"
L2 = "L2"
R2 = "R2"
L3 = "L3"
R3 = "R3"

BACK = "K"
START = "S"

MODE = "M"

LX = "LX"
LY = "LY"
RX = "RX"
RY = "RY"

#bytes
indices = {
  LX : 1, #[0,255]
  LY : 3,
  RX : 5,
  RY : 7,
  "flechas" : 12,# 128 +4 por direccion [128:4:160] +4*n
  "botones" : 11, #Letras [1,2,4,8] [A,B,X,Y]
  "detras"  : 11,  # [16, 32]
  "palancas" : 10, # L2 <128,R2 >128
  "cambio"  :  9,    # 0 igual, 128 nuevo
  "push"    :  12, # [129, 130] +1, +2
  "extra" : 11, #[64, 128] 0:normal
} 

masks = {
  UP :          0x01 ,
  UP_RIGHT :    0x02 ,
  RIGHT :       0x03 ,
  RIGHT_DOWN :  0x04 ,
  DOWN :        0x05 ,
  DOWN_LEFT :   0x06 ,
  LEFT :        0x07 ,
  LEFT_UP :     0x08 ,
  
  A : 1<<0 ,
  B : 1<<1 ,
  X : 1<<2 ,
  Y : 1<<3 ,
  
  L1 : 1<<4 ,
  R1 : 1<<5 ,
  BACK :  1<<6 ,
  START : 1<<7 ,
  
  #L2 : 0x4 ,
  #R2 : 0x8 ,

  L3 :    1<<0 ,
  R3 :    1<<1 ,

  MODE : 0x8
}

letras = {
  masks[X] : X ,
  masks[Y] : Y ,
  masks[B] : B ,
  masks[A] : A
}

flechas = {
  UP :          0x01 ,
  UP_RIGHT :    0x02 ,
  RIGHT :       0x03 ,
  RIGHT_DOWN :  0x04 ,
  DOWN :        0x05 ,
  DOWN_LEFT :   0x06 ,
  LEFT :        0x07 ,
  LEFT_UP :     0x08 ,
}