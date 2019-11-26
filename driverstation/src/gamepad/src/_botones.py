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
# R1 = "1"
L2 = "2"
# R2 = "2"
L3 = "3"
# R3 = "3"

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
  LY : 2,
  RX : 3,
  RY : 4,
  "botones" : 5, #[Flechas, Letras]
  "extra" : 6, # [LR][1-3], [BACK START]
  "modo" : 7 #[0, 8] 0:normal
} 

masks = {
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
  #R1 : 0x2 ,
  L2 : 0x4 ,
  #R2 : 0x8 ,

  BACK :   0x10 ,
  START : 0x20 ,
  L3 :    0x40 ,
  #R3 :    0x80 ,

  MODE : 0x8
}

letras = {
  masks[X] : X ,
  masks[Y] : Y ,
  masks[B] : B ,
  masks[A] : A
}