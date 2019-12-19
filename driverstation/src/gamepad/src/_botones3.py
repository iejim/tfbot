#!/usr/bin/env python2
# -*- coding: latin-1 -*-


# Valor reportado si no se presiona una flecha
NO_ARROWS = 0x0

CALC_FLECHAS = [
  UP,
  UP_RIGHT, 
  RIGHT,
  RIGHT_DOWN,
  DOWN,
  DOWN_LEFT,
  LEFT,
  LEFT_UP, 
  NO_PAD
  ] = [-3, -1, 2, 5, 3, 1, -2, -5, 0]

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

# Flechas
PADX = 'PX'
PADY = 'PY'

LX = "LX"
LY = "LY"
RX = "RX"
RY = "RY"

ABB_EVENT = (
    # D-PAD, aka HAT
    ('ABS_HAT0X', PADX),
    ('ABS_HAT0Y', PADY),
    ('ABS_Y', LY),
    ('ABS_X', LX),
    ('ABS_RY', RY),
    ('ABS_RX', RX),
    ('ABS_Z', L2),
    ('ABS_RZ', R2),

    # Face Buttons
    ('BTN_NORTH', Y),
    ('BTN_EAST', B),
    ('BTN_SOUTH', A),
    ('BTN_WEST', X),

    # Other buttons
    ('BTN_THUMBL', L3),
    ('BTN_THUMBR', R3),
    ('BTN_TL', L1),
    ('BTN_TR', R1),
    ('BTN_SELECT', START),
    ('BTN_START', BACK),
)

NOMBRES_EVT=dict(ABB_EVENT)

CATEGORIAS = [A_RX, A_RY, A_LX, A_LY, LETRAS, BOT_L, BOT_R, BOT_CONTROL, FLECHAS] = list(range(9))

TIPOS_EVT = dict((
    # D-PAD, aka HAT
    ('ABS_HAT0X', FLECHAS),
    ('ABS_HAT0Y', FLECHAS),
    ('ABS_Y', A_LY),
    ('ABS_X', A_LX),
    ('ABS_RY', A_RY),
    ('ABS_RX', A_RX),
    ('ABS_Z', BOT_L),
    ('ABS_RZ', BOT_R),

    # Face Buttons
    ('BTN_NORTH', LETRAS),
    ('BTN_EAST', LETRAS),
    ('BTN_SOUTH', LETRAS),
    ('BTN_WEST', LETRAS),

    # Other buttons
    ('BTN_THUMBL', BOT_L),
    ('BTN_THUMBR', BOT_R),
    ('BTN_TL', BOT_L),
    ('BTN_TR', BOT_R),
    ('BTN_SELECT', BOT_CONTROL),
    ('BTN_START', BOT_CONTROL),
))

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