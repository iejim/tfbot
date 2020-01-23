#!/bin/bash

Wifi=rfkill0
USR3=phy0tx
USR2=phy0rx
USR1=cpu0
USR0=mmc0

dir=/sys/class/leds

bbled="beaglebone:green:usr"
#cat $dir/wifi/trigger >  /home/ubuntu/trig

# Pudiera ser un while para esperar que este arriba el SoftAp (para poder setear el led del Wifi)
#sleep 2

echo $Wifi > $dir/wifi/trigger
echo $USR0 > $dir/${bbled}0/trigger
echo $USR1 > $dir/${bbled}1/trigger
echo $USR2 > $dir/${bbled}2/trigger
echo $USR3 > $dir/${bbled}3/trigger

#ls $dir/ > /home/ubuntu/lista
