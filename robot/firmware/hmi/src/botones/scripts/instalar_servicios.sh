#!/bin/bash


systemctl disable /opt/tfbot/services/ros_servos.service
systemctl disable /opt/tfbot/services/ros_javabot.service
systemctl disable /opt/tfbot/services/ros_leds.service
systemctl disable /opt/tfbot/services/ros_sabertooth.service

# Crear enlace a servicios
echo "Instalando enlaces a servicios"
# systemctl link /opt/tfbot/services/ros_teleop.service
# systemctl link /opt/tfbot/services/ros_sabertooth.service

systemctl link /opt/tfbot/services/ros_tfbot.service
systemctl link /opt/tfbot/services/ros_teleop.service
# systemctl link /opt/tfbot/services/ros_javabot.service
systemctl link /opt/tfbot/services/wd_botones.service

# Habilitar servicios
echo "Habilitando servicios."
systemctl enable wd_botones.service
# systemctl enable ros_teleop.service
# systemctl enable ros_sabertooth.service
