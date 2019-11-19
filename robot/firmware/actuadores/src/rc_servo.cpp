#include "rc/servo.h"

int rc_servo_init(void)
{
  return 1;
}

void rc_servo_cleanup(void)
{
  return;
}

int rc_servo_power_rail_en(int en)
{
  return en;
}

int rc_servo_set_esc_range(int min, int max)
{
  return min+max;
}

int rc_servo_send_pulse_us(int ch, int us)
{
  return us;
}

int rc_servo_send_pulse_normalized(int ch, double input)
{
  return (int)input*10;
}

int rc_servo_send_esc_pulse_normalized(int ch, double input)
{
  return (int)input*100;
}

int rc_servo_send_oneshot_pulse_normalized(int ch, double input)
{
  return (int)input*100;
}