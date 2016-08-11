
#include "contiki.h"
#include "dev/sky-sensors.h"
#include "dev/extSensor.h"


#define INPUT_CHANNEL  (1 << INCH_0)

#define INPUT_REFERENCE  SREF_1

#define PRESSURE_MEM   ADC12MEM0
/*---------------------------------------------------------------------------*/
static int
value(int type)
{
  return PRESSURE_MEM;
}
/*---------------------------------------------------------------------------*/
static int
status(int type)
{
  return sky_sensors_status(INPUT_CHANNEL, type);
}
/*---------------------------------------------------------------------------*/
static int
configure(int type, int c)
{ 
  return sky_sensors_configure(INPUT_CHANNEL, INPUT_REFERENCE, type, c);
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(extSensor, "extSensor", value, configure, status);