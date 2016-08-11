#!/bin/sh

echo Copying files
cp -i extSensor.c ~/contiki/platform/sky/dev/
cp -i extSensor.h ~/contiki/platform/sky/dev/
echo "CONTIKI_TARGET_SOURCEFILES += extSensor.c" >> ~/contiki/platform/sky/Makefile.sky
echo Done
