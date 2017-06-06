#!/bin/bash
if [ "$#" -ne 1 ]; then
  echo "Usage: ./run.sh \"device name\""
  exit 1
fi
export TOSAESP_DEVICE_NAME=$1
if [ ! -f "secret.sh" ]; then
  echo "Create secret.sh from template"
  exit 1
fi
. secret.sh
if [ ! -f "$TOSAESP_DEVICE_NAME.h" ]; then
  echo "Create $TOSAESP_DEVICE_NAME.h"
  exit 1
fi
cp $TOSAESP_DEVICE_NAME.h src/device.h
export TOSAESP_IP=COM4
if [ -f "$TOSAESP_DEVICE_NAME.ip" ]; then
  . ./$TOSAESP_DEVICE_NAME.ip
fi
export TOSAESP_LIB_DEPS="OneWire, DallasTemperature, PCF8574, NewPing"
export TOSAESP_BUILD_FLAGS=""
pio run --target upload
#if [ -f "$TOSAESP_DEVICE_NAME.ip" ]; then
#  until screen /dev/tty... 115200 | grep -m 1 "IP: "; do sleep 1 ; done
#fi
