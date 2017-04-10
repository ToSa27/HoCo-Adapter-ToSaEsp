#!/bin/bash
export TOSAESP_DEVICE_NAME=dg-heizung
export TOSAESP_IP=10.0.1.172
export TOSAESP_LIB_DEPS="OneWire, DallasTemperature, PCF8574"

export TOSAESP_OTA_PORT=8027
export TOSAESP_OTA_PASSWORD=tosa
export TOSAESP_BUILD_FLAGS=""
cp $TOSAESP_DEVICE_NAME.h src/config_device.h
pio run --target upload
