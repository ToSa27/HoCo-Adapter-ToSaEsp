#!/bin/bash
export TOSAESP_DEVICE_NAME=ag-zisterne
export TOSAESP_IP=10.0.1.166
export TOSAESP_LIB_DEPS="OneWire, DallasTemperature, NewPing"

export TOSAESP_OTA_PORT=8027
export TOSAESP_OTA_PASSWORD=tosa
export TOSAESP_BUILD_FLAGS=""
cp $TOSAESP_DEVICE_NAME.h src/config_device.h
pio run --target upload
