#!/bin/bash
cd "${0%/*}"
. ${HOCO_HOME}/data/config.sh
sudo apt-get -y install screen
sudo python -c "$(curl -fsSL https://raw.githubusercontent.com/platformio/platformio/master/scripts/get-platformio.py)"
platformio platform install https://github.com/platformio/platform-espressif8266.git#feature/stage

