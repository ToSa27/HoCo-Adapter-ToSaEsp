const fs = require('fs-extra');
const SerialPort = require("serialport");
const child_process = require('child_process');

var envs = {};
if (process.argv.length != 3) {
    console.log("Usage: node run.js {device name}");
    exit(1);
}
var device = process.argv[2];
envs["TOSAESP_DEVICE_NAME"] = device;
if (!fs.existsSync("config.json")) {
    console.log("Create config.json from template!");
    exit(1);
}
var config = JSON.parse(fs.readFileSync("config.json"));
envs["TOSAESP_OTA_PORT"] = config.ota.port;
envs["TOSAESP_OTA_PASS"] = config.ota.pass;
envs["TOSAESP_WIFI_SSID"] = config.wifi.ssid;
envs["TOSAESP_WIFI_PASS"] = config.wifi.pass;
envs["TOSAESP_MQTT_HOST"] = config.mqtt.host;
envs["TOSAESP_MQTT_PORT"] = config.mqtt.port;
envs["TOSAESP_MQTT_USER"] = config.mqtt.user;
envs["TOSAESP_MQTT_PASS"] = config.mqtt.pass;
if (!fs.existsSync(device + ".h")) {
    console.log("Create " + device + ".h file!");
    exit(1);
}
fs.copySync(device + ".h", "src/device.h");
var port = config.serial.port;
var network = {};
if (fs.existsSync("network.json"))
    network = JSON.parse(fs.readFileSync("network.json"));
if (network.hasOwnProperty(device)) {
    port = network[device];
    envs["TOSAESP_UPLOAD_FLAGS"] = "--port=" + config.ota.port + " --auth=" + config.ota.pass + " --progress";
} else {
    envs["TOSAESP_UPLOAD_FLAGS"] = "";
}
envs["TOSAESP_IP"] = port;
envs["TOSAESP_LIB_DEPS"] = "OneWire, DallasTemperature, PCF8574, NewPing";
envs["TOSAESP_BUILD_FLAGS"] = "";
console.log('pio starting for %s at %s', device, port);
var pio = child_process.spawn("pio", ["run", "-v", "--target", "upload"], { env: envs, stdio: "inherit" })
    .on('exit', (code) => {
        console.log('pio exited with code %d', code);
        if (code == 0 && !network.hasOwnProperty(device)) {
            console.log('scanning for ip address');
            var ipPrefix = "WiFi Connected: IP:";
            var sp = new SerialPort(config.serial.port, {
                baudRate: config.serial.speed,
                parser: SerialPort.parsers.readline('\n')
            }).on('data', (data) => {
                if (data.startsWith(ipPrefix)) {
                    var ip = data.substring(ipPrefix.length).trim();
                    console.log('detected ip %s', ip);
                    network[device] = ip;
                    fs.writeFileSync("network.json", JSON.stringify(network));
                    process.exit(0);
                }
            });
        }
    });