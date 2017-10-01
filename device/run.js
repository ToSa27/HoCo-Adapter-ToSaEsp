const fs = require('fs-extra');
const SerialPort = require("serialport");
const child_process = require('child_process');
const mqtt = require('mqtt');

function reset(envs) {
    var mqttc = mqtt.connect({
        host: envs["TOSAESP_MQTT_HOST"],
        port: envs["TOSAESP_MQTT_PORT"],
        user: envs["TOSAESP_MQTT_USER"],
        pass: envs["TOSAESP_MQTT_PASS"],
        secureProtocol: 'TLSv1_method'
    });
    mqttc.publish("hoco/esp_pio/" + envs["TOSAESP_DEVICE_NAME"] + "/$reboot", "");
    console.log('reboot command submitted');
};

function build(envs, upload, cb) {
    console.log('pio starting for %s at %s', envs["TOSAESP_DEVICE_NAME"], envs["TOSAESP_IP"]);
    var pio = child_process.spawn("pio", upload ? ["run", "-v", "--target", "upload"] : ["run", "-v"], { env: envs, stdio: "inherit" })
        .on('exit', (code) => {
            console.log('pio exited with code %d', code);
            cb(code);
        });
};

function upload_ota(envs, cb) {
    console.log('ota starting for %s at %s', envs["TOSAESP_DEVICE_NAME"], envs["TOSAESP_IP"]);
    var py = child_process.spawn("python", ["ota.py", "--debug", "--progress", "-i", envs["TOSAESP_IP"], "--port=" + envs["TOSAESP_OTA_PORT"], "--auth=" + envs["TOSAESP_OTA_PASS"], "-f", ".pioenvs/nodemcuv2/firmware.bin"], { stdio: "inherit" })
        .on('exit', (code) => {
            console.log('ota exited with code %d', code);
            cb(code);
        });
};

function scanip(port, speed, cb) {
    console.log('scanning for ip address');
    var ipPrefix = "WiFi Connected: IP:";
    var sp = new SerialPort(port, {
        baudRate: speed,
        parser: SerialPort.parsers.readline('\n')
    }).on('data', (data) => {
        if (data.startsWith(ipPrefix)) {
            var ip = data.substring(ipPrefix.length).trim();
            console.log('detected ip %s', ip);
            cb(ip);
            sp.close();
        }
    });
}

var envs = {};
if (process.argv.length != 3) {
    console.log("Usage: node run.js {device name}");
    process.exit(1);
}
var device = process.argv[2];
envs["TOSAESP_DEVICE_NAME"] = device;
if (!fs.existsSync("config.json")) {
    console.log("Create config.json from template!");
    process.exit(1);
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
    process.exit(1);
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
envs["TOSAESP_BUILD_FLAGS"] = "";
envs["TOSAESP_BOARD"] = "nodemcuv2";
envs["TOSAESP_BOARDFLASHMODE"] = "qio";
envs["TOSAESP_LIB_DEPS"] = "";
var headerReader = require('readline').createInterface({
    input: fs.createReadStream(device + ".h")
});
headerReader.on('line', (line) => {
    if (line.startsWith('//env:')) {
        var envname = line.substring(6);
        envname = envname.substring(0, envname.indexOf('=')).trim();
        var envval = line.substring(6);
        envval = envval.substring(envval.indexOf('=') + 1).trim();
        if (envval.length > 0) {
            console.log('ENV: ' + envname + ' : ' + (envs[envname] | "") + ' -> ' + envval);
            envs[envname] = envval;
        }
    }
});
headerReader.on('close', () => {
    build(envs, !network.hasOwnProperty(device), (code) => {
        if (code > 0)
            console.log("Build failed...");
        else if (network.hasOwnProperty(device)) {
            reset(envs);
            upload_ota(envs, (code) => {
                process.exit(code);
            });
        } else {
            scanip(config.serial.port, config.serial.speed, (ip) => {
                network[device] = ip;
                fs.writeFileSync("network.json", JSON.stringify(network));
                process.exit(0);
            });
        }
    });
});
