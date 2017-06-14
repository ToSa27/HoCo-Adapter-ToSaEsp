const fs = require('fs-extra');
const SerialPort = require("serialport");

var serialPort = "COM4";
var serialSpeed = 115200;
if (process.argv.length == 4) {
    serialPort = process.argv[2];
    serialSpeed = parseInt(process.argv[3]);
} else if (fs.existsSync("config.json")) {
    var config = JSON.parse(fs.readFileSync("config.json"));
    serialPort = config.serial.port;
    serialSpeed = config.serial.speed;
}
var port = new SerialPort(serialPort, {
    baudRate: serialSpeed,
    parser: SerialPort.parsers.readline('\n')
}).on('data', (data) => {
    console.log(serialPort + ': ' + data);
});