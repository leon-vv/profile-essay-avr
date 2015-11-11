var HID = require('node-hid');
var SerialPort = require("serialport").SerialPort;

var devices = HID.devices();
var controller = new HID.HID(devices[0].path);

var serialPort = new SerialPort("/dev/ttyUSB0", {
});

var buffer = new Buffer(3);

var i = 0;

setTimeout(function() {
    console.log(i);
}, 10000);

function writeLoop(port) {

    i += 1;
    port.write(buffer, function(error) {
        if(error) console.log(error);

        port.drain(function(error) {
            if(error) console.log(error);

            controller.read(function(error, data) {
                if(error) console.log(error);

                // Left stick y
                buffer.writeUInt8(255 - data[7], 0);
                // Right stick x
                buffer.writeUInt8(data[8], 1);
                // Right stick y
                buffer.writeUInt8(255 - data[9], 2);
                
                setTimeout(function() {
                    writeLoop(port);
                }, 236);
            });
        });
    });
}

serialPort.on("error", function(error) {
    console.log(error);
});

serialPort.on("open", function() {

    console.log("Connection opened");

    serialPort.on("data", function( data) {
        console.log(data);
    });
    writeLoop(serialPort);
});

