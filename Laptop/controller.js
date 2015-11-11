var HID = require('node-hid');
var SerialPort = require("serialport").SerialPort;

var devices = HID.devices();
var controller = new HID.HID(devices[0].path);

var serialPort = new SerialPort("/dev/ttyUSB0", {
    baudrate: 9600
});

var buffer = new Buffer(3);

serialPort.on("open", function() {

    for(;;) {
        serialPort.drain(function(error) {
            if(error) throw error;

            controller.read(function(error, data) {

                if(error) throw error;
                // Left stick y
                buffer.writeUInt8(255 - data[7], 0);
                // Right stick x
                buffer.writeUInt8(data[8], 1);
                // Right stick y
                buffer.writeUInt8(255 - data[9], 2);
                
                console.log("Writing to arduino: ", buffer);
                serialPort.write(buffer);
            });
        });
    }
});

