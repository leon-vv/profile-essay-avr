var HID = require('node-hid');
var SerialPort = require("serialport").SerialPort;

var app = require("express")();
var http = require("http").Server(app);
var io = require("socket.io")(http);


var devices = HID.devices();
var controller = new HID.HID(devices[0].path);

var serialPort = new SerialPort("/dev/ttyUSB0", {
	baudrate: 19200
});

var buffer = new Buffer(3);

serialPort.on("error", function(error) {
    console.log(error);
});

// Called when the connection with the Arduino is opened
serialPort.on("open", function() {

    console.log("Connection with controller opened");

	// Called when the Arduino sends some data.
    serialPort.on("data", function(data) {

		controller.read(function(error, data) {
			if(error) console.log(error);

			// Left stick y
			buffer.writeUInt8(255 - data[7], 0);
			// Right stick x
			buffer.writeUInt8(data[8], 1);
			// Right stick y
			buffer.writeUInt8(255 - data[9], 2);
			
			// Write the control bytes to the Arduino.	
			serialPort.write(buffer, function(error) {
				if(error) console.log(error);

				io.emit("control-data", {
					esc: buffer.readUInt8(0),
					servo1: buffer.readUInt8(1),
					servo2: buffer.readUInt8(2)
				});
			});
		});

    });
});

// Setup a small HTTP server such that
// we can keep track of the control bytes
// with a browser.
app.get("/", function(req, res) {
	res.sendFile(__dirname + "/index.html");	
});

io.on("connection", function(socket) {
	console.log("Connection with browser opened");
});

http.listen(8888, function() {
	console.log("Listening on localhost:8888");
});
