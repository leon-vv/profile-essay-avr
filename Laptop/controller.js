var HID = require('node-hid');

var devices = HID.devices();
var controller = new HID.HID(devices[0].path);

controller.on('data', function(data) {
    console.log('(' + data[6] + ',' + data[7] + ')', '(' + data[8] + ',' + data[9] + ')');
});
