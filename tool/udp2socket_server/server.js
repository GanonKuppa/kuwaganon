var client = require('socket.io-client');
var socket = client.connect('ws://devnotepc:3030');
var dgram = require('dgram');

var PORT = 2020;
var HOST = 'devnotepc';

var server = dgram.createSocket('udp4');

server.on('listening', function () {
    var address = server.address();
    console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

server.on('message', function (message, remote) {
    console.log(remote.address + ':' + remote.port +' - ' + message);
    socket.emit("udp_message",message.toString());
});

server.bind(PORT, HOST);