var express = require('express');
var app = require('express')();
var http = require('http').Server(app);
var io = require('socket.io')(http);

// http.createServer(function (req, res) {
//     res.writeHead(200, {'Content-Type': 'text/plain'});
//     res.end('quest3');
// }).listen(8080);

//on-click

app.get('/', function(req, res){
  res.sendFile(__dirname + '/test.html');
});

io.on('connection', function(socket){
  console.log('a user connected');
  socket.on('disconnect', function(){
    console.log('user disconnected');
  });
});

http.listen(8080, function() {
  console.log('listening on *:8080');
});
