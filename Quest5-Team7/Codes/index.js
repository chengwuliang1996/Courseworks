// Modules
var level = require('level');
var express = require('express');
var app = require('express')();
var http = require('http').Server(app);
var io = require('socket.io')(http);
var dgram = require('dgram');

// ESP UDP Host & Port
var MAC_PORT = 3000;
var MAC_HOST = '192.168.1.115';

// PI UDP Host & Port
var PI_PORT = 3000;
var PI_HOST = '192.168.1.117';

var data = []; //char data set
var code_list=[];
var DB_id=[];
var DB_code=[];
var DB_name=[];
var DB_valuepair=[] // [0]-refers to the id list; [1] refers to code list
var id;
var code;

// Send Data to the Web Server
var ESP_HOST = '192.168.1.139'
var ESP_PORT = 4000;


// ---------- ESP UDP Server Part ----------- // 
// send UDP msg to ESP
var found_message = new Buffer('1');
var client = dgram.createSocket('udp4');

// ---------- ESP UDP Client Part ----------- // 
// receive UDP msg from ESP

//Create socket listening to ESP msg
var server = dgram.createSocket('udp4');

var db = level('./mydb', {valueEncoding: 'json'});

//create UDP server
server.on('listening', function () {
    var address = server.address();
    console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

// On connection, print out received message
server.on('message', function (message, remote) {
    //parsing data received from ESP msg
    // if (message[0] !== tmp)
    // {
    //   tmp = message[0];
      // convert the message to int type
      data[0] = message[0] - 48 ;
      code_list[0]=(message[1] - 48).toString();
      code_list[1]=(message[2] - 48).toString();
      code_list[2]=(message[3] - 48).toString();
      code_list[3]=(message[4] - 48).toString();
      data[1] = code_list;
      id = data[1];
      code = code_list[0] + code_list[1] + code_list[2] + code_list[3];

      console.log(remote.address + ':' + remote.port +' - ' + message);
      console.log(data[0]);
      console.log(data);
      console.log(code);

      setTimeout(send, 1000);

      // Send Ok acknowledgement
      server.send("Ok!",remote.port,remote.address,function(error){
        if(error){
          console.log('MEH!');
        }
        else{
          console.log('Sent: Ok!');
        }
      });
});

// Bind server to port and IP
server.bind(MAC_PORT, MAC_HOST);


// ---------- HTTP Web Server Part ----------- // 
app.get('/', function(req, res){
    res.sendFile(__dirname + '/index.html');
});

// --------- HTTP Web Server Client connection ------ //
var clientConnected = 0;
io.on('connection', function(data){
    console.log("a user connected");
    clientConnected = 1;
    // readDB();
    data.on('disconnect', function(){
        console.log('user disconnected');
    });
});

function readDB(arg) {
    db.createReadStream()
      .on('data', function (data) {
        // console.log(data.key, '=', data.value)
        // Parsed the data into a structure but don't have to ...
        dataIn = {[data.key]: data.value};
        DB_id.push(data.value[0].id);
        DB_code.push(data.value[0].Code);
        DB_name.push(data.value[0].Name);
        DB_valuepair[0]=DB_id;
        DB_valuepair[1]=DB_code;
        DB_valuepair[2]=DB_name;
        // console.log(DB_id);
        // console.log(data.value[0].id)
        // console.log(DB_code);
      })
      .on('error', function (err) {
        console.log('Oh my!', err)
      })
      .on('close', function () {
        console.log('Stream closed')
      })
      .on('end', function () {
        console.log('Stream ended')
      })
  }

http.listen(8080,function(){
    console.log('listen on *: 8080');
})

var i;
function send(){
    var checked = 'Checked';
    const date = new Date().toString();
    var date_array = date.split(' ');
    var new_date = date_array[0] + " " + date_array[1] + " " + date_array[2] + " " + date_array[3] + " "  
    + date_array[4];
  
    for (i=0; i<DB_code.length;i++)
    {   
        // check if the client data matched the data in the DB
        if (code===DB_valuepair[1][i]){
            // [0]-refers to the id list; [1] refers to code list
            var value_pair = [{"id": DB_id[i], "Status": checked, "Name": DB_name[i]}];
            var msg = {[new_date]: value_pair};
            console.log("A client found!");
            console.log(msg);

            // ----- send UDP msg to ESP ----- //
            client.send(found_message, 0, found_message.length, ESP_PORT, ESP_HOST, function(err, bytes) {
                if (err) throw err;
                console.log('UDP message sent to ' + ESP_HOST +':'+ ESP_PORT);
                //client.close();
            });

            // ------ send client data to Web Server ------ // 
            io.emit('message',msg);
        }
    }
}

// send msg to Web Browser
readDB();