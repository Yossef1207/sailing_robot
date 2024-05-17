// -----------------------------------------------------------------
//		General settings and variables
// -----------------------------------------------------------------
const app = require('express')();
const express = require('express');
const server = require('http').createServer(app);
const io = require('socket.io')(server,{
	cors: {
		origin: '*',
	  }
});
const config = require("./config/config.js");
const { response } = require('express');
// comment for test commit
// Logger Routine
var logger = require('loglevel');
logger.enableAll();

// set allowed functions
app.use(function (req, res, next) {
    res.setHeader('Access-Control-Allow-Origin', '*');
    res.setHeader('Access-Control-Allow-Methods', 'GET, POST, OPTIONS, PUT, PATCH, DELETE');
    res.setHeader('Access-Control-Allow-Headers', 'X-Requested-With,content-type');
    res.setHeader('Access-Control-Allow-Credentials', true);
    next();
});
app.use(express.urlencoded({ extended: false }));   //is needed for the contents of post method
app.use(express.json());                            //is needed for the contents of post method

// define timeout-callback-function
const withTimeout = (onSuccess, onTimeout, timeout) => {  
	let called = false;
	const timer = setTimeout(() => {    
		if (called) return;
		called = true;    
		onTimeout();  
	}, timeout);
	return (...args) => { 
		if (called) return;
		called = true;
		clearTimeout(timer);
		onSuccess.apply(this, args);
	}
}

// --------------------------------------------------------------------------------
//	Routing - URL
// --------------------------------------------------------------------------------

/*app.post('/devices/editdevice/:id', (req, res) => {
	editentry
});
app.get('/devices/:raspi_id/getstatus', (req, res) => {   
	var raspi_id = this.raspi_socket.get(socket.id);
		try{
			socket.to("frontend").emit("status", "1");
		}catch (e){
			console.log("No Frontend connected!")
		}		
});*/

//connection routine
io.on('connection', (socket) => {
    console.log('a user connected: ' + socket.id);
	socket.on('log_on', message =>{
		if(message == 'frontend'){	//Frontend connection
			socket.join("frontend");
			console.log("frontend");	
		}else  {
			console.log("sailboat connected");	
		}
	});
	socket.on("gps_coordinates", message => {
		socket.to("frontend").emit("sailboat_data", message);
	})
	//disconnect routine
	socket.on('disconnect', message =>{
			console.log('a user has disconnected '+socket.id);
	});
});

// Start the server on Port config.PORT (in config.js)
server.listen(config.PORT, () => console.log('Server running on port ' + config.PORT));
