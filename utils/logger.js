const FS = require("fs");

function getTimePrefix(date) {
	return ('0' + date.getHours()).slice(-2) + ':' +
		('0' + date.getMinutes()).slice(-2) + ':' +
		('0' + date.getSeconds()).slice(-2) + ':' +
		('00' + date.getMilliseconds()).slice(-3);
}

class Logger {
	constructor(logFile) {
		this.filePath = logFile;
		this.sockets = [];
	}

	addSocket(socket) {
		this.sockets.push(socket);
	}

	debug(message) {
		FS.appendFile(this.filePath, getTimePrefix(new Date()) + ' | ' + message + '\n', function (err) {});
	}

	log(message) {
		this.trace(message);

		this.sockets.forEach(function(socket) {
			socket.emit('serveurrobottrace', message);
		});
	}
}

export default { Logger };