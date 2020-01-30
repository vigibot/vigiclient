const fs = require('fs');
const path = require('path');
const url = require('url');
const unzipper = require('unzipper');

function deleteFileOrFolder(path) {
	let stats = fs.lstatSync(path);
	if (stats.isDirectory()) {
		fs.readdirSync(path).forEach(function (file, index) {
			let curPath = path + "/" + file;
			if (fs.lstatSync(curPath).isDirectory()) { // recurse
				deleteFileOrFolder(curPath);
			} else { // delete file
				fs.unlinkSync(curPath);
			}
		});
		fs.rmdirSync(path);
	}
	else if (stats.isFile()) {
		fs.unlinkSync(path);
	}
};

function downloadAndFollowRedirect(uri) {
	let opts = url.parse(uri);
	opts.headers = {
		'User-Agent': 'javascript'
	};

	let protocol = opts.protocol.slice(0, -1);
	return new Promise(function (resolve, reject) {
		require(protocol).get(opts, function (response) {
			if (response.statusCode >= 200 && response.statusCode < 300) {
				resolve(response);
			} else if (response.headers.location) {
				resolve(downloadAndFollowRedirect(response.headers.location));
			} else {
				reject(new Error(response.statusCode + ' ' + response.statusMessage));
			}
		}).on('error', reject);
	});
};

function getLatestTagInfos(repoOwner, repoId) {
	return new Promise(function (resolve, reject) {
		let opts = url.parse('https://api.github.com/repos/' + repoOwner + '/' + repoId + '/tags');
		opts.headers = {
			'User-Agent': 'javascript'
		};
		require('https').get(opts, function (response) {
			if (response.statusCode >= 200 && response.statusCode < 300) {
				let body = '';
				response.on("data", function (chunk) {
					body += chunk;
				});

				response.on('end', function () {
					var tagList = [];
					try {
						tagList = JSON.parse(body);
					}
					catch(e) {
						return reject('Fail to parse response: ' + e);
					}

					if (tagList.length <= 0) {
						return reject('No tag found in this repo');
					}

					resolve(tagList[0]);
				});
			}
			else {
				reject('Bad response: ' + response.statusCode);
			}
		});
	});
}

function downloadAndUnzipTag(archiveUri, tmpFolder) {
	console.log('Download @ ' + archiveUri);

	return new Promise(function (resolve, reject) {
		// downloadAndFollowRedirect last archive
		downloadAndFollowRedirect(archiveUri).then(function (response) {
			response
				.pipe(unzipper.Extract({ path: tmpFolder })) // Will create tmpFolder if needed
				.on('close', resolve);
		}, reject);
	});
}

function replaceCurrentProject(originPath, currentProjetPath) {
	return new Promise(function (resolve, reject) {
		if (!fs.existsSync(currentProjetPath)) {
			reject('Folder ' + currentProjetPath + ' do not exist !');
		}
		
		try {
			fs.readdirSync(originPath).forEach(function (projectFolder) { // Zip contain a sub-folder...
				fs.readdirSync(originPath + path.sep + projectFolder).forEach(function (file) {
					let destinationFilePath = currentProjetPath + path.sep + file;
					if (fs.existsSync(destinationFilePath)) {
						deleteFileOrFolder(destinationFilePath);
					}
					fs.renameSync(originPath + path.sep + projectFolder + path.sep + file, destinationFilePath);
				});
			});
		}
		catch (e) {
			reject(e);
		}

		resolve();
	});
}

// Work !

const repoOwner = 'waveform80';
const repoId = 'picamera';
const tempFolderPath = 'tmp';

getLatestTagInfos(repoOwner, repoId).then(function (latestTag) {
	console.log('Distant version: ' + latestTag.name);

	let localVersion = require('./package.json').version;
	console.log('Local version: ' + localVersion);

	if (localVersion !== latestTag.name) {
		return downloadAndUnzipTag(latestTag.zipball_url, tempFolderPath).then(function() {
			return Promise.resolve(true);
		});
	}

	return Promise.resolve(false);
}).then(function (hasUpdated) {
	if (hasUpdated) {
		return replaceCurrentProject(tempFolderPath, 'wow');
	}
	
	return Promise.reject('No need to update !');
}).then(function () {
	console.log('Update done');
}).catch(function (e) {
	console.log('Fail: ' + e);
}).finally(function() {
	if (fs.existsSync(tempFolderPath)) {
		deleteFileOrFolder(tempFolderPath);

		console.log(tempFolderPath + ' folder deleted !');
	}
});
