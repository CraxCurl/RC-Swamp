const http = require('http');
const { spawn } = require('child_process');

process.env.PORT = '8899';
const proc = spawn('node', ['server.js'], { stdio: ['ignore', 'pipe', 'pipe'], env: process.env });

proc.stdout.on('data', (d) => console.log('[Server stdout]', d.toString().trim()));
proc.stderr.on('data', (d) => console.log('[Server stderr]', d.toString().trim()));

setTimeout(() => {
  http.get('http://localhost:8899/api/imu/export-csv', (res) => {
    let data = '';
    res.on('data', chunk => data += chunk);
    res.on('end', () => {
      console.log('STATUS:', res.statusCode);
      console.log('CONTENT-TYPE:', res.headers['content-type']);
      console.log('DISPOSITION:', res.headers['content-disposition']);
      console.log('CSV OUTPUT:');
      console.log(data);
      proc.kill();
      process.exit(0);
    });
  }).on('error', (err) => {
    console.error('HTTP error:', err.message);
    proc.kill();
    process.exit(1);
  });
}, 2000);
