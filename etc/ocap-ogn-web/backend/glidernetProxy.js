const net = require('net');
const events = require('events');
const express = require('express');
const WebSocket = require('ws');
const app = express();
const port = 3000;

// WebSocket server for Angular frontend
const wss = new WebSocket.Server({ port: 8080 });

// Connected WebSocket clients
const clients = new Set();

// WebSocket connection handling
wss.on('connection', (ws) => {
  console.log('WebSocket client connected');
  clients.add(ws);
  ws.on('close', () => {
    console.log('WebSocket client disconnected');
    clients.delete(ws);
  });
});

// APRS TCP client
const aprsClient = new net.Socket();
const aprsHost = 'aprs.glidernet.org';
const aprsPort = 14580;
const aprsUser = 'FOCACH';
const aprsPass = '-1'; // rx only
const aprsFilter = 'r/46.8182/8.2275/250';

const loginString = `user ${aprsUser} pass ${aprsPass} soft OcapOgnViewr 0.1 filter ${aprsFilter}\r\n`;

aprsClient.connect(aprsPort, aprsHost, () => {
  console.log('Connected to APRS server');
  aprsClient.write(loginString);
});

let buffer = '';
aprsClient.on('data', (data) => {
  buffer += data.toString();
  const lines = buffer.split('\r\n');
  buffer = lines.pop(); // Keep incomplete line in buffer
  lines.forEach((line) => {
    if (line && !line.startsWith('#')) { // ignore comment lines
      console.log('Received:', line);
      // Broadcast to all WebSocket clients
      clients.forEach((client) => {
        if (client.readyState === WebSocket.OPEN) {
          client.send(JSON.stringify({ message: line }));
        }
      });
    }
  });
});

aprsClient.on('error', (err) => {
  console.error('APRS connection error:', err.message);
  // Reconnect on error.
  setTimeout(() => aprsClient.connect(aprsPort, aprsHost), 5000);
});

aprsClient.on('close', () => {
  console.log('APRS connection closed');
});

// Express server for basic API (optional)
app.get('/health', (req, res) => {
  res.json({ status: 'Backend running' });
});

app.listen(port, () => {
  console.log(`Express server running on http://localhost:${port}`);
});

