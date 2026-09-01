#!/usr/bin/env node
/**
 * Simple Node.js HTTP server with CORS headers for SharedArrayBuffer support.
 * Enables Cross-Origin-Opener-Policy and Cross-Origin-Embedder-Policy.
 *
 * Usage: node demo/node_server.js
 *        PORT=8010 node demo/node_server.js
 */

import http from 'http';
import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

const DEFAULT_PORT = 8000;
const BASE_DIR = __dirname;

const mimeTypes = {
  '.html': 'text/html',
  '.js': 'text/javascript',
  '.css': 'text/css',
  '.json': 'application/json',
  '.png': 'image/png',
  '.jpg': 'image/jpg',
  '.gif': 'image/gif',
  '.svg': 'image/svg+xml',
  '.wav': 'audio/wav',
  '.mp4': 'video/mp4',
  '.woff': 'application/font-woff',
  '.ttf': 'application/font-ttf',
  '.eot': 'application/vnd.ms-fontobject',
  '.otf': 'application/font-otf',
  '.wasm': 'application/wasm',
};

const server = http.createServer((req, res) => {
  const urlPath = req.url.split('?')[0];
  let relativePath = urlPath === '/' ? 'index.html' : urlPath.replace(/^\//, '');
  let filePath = path.join(BASE_DIR, relativePath);

  if (fs.existsSync(filePath) && fs.statSync(filePath).isDirectory()) {
    filePath = path.join(filePath, 'index.html');
  }

  const extname = String(path.extname(filePath)).toLowerCase();
  const mimeType = mimeTypes[extname] || 'application/octet-stream';

  fs.readFile(filePath, (error, content) => {
    if (error) {
      if (error.code === 'ENOENT') {
        res.writeHead(404, { 'Content-Type': 'text/html' });
        res.end('<h1>404 Not Found</h1>', 'utf-8');
      } else {
        res.writeHead(500);
        res.end(`Server Error: ${error.code}`, 'utf-8');
      }
    } else {
      res.writeHead(200, {
        'Content-Type': mimeType,
        'Cross-Origin-Opener-Policy': 'same-origin',
        'Cross-Origin-Embedder-Policy': 'require-corp',
        'Cross-Origin-Resource-Policy': 'cross-origin',
        'Access-Control-Allow-Origin': '*',
        'Cache-Control':
          extname === '.js' || extname === '.wasm' ? 'no-cache, no-store, must-revalidate' : 'public, max-age=3600',
        Pragma: 'no-cache',
        Expires: '0',
      });
      res.end(content, 'utf-8');
    }
  });
});

const port = Number(process.env.PORT) || DEFAULT_PORT;

server.on('error', (e) => {
  if (e.code === 'EADDRINUSE') {
    console.error(`Port ${port} is already in use.`);
    console.error('Kill other node servers, then retry.');
    console.error(`Or pick another port: set PORT=${port + 1} && node demo/node_server.js`);
  } else {
    console.error('Server error:', e);
  }
  process.exit(1);
});

server.listen(port, () => {
  console.log(`Server running at http://localhost:${port}/`);
  console.log(`Serving files from: ${BASE_DIR}`);
  console.log('SharedArrayBuffer enabled (COOP & COEP headers set)');
  console.log('Press Ctrl+C to stop\n');
});
