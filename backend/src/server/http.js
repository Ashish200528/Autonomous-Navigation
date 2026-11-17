// Dashboard server: static page, MJPEG stream, JSON API, WebSocket state (10 Hz).
// Viewing needs nothing; anything that moves the robot or changes config needs the token.
import { createServer } from 'node:http';
import { readFile } from 'node:fs/promises';
import { timingSafeEqual } from 'node:crypto';
import { WebSocketServer } from 'ws';

const PUBLIC = new URL('../../public/', import.meta.url);
const TYPES = { html: 'text/html', js: 'text/javascript', css: 'text/css' };

export function startServer(app, { port, mjpeg_fps }, secret) {
  const okToken = (t) => typeof t === 'string' && t.length === secret.length && timingSafeEqual(Buffer.from(t), Buffer.from(secret));
  const json = (res, code, obj) => { res.writeHead(code, { 'content-type': 'application/json' }); res.end(JSON.stringify(obj)); };
  const body = (req) => new Promise((ok, bad) => {
    let s = '';
    req.on('data', (c) => { s += c; if (s.length > 1e5) req.destroy(); });
    req.on('end', () => { try { ok(s ? JSON.parse(s) : {}); } catch (e) { bad(e); } });
  });

  // MJPEG: one encode per tick, shared by all viewers; nothing is encoded when nobody watches.
  const viewers = new Set();
  let timer = null;
  const pump = async () => {
    const jpg = await app.jpeg().catch(() => null);
    if (!jpg) return;
    for (const res of viewers) res.write(`--f\r\nContent-Type: image/jpeg\r\nContent-Length: ${jpg.length}\r\n\r\n`), res.write(jpg), res.write('\r\n');
  };

  const routes = {
    'GET /api/health': (req, res) => json(res, 200, app.health()),
    'GET /api/config': (req, res) => json(res, 200, app.getConfig()),
    'PUT /api/config': async (req, res) => json(res, 200, app.patchConfig(await body(req))),
    'POST /api/mode': async (req, res) => json(res, 200, app.setMode((await body(req)).mode)),
    'POST /api/go': (req, res) => json(res, 200, app.go()),
    'POST /api/clear': (req, res) => json(res, 200, app.clear()),
    'POST /api/calibrate': async (req, res) => { const b = await body(req); json(res, 200, app.calibrate(b.class, Number(b.distance_m))); },
    'POST /api/calibrate/reset': (req, res) => json(res, 200, app.resetSelfcal()),
    'POST /api/record': async (req, res) => json(res, 200, app.record(!!(await body(req)).on)),
    'POST /api/map/clear': (req, res) => json(res, 200, app.clearMap()),
  };

  const server = createServer(async (req, res) => {
    const url = new URL(req.url, 'http://x');
    const key = `${req.method} ${url.pathname}`;
    try {
      if (routes[key]) {
        if (req.method !== 'GET' && !okToken(req.headers['x-token'])) return json(res, 401, { error: 'token required' });
        return await routes[key](req, res);
      }
      if (key === 'GET /stream.mjpg') {
        res.writeHead(200, { 'content-type': 'multipart/x-mixed-replace; boundary=f', 'cache-control': 'no-cache' });
        viewers.add(res);
        timer ??= setInterval(pump, 1000 / mjpeg_fps);
        req.on('close', () => { viewers.delete(res); if (!viewers.size) { clearInterval(timer); timer = null; } });
        return;
      }
      if (req.method === 'GET') {
        const name = url.pathname === '/' ? 'index.html' : url.pathname.slice(1);
        if (!/^[\w.-]+$/.test(name)) return json(res, 404, { error: 'not found' });
        const data = await readFile(new URL(name, PUBLIC));
        res.writeHead(200, { 'content-type': TYPES[name.split('.').pop()] ?? 'application/octet-stream' });
        return res.end(data);
      }
      json(res, 404, { error: 'not found' });
    } catch (e) {
      if (!res.headersSent) json(res, e.code === 'ENOENT' ? 404 : 400, { error: e.message });
    }
  });

  const wss = new WebSocketServer({ server, path: '/ws' });
  wss.on('connection', (ws) => {
    ws.on('message', (raw) => {
      let m;
      try { m = JSON.parse(raw); } catch { return; }
      if (!okToken(m.token)) return ws.send(JSON.stringify({ type: 'error', error: 'token required' }));
      if (m.type === 'drive') app.manual(Number(m.v) || 0, Number(m.w) || 0);
      else if (m.type === 'mode') app.setMode(m.mode);
    });
  }); // manual driving stops by itself 500 ms after the last drive message (arbiter)
  setInterval(() => {
    if (!wss.clients.size) return;
    const msg = JSON.stringify({ type: 'state', ...app.getState() });
    for (const c of wss.clients) if (c.readyState === 1) c.send(msg);
  }, 100);
  // 3D point cloud as binary Float32 [x, y, z, kind]* at 2 Hz (plan §8)
  setInterval(() => {
    if (!wss.clients.size) return;
    const buf = Buffer.from(app.cloud().buffer);
    for (const c of wss.clients) if (c.readyState === 1) c.send(buf);
  }, 500);

  server.listen(port, () => console.log(`dashboard on http://<pi-address>:${port}/`));
  return server;
}
