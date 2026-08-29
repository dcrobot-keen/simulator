// The simulator server. Two WebSocket endpoints:
//
//   :8765  Roboteq ASCII protocol — identical surface to robot-base/sim, so
//          robot-os-chromium's WebSocketTransport / dashboard connect
//          unchanged. `?C` encoder counts come from the physics body.
//   :8766  sensor stream — newline-delimited JSON, one frame per scan tick:
//          { t, groundTruth:{x,y,theta}, scan:{...} }. This is what a real
//          LIDAR + localization bridge would feed the nav layer
//          (roadmap.md Phase 7). Split from :8765 on purpose: on the real
//          robot the laser is a different device from the motor controller.
//
// Keeps the load-bearing safety property: the Roboteq RWD watchdog stops
// the wheels if no command arrives for RWD ms, independent of any client.

import { createServer } from 'node:http';
import { readFile } from 'node:fs/promises';
import { WebSocketServer } from 'ws';
import { encodeCommand, RoboteqDecoder } from './roboteq.js';
import { Robot, TB3 } from './robot.js';
import { scan, LDS01 } from './lidar.js';

export function startSimulator({
  world,
  robotPort = Number(process.env.SIM_PORT || 8765),
  sensorPort = Number(process.env.SIM_SENSOR_PORT || 8766),
  viewerPort = Number(process.env.SIM_VIEWER_PORT || 8767),
  rwdMs = Number(process.env.SIM_RWD_MS || 1000),
  tickHz = 50,
  scanHz = 10,
  start,
  log = (m) => console.log(`[sim ${new Date().toISOString()}] ${m}`),
} = {}) {
  const robot = new Robot(start ?? { x: world.bounds[0] / 2, y: world.bounds[1] / 2, theta: 0 });

  const state = {
    cmd: [0, 0],          // last !G per channel, -1000..1000
    motorEnabled: false,  // !MG / !EX
    estopped: false,      // latched by !EX or RWD
    voltage: 12.4,        // fake (TB3 is ~11.1V nominal)
    temperature: 30.0,
    lastCmdAt: Date.now(),
  };

  function stopMotors(reason) {
    if (state.estopped) return;
    state.cmd[0] = state.cmd[1] = 0;
    state.motorEnabled = false;
    state.estopped = true;
    log(`motors zeroed — ${reason}`);
  }

  function reply(ws, line) {
    if (ws.readyState === ws.OPEN) ws.send(encodeCommand(line));
  }

  function handleSub(ws, sub) {
    sub = sub.trim();
    if (sub === '') return;
    state.lastCmdAt = Date.now(); // any command feeds the RWD watchdog

    if (sub === '?FID') return reply(ws, 'FID=ROBOTEQ SIM (2D world) - see roadmap.md');
    if (sub === '?A') return reply(ws, `A=${Math.round(Math.abs(state.cmd[0]) / 2)}:${Math.round(Math.abs(state.cmd[1]) / 2)}`);
    if (sub === '?AI') return reply(ws, 'AI=0:0:0:0:0');
    if (sub === '?C') return reply(ws, `C=${robot.enc[0]}:${robot.enc[1]}`);
    if (sub === '?FF') return reply(ws, `FF=${state.motorEnabled ? 0 : 16}`);
    if (sub.startsWith('?T')) return reply(ws, `T=${state.temperature.toFixed(0)}`);
    if (sub.startsWith('?V')) return reply(ws, `V=${Math.round(state.voltage * 10)}`);
    if (sub === '?DI') return reply(ws, `DI=${state.estopped ? 0 : 1}:0:0`);

    if (sub === '^ECHOF 1') return reply(ws, '+');
    if (sub.startsWith('!R ')) return reply(ws, '+');
    if (sub.startsWith('!B ')) return reply(ws, '+');
    if (sub.startsWith('!AC ') || sub.startsWith('!DC ')) return reply(ws, '+');
    if (sub.startsWith('!C ')) { robot.enc[0] = robot.enc[1] = 0; return reply(ws, '+'); }
    if (sub === '!MG') { state.motorEnabled = true; state.estopped = false; log('!MG — motors enabled'); return reply(ws, '+'); }
    if (sub === '!EX') { stopMotors('!EX emergency stop'); return reply(ws, '+'); }

    const g = sub.match(/^!G\s+([12])\s+(-?\d+)$/);
    if (g) {
      const ch = Number(g[1]) - 1;
      const val = Math.max(-1000, Math.min(1000, Number(g[2])));
      if (state.motorEnabled && !state.estopped) state.cmd[ch] = val;
      return reply(ws, '+');
    }
    return reply(ws, '-');
  }

  // --- Roboteq endpoint ---------------------------------------------------
  const robotWss = new WebSocketServer({ port: robotPort });
  robotWss.on('connection', (ws) => {
    log(`roboteq client connected — RWD watchdog (re)armed`);
    state.lastCmdAt = Date.now();
    state.estopped = false;
    state.motorEnabled = false;
    state.cmd[0] = state.cmd[1] = 0;
    const dec = new RoboteqDecoder();
    ws.on('message', (data) => {
      for (const m of dec.push(new Uint8Array(data))) {
        if (m.type === 'line') for (const sub of m.raw.split('_')) handleSub(ws, sub);
      }
    });
    ws.on('close', () => log('roboteq client disconnected — RWD watchdog keeps running'));
    ws.on('error', () => {});
  });

  // --- sensor endpoint --------------------------------------------------
  const sensorWss = new WebSocketServer({ port: sensorPort });
  sensorWss.on('connection', (ws) => {
    log('sensor client connected');
    ws.send(JSON.stringify({ type: 'hello', world: { name: world.name, bounds: world.bounds, segments: world.segments }, robot: TB3, lidar: LDS01 }) + '\n');
    ws.on('error', () => {});
  });
  function broadcastSensor(obj) {
    const line = JSON.stringify(obj) + '\n';
    for (const c of sensorWss.clients) if (c.readyState === c.OPEN) c.send(line);
  }

  // --- sim loop -------------------------------------------------------
  const dt = 1 / tickHz;
  const physTimer = setInterval(() => {
    const units = state.estopped ? [0, 0] : state.cmd;
    robot.step(units, dt, world);
    if (!state.estopped && Date.now() - state.lastCmdAt > rwdMs) {
      stopMotors(`RWD: no serial command for >${rwdMs}ms`);
    }
  }, 1000 / tickHz);

  const scanTimer = setInterval(() => {
    const gt = robot.pose();
    broadcastSensor({ type: 'frame', t: Date.now(), groundTruth: gt, collided: robot.collided, scan: scan(world, gt) });
  }, 1000 / scanHz);

  // --- viewer (serves the one HTML file over http) --------------------
  const viewerUrl = new URL('../viewer.html', import.meta.url);
  const httpd = createServer(async (req, res) => {
    try {
      const html = await readFile(viewerUrl);
      res.writeHead(200, { 'Content-Type': 'text/html' });
      res.end(html);
    } catch (e) {
      res.writeHead(500);
      res.end(String(e));
    }
  });
  httpd.listen(viewerPort);

  log(`world "${world.name}" ${world.bounds[0]}x${world.bounds[1]}m — roboteq ws://127.0.0.1:${robotPort} (RWD ${rwdMs}ms), sensors ws://127.0.0.1:${sensorPort}, viewer http://127.0.0.1:${viewerPort}`);

  return {
    robot,
    state,
    stop() {
      clearInterval(physTimer);
      clearInterval(scanTimer);
      robotWss.close();
      sensorWss.close();
      httpd.close();
    },
  };
}
