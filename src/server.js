// The simulator server. Per robot, two WebSocket endpoints:
//
//   :8765  Roboteq ASCII protocol — identical surface to robot-base/sim, so
//          robot-os-chromium's WebSocketTransport / dashboard connect
//          unchanged. `?C` encoder counts come from the physics body.
//   :8766  sensor stream — newline-delimited JSON, one frame per scan tick:
//          { t, robotId, groundTruth:{x,y,theta}, odom, collided, scan:{...},
//            others:[{id,x,y,theta}] }. This is what a real LIDAR +
//          localization bridge would feed the nav layer (roadmap.md Phase 7).
//          Split from :8765 on purpose: on the real robot the laser is a
//          different device from the motor controller.
//
// Several robots share ONE world (2026-09-05): robot i listens on
// robotPort + 10*i / sensorPort + 10*i (8765/8766, 8775/8776, ...), so every
// existing client keeps its own pair of ports and needs no protocol change.
// Robots collide with each other and see each other on the LIDAR -- that is
// the point: a second robot is the "dynamic obstacle" the nav stack must
// stop for. The viewer (one http port) shows all of them.
//
// Keeps the load-bearing safety property: the Roboteq RWD watchdog stops
// the wheels if no command arrives for RWD ms, independent of any client.

import { createServer } from 'node:http';
import { readFile } from 'node:fs/promises';
import { WebSocketServer } from 'ws';
import { encodeCommand, RoboteqDecoder } from './roboteq.js';
import { Robot, TB3 } from './robot.js';
import { scan, LDS01 } from './lidar.js';
import { makeRng } from './noise.js';

/**
 * The world as one robot sees it: the shared walls plus every OTHER robot as a
 * disc. Implements the two methods Robot.step() and scan() use (raycast,
 * collides), so neither needs to know about multi-robot at all.
 */
class WorldView {
  constructor(world, self, others) {
    this.name = world.name;
    this.bounds = world.bounds;
    this.segments = world.segments;
    this._world = world;
    this._self = self;
    this._others = others; // array of Robot (live positions)
  }

  raycast(ox, oy, ang, max) {
    let best = this._world.raycast(ox, oy, ang, max);
    const dx = Math.cos(ang), dy = Math.sin(ang);
    for (const o of this._others) {
      if (o === this._self) continue;
      // ray-circle intersection, circle = other robot's body
      const cx = o.x - ox, cy = o.y - oy;
      const proj = cx * dx + cy * dy;
      if (proj <= 0) continue;
      const perp2 = cx * cx + cy * cy - proj * proj;
      const r2 = TB3.bodyRadius * TB3.bodyRadius;
      if (perp2 > r2) continue;
      const t = proj - Math.sqrt(r2 - perp2);
      if (t >= 0 && t < best) best = t;
    }
    return best;
  }

  collides(cx, cy, r) {
    if (this._world.collides(cx, cy, r)) return true;
    for (const o of this._others) {
      if (o === this._self) continue;
      const d = Math.hypot(o.x - cx, o.y - cy);
      if (d <= r + TB3.bodyRadius) return true;
    }
    return false;
  }
}

export function startSimulator({
  world,
  robotPort = Number(process.env.SIM_PORT || 8765),
  sensorPort = Number(process.env.SIM_SENSOR_PORT || 8766),
  viewerPort = Number(process.env.SIM_VIEWER_PORT || 8767),
  rwdMs = Number(process.env.SIM_RWD_MS || 1000),
  tickHz = 50,
  scanHz = 10,
  start,
  robots = null,  // [{ id, start:{x,y,theta}|null, robotPort?, sensorPort? }]; null = one robot from `start`
  noise = null,   // null = perfect sensors; see noise.js
  seed = 1,
  floor = null,   // { pngPath, width, height, extent:[minX,minY,maxX,maxY] } -> viewer background
  log = (m) => console.log(`[sim ${new Date().toISOString()}] ${m}`),
} = {}) {
  const rng = makeRng(seed);
  const specs = robots && robots.length ? robots : [{ id: 'robot', start }];
  const defaultStart = world.start ?? { x: world.bounds[0] / 2, y: world.bounds[1] / 2, theta: 0 };

  const sims = specs.map((spec, i) => ({
    id: spec.id ?? `robot-${i + 1}`,
    robot: new Robot(spec.start ?? defaultStart),
    robotPort: spec.robotPort ?? robotPort + 10 * i,
    sensorPort: spec.sensorPort ?? sensorPort + 10 * i,
    state: {
      cmd: [0, 0],          // last !G per channel, -1000..1000
      motorEnabled: false,  // !MG / !EX
      estopped: false,      // latched by !EX or RWD
      voltage: 12.4,        // fake (TB3 is ~11.1V nominal)
      temperature: 30.0,
      lastCmdAt: Date.now(),
    },
    robotWss: null,
    sensorWss: null,
  }));
  const bodies = sims.map((s) => s.robot);
  for (const sim of sims) sim.view = new WorldView(world, sim.robot, bodies);

  const tag = (sim) => (sims.length > 1 ? `[${sim.id}] ` : '');

  for (const sim of sims) {
    const { robot, state } = sim;

    function stopMotors(reason) {
      if (state.estopped) return;
      state.cmd[0] = state.cmd[1] = 0;
      state.motorEnabled = false;
      state.estopped = true;
      log(`${tag(sim)}motors zeroed — ${reason}`);
    }
    sim.stopMotors = stopMotors;

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
      if (sub === '!MG') { state.motorEnabled = true; state.estopped = false; log(`${tag(sim)}!MG — motors enabled`); return reply(ws, '+'); }
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

    // --- Roboteq endpoint -----------------------------------------------
    sim.robotWss = new WebSocketServer({ port: sim.robotPort });
    sim.robotWss.on('connection', (ws) => {
      log(`${tag(sim)}roboteq client connected — RWD watchdog (re)armed`);
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
      ws.on('close', () => log(`${tag(sim)}roboteq client disconnected — RWD watchdog keeps running`));
      ws.on('error', () => {});
    });

    // --- sensor endpoint -------------------------------------------------
    sim.sensorWss = new WebSocketServer({ port: sim.sensorPort });
    sim.sensorWss.on('connection', (ws) => {
      log(`${tag(sim)}sensor client connected`);
      ws.send(JSON.stringify({
        type: 'hello',
        robotId: sim.id,
        world: { name: world.name, bounds: world.bounds, segments: world.segments },
        robot: TB3, lidar: LDS01, noise,
        robots: sims.map((s) => ({ id: s.id, robotPort: s.robotPort, sensorPort: s.sensorPort })),
        floor: floor ? { url: '/floor.png', extent: floor.extent, width: floor.width, height: floor.height } : null,
      }) + '\n');
      ws.on('error', () => {});
    });
  }

  function broadcastSensor(sim, obj) {
    const line = JSON.stringify(obj) + '\n';
    for (const c of sim.sensorWss.clients) if (c.readyState === c.OPEN) c.send(line);
  }

  // --- sim loop ---------------------------------------------------------
  const dt = 1 / tickHz;
  const physTimer = setInterval(() => {
    for (const sim of sims) {
      const { robot, state } = sim;
      const units = state.estopped ? [0, 0] : state.cmd;
      robot.step(units, dt, sim.view, noise, rng);
      if (!state.estopped && Date.now() - state.lastCmdAt > rwdMs) {
        sim.stopMotors(`RWD: no serial command for >${rwdMs}ms`);
      }
    }
  }, 1000 / tickHz);

  const scanTimer = setInterval(() => {
    const poses = sims.map((s) => ({ id: s.id, ...s.robot.pose(), collided: s.robot.collided }));
    for (const sim of sims) {
      const gt = sim.robot.pose();
      broadcastSensor(sim, {
        type: 'frame', t: Date.now(), robotId: sim.id,
        groundTruth: gt, odom: sim.robot.odomPose(), collided: sim.robot.collided,
        scan: scan(sim.view, gt, LDS01, noise, rng),
        others: poses.filter((p) => p.id !== sim.id),
      });
    }
  }, 1000 / scanHz);

  // --- viewer (serves the one HTML file over http) ----------------------
  const viewerUrl = new URL('../viewer.html', import.meta.url);
  const httpd = createServer(async (req, res) => {
    try {
      const path = (req.url || '/').split('?')[0]; // the viewer cache-busts with ?t=...
      if (path === '/floor.png' && floor) {
        res.writeHead(200, { 'Content-Type': 'image/png', 'Cache-Control': 'no-cache' });
        res.end(await readFile(floor.pngPath));
        return;
      }
      // vendored three.js for the viewer's 3D mode (simulator/vendor, no CDN needed)
      if (path.startsWith('/vendor/') && !path.includes('..')) {
        const file = new URL('..' + path, import.meta.url);
        try {
          const body = await readFile(file);
          res.writeHead(200, { 'Content-Type': 'text/javascript', 'Cache-Control': 'max-age=86400' });
          res.end(body);
        } catch {
          res.writeHead(404); res.end('not found');
        }
        return;
      }
      if (path === '/robots.json') {
        res.writeHead(200, { 'Content-Type': 'application/json' });
        res.end(JSON.stringify(sims.map((s) => ({ id: s.id, robotPort: s.robotPort, sensorPort: s.sensorPort }))));
        return;
      }
      const html = await readFile(viewerUrl);
      res.writeHead(200, { 'Content-Type': 'text/html' });
      res.end(html);
    } catch (e) {
      res.writeHead(500);
      res.end(String(e));
    }
  });
  // the viewer is optional — a taken port shouldn't take down the sim
  httpd.on('error', (e) => log(`viewer http server not started (${e.code || e.message}) — sim continues`));
  httpd.listen(viewerPort);

  log(`world "${world.name}" ${world.bounds[0]}x${world.bounds[1]}m, noise ${noise ? `on (seed ${seed})` : 'off'} — viewer http://127.0.0.1:${viewerPort}`);
  for (const sim of sims) {
    const p = sim.robot.pose();
    log(`${tag(sim)}roboteq ws://127.0.0.1:${sim.robotPort} (RWD ${rwdMs}ms), sensors ws://127.0.0.1:${sim.sensorPort}, start (${p.x.toFixed(2)}, ${p.y.toFixed(2)}, ${p.theta.toFixed(2)})`);
  }

  return {
    robot: sims[0].robot,
    state: sims[0].state,
    robots: sims,
    stop() {
      clearInterval(physTimer);
      clearInterval(scanTimer);
      for (const sim of sims) {
        sim.robotWss.close();
        sim.sensorWss.close();
      }
      httpd.close();
    },
  };
}
