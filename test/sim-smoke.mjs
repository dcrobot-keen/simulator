// Smoke test for the 2D simulator, no browser: drives the Roboteq
// endpoint, watches the sensor stream, checks the body actually moves in
// the world, stops at a wall, produces a sane laser scan, and that the RWD
// watchdog still fires on silence.
//
//   node test/sim-smoke.mjs        (or: npm test)

import { spawn } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { dirname, resolve } from 'node:path';

const here = dirname(fileURLToPath(import.meta.url));
const ENTRY = resolve(here, '../src/index.js');
const RPORT = 8795;
const SPORT = 8796;
const RWD_MS = 400;

const wait = (ms) => new Promise((r) => setTimeout(r, ms));
let failures = 0;
function check(name, cond) {
  console.log(`${cond ? 'PASS' : 'FAIL'}  ${name}`);
  if (!cond) failures++;
}
const enc = new TextEncoder();

let simLog = '';
const sim = spawn(process.execPath, [ENTRY], {
  // start near the vertical wall at x=5.5 so a full-speed dash reaches it
  // in ~1s (TB3 Burger tops out at 0.22 m/s)
  env: { ...process.env, SIM_PORT: String(RPORT), SIM_SENSOR_PORT: String(SPORT), SIM_RWD_MS: String(RWD_MS), SIM_START: '5.15,3.0,0' },
  stdio: ['ignore', 'pipe', 'inherit'],
});
sim.stdout.on('data', (d) => { simLog += d.toString(); process.stdout.write(d); });
process.on('exit', () => sim.kill());

function open(url, onMsg) {
  const ws = new WebSocket(url);
  ws.binaryType = 'arraybuffer';
  ws.onmessage = (e) => onMsg(e.data);
  return new Promise((res, rej) => { ws.onopen = () => res(ws); ws.onerror = rej; });
}

try {
  await wait(500);

  // sensor stream (newline-delimited JSON)
  const frames = [];
  let hello = null;
  let sbuf = '';
  await open(`ws://127.0.0.1:${SPORT}`, (data) => {
    sbuf += typeof data === 'string' ? data : Buffer.from(data).toString();
    let nl;
    while ((nl = sbuf.indexOf('\n')) !== -1) {
      const line = sbuf.slice(0, nl); sbuf = sbuf.slice(nl + 1);
      if (!line.trim()) continue;
      const obj = JSON.parse(line);
      if (obj.type === 'hello') hello = obj; else frames.push(obj);
    }
  });
  await wait(300); // let hello + a few frames arrive
  check('sensor hello carries the world', hello && hello.world.segments.length >= 4 && Array.isArray(hello.world.bounds));
  check('sensor stream is producing frames', frames.length > 0);

  // roboteq endpoint
  const replies = [];
  let rbuf = '';
  const rob = await open(`ws://127.0.0.1:${RPORT}`, (data) => {
    rbuf += typeof data === 'string' ? data : Buffer.from(data).toString();
    let cr;
    while ((cr = rbuf.indexOf('\r')) !== -1) { replies.push(rbuf.slice(0, cr).trim()); rbuf = rbuf.slice(cr + 1); }
  });
  const send = (line) => rob.send(enc.encode(line + '\r'));

  const startX = frames.at(-1).groundTruth.x;

  // drive straight at full tilt toward the wall at x=5.5
  send('!MG');
  for (let i = 0; i < 25; i++) { send('!G 1 1000_!G 2 1000'); await wait(60); }
  await wait(200);

  const nowX = frames.at(-1).groundTruth.x;
  check(`body moved forward in the world (${startX.toFixed(2)} -> ${nowX.toFixed(2)} m)`, nowX - startX > 0.1);
  check('stopped at the wall (collision, x < 5.45)', nowX < 5.45 && frames.some((f) => f.collided));

  send('?C');
  await wait(50);
  const c = replies.filter((r) => r.startsWith('C=')).at(-1);
  const [cl, crv] = c.slice(2).split(':').map(Number);
  check(`encoders advanced and track together (C=${cl}:${crv})`, cl > 0 && crv > 0 && Math.abs(cl - crv) / cl < 0.05);

  const s = frames.at(-1).scan;
  const allInRange = s.ranges.length === 360 && s.ranges.every((r) => r >= s.rangeMin - 1e-6 && r <= s.rangeMax + 1e-6);
  check('laser scan: 360 beams, all within range', allInRange);
  check('forward beam sees the wall ahead (< rangeMax)', s.ranges[0] < s.rangeMax);

  // go quiet -> RWD watchdog. Poll for the log line so we can then check
  // the body is actually stopped *after* it fired (not still coasting).
  const logMark = simLog.length;
  const t0 = Date.now();
  let fired = false;
  while (Date.now() - t0 < RWD_MS + 400) {
    if (simLog.slice(logMark).includes('RWD: no serial command')) { fired = true; break; }
    await wait(20);
  }
  check('RWD watchdog stopped the wheels on silence', fired);
  const xAtFire = frames.at(-1).groundTruth.x;
  await wait(250);
  check('body stays put after the watchdog fired', Math.abs(frames.at(-1).groundTruth.x - xAtFire) < 0.01);

  // rotate in place after re-enable
  send('!MG');
  const thBefore = frames.at(-1).groundTruth.theta;
  for (let i = 0; i < 8; i++) { send('!G 1 -500_!G 2 500'); await wait(80); }
  await wait(150);
  check('rotates in place on a spin command', Math.abs(frames.at(-1).groundTruth.theta - thBefore) > 0.3);

  rob.close();
  await wait(50);
} catch (err) {
  console.log(`FAIL  unexpected error: ${err.stack || err}`);
  failures++;
}

sim.kill();
console.log(failures === 0 ? '\nall simulator smoke checks passed' : `\n${failures} check(s) failed`);
process.exit(failures === 0 ? 0 : 1);
