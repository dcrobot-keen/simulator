// Checks the sensor-noise model, no browser. Compares a noisy run against
// a noise-off run of the same drive (same seed):
//   - laser: standing still, beams jitter and some drop out with noise,
//     and don't without it
//   - odometry: the dead-reckoned pose (sim's `odom`, integrated from the
//     encoders) drifts from ground truth under wheel slip, and tracks it
//     exactly without
//
//   node test/noise-smoke.mjs

import { spawn } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { dirname, resolve } from 'node:path';

const here = dirname(fileURLToPath(import.meta.url));
const ENTRY = resolve(here, '../src/index.js');
const wait = (ms) => new Promise((r) => setTimeout(r, ms));

let failures = 0;
const check = (name, cond) => { console.log(`${cond ? 'PASS' : 'FAIL'}  ${name}`); if (!cond) failures++; };
const enc = new TextEncoder();
const td = new TextDecoder();
const asText = (d) => (typeof d === 'string' ? d : td.decode(d));

function openJsonStream(url, onObj) {
  const ws = new WebSocket(url);
  ws.binaryType = 'arraybuffer';
  let buf = '';
  ws.onmessage = (e) => {
    buf += asText(e.data);
    let nl;
    while ((nl = buf.indexOf('\n')) !== -1) { const l = buf.slice(0, nl); buf = buf.slice(nl + 1); if (l.trim()) onObj(JSON.parse(l)); }
  };
  return new Promise((res, rej) => { ws.onopen = () => res(ws); ws.onerror = rej; });
}
function openRoboteq(url) {
  const ws = new WebSocket(url);
  ws.binaryType = 'arraybuffer';
  return new Promise((res, rej) => { ws.onopen = () => res(ws); ws.onerror = rej; });
}

async function run(noise, n) {
  const R = 8810 + n * 4, S = R + 1, V = R + 2;
  const sim = spawn(process.execPath, [ENTRY], {
    env: {
      ...process.env, SIM_PORT: String(R), SIM_SENSOR_PORT: String(S), SIM_VIEWER_PORT: String(V),
      SIM_NOISE: noise, SIM_SEED: '3', SIM_WORLD: 'worlds/room.world.json', SIM_START: '4.0,3.0,0',
    },
    stdio: ['ignore', 'ignore', 'inherit'],
  });
  try {
    await wait(500);
    const frames = [];
    await openJsonStream(`ws://127.0.0.1:${S}`, (o) => { if (o.type === 'frame') frames.push(o); });
    const rob = await openRoboteq(`ws://127.0.0.1:${R}`);
    const send = (s) => rob.send(enc.encode(s + '\r'));

    send('!MG');
    for (let i = 0; i < 55; i++) { send('!G 1 800_!G 2 800'); await wait(50); } // ~2.7s, ~0.4 m
    for (let i = 0; i < 8; i++) { send('!G 1 0_!G 2 0'); await wait(50); }       // stop + keepalive
    await wait(400);                                                             // settle
    const still = frames.length;
    await wait(1500);                                                           // ~15 fresh frames, stationary
    rob.close();

    const last = frames.at(-1);
    return {
      driftEnd: Math.hypot(last.groundTruth.x - last.odom.x, last.groundTruth.y - last.odom.y),
      stillScans: frames.slice(still).map((f) => f.scan.ranges),
      rangeMax: last.scan.rangeMax,
      geomNoReturn: frames.slice(still)[0].scan.ranges.filter((r) => r >= last.scan.rangeMax - 1e-6).length,
    };
  } finally {
    sim.kill();
  }
}

try {
  const off = await run('off', 0);
  const on = await run('default', 1);

  // --- laser noise (measured while stationary) ------------------------
  const spread = (scans, beam) => {
    const vals = scans.map((r) => r[beam]);
    return Math.max(...vals) - Math.min(...vals);
  };
  const offSpread = spread(off.stillScans, 0);
  const onSpread = spread(on.stillScans, 0);
  check(`no-noise scan is steady when stopped (forward-beam spread ${offSpread.toFixed(4)} m)`, offSpread < 0.003);
  check(`noisy scan jitters (forward-beam spread ${onSpread.toFixed(4)} m)`, onSpread > 0.01);

  const onNoReturn = on.stillScans.at(-1).filter((r) => r >= on.rangeMax - 1e-6).length;
  check(`dropouts add no-return beams (geom ${off.geomNoReturn} -> noisy ${onNoReturn})`, onNoReturn > off.geomNoReturn);

  // --- odometry slip -----------------------------------------------
  check(`no-noise: dead reckoning tracks ground truth (drift ${off.driftEnd.toFixed(4)} m)`, off.driftEnd < 0.005);
  check(`noisy: dead reckoning drifts from ground truth (drift ${on.driftEnd.toFixed(3)} m over ~0.4 m driven)`,
    on.driftEnd > 0.015 && on.driftEnd > 5 * off.driftEnd);
} catch (err) {
  console.log(`FAIL  unexpected error: ${err.stack || err}`);
  failures++;
}

console.log(failures === 0 ? '\nall noise smoke checks passed' : `\n${failures} check(s) failed`);
process.exit(failures === 0 ? 0 : 1);
