// Entry point: load a world file and start the simulator.
//
//   node src/index.js                       # default world, default noise
//   SIM_WORLD=worlds/corridor.world.json node src/index.js
//   SIM_NOISE=off node src/index.js         # off | low | default | high
//   SIM_SEED=7 node src/index.js            # reproducible noise
//   SIM_RWD_MS=300 node src/index.js        # fast watchdog for testing
//   SIM_START="1,1,0" node src/index.js     # start pose, overrides world's
//
// Then drive it: open http://127.0.0.1:8767 (viewer), or point
// robot-os-chromium's dashboard (local mode) at ws://127.0.0.1:8765.

import { readFile, readdir } from 'node:fs/promises';
import { World } from './world.js';
import { startSimulator } from './server.js';
import { resolveNoise } from './noise.js';

const root = new URL('..', import.meta.url);
const worldPath = process.env.SIM_WORLD || 'worlds/room.world.json';
let world;
try {
  world = new World(JSON.parse(await readFile(new URL(worldPath, root))));
} catch (e) {
  const avail = (await readdir(new URL('worlds/', root))).filter((f) => f.endsWith('.world.json'));
  console.error(`could not load world "${worldPath}": ${e.message}\navailable: ${avail.join(', ')}`);
  process.exit(1);
}

let start;
if (process.env.SIM_START) {
  const [x, y, theta] = process.env.SIM_START.split(',').map(Number);
  start = { x, y, theta };
}

const noise = resolveNoise(process.env.SIM_NOISE || 'default');
const seed = Number(process.env.SIM_SEED || 1);

const sim = startSimulator({ world, start, noise, seed });

process.on('SIGINT', () => { sim.stop(); process.exit(0); });
