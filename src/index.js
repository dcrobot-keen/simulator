// Entry point: load a world file and start the simulator.
//
//   node src/index.js                 # default world (worlds/room.world.json)
//   SIM_WORLD=worlds/open.world.json node src/index.js
//   SIM_RWD_MS=300 node src/index.js  # fast watchdog for testing
//   SIM_START="1,1,0" node src/index.js   # start pose "x,y,theta" (m, m, rad)
//
// Then drive it: open viewer.html in Chrome, or point
// robot-os-chromium's dashboard (local mode) at ws://127.0.0.1:8765.

import { readFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { World } from './world.js';
import { startSimulator } from './server.js';

const worldPath = process.env.SIM_WORLD || 'worlds/room.world.json';
const url = new URL(worldPath, new URL('..', import.meta.url));
const world = new World(JSON.parse(await readFile(url)));

let start;
if (process.env.SIM_START) {
  const [x, y, theta] = process.env.SIM_START.split(',').map(Number);
  start = { x, y, theta };
}

const sim = startSimulator({ world, start });

process.on('SIGINT', () => { sim.stop(); process.exit(0); });
