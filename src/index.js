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
import { parseSlicemap, toWorld } from './slicemap.js';
import { startSimulator } from './server.js';
import { resolveNoise } from './noise.js';

const root = new URL('..', import.meta.url);
const worldPath = process.env.SIM_WORLD || 'worlds/room.world.json';
let world;
try {
  const raw = JSON.parse(await readFile(new URL(worldPath, root)));
  // A slicemap-v1 (scan-to-map-studio slice, or its merged output published
  // straight into worlds/ by the alignment workspace) is accepted as a world
  // directly -- same conversion as scripts/slicemap-to-world.mjs, no extra step.
  world = new World(raw.format === 'slicemap-v1'
    ? toWorld(parseSlicemap(raw), { name: worldPath.split('/').pop().replace(/\.slicemap\.json$|\.json$/, '') })
    : raw);
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
