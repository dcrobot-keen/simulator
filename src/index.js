// Entry point: load a world file and start the simulator.
//
//   node src/index.js                       # default world, default noise
//   SIM_WORLD=worlds/corridor.world.json node src/index.js
//   SIM_NOISE=off node src/index.js         # off | low | default | high
//   SIM_SEED=7 node src/index.js            # reproducible noise
//   SIM_RWD_MS=300 node src/index.js        # fast watchdog for testing
//   SIM_START="1,1,0" node src/index.js     # start pose, overrides world's
//   SIM_ROBOTS="tb3-sim-01@auto;tb3-sim-02@9.28,9.47,0" node src/index.js
//       # several robots in ONE world; robot i listens on 8765+10i / 8766+10i.
//       # "auto" (or no @pose) = the world's default spawn. They collide with and
//       # see each other on the LIDAR -- the moving obstacle the nav stack must stop for.
//
// Then drive it: open http://127.0.0.1:8767 (viewer), or point
// robot-os-chromium's dashboard (local mode) at ws://127.0.0.1:8765.

import { readFile, readdir } from 'node:fs/promises';
import { existsSync } from 'node:fs';
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

// A slicemap published by the alignment workspace may come with the app's floor
// image composited on the same grid (<stem>.floor.png/.json). The viewer draws it
// under the walls; the grid corner is the world origin, so the extent is just
// [0, 0, cols*r, rows*r] -- the same rule pathfinder uses.
let floor = null;
{
  const stem = new URL(worldPath, root).pathname.replace(/\.slicemap\.json$|\.json$/i, '');
  const pngPath = decodeURIComponent(stem) + '.floor.png';
  const jsonPath = decodeURIComponent(stem) + '.floor.json';
  if (existsSync(pngPath) && existsSync(jsonPath)) {
    try {
      const meta = JSON.parse(await readFile(jsonPath, 'utf-8'));
      floor = { pngPath, width: meta.width_px, height: meta.height_px, extent: [0, 0, meta.width_px * meta.resolution, meta.height_px * meta.resolution] };
    } catch (e) {
      console.error(`floor image sidecar unreadable (${e.message}) -- viewer draws walls only`);
    }
  }
}

let start;
if (process.env.SIM_START) {
  const [x, y, theta] = process.env.SIM_START.split(',').map(Number);
  start = { x, y, theta };
}

const noise = resolveNoise(process.env.SIM_NOISE || 'default');
const seed = Number(process.env.SIM_SEED || 1);

// SIM_ROBOTS: "id@x,y,theta;id2@auto;..." -> several robots sharing the world.
let robots = null;
if (process.env.SIM_ROBOTS) {
  robots = process.env.SIM_ROBOTS.split(';').map((entry) => entry.trim()).filter(Boolean).map((entry, i) => {
    const [id, pose] = entry.split('@');
    let rs = null;
    if (pose && pose !== 'auto') {
      const [x, y, theta] = pose.split(',').map(Number);
      if ([x, y].some((v) => !Number.isFinite(v))) throw new Error(`SIM_ROBOTS: bad pose in "${entry}"`);
      rs = { x, y, theta: Number.isFinite(theta) ? theta : 0 };
    }
    return { id: id || `robot-${i + 1}`, start: rs ?? (i === 0 ? start : null) };
  });
}

const sim = startSimulator({ world, start, robots, noise, seed, floor });

process.on('SIGINT', () => { sim.stop(); process.exit(0); });
