// CLI: convert a scan-to-map-studio slicemap-v1 JSON into a simulator
// worlds/*.world.json, so `projects/<room>` becomes a driveable sim world
// (roadmap.md Phase 9, step 2). The sim then IS that room, for testing
// iPhone-map-relative localization against ground truth without hardware.
//
//   node scripts/slicemap-to-world.mjs bedroom_tb3.json worlds/bedroom.world.json
//   node scripts/slicemap-to-world.mjs bedroom_tb3.json worlds/bedroom.world.json --walls-only --start 1.2,0.8,0

import { readFileSync, writeFileSync } from 'node:fs';
import { basename } from 'node:path';
import { parseSlicemap, toWorld } from '../src/slicemap.js';

const [, , inPath, outPath, ...rest] = process.argv;
if (!inPath || !outPath) {
  console.error('usage: node scripts/slicemap-to-world.mjs <slicemap.json> <out.world.json> [--walls-only] [--start x,y,theta]');
  process.exit(2);
}

const wallsOnly = rest.includes('--walls-only');
const startArg = rest[rest.indexOf('--start') + 1];
const start = rest.includes('--start') && startArg
  ? startArg.split(',').map(Number)
  : undefined;

const slice = parseSlicemap(JSON.parse(readFileSync(inPath, 'utf-8')));
const name = basename(outPath).replace(/\.world\.json$/, '').replace(/\.json$/, '');
const world = toWorld(slice, { wallsOnly, start, name });

// compact: one wall segment per line (rounded), rather than 2000 lines
const round = (n) => Math.round(n * 1e4) / 1e4;
const wallLines = world.walls.map((s) => `    [${s.map(round).join(', ')}]`).join(',\n');
const json =
  `{\n  "name": ${JSON.stringify(world.name)},\n` +
  `  "bounds": [${world.bounds.map(round).join(', ')}],\n` +
  `  "start": [${world.start.map(round).join(', ')}],\n` +
  `  "walls": [\n${wallLines}\n  ]\n}\n`;
writeFileSync(outPath, json);

const occ = [...slice.codes].filter((c) => c >= 2).length;
console.log(
  `${inPath}: ${slice.cols}x${slice.rows} @ ${slice.resolution}m (z=${slice.z}±${slice.band}), ` +
  `${occ} occupied cells -> ${world.walls.length} wall segments`,
);
console.log(`bounds ${world.bounds.map((n) => n.toFixed(2))}, start ${world.start.map((n) => n.toFixed(2))}`);
console.log(`wrote ${outPath}`);
