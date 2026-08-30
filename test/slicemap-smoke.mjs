// Smoke test for the slicemap-v1 loader + world conversion (src/slicemap.js).
// No browser, no scan-to-map-studio: builds a slicemap-v1 blob by hand,
// converts it, and checks the wall segments land where the occupied cells
// are and that collinear runs got merged.
//
//   node test/slicemap-smoke.mjs

import { parseSlicemap, toWorld, SLICE_CODE } from '../src/slicemap.js';

let failures = 0;
const check = (name, cond, extra = '') => {
  console.log(`${cond ? 'PASS' : 'FAIL'}  ${name}${extra ? '  (' + extra + ')' : ''}`);
  if (!cond) failures++;
};

const b64 = (u8) => Buffer.from(u8).toString('base64');

// --- hand-built slicemap: 10x8 grid @ 0.1 m, a 6-cell horizontal wall run
//     on row 2 (cols 2..7) + one furniture cell at (5,5) --------------------
const cols = 10, rows = 8, res = 0.1;
const codes = new Uint8Array(cols * rows).fill(SLICE_CODE.FREE);
for (let c = 2; c < 8; c++) codes[2 * cols + c] = SLICE_CODE.OCC_WALL;
codes[5 * cols + 5] = SLICE_CODE.OCC_FURNITURE;

const blob = {
  format: 'slicemap-v1', z: 0.18, band: 0.05, resolution: res,
  origin: [-1.0, -2.0], cols, rows, data: b64(codes),
};

// --- parse ---
const slice = parseSlicemap(blob);
check('parseSlicemap: dims + frame', slice.cols === cols && slice.rows === rows && slice.resolution === res && slice.origin[0] === -1.0);
check('parseSlicemap: codes length', slice.codes.length === cols * rows);
check('parseSlicemap: rejects a non-slicemap object', (() => { try { parseSlicemap({ format: 'x' }); return false; } catch { return true; } })());

// --- toWorld: all occupied ---
const w = toWorld(slice, { name: 'test' });
check('toWorld: bounds = grid extent', w.bounds[0] === cols * res && w.bounds[1] === rows * res);

// the 6-cell wall run on row 2 -> its bottom & top exposed edges should each
// be ONE merged segment (x 0.2..0.8 at y 0.2 and y 0.3), not 6.
const horiz = w.walls.filter(([x1, y1, x2, y2]) => y1 === y2);
const bottom = horiz.find(([x1, y1, x2]) => Math.abs(y1 - 0.2) < 1e-9 && Math.abs(x1 - 0.2) < 1e-9 && Math.abs(x2 - 0.8) < 1e-9);
check('toWorld: 6-cell wall run merged into one bottom segment', !!bottom, JSON.stringify(w.walls.filter(([,y1,,y2]) => Math.abs(y1-0.2)<1e-9 && y1===y2)));
check('toWorld: far fewer segments than occupied cell edges', w.walls.length <= 8, `${w.walls.length} segments for 7 occupied cells`);

// --- walls-only skips the furniture cell ---
const wOnly = toWorld(slice, { wallsOnly: true });
const totalLen = (walls) => walls.reduce((s, [x1, y1, x2, y2]) => s + Math.hypot(x2 - x1, y2 - y1), 0);
check('toWorld walls-only: shorter total wall length than all-occupied (furniture dropped)',
  totalLen(wOnly.walls) < totalLen(w.walls), `${totalLen(wOnly.walls).toFixed(2)} < ${totalLen(w.walls).toFixed(2)}`);

// --- explicit start passes through; default start is inside the grid ---
const wStart = toWorld(slice, { start: [0.55, 0.65, 1.57] });
check('toWorld: explicit --start passes through', JSON.stringify(wStart.start) === JSON.stringify([0.55, 0.65, 1.57]));
check('toWorld: default start is a free-cell centroid inside bounds',
  w.start[0] > 0 && w.start[0] < w.bounds[0] && w.start[1] > 0 && w.start[1] < w.bounds[1], JSON.stringify(w.start));

// --- the produced world.json shape is what World() accepts ---
check('toWorld: shape = { name, bounds, walls, start }',
  typeof w.name === 'string' && Array.isArray(w.bounds) && Array.isArray(w.walls) && Array.isArray(w.start)
  && w.walls.every((s) => Array.isArray(s) && s.length === 4));

console.log(failures === 0 ? '\nall slicemap smoke checks passed' : `\n${failures} check(s) failed`);
process.exit(failures === 0 ? 0 : 1);
