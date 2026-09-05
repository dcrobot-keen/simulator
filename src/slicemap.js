// slicemap-v1 loader + conversion to the simulator's wall-segment world.
//
// A slicemap-v1 file is produced by scan-to-map-studio/scripts/slice_map.py:
// a 2D occupancy grid sliced from an iPhone LiDAR scan at a robot's LiDAR
// mount height (roadmap.md Phase 9). JSON:
//   { format:"slicemap-v1", z, band, resolution, origin:[x,y], cols, rows,
//     data:<base64 row-major uint8 codes, row 0 = min y> }
// codes: 0 unknown, 1 free, 2 occupied (furniture / unspecified), 3 occupied-wall.
//
// toWorld() turns it into { name, bounds, walls, start } — occupied cells
// become axis-aligned wall segments (exposed edges only, then collinear
// runs merged so the raycaster isn't iterating thousands of unit edges).
// unknown/free both become open space: the sim world is "the room as the
// iPhone slice saw it at height z".

export const SLICE_CODE = { UNKNOWN: 0, FREE: 1, OCC_FURNITURE: 2, OCC_WALL: 3 };

const b64decode = (s) =>
  typeof atob === 'function'
    ? Uint8Array.from(atob(s), (c) => c.charCodeAt(0))
    : new Uint8Array(Buffer.from(s, 'base64'));

/** Parse a slicemap-v1 object. Returns { z, band, resolution, origin:[x,y], cols, rows, codes:Uint8Array }. */
export function parseSlicemap(obj) {
  if (!obj || obj.format !== 'slicemap-v1') throw new Error('not a slicemap-v1 object');
  const codes = b64decode(obj.data);
  if (codes.length !== obj.cols * obj.rows) {
    throw new Error(`slicemap data length ${codes.length} != cols*rows ${obj.cols * obj.rows}`);
  }
  return {
    z: obj.z, band: obj.band, resolution: obj.resolution,
    origin: [obj.origin[0], obj.origin[1]], cols: obj.cols, rows: obj.rows, codes,
  };
}

/**
 * slicemap -> simulator world.json shape.
 * @param {object} slice - parseSlicemap() output
 * @param {object} [opts]
 * @param {boolean} [opts.wallsOnly=false] - only OCC_WALL cells become walls (skip furniture)
 * @param {[number,number,number]} [opts.start] - spawn pose in world coords (origin-relative); default = centroid of free cells
 * @param {string} [opts.name='slice']
 * @returns {{ name, bounds:[number,number], walls:number[][], start:[number,number,number] }}
 */
export function toWorld(slice, { wallsOnly = false, start, name = 'slice' } = {}) {
  const { cols, rows, resolution: r, codes } = slice;
  const occ = (c, row) => {
    if (c < 0 || c >= cols || row < 0 || row >= rows) return false;
    const v = codes[row * cols + c];
    return wallsOnly ? v === SLICE_CODE.OCC_WALL : v === SLICE_CODE.OCC_WALL || v === SLICE_CODE.OCC_FURNITURE;
  };
  // 'wall' | 'furniture' of the occupied cell that owns an edge -- carried on the segment
  // (5th element) so the viewers can draw walls and furniture differently (3D heights).
  const kindOf = (c, row) => (codes[row * cols + c] === SLICE_CODE.OCC_WALL ? 'wall' : 'furniture');

  // Exposed unit edges, keyed for collinear-run merging.
  // Horizontal edge on grid line y=row (world y = row*r), spanning col..col+1.
  // Vertical edge on grid line x=col (world x = col*r), spanning row..row+1.
  const hEdges = new Map(); // row -> Set of col (edge = segment [col, col+1] along grid line y=row)
  const vEdges = new Map(); // col -> Set of row (edge = segment [row, row+1] along grid line x=col)

  // map key = "<grid line>|<kind>" so runs of different kinds never merge into one segment
  const bucket = (map, k) => { let s = map.get(k); if (!s) map.set(k, (s = new Set())); return s; };
  const addH = (row, col, kind) => bucket(hEdges, `${row}|${kind}`).add(col);
  const addV = (col, row, kind) => bucket(vEdges, `${col}|${kind}`).add(row);

  for (let row = 0; row < rows; row++) {
    for (let col = 0; col < cols; col++) {
      if (!occ(col, row)) continue;
      const kind = kindOf(col, row);
      if (!occ(col, row - 1)) addH(row, col, kind);       // bottom edge
      if (!occ(col, row + 1)) addH(row + 1, col, kind);   // top edge
      if (!occ(col - 1, row)) addV(col, row, kind);       // left edge
      if (!occ(col + 1, row)) addV(col + 1, row, kind);   // right edge
    }
  }

  const walls = [];
  const mergeRuns = (map, makeSeg) => {
    for (const [key, set] of map) {
      const [lineStr, kind] = key.split('|');
      const line = Number(lineStr);
      const idxs = [...set].sort((a, b) => a - b);
      let runStart = idxs[0];
      let prev = idxs[0];
      for (let i = 1; i <= idxs.length; i++) {
        if (i < idxs.length && idxs[i] === prev + 1) { prev = idxs[i]; continue; }
        walls.push([...makeSeg(line, runStart, prev + 1), kind]);
        if (i < idxs.length) { runStart = idxs[i]; prev = idxs[i]; }
      }
    }
  };
  mergeRuns(hEdges, (row, c0, c1) => [c0 * r, row * r, c1 * r, row * r]);
  mergeRuns(vEdges, (col, r0, r1) => [col * r, r0 * r, col * r, r1 * r]);

  const bounds = [cols * r, rows * r];

  const spawn = start ?? defaultSpawn(slice, wallsOnly, bounds);

  return { name, bounds, walls, start: spawn };
}

/**
 * Default spawn: the free cell farthest from anything that becomes a wall
 * (multi-source BFS distance transform), ties broken toward the free-cell
 * centroid. The centroid alone is a bad spawn -- on a real scan it often
 * lands within the body radius of furniture, and the sim then reports
 * "touching a wall" forever because translation is dropped on collision.
 * Unknown cells count as open (they are open space in the world), so only
 * occupied cells and the grid boundary push the spawn away.
 */
export function defaultSpawn(slice, wallsOnly, bounds) {
  const { cols, rows, resolution: r, codes } = slice;
  const isWall = (v) => (wallsOnly ? v === SLICE_CODE.OCC_WALL : v === SLICE_CODE.OCC_WALL || v === SLICE_CODE.OCC_FURNITURE);
  const dist = new Int32Array(cols * rows).fill(-1);
  const queue = [];
  let sx = 0, sy = 0, n = 0;
  for (let i = 0; i < codes.length; i++) {
    if (isWall(codes[i])) { dist[i] = 0; queue.push(i); }
    if (codes[i] === SLICE_CODE.FREE) { sx += (i % cols) + 0.5; sy += Math.floor(i / cols) + 0.5; n++; }
  }
  if (n === 0) return [bounds[0] / 2, bounds[1] / 2, 0];
  const cx = sx / n, cy = sy / n;
  for (let col = 0; col < cols; col++) for (const row of [0, rows - 1]) { const i = row * cols + col; if (dist[i] < 0) { dist[i] = 1; queue.push(i); } }
  for (let row = 0; row < rows; row++) for (const col of [0, cols - 1]) { const i = row * cols + col; if (dist[i] < 0) { dist[i] = 1; queue.push(i); } }
  for (let q = 0; q < queue.length; q++) {
    const i = queue[q], col = i % cols, row = Math.floor(i / cols), d = dist[i] + 1;
    for (const [dc, dr] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) {
      const c2 = col + dc, r2 = row + dr;
      if (c2 < 0 || c2 >= cols || r2 < 0 || r2 >= rows) continue;
      const j = r2 * cols + c2;
      if (dist[j] < 0) { dist[j] = d; queue.push(j); }
    }
  }
  let best = -1, bestD = -1, bestC = Infinity;
  for (let i = 0; i < codes.length; i++) {
    if (codes[i] !== SLICE_CODE.FREE) continue;
    const dc = (i % cols) + 0.5 - cx, dr = Math.floor(i / cols) + 0.5 - cy, c = dc * dc + dr * dr;
    if (dist[i] > bestD || (dist[i] === bestD && c < bestC)) { best = i; bestD = dist[i]; bestC = c; }
  }
  return [((best % cols) + 0.5) * r, (Math.floor(best / cols) + 0.5) * r, 0];
}
