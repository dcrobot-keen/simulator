// The 2D world: static wall segments in metres, and the two queries the
// rest of the sim needs against them — a ray cast (for the LIDAR) and a
// circle overlap test (for robot collision). No dynamics, no 3D.
//
// A world file is JSON: { name, bounds:[w,h], walls:[[x1,y1,x2,y2], ...],
// start:[x,y,theta] }. The outer bounds are added as four walls
// automatically; `start` is a default spawn pose (SIM_START overrides it).

export class World {
  constructor({ name = 'world', bounds = [10, 10], walls = [], start = null } = {}) {
    this.name = name;
    this.bounds = bounds;
    this.start = start && { x: start[0], y: start[1], theta: start[2] ?? 0 };
    const [w, h] = bounds;
    // segments as {x1,y1,x2,y2}; origin at a corner, +x right, +y up
    // kind: 'border' (outer bounds) | 'wall' | 'furniture' (slicemap worlds tag their edges;
    // plain world files default to 'wall'). Only the viewers care -- physics treats all alike.
    this.segments = [
      { x1: 0, y1: 0, x2: w, y2: 0, kind: 'border' },
      { x1: w, y1: 0, x2: w, y2: h, kind: 'border' },
      { x1: w, y1: h, x2: 0, y2: h, kind: 'border' },
      { x1: 0, y1: h, x2: 0, y2: 0, kind: 'border' },
      ...walls.map(([x1, y1, x2, y2, kind]) => ({ x1, y1, x2, y2, kind: kind ?? 'wall' })),
    ];
  }

  // Distance from (ox,oy) along heading `ang` to the nearest segment, or
  // `max` if nothing is hit within `max`.
  raycast(ox, oy, ang, max) {
    const dx = Math.cos(ang);
    const dy = Math.sin(ang);
    let best = max;
    for (const s of this.segments) {
      const sx = s.x2 - s.x1;
      const sy = s.y2 - s.y1;
      const denom = dx * sy - dy * sx;
      if (Math.abs(denom) < 1e-12) continue; // parallel
      const t = ((s.x1 - ox) * sy - (s.y1 - oy) * sx) / denom; // along the ray
      const u = ((s.x1 - ox) * dy - (s.y1 - oy) * dx) / denom; // along the segment
      if (t >= 0 && t < best && u >= 0 && u <= 1) best = t;
    }
    return best;
  }

  // Is a disc of radius r centred at (cx,cy) touching any segment?
  collides(cx, cy, r) {
    for (const s of this.segments) {
      const vx = s.x2 - s.x1;
      const vy = s.y2 - s.y1;
      const wx = cx - s.x1;
      const wy = cy - s.y1;
      const len2 = vx * vx + vy * vy || 1e-12;
      let t = (wx * vx + wy * vy) / len2;
      t = Math.max(0, Math.min(1, t));
      const px = s.x1 + t * vx;
      const py = s.y1 + t * vy;
      const d2 = (cx - px) ** 2 + (cy - py) ** 2;
      if (d2 <= r * r) return true;
    }
    return false;
  }
}
