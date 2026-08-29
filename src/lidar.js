// A 2D laser scan by ray casting against the world. Modeled on the
// TurtleBot3's LDS-01: full 360°, 1° steps, 0.12–3.5 m range. Beams that
// hit nothing inside range report the max range (the nav layer treats
// >= rangeMax as "no return").
//
// With a `noise` config (see noise.js) each beam gets gaussian range error
// and a chance of dropping out (reported as rangeMax). Without it the scan
// is exact geometry.

import { gaussian } from './noise.js';

export const LDS01 = {
  count: 360,
  angleMin: 0,
  angleMax: 2 * Math.PI,
  rangeMin: 0.12,
  rangeMax: 3.5,
};

export function scan(world, pose, spec = LDS01, noise = null, rng = Math.random) {
  const inc = (spec.angleMax - spec.angleMin) / spec.count;
  const ranges = new Array(spec.count);
  for (let i = 0; i < spec.count; i++) {
    const a = pose.theta + spec.angleMin + i * inc;
    let d = world.raycast(pose.x, pose.y, a, spec.rangeMax);

    if (noise && noise.lidar) {
      if (d < spec.rangeMax && noise.lidar.dropout && rng() < noise.lidar.dropout) {
        d = spec.rangeMax; // no return this beam
      } else if (d < spec.rangeMax && noise.lidar.rangeSigma) {
        d += gaussian(rng) * noise.lidar.rangeSigma;
      }
    }

    d = Math.max(spec.rangeMin, Math.min(spec.rangeMax, d));
    ranges[i] = Number(d.toFixed(3));
  }
  return {
    angleMin: spec.angleMin,
    angleMax: spec.angleMax,
    angleIncrement: inc,
    rangeMin: spec.rangeMin,
    rangeMax: spec.rangeMax,
    ranges,
  };
}
