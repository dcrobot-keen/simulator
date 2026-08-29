// A 2D laser scan by ray casting against the world. Modeled on the
// TurtleBot3's LDS-01: full 360°, 1° steps, 0.12–3.5 m range. Beams that
// hit nothing inside range report the max range (SICK/LDS convention
// varies; the nav layer treats >= rangeMax as "no return").

export const LDS01 = {
  count: 360,
  angleMin: 0,
  angleMax: 2 * Math.PI,
  rangeMin: 0.12,
  rangeMax: 3.5,
};

export function scan(world, pose, spec = LDS01) {
  const inc = (spec.angleMax - spec.angleMin) / spec.count;
  const ranges = new Array(spec.count);
  for (let i = 0; i < spec.count; i++) {
    const a = pose.theta + spec.angleMin + i * inc;
    const d = world.raycast(pose.x, pose.y, a, spec.rangeMax);
    ranges[i] = d < spec.rangeMin ? spec.rangeMin : Number(d.toFixed(3));
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
