// Seeded RNG + noise presets. Everything random in the sim goes through one
// rng so a run is reproducible with SIM_SEED; SIM_NOISE picks how much.
//
// Modeled loosely on a hobby 2D LIDAR (LDS-01) + differential-drive
// odometry:
//   lidar.rangeSigma  gaussian stddev added to each beam's distance (m)
//   lidar.dropout     probability a beam returns nothing (dark/glossy/grazing)
//   odom.slipSigma    per-wheel multiplicative error between wheel speed and
//                     actual ground speed — the encoders still count the
//                     wheel exactly, so this is what makes dead-reckoning
//                     drift away from ground truth.

export function makeRng(seed = 1) {
  let s = seed >>> 0;
  return function rng() {
    s |= 0; s = (s + 0x6d2b79f5) | 0;
    let t = Math.imul(s ^ (s >>> 15), 1 | s);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

// standard normal via Box–Muller
export function gaussian(rng) {
  let u = 0, v = 0;
  while (u === 0) u = rng();
  while (v === 0) v = rng();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

export const NOISE_PRESETS = {
  off: null,
  low: {
    lidar: { rangeSigma: 0.008, dropout: 0.005 },
    odom: { slipSigma: 0.01 },
  },
  default: {
    lidar: { rangeSigma: 0.015, dropout: 0.02 },
    odom: { slipSigma: 0.04 },
  },
  high: {
    lidar: { rangeSigma: 0.03, dropout: 0.06 },
    odom: { slipSigma: 0.07 },
  },
};

export function resolveNoise(name = 'default') {
  if (name === 'off' || name === '0' || name === 'none') return null;
  return NOISE_PRESETS[name] ?? NOISE_PRESETS.default;
}
