// Differential-drive kinematic body, TurtleBot3 Burger sized. No inertia.
// The pose is the integral of the wheel speeds, except: a step that would
// drive the body into a wall keeps the rotation and drops the translation
// (so you can turn away). With a `noise` config the encoders still count
// the commanded wheel rotation exactly but the body moves along a slightly
// perturbed path (wheel slip), so dead-reckoning from `?C` drifts.
//
// Input is Roboteq motor-command units per channel (-1000..1000); ±1000 is
// mapped to ±max wheel angular velocity. Output is encoder counts, which
// is what the Roboteq `?C` query reports back up the stack.

import { gaussian } from './noise.js';

// TurtleBot3 Burger
export const TB3 = {
  wheelRadius: 0.033,      // m
  wheelSeparation: 0.160,  // m
  bodyRadius: 0.11,        // m, for collision
  maxLinear: 0.22,         // m/s  (TB3 Burger spec)
  countsPerRev: 4096,      // encoder resolution (assumed)
};

const MAX_WHEEL_W = TB3.maxLinear / TB3.wheelRadius; // rad/s at ±1000 units

export class Robot {
  constructor({ x = 1, y = 1, theta = 0 } = {}) {
    this.x = x;
    this.y = y;
    this.theta = theta;
    this.enc = [0, 0];         // accumulated counts, [left, right]
    this._encFrac = [0, 0];    // sub-count remainder so slow speeds still integrate
    this.collided = false;
    // dead-reckoned pose: what a downstream OdometryNode gets by
    // integrating ?C with the nominal kinematic model. Equals ground truth
    // exactly with noise off; diverges under wheel slip.
    this.odom = { x, y, theta };
    this._bias = null;          // persistent per-wheel scale error (set on first noisy step)
  }

  // units: [left, right] in -1000..1000. dt in seconds. world for collision.
  // noise: optional {odom:{slipSigma}}. rng: () => [0,1).
  step(units, dt, world, noise = null, rng = Math.random) {
    const wL = (Math.max(-1000, Math.min(1000, units[0])) / 1000) * MAX_WHEEL_W;
    const wR = (Math.max(-1000, Math.min(1000, units[1])) / 1000) * MAX_WHEEL_W;

    // encoder counts advance with the *commanded* wheel rotation (the
    // encoder is on the wheel), even on collision
    for (let i = 0; i < 2; i++) {
      const w = i === 0 ? wL : wR;
      const revs = (w * dt) / (2 * Math.PI);
      const exact = revs * TB3.countsPerRev + this._encFrac[i];
      const whole = Math.trunc(exact);
      this.enc[i] += whole;
      this._encFrac[i] = exact - whole;
    }

    // dead reckoning: nominal kinematic model on the commanded speeds
    {
      const v = (wL + wR) / 2 * TB3.wheelRadius;
      const om = (wR - wL) / TB3.wheelSeparation * TB3.wheelRadius;
      this.odom.theta += om * dt;
      this.odom.x += v * Math.cos(this.odom.theta) * dt;
      this.odom.y += v * Math.sin(this.odom.theta) * dt;
    }

    // actual ground speed differs from wheel speed under slip. The
    // persistent part is what makes dead reckoning drift: a mild common
    // scale error plus an asymmetry (turn bias) so the body quietly veers
    // while odometry thinks it's going straight. Plus a small per-step
    // jitter on top.
    let aL = wL, aR = wR;
    if (noise && noise.odom && noise.odom.slipSigma) {
      const s = noise.odom.slipSigma;
      if (!this._bias) {
        const scale = 1 + gaussian(rng) * s * 0.5;
        const turn = gaussian(rng) * s;
        this._bias = [scale * (1 + turn), scale * (1 - turn)];
      }
      aL = wL * this._bias[0] * (1 + gaussian(rng) * s * 0.3);
      aR = wR * this._bias[1] * (1 + gaussian(rng) * s * 0.3);
    }

    const vL = aL * TB3.wheelRadius;
    const vR = aR * TB3.wheelRadius;
    const v = (vL + vR) / 2;
    const omega = (vR - vL) / TB3.wheelSeparation;

    const nTheta = this.theta + omega * dt;
    const nx = this.x + v * Math.cos(nTheta) * dt;
    const ny = this.y + v * Math.sin(nTheta) * dt;

    this.theta = nTheta;
    if (world && world.collides(nx, ny, TB3.bodyRadius)) {
      this.collided = true; // rotation kept, translation dropped
    } else {
      this.collided = false;
      this.x = nx;
      this.y = ny;
    }
  }

  pose() { return { x: this.x, y: this.y, theta: wrap(this.theta) }; }
  odomPose() { return { x: this.odom.x, y: this.odom.y, theta: wrap(this.odom.theta) }; }
}

function wrap(t) {
  t %= 2 * Math.PI;
  if (t > Math.PI) t -= 2 * Math.PI;
  if (t <= -Math.PI) t += 2 * Math.PI;
  return t;
}
