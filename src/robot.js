// Differential-drive kinematic body, TurtleBot3 Burger sized. No inertia,
// no wheel slip — the pose is the exact integral of the commanded wheel
// speeds, except that a step which would drive the body into a wall keeps
// the rotation and drops the translation (so you can turn away).
//
// Input is Roboteq motor-command units per channel (-1000..1000); ±1000 is
// mapped to ±max wheel angular velocity. Output is encoder counts, which
// is what the Roboteq `?C` query reports back up the stack.

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
  }

  // units: [left, right] in -1000..1000. dt in seconds. world for collision.
  step(units, dt, world) {
    const wL = (Math.max(-1000, Math.min(1000, units[0])) / 1000) * MAX_WHEEL_W;
    const wR = (Math.max(-1000, Math.min(1000, units[1])) / 1000) * MAX_WHEEL_W;

    // encoder counts always advance with the wheels (even on collision —
    // the wheels keep turning against the wall)
    for (let i = 0; i < 2; i++) {
      const w = i === 0 ? wL : wR;
      const revs = (w * dt) / (2 * Math.PI);
      const exact = revs * TB3.countsPerRev + this._encFrac[i];
      const whole = Math.trunc(exact);
      this.enc[i] += whole;
      this._encFrac[i] = exact - whole;
    }

    const vL = wL * TB3.wheelRadius;
    const vR = wR * TB3.wheelRadius;
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

  pose() {
    // wrap theta to (-pi, pi]
    let t = this.theta % (2 * Math.PI);
    if (t > Math.PI) t -= 2 * Math.PI;
    if (t <= -Math.PI) t += 2 * Math.PI;
    return { x: this.x, y: this.y, theta: t };
  }
}
