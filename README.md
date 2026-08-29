# simulator — lightweight 2D world sim

Stands in for a real robot so the stack can be exercised end to end without
hardware or Gazebo. A differential-drive body (TurtleBot3 Burger sized) in
a 2D world, no 3D, no inertia, no wheel slip.

- **`ws://127.0.0.1:8765`** — the same Roboteq ASCII protocol as
  `robot-base/sim` (so `robot-os-chromium`'s `WebSocketTransport` /
  dashboard connect unchanged). `?C` encoder counts come from the physics
  body; the RWD watchdog still stops the wheels on serial silence.
- **`ws://127.0.0.1:8766`** — JSON sensor stream, one line per scan tick:
  `{ t, groundTruth:{x,y,theta}, collided, scan:{ ranges, angleIncrement, … } }`,
  preceded by one `{ type:"hello", world, robot, lidar }`. This is what a
  real LIDAR + localization bridge would feed the nav layer
  (roadmap.md Phase 7). Split from :8765 on purpose — on the real robot
  the laser is a different device from the motor controller.

The `firmware` here speaks Former's Roboteq protocol but the body is
TB3-sized; `robot-os-chromium/manifests/former.manifest.json` drives it
unchanged (±1000 units → ±max wheel speed).

## Run

```sh
cd simulator
npm install          # just `ws`
npm start            # world "room" 8×6 m; ws 8765 (roboteq) + 8766 (sensors)
```

Env: `SIM_WORLD=worlds/other.world.json`, `SIM_RWD_MS=300`,
`SIM_START="x,y,theta"`, `SIM_PORT`, `SIM_SENSOR_PORT`.

## Drive it

**Quick view:** open `viewer.html` in Chrome (works from `file://`) — arrow
keys / WASD to drive, `Space` = E-STOP, `R` = re-enable. Shows the world,
the body, and the laser.

**Full stack:** run `robot-os-chromium`'s `serve-dashboard.mjs`, open the
dashboard in **local** mode — the SharedWorker connects to
`ws://127.0.0.1:8765`, i.e. this sim. Slider / gamepad teleop drives the
body; watch it move (and hit walls) in `viewer.html` alongside.

## Test

```sh
npm test             # test/sim-smoke.mjs — 10 checks, no browser
```

Drives the Roboteq endpoint, watches the sensor stream: body moves in the
world, stops at a wall, encoders track, laser scan is sane, RWD watchdog
fires on silence, spins in place on a rotate command.

## Worlds

`worlds/*.world.json`: `{ name, bounds:[w,h], walls:[[x1,y1,x2,y2], …] }`
in metres. The outer `bounds` rectangle is added as walls automatically.

## Not this

3D physics, sensor noise, TB3's real ROS stack, Gazebo. When we need those
(sensor realism for SLAM tuning), that's the separate Gazebo + TB3 track —
see roadmap.md.
