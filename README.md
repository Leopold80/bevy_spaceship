# Bevy Spacecraft

Apollo-style lunar module visualization and quaternion attitude-control experiments in Bevy.

## Branches

- `main`: baseline Apollo-style lunar module scene.
- `experiment/quaternion-attitude-control`: kinematic quaternion attitude-control demo with logs and visual verification.

## Quaternion Attitude-Control Demo

The experiment branch validates the simplified kinematic outer-loop law:

```text
q_e = q_d^-1 * q
if q_e0 < 0, q_e = -q_e
omega_c = -kp * q_e0 * q_ev
```

The implementation intentionally models only kinematics. It does not include rigid-body dynamics,
inertia, actuator torque, saturation, or an inner angular-rate loop.

## Run

```bash
cargo run
```

Controls in the visual demo:

- `Space` or `R`: reset the current scenario.
- `1`, `2`, `3`: switch between repeatable initial attitude scenarios.
- `P`: pause or resume convergence.

The right-side overlay shows two coordinate frames at the same origin:

- Thick frame: desired attitude `q_d`.
- Thin translucent frame: current attitude `q`.

As the controller converges, the current frame should overlap the desired frame.

## Headless Log Verification

Use the headless mode when GPU rendering is unavailable:

```bash
cargo run -- --headless-log
```

This writes:

```text
logs/attitude_kinematics.csv
```

Expected trends:

- `qe0 >= 0`, showing unwind avoidance.
- `qev_norm` decreases toward zero.
- `error_angle_rad` decreases toward zero.
- `omega_norm` decreases as the error shrinks.

## Theory Notes

The corrected derivation is kept in:

```text
docs/quaternion_attitude_control.md
```

The Bevy-independent control code is in:

```text
src/attitude_control.rs
```

The scene, controls, HUD, logging, and coordinate-frame visualization are in:

```text
src/main.rs
```

## Checks

```bash
cargo fmt --check
cargo check
cargo test
```
