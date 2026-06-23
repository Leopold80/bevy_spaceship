# Bevy Spacecraft

Baseline Apollo-style lunar module visualization in Bevy.

## Branches

- `main`: baseline Apollo-style lunar module scene.
- `experiment/quaternion-attitude-control`: quaternion attitude-control experiment with corrected notes, visual frame overlay, and CSV log verification.

## Run Baseline Scene

```bash
cargo run
```

## Experiment Branch

Switch to the experiment branch for the kinematic quaternion attitude-control demo:

```bash
git switch experiment/quaternion-attitude-control
cargo run
```

The experiment validates:

```text
q_e = q_d^-1 * q
if q_e0 < 0, q_e = -q_e
omega_c = -kp * q_e0 * q_ev
```

It also includes a headless CSV log mode:

```bash
cargo run -- --headless-log
```

## Checks

```bash
cargo fmt --check
cargo check
cargo test
```
