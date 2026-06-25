/// Centralized timing constants for the Apollo MuJoCo control demo.
///
/// Keep the numerical time base here so MuJoCo integration, controller
/// sampling, and zero-order hold logic cannot silently drift apart.
pub const APOLLO_MUJOCO_TIMESTEP_MICROS: u64 = 2_000;
pub const APOLLO_CONTROL_HOLD_STEPS: usize = 10;

pub const APOLLO_MUJOCO_TIMESTEP_SECS: f64 = APOLLO_MUJOCO_TIMESTEP_MICROS as f64 / 1_000_000.0;
pub const APOLLO_MUJOCO_TIMESTEP_SECS_F32: f32 =
    APOLLO_MUJOCO_TIMESTEP_MICROS as f32 / 1_000_000.0;
pub const APOLLO_CONTROLLER_TIMESTEP_SECS: f32 =
    APOLLO_MUJOCO_TIMESTEP_SECS_F32 * APOLLO_CONTROL_HOLD_STEPS as f32;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn controller_timestep_matches_mujoco_hold_steps() {
        assert_eq!(APOLLO_MUJOCO_TIMESTEP_MICROS, 2_000);
        assert_eq!(APOLLO_CONTROL_HOLD_STEPS, 10);
        assert!((APOLLO_CONTROLLER_TIMESTEP_SECS - 0.020).abs() < 1e-6);
    }
}
