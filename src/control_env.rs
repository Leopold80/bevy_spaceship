use crate::control_law::{ApolloController, CascadedAttitudeController};
use crate::mujoco_dynamics::{ApolloDynamics, ApolloDynamicsState, ApolloWrench};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

pub const APOLLO_CONTROLLER_DT_SECS: f32 = 0.020;

#[derive(Clone, Copy, Debug)]
pub struct ApolloControlSnapshot {
    pub state: ApolloDynamicsState,
    pub sim_time_secs: f32,
    pub control_tick: u64,
    pub running: bool,
}

#[derive(Debug)]
struct SharedApolloInner {
    snapshot: ApolloControlSnapshot,
}

pub struct ApolloControlEnv<C: ApolloController> {
    dynamics: ApolloDynamics,
    controller: C,
    controller_dt_secs: f32,
    control_hold_steps: usize,
    sim_time_secs: f32,
    control_tick: u64,
    held_wrench: ApolloWrench,
}

impl<C: ApolloController> ApolloControlEnv<C> {
    pub fn new(
        dynamics: ApolloDynamics,
        controller_dt_secs: f32,
        controller: C,
    ) -> Result<Self, String> {
        let simulation_dt_secs = dynamics.simulation_dt_secs();
        let control_hold_steps =
            fixed_hold_steps(simulation_dt_secs, controller_dt_secs).ok_or_else(|| {
                format!(
                    "controller dt {controller_dt_secs} must be an integer multiple of simulation dt {simulation_dt_secs}"
                )
            })?;

        Ok(Self {
            dynamics,
            controller,
            controller_dt_secs,
            control_hold_steps,
            sim_time_secs: 0.0,
            control_tick: 0,
            held_wrench: ApolloWrench::default(),
        })
    }

    pub fn step_control_tick(&mut self) -> ApolloControlSnapshot {
        let state_before_control = self.dynamics.state();
        self.held_wrench = self
            .controller
            .update(state_before_control, self.sim_time_secs);

        let mut state = self.dynamics.state();
        for _ in 0..self.control_hold_steps {
            state = self.dynamics.step(self.held_wrench);
        }

        self.control_tick += 1;
        self.sim_time_secs += self.controller_dt_secs;
        self.snapshot_with_state(state)
    }

    pub fn reset(&mut self) -> ApolloControlSnapshot {
        self.dynamics.reset();
        self.controller.reset();
        self.sim_time_secs = 0.0;
        self.control_tick = 0;
        self.held_wrench = ApolloWrench::default();
        self.snapshot()
    }

    pub fn snapshot(&self) -> ApolloControlSnapshot {
        self.snapshot_with_state(self.dynamics.state())
    }

    pub fn control_hold_steps(&self) -> usize {
        self.control_hold_steps
    }

    pub fn controller_dt_secs(&self) -> f32 {
        self.controller_dt_secs
    }

    fn snapshot_with_state(&self, state: ApolloDynamicsState) -> ApolloControlSnapshot {
        ApolloControlSnapshot {
            state,
            sim_time_secs: self.sim_time_secs,
            control_tick: self.control_tick,
            running: true,
        }
    }
}

impl ApolloControlEnv<CascadedAttitudeController> {
    pub fn new_attitude_control(
        dynamics: ApolloDynamics,
        controller_dt_secs: f32,
    ) -> Result<Self, String> {
        Self::new(
            dynamics,
            controller_dt_secs,
            CascadedAttitudeController::default(),
        )
    }
}

#[derive(Clone)]
pub struct SharedApolloState {
    inner: Arc<Mutex<SharedApolloInner>>,
    reset_requested: Arc<AtomicBool>,
    running: Arc<AtomicBool>,
    worker: Arc<Mutex<Option<JoinHandle<()>>>>,
}

impl SharedApolloState {
    pub fn start() -> Result<Self, String> {
        let dynamics = ApolloDynamics::new()?;
        let env = ApolloControlEnv::new_attitude_control(dynamics, APOLLO_CONTROLLER_DT_SECS)?;
        let initial_snapshot = env.snapshot();

        let shared = Self {
            inner: Arc::new(Mutex::new(SharedApolloInner {
                snapshot: initial_snapshot,
            })),
            reset_requested: Arc::new(AtomicBool::new(false)),
            running: Arc::new(AtomicBool::new(true)),
            worker: Arc::new(Mutex::new(None)),
        };

        let thread_shared = shared.clone_without_worker();
        let handle = thread::spawn(move || run_simulation_loop(env, thread_shared));
        *shared
            .worker
            .lock()
            .expect("simulation worker mutex poisoned") = Some(handle);

        Ok(shared)
    }

    pub fn snapshot(&self) -> ApolloControlSnapshot {
        let mut snapshot = self
            .inner
            .lock()
            .expect("Apollo state mutex poisoned")
            .snapshot;
        snapshot.running = self.running.load(Ordering::Acquire);
        snapshot
    }

    pub fn request_reset(&self) {
        self.reset_requested.store(true, Ordering::Release);
    }

    pub fn stop(&self) {
        self.running.store(false, Ordering::Release);
        if let Some(handle) = self
            .worker
            .lock()
            .expect("simulation worker mutex poisoned")
            .take()
        {
            let _ = handle.join();
        }
    }

    fn publish(&self, snapshot: ApolloControlSnapshot) {
        self.inner
            .lock()
            .expect("Apollo state mutex poisoned")
            .snapshot = snapshot;
    }

    fn take_reset_request(&self) -> bool {
        self.reset_requested.swap(false, Ordering::AcqRel)
    }

    fn clone_without_worker(&self) -> Self {
        Self {
            inner: self.inner.clone(),
            reset_requested: self.reset_requested.clone(),
            running: self.running.clone(),
            worker: Arc::new(Mutex::new(None)),
        }
    }
}

impl Drop for SharedApolloState {
    fn drop(&mut self) {
        if Arc::strong_count(&self.worker) == 1 {
            self.stop();
        }
    }
}

fn run_simulation_loop<C: ApolloController>(
    mut env: ApolloControlEnv<C>,
    shared: SharedApolloState,
) {
    let tick_duration = Duration::from_secs_f32(env.controller_dt_secs());

    while shared.running.load(Ordering::Acquire) {
        let tick_start = Instant::now();
        let snapshot = if shared.take_reset_request() {
            env.reset()
        } else {
            env.step_control_tick()
        };
        shared.publish(snapshot);

        if let Some(remaining) = tick_duration.checked_sub(tick_start.elapsed()) {
            thread::sleep(remaining);
        }
    }

    let mut snapshot = env.snapshot();
    snapshot.running = false;
    shared.publish(snapshot);
}

fn fixed_hold_steps(simulation_dt_secs: f32, controller_dt_secs: f32) -> Option<usize> {
    if simulation_dt_secs <= 0.0 || controller_dt_secs <= 0.0 {
        return None;
    }

    let ratio = controller_dt_secs / simulation_dt_secs;
    let rounded = ratio.round();
    if (ratio - rounded).abs() <= 1e-4 {
        Some(rounded as usize)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::apollo_spec::APOLLO_MUJOCO_TIMESTEP_SECS;
    use crate::control_law::ApolloController;
    use std::sync::atomic::{AtomicUsize, Ordering};

    fn approx_eq(lhs: f32, rhs: f32) -> bool {
        (lhs - rhs).abs() < 1e-6
    }

    struct CountingController {
        updates: Arc<AtomicUsize>,
    }

    struct ZeroWrenchController;

    impl ApolloController for CountingController {
        fn update(&mut self, _state: ApolloDynamicsState, _sim_time_secs: f32) -> ApolloWrench {
            self.updates.fetch_add(1, Ordering::AcqRel);
            ApolloWrench::default()
        }
    }

    impl ApolloController for ZeroWrenchController {
        fn update(&mut self, _state: ApolloDynamicsState, _sim_time_secs: f32) -> ApolloWrench {
            ApolloWrench::default()
        }
    }

    #[test]
    fn controller_dt_is_integer_multiple_of_simulation_dt() {
        let dynamics = ApolloDynamics::new().expect("Apollo MuJoCo model should load");
        assert!(approx_eq(
            dynamics.simulation_dt_secs(),
            APOLLO_MUJOCO_TIMESTEP_SECS as f32
        ));

        let env = ApolloControlEnv::new(dynamics, APOLLO_CONTROLLER_DT_SECS, ZeroWrenchController)
            .expect("controller dt should align with simulation dt");
        assert_eq!(env.control_hold_steps(), 10);
    }

    #[test]
    fn control_tick_advances_exactly_one_controller_period() {
        let dynamics = ApolloDynamics::new().expect("Apollo MuJoCo model should load");
        let mut env =
            ApolloControlEnv::new(dynamics, APOLLO_CONTROLLER_DT_SECS, ZeroWrenchController)
                .expect("controller dt should align with simulation dt");

        let initial = env.snapshot();
        let next = env.step_control_tick();

        assert_eq!(initial.control_tick, 0);
        assert_eq!(next.control_tick, 1);
        assert!(approx_eq(next.sim_time_secs, APOLLO_CONTROLLER_DT_SECS));
    }

    #[test]
    fn reset_restores_control_clock() {
        let dynamics = ApolloDynamics::new().expect("Apollo MuJoCo model should load");
        let mut env =
            ApolloControlEnv::new(dynamics, APOLLO_CONTROLLER_DT_SECS, ZeroWrenchController)
                .expect("controller dt should align with simulation dt");

        env.step_control_tick();
        let reset = env.reset();

        assert_eq!(reset.control_tick, 0);
        assert!(approx_eq(reset.sim_time_secs, 0.0));
        assert!(reset.state.position.is_finite());
        assert!(reset.state.rotation.is_finite());
    }

    #[test]
    fn controller_updates_once_per_control_tick() {
        let dynamics = ApolloDynamics::new().expect("Apollo MuJoCo model should load");
        let updates = Arc::new(AtomicUsize::new(0));
        let controller = CountingController {
            updates: updates.clone(),
        };
        let mut env = ApolloControlEnv::new(dynamics, APOLLO_CONTROLLER_DT_SECS, controller)
            .expect("controller dt should align with simulation dt");

        env.step_control_tick();
        env.step_control_tick();

        assert_eq!(updates.load(Ordering::Acquire), 2);
        assert_eq!(env.control_hold_steps(), 10);
    }
}
