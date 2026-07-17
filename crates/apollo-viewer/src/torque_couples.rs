//! 仅供交互演示使用的确定性 RCS 纯力偶选择。
//!
//! 这里不接受期望六维 wrench，也不属于 plant/领域契约；它只从共享推进规格
//! 穷举出六个便于人工检查的正负轴喷口组合。

use glam::DVec3;

const FORCE_ZERO_TOLERANCE_N: f64 = 1.0e-9;
const CROSS_AXIS_TORQUE_RATIO: f64 = 1.0e-9;
const SCORE_TIE_TOLERANCE_NM: f64 = 1.0e-9;

/// 从共享 RCS 规格抽取的选择器输入；`stable_index` 同时作为最终 tie-break。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ThrusterCandidate {
    pub stable_index: usize,
    pub position_body_m: DVec3,
    /// 飞船受力方向，而不是喷管/羽流方向。
    pub force_direction_body: DVec3,
    pub maximum_thrust_n: f64,
}

/// 某一正负机体轴对应的纯力偶喷口集合。
#[derive(Clone, Debug, PartialEq)]
pub struct AxisTorqueSet {
    pub target_axis_body: DVec3,
    pub thruster_indices: Vec<usize>,
    pub torque_about_com_body_nm: DVec3,
}

/// 依次返回 `+X, -X, +Y, -Y, +Z, -Z` 六个纯力偶。
///
/// 优先选择两喷口组合；某轴不存在两喷口纯力偶时才尝试四喷口组合。
/// 候选必须净力为零、横轴力矩相对目标轴不超过容差，再按目标轴力矩最大、
/// 稳定索引字典序最小选择。
pub fn select_axis_torque_sets(
    thrusters: &[ThrusterCandidate],
    center_of_mass_body_m: DVec3,
) -> Result<[AxisTorqueSet; 6], &'static str> {
    if thrusters.len() < 2
        || !center_of_mass_body_m.is_finite()
        || thrusters.iter().any(|thruster| {
            !thruster.position_body_m.is_finite()
                || !thruster.force_direction_body.is_finite()
                || !thruster.force_direction_body.is_normalized()
                || !thruster.maximum_thrust_n.is_finite()
                || thruster.maximum_thrust_n <= 0.0
        })
    {
        return Err("RCS 纯力偶选择器收到无效规格");
    }

    let axes = [
        DVec3::X,
        DVec3::NEG_X,
        DVec3::Y,
        DVec3::NEG_Y,
        DVec3::Z,
        DVec3::NEG_Z,
    ];
    let mut selected = Vec::with_capacity(axes.len());
    for axis in axes {
        let best = best_combination(thrusters, center_of_mass_body_m, axis, 2)
            .or_else(|| best_combination(thrusters, center_of_mass_body_m, axis, 4))
            .ok_or("共享 RCS 规格无法构成全部六个纯力偶")?;
        selected.push(AxisTorqueSet {
            target_axis_body: axis,
            thruster_indices: best.indices,
            torque_about_com_body_nm: best.torque,
        });
    }
    selected
        .try_into()
        .map_err(|_| "内部错误：纯力偶数量不是六个")
}

#[derive(Clone, Debug)]
struct Combination {
    indices: Vec<usize>,
    torque: DVec3,
    target_score_nm: f64,
}

fn best_combination(
    thrusters: &[ThrusterCandidate],
    center_of_mass_body_m: DVec3,
    target_axis: DVec3,
    count: usize,
) -> Option<Combination> {
    let mut best = None;
    let mut indices = Vec::with_capacity(count);
    visit_combinations(thrusters.len(), count, 0, &mut indices, &mut |indices| {
        let (force, torque) = combined_wrench(thrusters, center_of_mass_body_m, indices);
        if force.length() > FORCE_ZERO_TOLERANCE_N {
            return;
        }
        let target_score_nm = torque.dot(target_axis);
        if target_score_nm <= 0.0 {
            return;
        }
        let cross_axis = torque - target_axis * target_score_nm;
        if cross_axis.length() / target_score_nm > CROSS_AXIS_TORQUE_RATIO {
            return;
        }

        let mut stable_indices: Vec<_> = indices
            .iter()
            .map(|index| thrusters[*index].stable_index)
            .collect();
        stable_indices.sort_unstable();
        let candidate = Combination {
            indices: stable_indices,
            torque,
            target_score_nm,
        };
        if best
            .as_ref()
            .is_none_or(|current| is_better(&candidate, current))
        {
            best = Some(candidate);
        }
    });
    best
}

fn visit_combinations(
    item_count: usize,
    choose: usize,
    start: usize,
    indices: &mut Vec<usize>,
    visitor: &mut impl FnMut(&[usize]),
) {
    if indices.len() == choose {
        visitor(indices);
        return;
    }
    let remaining = choose - indices.len();
    if remaining > item_count.saturating_sub(start) {
        return;
    }
    let last_start = item_count - remaining;
    for index in start..=last_start {
        indices.push(index);
        visit_combinations(item_count, choose, index + 1, indices, visitor);
        indices.pop();
    }
}

fn combined_wrench(
    thrusters: &[ThrusterCandidate],
    center_of_mass_body_m: DVec3,
    indices: &[usize],
) -> (DVec3, DVec3) {
    indices.iter().fold(
        (DVec3::ZERO, DVec3::ZERO),
        |(total_force, total_torque), index| {
            let thruster = thrusters[*index];
            let force = thruster.force_direction_body * thruster.maximum_thrust_n;
            let lever = thruster.position_body_m - center_of_mass_body_m;
            (total_force + force, total_torque + lever.cross(force))
        },
    )
}

fn is_better(candidate: &Combination, current: &Combination) -> bool {
    candidate.target_score_nm > current.target_score_nm + SCORE_TIE_TOLERANCE_NM
        || ((candidate.target_score_nm - current.target_score_nm).abs() <= SCORE_TIE_TOLERANCE_NM
            && candidate.indices < current.indices)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn candidate(index: usize, position: DVec3, force_direction: DVec3) -> ThrusterCandidate {
        ThrusterCandidate {
            stable_index: index,
            position_body_m: position,
            force_direction_body: force_direction,
            maximum_thrust_n: 100.0,
        }
    }

    fn symmetric_test_thrusters() -> Vec<ThrusterCandidate> {
        vec![
            // +X / -X 力偶：Y 方向力臂配合正反 Z 力。
            candidate(0, DVec3::Y, DVec3::Z),
            candidate(1, DVec3::NEG_Y, DVec3::NEG_Z),
            candidate(2, DVec3::Y, DVec3::NEG_Z),
            candidate(3, DVec3::NEG_Y, DVec3::Z),
            // +Y / -Y 力偶：Z 方向力臂配合正反 X 力。
            candidate(4, DVec3::Z, DVec3::X),
            candidate(5, DVec3::NEG_Z, DVec3::NEG_X),
            candidate(6, DVec3::Z, DVec3::NEG_X),
            candidate(7, DVec3::NEG_Z, DVec3::X),
            // +Z / -Z 力偶：X 方向力臂配合正反 Y 力。
            candidate(8, DVec3::X, DVec3::Y),
            candidate(9, DVec3::NEG_X, DVec3::NEG_Y),
            candidate(10, DVec3::X, DVec3::NEG_Y),
            candidate(11, DVec3::NEG_X, DVec3::Y),
        ]
    }

    #[test]
    fn selects_all_six_axis_signs_as_pure_two_thruster_couples() {
        let thrusters = symmetric_test_thrusters();
        let selected = select_axis_torque_sets(&thrusters, DVec3::ZERO).unwrap();
        for set in selected {
            assert_eq!(set.thruster_indices.len(), 2);
            let axis_score = set.torque_about_com_body_nm.dot(set.target_axis_body);
            assert!(axis_score > 0.0);
            assert!(
                (set.torque_about_com_body_nm - set.target_axis_body * axis_score).length()
                    <= 1.0e-12
            );
            let positions: Vec<_> = set
                .thruster_indices
                .iter()
                .map(|stable_index| {
                    thrusters
                        .iter()
                        .position(|thruster| thruster.stable_index == *stable_index)
                        .unwrap()
                })
                .collect();
            let (force, _) = combined_wrench(&thrusters, DVec3::ZERO, &positions);
            assert!(force.length() <= FORCE_ZERO_TOLERANCE_N);
        }
    }

    #[test]
    fn equal_torque_uses_stable_index_lexicographic_tie_break() {
        let mut thrusters = symmetric_test_thrusters();
        thrusters.push(candidate(12, DVec3::Y, DVec3::Z));
        thrusters.push(candidate(13, DVec3::NEG_Y, DVec3::NEG_Z));
        let selected = select_axis_torque_sets(&thrusters, DVec3::ZERO).unwrap();
        assert_eq!(selected[0].thruster_indices, vec![0, 1]);
    }

    #[test]
    fn rejects_non_normalized_force_direction() {
        let mut thrusters = symmetric_test_thrusters();
        thrusters[0].force_direction_body = DVec3::new(2.0, 0.0, 0.0);
        assert!(select_axis_torque_sets(&thrusters, DVec3::ZERO).is_err());
    }

    #[test]
    fn shared_apollo11_spec_produces_six_deterministic_pure_couples() {
        let spec = apollo_core::ApolloPropulsionSpec::apollo11_touchdown();
        let candidates: Vec<_> = spec
            .rcs_thrusters
            .iter()
            .map(|thruster| ThrusterCandidate {
                stable_index: thruster.id.index(),
                position_body_m: thruster.position_body_m,
                force_direction_body: thruster.force_direction_body,
                maximum_thrust_n: thruster.steady_thrust_n,
            })
            .collect();
        let selected = select_axis_torque_sets(&candidates, apollo_core::center_of_mass_body_m())
            .expect("Apollo 11 RCS topology must provide six pure couples");
        assert_eq!(selected.len(), 6);
        assert!(selected.iter().all(|set| set.thruster_indices.len() == 2));
    }
}
