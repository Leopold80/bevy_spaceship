use glam::{DQuat, DVec3};

/// 后端生成的 Apollo 刚体名称。
pub const APOLLO_BODY_NAME: &str = "apollo_lander";
/// 后端生成的 Apollo 自由关节名称。
pub const APOLLO_FREEJOINT_NAME: &str = "apollo_freejoint";

/// Apollo 11 月球舱实际轻载着陆工况质量。
///
/// 数据源：NASA NTRS 20260000331，*Apollo 11 Lunar Module Touchdown
/// Dynamics Reconstruction Verification and Validation for Human Landing Systems*，
/// Table 1 的 Apollo 11 actual light touchdown 列。
pub const APOLLO_TOUCHDOWN_MASS_KG: f64 = 4_932.0;

// NASA LM 轴：X 垂直、Y 横向、Z 向前；本模型：X 向右、Y 向上、Z 向前。
// 因此 code Ixx = LM Iyy，code Iyy = LM Ixx，code Izz = LM Izz。
/// 机体 X 轴转动惯量，kg·m²。
pub const APOLLO_IXX_KG_M2: f64 = 6_332.0;
/// 机体 Y 轴转动惯量，kg·m²。
pub const APOLLO_IYY_KG_M2: f64 = 7_953.0;
/// 机体 Z 轴转动惯量，kg·m²。
pub const APOLLO_IZZ_KG_M2: f64 = 5_879.0;

/// 可视部件的后端中立材质标签。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ApolloMaterial {
    /// 金色表面。
    Gold,
    /// 金属箔表面。
    Foil,
    /// 裸露金属表面。
    Metal,
    /// 深色表面。
    Dark,
    /// 白色表面。
    White,
}

/// 不依赖任何渲染或物理后端的 Apollo 几何。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ApolloShape {
    /// 长方体。
    Cuboid {
        /// 缩放前的 XYZ 全尺寸，米。
        size_m: DVec3,
    },
    /// 局部 Y 轴上的圆柱体。
    Cylinder {
        /// 缩放前半径，米。
        radius_m: f64,
        /// 缩放前高度，米。
        height_m: f64,
        /// 可视化细分数；物理后端可忽略。
        resolution: u32,
    },
    /// 球体。
    Sphere {
        /// 缩放前半径，米。
        radius_m: f64,
    },
    /// 端点直接使用机体坐标系，便于后端生成 `fromto`/胶囊体。
    Strut {
        /// 机体系起点，米。
        start_body_m: DVec3,
        /// 机体系终点，米。
        end_body_m: DVec3,
        /// 杆件半径，米。
        radius_m: f64,
        /// 可视化细分数；物理后端可忽略。
        resolution: u32,
    },
}

/// 仅面向可视化的部件描述，不携带质量或惯量。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ApolloVisualPart {
    /// 稳定部件名。
    pub name: &'static str,
    /// 后端中立几何。
    pub shape: ApolloShape,
    /// 可视材质标签。
    pub material: ApolloMaterial,
    /// 部件原点在机体系中的位置，米。
    pub translation_body_m: DVec3,
    /// 从部件局部系到机体系的旋转。
    pub rotation_part_to_body: DQuat,
    /// 各轴无量纲缩放。
    pub scale: DVec3,
}

/// 仅面向碰撞/物理几何生成的部件描述，不携带质量或材质。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ApolloCollisionPart {
    /// 稳定部件名。
    pub name: &'static str,
    /// 后端中立几何。
    pub shape: ApolloShape,
    /// 部件原点在机体系中的位置，米。
    pub translation_body_m: DVec3,
    /// 从部件局部系到机体系的旋转。
    pub rotation_part_to_body: DQuat,
    /// 各轴无量纲缩放。
    pub scale: DVec3,
}

/// 用于保留旧模型质心算法的离散质量点。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ApolloMassPoint {
    /// 对应的稳定部件名。
    pub name: &'static str,
    /// 机体系质量点位置，米。
    pub position_body_m: DVec3,
    /// 归一化后质量，kg。
    pub mass_kg: f64,
}

/// 物理后端构建单刚体所需的质量属性。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ApolloModelSpec {
    /// 模型稳定名。
    pub name: &'static str,
    /// 总质量，kg。
    pub mass_kg: f64,
    /// 机体系质心位置，米。
    pub center_of_mass_body_m: DVec3,
    /// 绕质心、沿机体 XYZ 轴的对角惯量，kg·m²。
    pub diagonal_inertia_body_kg_m2: DVec3,
}

impl ApolloModelSpec {
    /// Apollo 11 实际轻载着陆工况。
    pub fn touchdown() -> Self {
        Self {
            name: APOLLO_BODY_NAME,
            mass_kg: total_physics_mass_kg(),
            center_of_mass_body_m: center_of_mass_body_m(),
            diagonal_inertia_body_kg_m2: DVec3::new(
                APOLLO_IXX_KG_M2,
                APOLLO_IYY_KG_M2,
                APOLLO_IZZ_KG_M2,
            ),
        }
    }
}

#[derive(Clone, Copy)]
struct RawPart {
    name: &'static str,
    shape: ApolloShape,
    material: ApolloMaterial,
    translation_body_m: DVec3,
    rotation_part_to_body: DQuat,
    scale: DVec3,
    /// 与原实现一致的相对质量权重；对外导出前归一化到着陆总质量。
    mass_weight: Option<f64>,
}

/// 完整的可视部件清单。
pub fn apollo_visual_parts() -> Vec<ApolloVisualPart> {
    raw_parts()
        .into_iter()
        .map(|part| ApolloVisualPart {
            name: part.name,
            shape: part.shape,
            material: part.material,
            translation_body_m: part.translation_body_m,
            rotation_part_to_body: part.rotation_part_to_body,
            scale: part.scale,
        })
        .collect()
}

/// 参与物理外形的部件清单。
///
/// 总质量、质心与惯量由 [`ApolloModelSpec`] 统一提供，不在几何上重复指定。
pub fn apollo_collision_parts() -> Vec<ApolloCollisionPart> {
    raw_parts()
        .into_iter()
        .filter(|part| part.mass_weight.is_some())
        .map(|part| ApolloCollisionPart {
            name: part.name,
            shape: part.shape,
            translation_body_m: part.translation_body_m,
            rotation_part_to_body: part.rotation_part_to_body,
            scale: part.scale,
        })
        .collect()
}

/// 保留旧模型质心分布的归一化质量点。
pub fn apollo_mass_points() -> Vec<ApolloMassPoint> {
    let raw = raw_parts();
    let raw_total: f64 = raw.iter().filter_map(|part| part.mass_weight).sum();
    let scale = APOLLO_TOUCHDOWN_MASS_KG / raw_total;

    raw.into_iter()
        .filter_map(|part| {
            part.mass_weight.map(|weight| ApolloMassPoint {
                name: part.name,
                // 有意使用旧 `ApolloPart.translation` 作为质量点；这保持当前 CoM 数值。
                position_body_m: part.translation_body_m,
                mass_kg: weight * scale,
            })
        })
        .collect()
}

/// 返回归一化质量点的质量之和，kg。
pub fn total_physics_mass_kg() -> f64 {
    apollo_mass_points()
        .into_iter()
        .map(|point| point.mass_kg)
        .sum()
}

/// 按归一化质量点计算机体系质心，米。
pub fn center_of_mass_body_m() -> DVec3 {
    let (weighted_position, total_mass) = apollo_mass_points().into_iter().fold(
        (DVec3::ZERO, 0.0),
        |(weighted_position, total_mass), point| {
            (
                weighted_position + point.position_body_m * point.mass_kg,
                total_mass + point.mass_kg,
            )
        },
    );

    if total_mass > 0.0 {
        weighted_position / total_mass
    } else {
        DVec3::ZERO
    }
}

fn raw_parts() -> Vec<RawPart> {
    let mut parts = vec![
        RawPart {
            name: "descent_stage",
            shape: ApolloShape::Cylinder {
                radius_m: 2.5,
                height_m: 2.5,
                resolution: 8,
            },
            material: ApolloMaterial::Metal,
            translation_body_m: DVec3::new(0.0, 1.24, 0.0),
            rotation_part_to_body: DQuat::from_rotation_y(std::f64::consts::PI / 8.0),
            scale: DVec3::new(1.18, 0.82, 1.0),
            mass_weight: Some(3_270.0),
        },
        RawPart {
            name: "interstage_adapter",
            shape: ApolloShape::Cylinder {
                radius_m: 1.5,
                height_m: 0.24,
                resolution: 16,
            },
            material: ApolloMaterial::Gold,
            translation_body_m: DVec3::new(0.0, 2.345, 0.0),
            rotation_part_to_body: DQuat::IDENTITY,
            scale: DVec3::new(1.0, 1.0, 0.92),
            mass_weight: None,
        },
        RawPart {
            name: "ascent_stage",
            shape: ApolloShape::Cylinder {
                radius_m: 1.64,
                height_m: 1.44,
                resolution: 8,
            },
            material: ApolloMaterial::Gold,
            translation_body_m: DVec3::new(0.0, 3.0, 0.0),
            rotation_part_to_body: DQuat::from_rotation_y(std::f64::consts::PI / 8.0),
            scale: DVec3::new(1.0, 0.8, 0.92),
            mass_weight: Some(3_510.0),
        },
        RawPart {
            name: "docking_adapter",
            shape: ApolloShape::Cylinder {
                radius_m: 0.68,
                height_m: 0.32,
                resolution: 24,
            },
            material: ApolloMaterial::White,
            translation_body_m: DVec3::new(0.0, 3.7, 0.0),
            rotation_part_to_body: DQuat::IDENTITY,
            scale: DVec3::ONE,
            mass_weight: None,
        },
        RawPart {
            name: "docking_hatch",
            shape: ApolloShape::Cylinder {
                radius_m: 0.76,
                height_m: 0.56,
                resolution: 24,
            },
            material: ApolloMaterial::White,
            translation_body_m: DVec3::new(0.0, 4.1, 0.0),
            rotation_part_to_body: DQuat::IDENTITY,
            scale: DVec3::ONE,
            mass_weight: Some(130.0),
        },
        RawPart {
            name: "front_window",
            shape: ApolloShape::Cuboid {
                size_m: DVec3::new(1.48, 1.04, 0.16),
            },
            material: ApolloMaterial::Dark,
            translation_body_m: DVec3::new(0.0, 1.56, 2.06),
            rotation_part_to_body: DQuat::IDENTITY,
            scale: DVec3::ONE,
            mass_weight: None,
        },
        RawPart {
            name: "side_window",
            shape: ApolloShape::Cuboid {
                size_m: DVec3::new(0.68, 0.84, 0.18),
            },
            material: ApolloMaterial::Dark,
            translation_body_m: DVec3::new(0.96, 2.52, 1.56),
            rotation_part_to_body: DQuat::from_rotation_y(-0.32),
            scale: DVec3::ONE,
            mass_weight: None,
        },
        RawPart {
            name: "left_foil_panel",
            shape: ApolloShape::Cuboid {
                size_m: DVec3::new(0.64, 1.72, 0.24),
            },
            material: ApolloMaterial::Gold,
            translation_body_m: DVec3::new(-1.9, 1.1, 0.16),
            rotation_part_to_body: DQuat::from_rotation_z(0.28),
            scale: DVec3::ONE,
            mass_weight: None,
        },
        RawPart {
            name: "right_foil_panel",
            shape: ApolloShape::Cuboid {
                size_m: DVec3::new(0.64, 1.72, 0.24),
            },
            material: ApolloMaterial::Gold,
            translation_body_m: DVec3::new(1.9, 1.1, -0.16),
            rotation_part_to_body: DQuat::from_rotation_z(-0.28),
            scale: DVec3::ONE,
            mass_weight: None,
        },
        RawPart {
            name: "dish",
            shape: ApolloShape::Sphere { radius_m: 0.56 },
            material: ApolloMaterial::Dark,
            translation_body_m: DVec3::new(-2.36, 4.16, -1.56),
            rotation_part_to_body: DQuat::IDENTITY,
            scale: DVec3::new(1.0, 0.12, 1.0),
            mass_weight: None,
        },
        RawPart {
            name: "dish_mount",
            shape: ApolloShape::Cylinder {
                radius_m: 0.07,
                height_m: 0.32,
                resolution: 10,
            },
            material: ApolloMaterial::Metal,
            translation_body_m: DVec3::new(-2.36, 3.94, -1.56),
            rotation_part_to_body: DQuat::IDENTITY,
            scale: DVec3::ONE,
            mass_weight: None,
        },
        RawPart {
            name: "antenna_brace",
            shape: ApolloShape::Strut {
                start_body_m: DVec3::new(-1.3, 3.42, -0.72),
                end_body_m: DVec3::new(-2.36, 3.94, -1.56),
                radius_m: 0.036,
                resolution: 8,
            },
            material: ApolloMaterial::Metal,
            translation_body_m: DVec3::ZERO,
            rotation_part_to_body: DQuat::IDENTITY,
            scale: DVec3::ONE,
            mass_weight: None,
        },
        RawPart {
            name: "side_antenna",
            shape: ApolloShape::Cylinder {
                radius_m: 0.16,
                height_m: 1.4,
                resolution: 16,
            },
            material: ApolloMaterial::Metal,
            translation_body_m: DVec3::new(1.12, 3.84, -0.7),
            rotation_part_to_body: DQuat::from_rotation_z(0.35),
            scale: DVec3::ONE,
            mass_weight: None,
        },
    ];

    for index in 0..4 {
        let angle = index as f64 * std::f64::consts::FRAC_PI_2 + std::f64::consts::FRAC_PI_4;
        let direction = DVec3::new(angle.cos(), 0.0, angle.sin());
        let foot = direction * 4.1 + DVec3::new(0.0, -2.32, 0.0);
        let leg_mount = direction * 2.04;

        parts.push(RawPart {
            name: match index {
                0 => "landing_strut_front_right",
                1 => "landing_strut_back_right",
                2 => "landing_strut_back_left",
                _ => "landing_strut_front_left",
            },
            shape: ApolloShape::Strut {
                start_body_m: leg_mount,
                end_body_m: foot,
                radius_m: 0.07,
                resolution: 12,
            },
            material: ApolloMaterial::Metal,
            translation_body_m: DVec3::ZERO,
            rotation_part_to_body: DQuat::IDENTITY,
            scale: DVec3::ONE,
            mass_weight: Some(64.0),
        });

        parts.push(RawPart {
            name: match index {
                0 => "foot_front_right",
                1 => "foot_back_right",
                2 => "foot_back_left",
                _ => "foot_front_left",
            },
            shape: ApolloShape::Cylinder {
                radius_m: 0.76,
                height_m: 0.18,
                resolution: 32,
            },
            material: ApolloMaterial::Foil,
            translation_body_m: foot,
            rotation_part_to_body: DQuat::IDENTITY,
            scale: DVec3::ONE,
            mass_weight: Some(40.0),
        });

        parts.push(RawPart {
            name: match index {
                0 => "leg_fairing_front_right",
                1 => "leg_fairing_back_right",
                2 => "leg_fairing_back_left",
                _ => "leg_fairing_front_left",
            },
            shape: ApolloShape::Cuboid {
                size_m: DVec3::new(0.4, 0.28, 0.36),
            },
            material: ApolloMaterial::Dark,
            translation_body_m: leg_mount + direction * 0.16 + DVec3::new(0.0, 0.02, 0.0),
            rotation_part_to_body: DQuat::from_rotation_y(-angle),
            scale: DVec3::ONE,
            mass_weight: None,
        });
    }

    parts
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn visual_collision_and_mass_surfaces_are_separate() {
        let visuals = apollo_visual_parts();
        let collisions = apollo_collision_parts();
        let mass_points = apollo_mass_points();

        assert_eq!(visuals.len(), 25);
        assert_eq!(collisions.len(), 11);
        assert_eq!(mass_points.len(), 11);
        assert!(visuals.iter().any(|part| part.name == "interstage_adapter"));
        assert!(
            !collisions
                .iter()
                .any(|part| part.name == "interstage_adapter")
        );

        let collision_names: Vec<_> = collisions.iter().map(|part| part.name).collect();
        let mass_names: Vec<_> = mass_points.iter().map(|point| point.name).collect();
        assert_eq!(collision_names, mass_names);
    }

    #[test]
    fn touchdown_mass_inertia_and_center_of_mass_match_baseline() {
        let spec = ApolloModelSpec::touchdown();
        assert!((spec.mass_kg - 4_932.0).abs() < 1.0e-9);
        assert_eq!(
            spec.diagonal_inertia_body_kg_m2,
            DVec3::new(6_332.0, 7_953.0, 5_879.0)
        );

        // 与旧算法一致：使用部件 translation 的相对质量加权，归一化不改变 CoM。
        let expected_y =
            (3_270.0 * 1.24 + 3_510.0 * 3.0 + 130.0 * 4.1 + 4.0 * 40.0 * -2.32) / 7_326.0;
        assert!(spec.center_of_mass_body_m.x.abs() < 1.0e-15);
        assert!((spec.center_of_mass_body_m.y - expected_y).abs() < 1.0e-12);
        assert!(spec.center_of_mass_body_m.z.abs() < 1.0e-15);
    }

    #[test]
    fn collision_shapes_preserve_landing_gear_geometry() {
        let descent = apollo_collision_parts()
            .into_iter()
            .find(|part| part.name == "descent_stage")
            .unwrap();
        assert!(matches!(
            descent.shape,
            ApolloShape::Cylinder {
                radius_m: 2.5,
                height_m: 2.5,
                ..
            }
        ));

        for strut in apollo_collision_parts()
            .into_iter()
            .filter(|part| part.name.starts_with("landing_strut"))
        {
            let ApolloShape::Strut {
                start_body_m,
                end_body_m,
                ..
            } = strut.shape
            else {
                unreachable!();
            };
            assert!(end_body_m.y < start_body_m.y);
            let start_radius = start_body_m.x.hypot(start_body_m.z);
            let end_radius = end_body_m.x.hypot(end_body_m.z);
            assert!(end_radius > start_radius + 1.0);
        }
    }

    #[test]
    fn all_public_model_numbers_are_finite_and_physical() {
        for part in apollo_visual_parts() {
            assert!(part.translation_body_m.is_finite());
            assert!(part.rotation_part_to_body.is_finite());
            assert!(part.scale.is_finite());
            assert!(part.scale.cmpgt(DVec3::ZERO).all());
        }
        for point in apollo_mass_points() {
            assert!(point.position_body_m.is_finite());
            assert!(point.mass_kg.is_finite() && point.mass_kg > 0.0);
        }
    }
}
