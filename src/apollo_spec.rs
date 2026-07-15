use bevy::prelude::*;
use std::fmt::Write;

pub const APOLLO_BODY_NAME: &str = "apollo_lander";
pub const APOLLO_FREEJOINT_NAME: &str = "apollo_freejoint";
pub const APOLLO_MUJOCO_TIMESTEP_SECS: f64 = 0.002;

/// Apollo 11 LM 实际着陆工况质量属性。
///
/// 数据源：NASA NTRS 20260000331, *Apollo 11 Lunar Module Touchdown
/// Dynamics Reconstruction Verification and Validation for Human Landing Systems*,
/// Table 1 的 Apollo 11 实际轻载着陆列。
///
/// NASA LM 本体轴定义：X 垂直向上、Y 横向向右、Z 向前。
/// 本项目模型轴定义：X 向右、Y 向上、Z 向前。因此映射为：
///   code Ixx = LM Iyy
///   code Iyy = LM Ixx
///   code Izz = LM Izz
pub const APOLLO_TOUCHDOWN_MASS_KG: f32 = 4_932.0;
const APOLLO_LM_IXX_VERTICAL: f32 = 7_953.0;
const APOLLO_LM_IYY_LATERAL: f32 = 6_332.0;
const APOLLO_LM_IZZ_FORWARD: f32 = 5_879.0;

pub const APOLLO_IXX: f32 = APOLLO_LM_IYY_LATERAL;
pub const APOLLO_IYY: f32 = APOLLO_LM_IXX_VERTICAL;
pub const APOLLO_IZZ: f32 = APOLLO_LM_IZZ_FORWARD;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ApolloMaterial {
    Gold,
    Foil,
    Metal,
    Dark,
    White,
}

#[derive(Clone, Copy, Debug)]
pub enum ApolloShape {
    Cuboid {
        size: Vec3,
    },
    Cylinder {
        radius: f32,
        height: f32,
        resolution: u32,
    },
    Sphere {
        radius: f32,
    },
    Strut {
        start: Vec3,
        end: Vec3,
        radius: f32,
        resolution: u32,
    },
}

#[derive(Clone, Copy, Debug)]
pub struct ApolloPart {
    pub name: &'static str,
    pub shape: ApolloShape,
    pub material: ApolloMaterial,
    pub translation: Vec3,
    pub rotation: Quat,
    pub scale: Vec3,
    pub physics_mass: Option<f32>,
}

impl ApolloPart {
    pub fn visual_transform(self) -> Transform {
        Transform::from_translation(self.translation)
            .with_rotation(self.rotation)
            .with_scale(self.scale)
    }
}

pub fn apollo_parts() -> Vec<ApolloPart> {
    let mut parts = vec![
        // 下列数值作为当前粗略部件模型的相对质量权重；函数返回前会按
        // NASA Apollo 11 实际着陆总质量统一归一化，保留现有质心分布。
        ApolloPart {
            name: "descent_stage",
            shape: ApolloShape::Cylinder {
                radius: 2.5,
                height: 2.5,
                resolution: 8,
            },
            material: ApolloMaterial::Metal,
            translation: Vec3::new(0.0, 1.24, 0.0),
            rotation: Quat::from_rotation_y(std::f32::consts::PI / 8.0),
            scale: Vec3::new(1.18, 0.82, 1.0),
            physics_mass: Some(3270.0),
        },
        // 视觉转接环同时嵌入下降级顶面和上升级底面，消除两级之间的悬空缝隙。
        // 该零件不参与质量计算，因此不会因外形修复改变飞行动力学参数。
        ApolloPart {
            name: "interstage_adapter",
            shape: ApolloShape::Cylinder {
                radius: 1.5,
                height: 0.24,
                resolution: 16,
            },
            material: ApolloMaterial::Gold,
            translation: Vec3::new(0.0, 2.345, 0.0),
            rotation: Quat::IDENTITY,
            scale: Vec3::new(1.0, 1.0, 0.92),
            physics_mass: None,
        },
        ApolloPart {
            name: "ascent_stage",
            shape: ApolloShape::Cylinder {
                radius: 1.64,
                height: 1.44,
                resolution: 8,
            },
            material: ApolloMaterial::Gold,
            translation: Vec3::new(0.0, 3.0, 0.0),
            rotation: Quat::from_rotation_y(std::f32::consts::PI / 8.0),
            scale: Vec3::new(1.0, 0.8, 0.92),
            physics_mass: Some(3510.0),
        },
        // 对接舱口原本与上升级顶面之间存在可见空隙；用短转接颈桥接两者。
        ApolloPart {
            name: "docking_adapter",
            shape: ApolloShape::Cylinder {
                radius: 0.68,
                height: 0.32,
                resolution: 24,
            },
            material: ApolloMaterial::White,
            translation: Vec3::new(0.0, 3.7, 0.0),
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
            physics_mass: None,
        },
        ApolloPart {
            name: "docking_hatch",
            shape: ApolloShape::Cylinder {
                radius: 0.76,
                height: 0.56,
                resolution: 24,
            },
            material: ApolloMaterial::White,
            translation: Vec3::new(0.0, 4.1, 0.0),
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
            physics_mass: Some(130.0),
        },
        ApolloPart {
            name: "front_window",
            shape: ApolloShape::Cuboid {
                size: Vec3::new(1.48, 1.04, 0.16),
            },
            material: ApolloMaterial::Dark,
            translation: Vec3::new(0.0, 1.56, 2.06),
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
            physics_mass: None,
        },
        ApolloPart {
            name: "side_window",
            shape: ApolloShape::Cuboid {
                size: Vec3::new(0.68, 0.84, 0.18),
            },
            material: ApolloMaterial::Dark,
            translation: Vec3::new(0.96, 2.52, 1.56),
            rotation: Quat::from_rotation_y(-0.32),
            scale: Vec3::ONE,
            physics_mass: None,
        },
        ApolloPart {
            name: "left_foil_panel",
            shape: ApolloShape::Cuboid {
                size: Vec3::new(0.64, 1.72, 0.24),
            },
            material: ApolloMaterial::Gold,
            translation: Vec3::new(-1.9, 1.1, 0.16),
            rotation: Quat::from_rotation_z(0.28),
            scale: Vec3::ONE,
            physics_mass: None,
        },
        ApolloPart {
            name: "right_foil_panel",
            shape: ApolloShape::Cuboid {
                size: Vec3::new(0.64, 1.72, 0.24),
            },
            material: ApolloMaterial::Gold,
            translation: Vec3::new(1.9, 1.1, -0.16),
            rotation: Quat::from_rotation_z(-0.28),
            scale: Vec3::ONE,
            physics_mass: None,
        },
        ApolloPart {
            name: "dish",
            shape: ApolloShape::Sphere { radius: 0.56 },
            material: ApolloMaterial::Dark,
            translation: Vec3::new(-2.36, 4.16, -1.56),
            rotation: Quat::IDENTITY,
            scale: Vec3::new(1.0, 0.12, 1.0),
            physics_mass: None,
        },
        ApolloPart {
            name: "dish_mount",
            shape: ApolloShape::Cylinder {
                radius: 0.07,
                height: 0.32,
                resolution: 10,
            },
            material: ApolloMaterial::Metal,
            translation: Vec3::new(-2.36, 3.94, -1.56),
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
            physics_mass: None,
        },
        ApolloPart {
            name: "antenna_brace",
            shape: ApolloShape::Strut {
                // 两端分别嵌入上升级壳体和碟形天线底座，避免近看时悬空。
                start: Vec3::new(-1.3, 3.42, -0.72),
                end: Vec3::new(-2.36, 3.94, -1.56),
                radius: 0.036,
                resolution: 8,
            },
            material: ApolloMaterial::Metal,
            translation: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
            physics_mass: None,
        },
        ApolloPart {
            name: "side_antenna",
            shape: ApolloShape::Cylinder {
                radius: 0.16,
                height: 1.4,
                resolution: 16,
            },
            material: ApolloMaterial::Metal,
            // 收回天线，使倾斜圆柱的下端落入上升级外壳内。
            translation: Vec3::new(1.12, 3.84, -0.7),
            rotation: Quat::from_rotation_z(0.35),
            scale: Vec3::ONE,
            physics_mass: None,
        },
    ];

    // Landing gear geometry constants scaled for the ~1:2→1:1 size correction.
    // Direction angles place four legs at 45° diagonals relative to the body axes.
    for i in 0..4 {
        let angle = i as f32 * std::f32::consts::FRAC_PI_2 + std::f32::consts::PI / 4.0;
        let dir = Vec3::new(angle.cos(), 0.0, angle.sin());
        let foot = dir * 4.1 + Vec3::new(0.0, -2.32, 0.0);
        let leg_mount = dir * 2.04 + Vec3::new(0.0, 0.0, 0.0);

        // 原始权重 64，整机归一化后每根约 43.1 kg。
        parts.push(ApolloPart {
            name: match i {
                0 => "landing_strut_front_right",
                1 => "landing_strut_back_right",
                2 => "landing_strut_back_left",
                _ => "landing_strut_front_left",
            },
            shape: ApolloShape::Strut {
                start: leg_mount,
                end: foot,
                radius: 0.07,
                resolution: 12,
            },
            material: ApolloMaterial::Metal,
            translation: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
            physics_mass: Some(64.0),
        });

        // 原始权重 40，整机归一化后每个约 26.9 kg。
        parts.push(ApolloPart {
            name: match i {
                0 => "foot_front_right",
                1 => "foot_back_right",
                2 => "foot_back_left",
                _ => "foot_front_left",
            },
            shape: ApolloShape::Cylinder {
                radius: 0.76,
                height: 0.18,
                resolution: 32,
            },
            material: ApolloMaterial::Foil,
            translation: foot,
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
            physics_mass: Some(40.0),
        });

        parts.push(ApolloPart {
            name: match i {
                0 => "leg_fairing_front_right",
                1 => "leg_fairing_back_right",
                2 => "leg_fairing_back_left",
                _ => "leg_fairing_front_left",
            },
            shape: ApolloShape::Cuboid {
                size: Vec3::new(0.4, 0.28, 0.36),
            },
            material: ApolloMaterial::Dark,
            translation: leg_mount + dir * 0.16 + Vec3::new(0.0, 0.02, 0.0),
            rotation: Quat::from_rotation_y(-angle),
            scale: Vec3::ONE,
            physics_mass: None,
        });
    }

    let raw_total_mass: f32 = parts.iter().filter_map(|part| part.physics_mass).sum();
    let mass_scale = APOLLO_TOUCHDOWN_MASS_KG / raw_total_mass;
    for part in &mut parts {
        if let Some(mass) = &mut part.physics_mass {
            *mass *= mass_scale;
        }
    }

    parts
}

/// Sum of all `physics_mass` values across `apollo_parts()`.
pub fn total_physics_mass() -> f32 {
    apollo_parts()
        .into_iter()
        .filter_map(|p| p.physics_mass)
        .sum()
}

/// Mass-weighted centre of mass of the physics-bearing parts, in body frame.
pub fn center_of_mass() -> Vec3 {
    let (weighted_sum, total_mass) = apollo_parts()
        .into_iter()
        .filter_map(|p| p.physics_mass.map(|m| (p.translation * m, m)))
        .fold((Vec3::ZERO, 0.0_f32), |(acc_w, acc_m), (w, m)| {
            (acc_w + w, acc_m + m)
        });
    if total_mass > 0.0 {
        weighted_sum / total_mass
    } else {
        Vec3::ZERO
    }
}

pub fn apollo_mjcf_xml() -> String {
    let mut xml = String::new();
    writeln!(xml, "<mujoco model=\"apollo_lander\">").unwrap();
    writeln!(
        xml,
        "  <option timestep=\"{APOLLO_MUJOCO_TIMESTEP_SECS}\" gravity=\"0 0 0\" integrator=\"RK4\"/>"
    )
    .unwrap();
    writeln!(xml, "  <compiler angle=\"radian\"/>").unwrap();
    writeln!(xml, "  <worldbody>").unwrap();
    writeln!(
        xml,
        "    <light name=\"key\" pos=\"0 6 6\" dir=\"0 -1 -1\"/>"
    )
    .unwrap();
    writeln!(xml, "    <body name=\"{APOLLO_BODY_NAME}\" pos=\"0 0 0\">").unwrap();
    writeln!(xml, "      <freejoint name=\"{APOLLO_FREEJOINT_NAME}\"/>").unwrap();

    // Body-level mass and centre of mass are computed from apollo_parts()
    // so they stay in sync when part masses change.  The diagonal inertia
    // is the Apollo 11 lunar-landing reference — a design target that
    // simplified cylinder geometry cannot reproduce on its own.
    let mass = total_physics_mass();
    let com = center_of_mass();
    writeln!(
        xml,
        "      <inertial pos=\"{com_x:.4} {com_y:.4} {com_z:.4}\" \
         mass=\"{mass:.3}\" \
         diaginertia=\"{APOLLO_IXX:.0} {APOLLO_IYY:.0} {APOLLO_IZZ:.0}\"/>",
        com_x = com.x,
        com_y = com.y,
        com_z = com.z,
    )
    .unwrap();

    // Geoms carry zero mass: the <inertial> element above provides the
    // total body mass and inertia.  Keeping geoms massless keeps the
    // single-source-of-truth property — the part masses in apollo_parts()
    // define the total, not the per-geom XML attributes.
    for part in apollo_parts()
        .into_iter()
        .filter(|part| part.physics_mass.is_some())
    {
        write_mjcf_geom(&mut xml, part, 0.0);
    }

    writeln!(xml, "    </body>").unwrap();
    writeln!(xml, "  </worldbody>").unwrap();
    writeln!(xml, "</mujoco>").unwrap();
    xml
}

fn write_mjcf_geom(xml: &mut String, part: ApolloPart, mass: f32) {
    match part.shape {
        ApolloShape::Cuboid { size } => {
            let half = size * part.scale * 0.5;
            writeln!(
                xml,
                "      <geom name=\"{}\" type=\"box\" pos=\"{}\" size=\"{}\" mass=\"{mass:.3}\"/>",
                part.name,
                mj_vec(part.translation),
                mj_vec(half)
            )
            .unwrap();
        }
        ApolloShape::Cylinder { radius, height, .. } => {
            let start = part.translation - Vec3::Y * height * part.scale.y * 0.5;
            let end = part.translation + Vec3::Y * height * part.scale.y * 0.5;
            let radius = radius * ((part.scale.x + part.scale.z) * 0.5);
            writeln!(
                xml,
                "      <geom name=\"{}\" type=\"cylinder\" fromto=\"{} {}\" size=\"{radius:.4}\" mass=\"{mass:.3}\"/>",
                part.name,
                mj_vec(start),
                mj_vec(end)
            )
            .unwrap();
        }
        ApolloShape::Sphere { radius } => {
            let radius = radius * part.scale.max_element();
            writeln!(
                xml,
                "      <geom name=\"{}\" type=\"sphere\" pos=\"{}\" size=\"{radius:.4}\" mass=\"{mass:.3}\"/>",
                part.name,
                mj_vec(part.translation),
            )
            .unwrap();
        }
        ApolloShape::Strut {
            start, end, radius, ..
        } => {
            writeln!(
                xml,
                "      <geom name=\"{}\" type=\"capsule\" fromto=\"{} {}\" size=\"{radius:.4}\" mass=\"{mass:.3}\"/>",
                part.name,
                mj_vec(start),
                mj_vec(end)
            )
            .unwrap();
        }
    }
}

fn mj_vec(v: Vec3) -> String {
    format!("{:.5} {:.5} {:.5}", v.x, v.y, v.z)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn part_named(name: &str) -> ApolloPart {
        apollo_parts()
            .into_iter()
            .find(|part| part.name == name)
            .unwrap_or_else(|| panic!("missing Apollo part: {name}"))
    }

    fn cylinder_vertical_bounds(part: ApolloPart) -> (f32, f32) {
        let ApolloShape::Cylinder { height, .. } = part.shape else {
            panic!("{} should be a cylinder", part.name);
        };
        let half_height = height * part.scale.y * 0.5;
        (
            part.translation.y - half_height,
            part.translation.y + half_height,
        )
    }

    fn cylinder_horizontal_radii(part: ApolloPart) -> Vec2 {
        let ApolloShape::Cylinder { radius, .. } = part.shape else {
            panic!("{} should be a cylinder", part.name);
        };
        Vec2::new(radius * part.scale.x, radius * part.scale.z)
    }

    #[test]
    fn mjcf_contains_named_free_body() {
        let xml = apollo_mjcf_xml();
        assert!(xml.contains(APOLLO_BODY_NAME));
        assert!(xml.contains(APOLLO_FREEJOINT_NAME));
        assert!(xml.contains("descent_stage"));
        assert!(xml.contains("mass=\"4932.000\""));
        assert!(xml.contains("diaginertia=\"6332 7953 5879\""));
    }

    #[test]
    fn nasa_apollo_11_touchdown_inertia_axes_map_to_model_axes() {
        // NASA LM: X 垂直、Y 横向、Z 向前；模型：X 横向、Y 垂直、Z 向前。
        assert_eq!(APOLLO_IXX, APOLLO_LM_IYY_LATERAL);
        assert_eq!(APOLLO_IYY, APOLLO_LM_IXX_VERTICAL);
        assert_eq!(APOLLO_IZZ, APOLLO_LM_IZZ_FORWARD);
    }

    #[test]
    fn landing_gear_starts_below_descent_stage_skirt() {
        // descent_stage translation.y=1.24, cylinder height=2.5, scale.y=0.82
        let descent_stage_bottom = 1.24 - 2.5 * 0.82 * 0.5;

        for part in apollo_parts()
            .into_iter()
            .filter(|part| part.name.starts_with("landing_strut"))
        {
            let ApolloShape::Strut { start, end, .. } = part.shape else {
                panic!("{} should be a strut", part.name);
            };
            let start_radial = Vec2::new(start.x, start.z).length();
            let end_radial = Vec2::new(end.x, end.z).length();

            assert!(
                start.y <= descent_stage_bottom - 0.02,
                "{} starts above the descent-stage lower skirt: {}",
                part.name,
                start.y
            );
            assert!(
                end.y < start.y,
                "{} should angle downward away from the lander",
                part.name
            );
            // Radial sweep must be at least 1.0 m outward (scaled from 0.5 m).
            assert!(
                end_radial > start_radial + 1.0,
                "{} should sweep outward toward the footpad",
                part.name
            );
        }
    }

    #[test]
    fn adapters_bridge_the_main_stage_stack_without_affecting_mass() {
        let descent_part = part_named("descent_stage");
        let descent = cylinder_vertical_bounds(descent_part);
        let interstage_part = part_named("interstage_adapter");
        let interstage = cylinder_vertical_bounds(interstage_part);
        let ascent_part = part_named("ascent_stage");
        let ascent = cylinder_vertical_bounds(ascent_part);
        let docking_adapter_part = part_named("docking_adapter");
        let docking_adapter = cylinder_vertical_bounds(docking_adapter_part);
        let docking_hatch_part = part_named("docking_hatch");
        let docking_hatch = cylinder_vertical_bounds(docking_hatch_part);

        assert!(interstage.0 < descent.1 && interstage.1 > ascent.0);
        assert!(docking_adapter.0 < ascent.1 && docking_adapter.1 > docking_hatch.0);

        let descent_radii = cylinder_horizontal_radii(descent_part);
        let interstage_radii = cylinder_horizontal_radii(interstage_part);
        let ascent_radii = cylinder_horizontal_radii(ascent_part);
        let docking_adapter_radii = cylinder_horizontal_radii(docking_adapter_part);
        let docking_hatch_radii = cylinder_horizontal_radii(docking_hatch_part);
        assert!(interstage_radii.cmpgt(Vec2::ZERO).all());
        assert!(interstage_radii.cmple(descent_radii).all());
        assert!(interstage_radii.cmple(ascent_radii).all());
        assert!(docking_adapter_radii.cmpgt(Vec2::ZERO).all());
        assert!(docking_adapter_radii.cmple(ascent_radii).all());
        assert!(docking_adapter_radii.cmple(docking_hatch_radii).all());

        assert!(interstage_part.physics_mass.is_none());
        assert!(docking_adapter_part.physics_mass.is_none());
        assert!((total_physics_mass() - APOLLO_TOUCHDOWN_MASS_KG).abs() < 0.01);

        let xml = apollo_mjcf_xml();
        assert!(!xml.contains("interstage_adapter"));
        assert!(!xml.contains("docking_adapter"));
    }

    #[test]
    fn upper_antennas_are_anchored_to_the_lander() {
        let ascent = part_named("ascent_stage");
        let ApolloShape::Cylinder { radius, height, .. } = ascent.shape else {
            panic!("ascent_stage should be a cylinder");
        };
        let ascent_bottom = ascent.translation.y - height * ascent.scale.y * 0.5;
        let ascent_top = ascent.translation.y + height * ascent.scale.y * 0.5;

        let side_antenna = part_named("side_antenna");
        let ApolloShape::Cylinder { height, .. } = side_antenna.shape else {
            panic!("side_antenna should be a cylinder");
        };
        let antenna_axis = side_antenna.rotation * Vec3::Y;
        let antenna_bottom =
            side_antenna.translation - antenna_axis * height * side_antenna.scale.y * 0.5;
        let normalized_radius = Vec2::new(
            antenna_bottom.x / (radius * ascent.scale.x),
            antenna_bottom.z / (radius * ascent.scale.z),
        )
        .length();
        assert!(normalized_radius < 1.0);
        assert!((ascent_bottom..=ascent_top).contains(&antenna_bottom.y));

        let brace = part_named("antenna_brace");
        let ApolloShape::Strut { start, end, .. } = brace.shape else {
            panic!("antenna_brace should be a strut");
        };
        let brace_start_radius = Vec2::new(
            start.x / (radius * ascent.scale.x),
            start.z / (radius * ascent.scale.z),
        )
        .length();
        assert!(brace_start_radius < 1.0);
        assert!((ascent_bottom..=ascent_top).contains(&start.y));

        let dish_mount = part_named("dish_mount");
        assert_eq!(end, dish_mount.translation);
        let (_, dish_mount_top) = cylinder_vertical_bounds(dish_mount);
        let dish = part_named("dish");
        let ApolloShape::Sphere { radius } = dish.shape else {
            panic!("dish should be a sphere");
        };
        let dish_bottom = dish.translation.y - radius * dish.scale.y;
        assert!(dish_mount_top > dish_bottom);
    }
}
