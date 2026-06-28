use bevy::prelude::*;
use std::fmt::Write;

pub const APOLLO_BODY_NAME: &str = "apollo_lander";
pub const APOLLO_FREEJOINT_NAME: &str = "apollo_freejoint";
pub const APOLLO_MUJOCO_TIMESTEP_SECS: f64 = 0.002;

/// Apollo 11 LM lunar-landing diagonal moment-of-inertia (kg·m²).
///
/// source: SNA-8-D-027III Rev.2 CSM/LM Spacecraft Operational Data Book
/// Vol.3, Apollo 11 Mission Report Appendix A.6.  Original values in
/// slug-ft² converted to SI (1 slug-ft² = 1.35581795 kg·m²).
///
/// Coordinate mapping: LM (X fwd, Y right, Z down) → code (X right, Y up, Z fwd).
///   code Ixx = LM Iyy = 13 867 slug-ft² → 18 801 kg·m²
///   code Iyy = LM Izz = 16 204 slug-ft² → 21 970 kg·m²
///   code Izz = LM Ixx = 12 582 slug-ft² → 17 059 kg·m²
pub const APOLLO_IXX: f32 = 18_801.0;
pub const APOLLO_IYY: f32 = 21_970.0;
pub const APOLLO_IZZ: f32 = 17_059.0;

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
        // Apollo LM at lunar landing: ~7,327 kg total. Masses are scaled so
        // that the MuJoCo-computed body mass matches that figure.
        // The dry masses below are the real Apollo LM breakdown; the
        // 1.6× multiplier accounts for propellant, crew, and consumables
        // still aboard at landing.
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
                start: Vec3::new(-1.48, 3.44, -0.9),
                end: Vec3::new(-2.24, 3.84, -1.48),
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
            translation: Vec3::new(1.44, 3.84, -0.7),
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

        // ~64 kg per landing strut (scaled to total landing mass).
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

        // ~40 kg per footpad.
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

    #[test]
    fn mjcf_contains_named_free_body() {
        let xml = apollo_mjcf_xml();
        assert!(xml.contains(APOLLO_BODY_NAME));
        assert!(xml.contains(APOLLO_FREEJOINT_NAME));
        assert!(xml.contains("descent_stage"));
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
}
