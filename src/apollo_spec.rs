use bevy::prelude::*;
use std::fmt::Write;

pub const APOLLO_BODY_NAME: &str = "apollo_lander";
pub const APOLLO_FREEJOINT_NAME: &str = "apollo_freejoint";
pub const APOLLO_MUJOCO_TIMESTEP_SECS: f64 = 0.002;

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
        ApolloPart {
            name: "descent_stage",
            shape: ApolloShape::Cylinder {
                radius: 1.25,
                height: 1.25,
                resolution: 8,
            },
            material: ApolloMaterial::Metal,
            translation: Vec3::new(0.0, 0.62, 0.0),
            rotation: Quat::from_rotation_y(std::f32::consts::PI / 8.0),
            scale: Vec3::new(1.18, 0.82, 1.0),
            physics_mass: Some(840.0),
        },
        ApolloPart {
            name: "ascent_stage",
            shape: ApolloShape::Cylinder {
                radius: 0.82,
                height: 0.72,
                resolution: 8,
            },
            material: ApolloMaterial::Gold,
            translation: Vec3::new(0.0, 1.5, 0.0),
            rotation: Quat::from_rotation_y(std::f32::consts::PI / 8.0),
            scale: Vec3::new(1.0, 0.8, 0.92),
            physics_mass: Some(420.0),
        },
        ApolloPart {
            name: "docking_hatch",
            shape: ApolloShape::Cylinder {
                radius: 0.38,
                height: 0.28,
                resolution: 24,
            },
            material: ApolloMaterial::White,
            translation: Vec3::new(0.0, 2.05, 0.0),
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
            physics_mass: Some(45.0),
        },
        ApolloPart {
            name: "front_window",
            shape: ApolloShape::Cuboid {
                size: Vec3::new(0.74, 0.52, 0.08),
            },
            material: ApolloMaterial::Dark,
            translation: Vec3::new(0.0, 0.78, 1.03),
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
            physics_mass: None,
        },
        ApolloPart {
            name: "side_window",
            shape: ApolloShape::Cuboid {
                size: Vec3::new(0.34, 0.42, 0.09),
            },
            material: ApolloMaterial::Dark,
            translation: Vec3::new(0.48, 1.26, 0.78),
            rotation: Quat::from_rotation_y(-0.32),
            scale: Vec3::ONE,
            physics_mass: None,
        },
        ApolloPart {
            name: "left_foil_panel",
            shape: ApolloShape::Cuboid {
                size: Vec3::new(0.32, 0.86, 0.12),
            },
            material: ApolloMaterial::Gold,
            translation: Vec3::new(-0.95, 0.55, 0.08),
            rotation: Quat::from_rotation_z(0.28),
            scale: Vec3::ONE,
            physics_mass: None,
        },
        ApolloPart {
            name: "right_foil_panel",
            shape: ApolloShape::Cuboid {
                size: Vec3::new(0.32, 0.86, 0.12),
            },
            material: ApolloMaterial::Gold,
            translation: Vec3::new(0.95, 0.55, -0.08),
            rotation: Quat::from_rotation_z(-0.28),
            scale: Vec3::ONE,
            physics_mass: None,
        },
        ApolloPart {
            name: "dish",
            shape: ApolloShape::Sphere { radius: 0.28 },
            material: ApolloMaterial::Dark,
            translation: Vec3::new(-1.18, 2.08, -0.78),
            rotation: Quat::IDENTITY,
            scale: Vec3::new(1.0, 0.12, 1.0),
            physics_mass: None,
        },
        ApolloPart {
            name: "dish_mount",
            shape: ApolloShape::Cylinder {
                radius: 0.035,
                height: 0.16,
                resolution: 10,
            },
            material: ApolloMaterial::Metal,
            translation: Vec3::new(-1.18, 1.97, -0.78),
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
            physics_mass: None,
        },
        ApolloPart {
            name: "antenna_brace",
            shape: ApolloShape::Strut {
                start: Vec3::new(-0.74, 1.72, -0.45),
                end: Vec3::new(-1.12, 1.92, -0.74),
                radius: 0.018,
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
                radius: 0.08,
                height: 0.7,
                resolution: 16,
            },
            material: ApolloMaterial::Metal,
            translation: Vec3::new(0.72, 1.92, -0.35),
            rotation: Quat::from_rotation_z(0.35),
            scale: Vec3::ONE,
            physics_mass: None,
        },
    ];

    for i in 0..4 {
        let angle = i as f32 * std::f32::consts::FRAC_PI_2 + std::f32::consts::PI / 4.0;
        let dir = Vec3::new(angle.cos(), 0.0, angle.sin());
        let foot = dir * 2.05 + Vec3::new(0.0, -1.16, 0.0);
        let upper = dir * 0.78 + Vec3::new(0.0, 0.18, 0.0);

        parts.push(ApolloPart {
            name: match i {
                0 => "landing_strut_front_right",
                1 => "landing_strut_back_right",
                2 => "landing_strut_back_left",
                _ => "landing_strut_front_left",
            },
            shape: ApolloShape::Strut {
                start: upper,
                end: foot,
                radius: 0.035,
                resolution: 12,
            },
            material: ApolloMaterial::Metal,
            translation: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
            physics_mass: Some(24.0),
        });

        parts.push(ApolloPart {
            name: match i {
                0 => "landing_brace_front_right",
                1 => "landing_brace_back_right",
                2 => "landing_brace_back_left",
                _ => "landing_brace_front_left",
            },
            shape: ApolloShape::Strut {
                start: Vec3::new(0.0, 0.15, 0.0),
                end: foot + Vec3::new(0.0, 0.22, 0.0),
                radius: 0.022,
                resolution: 10,
            },
            material: ApolloMaterial::Metal,
            translation: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
            physics_mass: Some(12.0),
        });

        parts.push(ApolloPart {
            name: match i {
                0 => "foot_front_right",
                1 => "foot_back_right",
                2 => "foot_back_left",
                _ => "foot_front_left",
            },
            shape: ApolloShape::Cylinder {
                radius: 0.38,
                height: 0.09,
                resolution: 32,
            },
            material: ApolloMaterial::Foil,
            translation: foot,
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
            physics_mass: Some(18.0),
        });

        parts.push(ApolloPart {
            name: match i {
                0 => "leg_fairing_front_right",
                1 => "leg_fairing_back_right",
                2 => "leg_fairing_back_left",
                _ => "leg_fairing_front_left",
            },
            shape: ApolloShape::Cuboid {
                size: Vec3::new(0.12, 0.42, 0.12),
            },
            material: ApolloMaterial::Dark,
            translation: upper + dir * 0.18,
            rotation: Quat::from_rotation_y(-angle),
            scale: Vec3::ONE,
            physics_mass: None,
        });
    }

    parts
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

    for part in apollo_parts()
        .into_iter()
        .filter(|part| part.physics_mass.is_some())
    {
        write_mjcf_geom(&mut xml, part);
    }

    writeln!(xml, "    </body>").unwrap();
    writeln!(xml, "  </worldbody>").unwrap();
    writeln!(xml, "</mujoco>").unwrap();
    xml
}

fn write_mjcf_geom(xml: &mut String, part: ApolloPart) {
    let mass = part.physics_mass.unwrap();
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
}
