use crate::PlantError;
use apollo_core::{
    APOLLO_FREEJOINT_NAME, ApolloCollisionPart, ApolloModelSpec, ApolloShape, SimulationTiming,
    apollo_collision_parts,
};
use glam::{DQuat, DVec3};
use std::fmt::Write;

/// 从后端中立的 Apollo 规格生成单自由刚体 MJCF。
///
/// 当前基线有意保持零重力、无接触和理想六维 wrench 输入；更高保真的
/// 月球重力、执行器和变质量模型应作为独立后端能力加入。
pub fn generate_apollo_mjcf(
    model_spec: ApolloModelSpec,
    timing: SimulationTiming,
) -> Result<String, PlantError> {
    validate_model_spec(model_spec)?;

    let collision_parts = apollo_collision_parts();
    for part in &collision_parts {
        validate_collision_part(*part)?;
    }

    let mut xml = String::new();
    writeln!(xml, "<mujoco model=\"apollo_lander\">").unwrap();
    writeln!(
        xml,
        "  <option timestep=\"{:.9}\" gravity=\"0 0 0\" integrator=\"RK4\"/>",
        timing.physics_step_seconds()
    )
    .unwrap();
    writeln!(
        xml,
        "  <compiler angle=\"radian\" inertiafromgeom=\"false\" alignfree=\"false\"/>"
    )
    .unwrap();
    writeln!(xml, "  <worldbody>").unwrap();
    writeln!(xml, "    <body name=\"{}\" pos=\"0 0 0\">", model_spec.name).unwrap();
    writeln!(xml, "      <freejoint name=\"{APOLLO_FREEJOINT_NAME}\"/>").unwrap();
    writeln!(
        xml,
        "      <inertial pos=\"{}\" mass=\"{:.9}\" diaginertia=\"{}\"/>",
        mj_vec(model_spec.center_of_mass_body_m),
        model_spec.mass_kg,
        mj_vec(model_spec.diagonal_inertia_body_kg_m2),
    )
    .unwrap();

    // 质量和惯量只由 body 的 inertial 元素提供，几何体不重复贡献质量。
    for part in collision_parts {
        write_collision_geom(&mut xml, part);
    }

    writeln!(xml, "    </body>").unwrap();
    writeln!(xml, "  </worldbody>").unwrap();
    writeln!(xml, "</mujoco>").unwrap();
    Ok(xml)
}

fn validate_model_spec(spec: ApolloModelSpec) -> Result<(), PlantError> {
    if spec.name.is_empty()
        || !spec
            .name
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'_' | b'-' | b'.'))
    {
        return Err(PlantError::InvalidModelSpec(
            "name must be a non-empty ASCII identifier".to_owned(),
        ));
    }
    if !spec.mass_kg.is_finite() || spec.mass_kg <= 0.0 {
        return Err(PlantError::InvalidModelSpec(
            "mass_kg must be finite and positive".to_owned(),
        ));
    }
    if !spec.center_of_mass_body_m.is_finite() {
        return Err(PlantError::InvalidModelSpec(
            "center_of_mass_body_m must be finite".to_owned(),
        ));
    }

    let inertia = spec.diagonal_inertia_body_kg_m2;
    if !inertia.is_finite() || !inertia.cmpgt(DVec3::ZERO).all() {
        return Err(PlantError::InvalidModelSpec(
            "diagonal_inertia_body_kg_m2 must be finite and positive".to_owned(),
        ));
    }
    if inertia.x + inertia.y < inertia.z
        || inertia.x + inertia.z < inertia.y
        || inertia.y + inertia.z < inertia.x
    {
        return Err(PlantError::InvalidModelSpec(
            "diagonal inertia must satisfy the triangle inequalities".to_owned(),
        ));
    }
    Ok(())
}

fn validate_collision_part(part: ApolloCollisionPart) -> Result<(), PlantError> {
    let invalid = |reason: &str| {
        PlantError::InvalidModelSpec(format!("collision part '{}': {reason}", part.name))
    };

    if part.name.is_empty() {
        return Err(invalid("name must not be empty"));
    }
    if !part.translation_body_m.is_finite()
        || !part.rotation_part_to_body.is_finite()
        || !part.scale.is_finite()
        || !part.scale.cmpgt(DVec3::ZERO).all()
    {
        return Err(invalid("transform must be finite with positive scale"));
    }
    if (part.rotation_part_to_body.length() - 1.0).abs() > 1.0e-9 {
        return Err(invalid("rotation_part_to_body must be a unit quaternion"));
    }

    let dimensions_are_valid = match part.shape {
        ApolloShape::Cuboid { size_m } => size_m.is_finite() && size_m.cmpgt(DVec3::ZERO).all(),
        ApolloShape::Cylinder {
            radius_m, height_m, ..
        } => radius_m.is_finite() && radius_m > 0.0 && height_m.is_finite() && height_m > 0.0,
        ApolloShape::Sphere { radius_m } => radius_m.is_finite() && radius_m > 0.0,
        ApolloShape::Strut {
            start_body_m,
            end_body_m,
            radius_m,
            ..
        } => {
            start_body_m.is_finite()
                && end_body_m.is_finite()
                && start_body_m != end_body_m
                && radius_m.is_finite()
                && radius_m > 0.0
        }
    };
    if !dimensions_are_valid {
        return Err(invalid("shape dimensions must be finite and positive"));
    }
    Ok(())
}

fn write_collision_geom(xml: &mut String, part: ApolloCollisionPart) {
    match part.shape {
        ApolloShape::Cuboid { size_m } => {
            let half_size = size_m * part.scale * 0.5;
            writeln!(
                xml,
                "      <geom name=\"{}\" type=\"box\" pos=\"{}\" quat=\"{}\" size=\"{}\" mass=\"0\"/>",
                part.name,
                mj_vec(part.translation_body_m),
                mj_quat(part.rotation_part_to_body),
                mj_vec(half_size),
            )
            .unwrap();
        }
        ApolloShape::Cylinder {
            radius_m, height_m, ..
        } => {
            // 中立几何中的圆柱轴为局部 +Y。用 fromto 明确表达轴向，避免
            // MuJoCo 圆柱默认 +Z 轴与 Bevy 约定不同。
            let axis_body = part.rotation_part_to_body * DVec3::Y;
            let half_axis = axis_body * (height_m * part.scale.y * 0.5);
            let start = part.translation_body_m - half_axis;
            let end = part.translation_body_m + half_axis;
            let radius = radius_m * (part.scale.x + part.scale.z) * 0.5;
            writeln!(
                xml,
                "      <geom name=\"{}\" type=\"cylinder\" fromto=\"{} {}\" size=\"{radius:.9}\" mass=\"0\"/>",
                part.name,
                mj_vec(start),
                mj_vec(end),
            )
            .unwrap();
        }
        ApolloShape::Sphere { radius_m } => {
            let radius = radius_m * part.scale.max_element();
            writeln!(
                xml,
                "      <geom name=\"{}\" type=\"sphere\" pos=\"{}\" size=\"{radius:.9}\" mass=\"0\"/>",
                part.name,
                mj_vec(part.translation_body_m),
            )
            .unwrap();
        }
        ApolloShape::Strut {
            start_body_m,
            end_body_m,
            radius_m,
            ..
        } => {
            writeln!(
                xml,
                "      <geom name=\"{}\" type=\"capsule\" fromto=\"{} {}\" size=\"{radius_m:.9}\" mass=\"0\"/>",
                part.name,
                mj_vec(start_body_m),
                mj_vec(end_body_m),
            )
            .unwrap();
        }
    }
}

fn mj_vec(value: DVec3) -> String {
    format!("{:.9} {:.9} {:.9}", value.x, value.y, value.z)
}

/// MJCF 四元数顺序为 w, x, y, z。
fn mj_quat(value: DQuat) -> String {
    format!(
        "{:.9} {:.9} {:.9} {:.9}",
        value.w, value.x, value.y, value.z
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn generated_mjcf_preserves_timing_names_and_mass_properties() {
        let xml = generate_apollo_mjcf(ApolloModelSpec::touchdown(), SimulationTiming::APOLLO)
            .expect("Apollo MJCF should be generated");

        assert!(xml.contains("timestep=\"0.002000000\""));
        assert!(xml.contains("alignfree=\"false\""));
        assert!(xml.contains("name=\"apollo_lander\""));
        assert!(xml.contains("name=\"apollo_freejoint\""));
        assert!(xml.contains("mass=\"4932.000000000\""));
        assert!(xml.contains("diaginertia=\"6332.000000000 7953.000000000 5879.000000000\""));
        assert!(xml.contains("name=\"descent_stage\""));
        assert!(!xml.contains("interstage_adapter"));
    }

    #[test]
    fn invalid_model_spec_is_rejected_before_mujoco() {
        let invalid = ApolloModelSpec {
            mass_kg: f64::NAN,
            ..ApolloModelSpec::touchdown()
        };
        assert!(matches!(
            generate_apollo_mjcf(invalid, SimulationTiming::APOLLO),
            Err(PlantError::InvalidModelSpec(_))
        ));
    }
}
