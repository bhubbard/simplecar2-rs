//! Rigorous Physical Accuracy & Kinematic Parity Benchmark Tests
//! Evaluates Hooke's law suspension equilibrium, Ackermann steering geometry, and differential torque parity.

use approx::assert_relative_eq;
use glam::Vec3;
use simplecar2::differential::Differential;
use simplecar2::steering::{Steering, SteeringConfig};
use simplecar2::suspension::{Suspension, SuspensionConfig, SuspensionState};

#[test]
fn test_suspension_hooke_equilibrium_accuracy() {
    let k = 25_000.0f32; // 25 kN/m
    let config = SuspensionConfig {
        rest_length: 0.6,
        spring_stiffness: k,
        compression_damping: 1_500.0,
        rebound_damping: 2_000.0,
        max_travel: 0.35,
        force_clamp: 100_000.0,
    };
    let suspension = Suspension::new(config);
    let mut state = SuspensionState::new(config.rest_length);

    // Quarter car mass m = 350 kg, W = m * g = 3433.5 N
    let expected_weight = 350.0 * 9.81f32; // 3433.5 N
    let expected_compression = expected_weight / k; // 0.13734 m

    let hit_dist = config.rest_length - expected_compression;
    let mount_pos = Vec3::new(0.0, 1.0, 0.0);
    let down = Vec3::NEG_Y;
    let dt = 0.01667;

    // Evaluate twice so compression velocity is 0 (static equilibrium)
    let _ = suspension.evaluate(
        &mut state,
        Some(hit_dist),
        Some(mount_pos + down * hit_dist),
        Some(Vec3::Y),
        0.0,
        mount_pos,
        down,
        dt,
    );
    let force = suspension.evaluate(
        &mut state,
        Some(hit_dist),
        Some(mount_pos + down * hit_dist),
        Some(Vec3::Y),
        0.0,
        mount_pos,
        down,
        dt,
    );

    assert_relative_eq!(state.compression, expected_compression, epsilon = 1e-4);
    assert_relative_eq!(force.spring_force, expected_weight, epsilon = 0.1);
    assert_relative_eq!(force.damping_force, 0.0, epsilon = 0.01);
    assert_relative_eq!(force.total_normal_force, expected_weight, epsilon = 0.1);
}

#[test]
fn test_ackermann_steering_kinematic_parity() {
    let wheelbase = 2.6f32;
    let track_width = 1.6f32;

    let config = SteeringConfig {
        wheelbase,
        track_width,
        max_steer_angle_rad: 35.0_f32.to_radians(),
        ..Default::default()
    };

    let steering = Steering::new(config);

    // Left turn at 20 degrees:
    let steer_rad = 20.0_f32.to_radians();
    let angles = steering.calculate_ackermann(steer_rad);

    // In pure Ackermann geometry, the inner wheel (left) turns sharper than the outer wheel (right):
    assert!(
        angles.left_rad > angles.right_rad,
        "Inner angle ({}) must exceed outer angle ({}) in Ackermann steering",
        angles.left_rad,
        angles.right_rad
    );

    // Both wheels turn in the expected positive direction:
    assert!(angles.left_rad > 0.0);
    assert!(angles.right_rad > 0.0);

    // Analytical Ackermann equations:
    // cot(delta_outer) - cot(delta_inner) = track_width / wheelbase
    let cot_outer = 1.0 / angles.right_rad.tan();
    let cot_inner = 1.0 / angles.left_rad.tan();
    let diff = cot_outer - cot_inner;
    let expected_diff = track_width / wheelbase;

    assert_relative_eq!(diff, expected_diff, epsilon = 1e-3);
}

#[test]
fn test_open_differential_torque_parity() {
    let diff = Differential::open();
    let input_torque = 500.0f32; // 500 Nm
    let split = diff.distribute_torque(input_torque, 100.0, 150.0, 2000.0, 2000.0);

    let expected_half = input_torque * diff.mechanical_efficiency * 0.5;
    assert_relative_eq!(split.torque_left, expected_half, epsilon = 1e-3);
    assert_relative_eq!(split.torque_right, expected_half, epsilon = 1e-3);

    let total_output = split.torque_left + split.torque_right;
    assert_relative_eq!(total_output, input_torque * diff.mechanical_efficiency, epsilon = 1e-3);
}
