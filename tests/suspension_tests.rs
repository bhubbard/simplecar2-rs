use glam::Vec3;
use simplecar2::suspension::{AntiRollBar, Suspension, SuspensionConfig, SuspensionState};

#[test]
fn test_suspension_spring_and_damping() {
    let config = SuspensionConfig {
        rest_length: 0.5,
        spring_stiffness: 20_000.0,
        compression_damping: 2_000.0,
        rebound_damping: 3_000.0,
        max_travel: 0.3,
        force_clamp: 50_000.0,
    };
    let suspension = Suspension::new(config);
    let mut state = SuspensionState::new(config.rest_length);

    let mount_pos = Vec3::new(0.0, 1.0, 0.0);
    let down = Vec3::NEG_Y;
    let dt = 0.02;

    // First step: wheel compressed by 0.1m (distance = 0.4m)
    let force1 = suspension.evaluate(
        &mut state,
        Some(0.4),
        Some(Vec3::new(0.0, 0.6, 0.0)),
        Some(Vec3::Y),
        0.0,
        mount_pos,
        down,
        dt,
    );

    assert!(state.is_grounded);
    assert!((state.compression - 0.1).abs() < 1e-5);
    // Compression increased from 0 to 0.1 over 0.02s => velocity = 0.1 / 0.02 = 5.0 m/s
    assert!((state.compression_velocity - 5.0).abs() < 1e-4);

    // Spring force = 20,000 * 0.1 = 2,000 N
    assert!((force1.spring_force - 2_000.0).abs() < 1e-2);
    // Damping force = 2,000 * 5.0 = 10,000 N
    assert!((force1.damping_force - 10_000.0).abs() < 1e-2);
    // Total normal force = 2,000 + 10,000 = 12,000 N
    assert!((force1.total_normal_force - 12_000.0).abs() < 1e-2);
    assert!((force1.force_vector.y - 12_000.0).abs() < 1e-2);

    // Second step at constant compression (v = 0)
    let force2 = suspension.evaluate(
        &mut state,
        Some(0.4),
        Some(Vec3::new(0.0, 0.6, 0.0)),
        Some(Vec3::Y),
        0.0,
        mount_pos,
        down,
        dt,
    );
    assert!((force2.spring_force - 2_000.0).abs() < 1e-2);
    assert!((force2.damping_force - 0.0).abs() < 1e-2);
    assert!((force2.total_normal_force - 2_000.0).abs() < 1e-2);
}

#[test]
fn test_anti_roll_bar() {
    let arb = AntiRollBar::new(10_000.0);

    let mut left_state = SuspensionState::new(0.5);
    left_state.is_grounded = true;
    left_state.compression = 0.15; // Left wheel compressed by 15cm

    let mut right_state = SuspensionState::new(0.5);
    right_state.is_grounded = true;
    right_state.compression = 0.05; // Right wheel compressed by 5cm

    let (left_adj, right_adj) = arb.calculate_forces(&left_state, &right_state);

    // delta = 0.15 - 0.05 = 0.10 m
    // arb_force = 10_000 * 0.10 = 1,000 N
    assert!((left_adj - (-1_000.0)).abs() < 1e-4);
    assert!((right_adj - 1_000.0).abs() < 1e-4);
}

#[test]
fn test_suspension_airborne() {
    let suspension = Suspension::new(SuspensionConfig::default());
    let mut state = SuspensionState::new(0.5);

    let force = suspension.evaluate(
        &mut state,
        None, // No ground contact
        None,
        None,
        0.0,
        Vec3::ZERO,
        Vec3::NEG_Y,
        0.02,
    );

    assert!(!state.is_grounded);
    assert_eq!(force.total_normal_force, 0.0);
    assert_eq!(force.force_vector, Vec3::ZERO);
}
