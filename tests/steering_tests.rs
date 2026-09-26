use simplecar2::steering::{Steering, SteeringConfig, SteeringState};

#[test]
fn test_ackermann_geometry_left_turn() {
    let config = SteeringConfig {
        wheelbase: 2.7,
        track_width: 1.6,
        max_steer_angle_rad: 35.0_f32.to_radians(),
        ..Default::default()
    };
    let steering = Steering::new(config);

    // Turning left (positive angle)
    let steer_rad = 20.0_f32.to_radians();
    let angles = steering.calculate_ackermann(steer_rad);

    // In a left turn, inner wheel (left) turns sharper than outer wheel (right)
    assert!(angles.left_rad > angles.right_rad);
    assert!(angles.left_rad > 0.0);
    assert!(angles.right_rad > 0.0);
}

#[test]
fn test_ackermann_geometry_right_turn() {
    let config = SteeringConfig {
        wheelbase: 2.7,
        track_width: 1.6,
        max_steer_angle_rad: 35.0_f32.to_radians(),
        ..Default::default()
    };
    let steering = Steering::new(config);

    // Turning right (negative angle)
    let steer_rad = -20.0_f32.to_radians();
    let angles = steering.calculate_ackermann(steer_rad);

    // In a right turn, inner wheel (right) turns sharper than outer wheel (left)
    // Angles are negative: |right| > |left| => right < left
    assert!(angles.right_rad.abs() > angles.left_rad.abs());
    assert!(angles.left_rad < 0.0);
    assert!(angles.right_rad < 0.0);
}

#[test]
fn test_speed_sensitive_steering_reduction() {
    let config = SteeringConfig {
        max_steer_angle_rad: 30.0_f32.to_radians(),
        speed_sensitivity_factor: 0.04,
        ..Default::default()
    };
    let steering = Steering::new(config);

    let low_speed_angle = steering.speed_attenuated_angle(1.0, 0.0);
    let mid_speed_angle = steering.speed_attenuated_angle(1.0, 20.0); // 72 km/h
    let high_speed_angle = steering.speed_attenuated_angle(1.0, 50.0); // 180 km/h

    // Angle must strictly decrease with speed
    assert!(low_speed_angle > mid_speed_angle);
    assert!(mid_speed_angle > high_speed_angle);

    // At 50 m/s with factor 0.04: divisor = 1.0 + 50 * 0.04 = 3.0 => angle is 1/3 of max
    let expected_high = config.max_steer_angle_rad / 3.0;
    assert!((high_speed_angle - expected_high).abs() < 1e-4);
}

#[test]
fn test_steering_state_update_smoothing() {
    let steering = Steering::new(SteeringConfig::default());
    let mut state = SteeringState::new();

    // Step with full left input
    steering.update(&mut state, 1.0, 10.0, 0.02);
    assert!(state.smoothed_input > 0.0 && state.smoothed_input <= 1.0);
    assert!(state.current_angles.left_rad > 0.0);
}
