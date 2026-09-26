use simplecar2::differential::Differential;

#[test]
fn test_open_differential() {
    let diff = Differential::open();
    let out = diff.distribute_torque(100.0, 50.0, 50.0, 3000.0, 3000.0);

    let expected_half = 100.0 * diff.mechanical_efficiency * 0.5;
    assert!((out.torque_left - expected_half).abs() < 1e-4);
    assert!((out.torque_right - expected_half).abs() < 1e-4);
    assert_eq!(out.diff_angular_velocity, 50.0);
    assert_eq!(out.lock_torque, 0.0);

    // Speed discrepancy: open diff STILL splits torque 50/50
    let out_spin = diff.distribute_torque(100.0, 100.0, 20.0, 3000.0, 3000.0);
    assert!((out_spin.torque_left - expected_half).abs() < 1e-4);
    assert!((out_spin.torque_right - expected_half).abs() < 1e-4);
}

#[test]
fn test_locked_differential() {
    let diff = Differential::locked();

    // Equal load => 50/50
    let out_equal = diff.distribute_torque(200.0, 40.0, 40.0, 2500.0, 2500.0);
    let total_eff = 200.0 * diff.mechanical_efficiency;
    assert!((out_equal.torque_left - total_eff * 0.5).abs() < 1e-3);
    assert!((out_equal.torque_right - total_eff * 0.5).abs() < 1e-3);

    // Unbalanced load: Left has 75% load, Right has 25% load
    let out_unbalanced = diff.distribute_torque(200.0, 40.0, 40.0, 3750.0, 1250.0);
    assert!((out_unbalanced.torque_left - total_eff * 0.75).abs() < 1e-3);
    assert!((out_unbalanced.torque_right - total_eff * 0.25).abs() < 1e-3);
}

#[test]
fn test_limited_slip_differential() {
    let diff = Differential::limited_slip(20.0, 50.0, 3.0);

    // Zero delta omega => equal split
    let out_straight = diff.distribute_torque(200.0, 50.0, 50.0, 3000.0, 3000.0);
    assert!((out_straight.torque_left - out_straight.torque_right).abs() < 1e-4);

    // Left wheel spinning faster (omega_left = 60, omega_right = 40, delta = 20)
    // LSD should transfer torque AWAY from left wheel TO right wheel
    let out_slip = diff.distribute_torque(200.0, 60.0, 40.0, 3000.0, 3000.0);
    assert!(out_slip.torque_right > out_slip.torque_left);
    assert!(out_slip.lock_torque > 0.0);
}
