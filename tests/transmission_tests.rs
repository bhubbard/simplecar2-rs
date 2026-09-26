use simplecar2::transmission::{Gear, Transmission, TransmissionConfig, TransmissionState};

#[test]
fn test_gear_ratios_and_total_ratio() {
    let config = TransmissionConfig {
        forward_ratios: vec![3.5, 2.0, 1.4, 1.0, 0.8, 0.65],
        reverse_ratio: -3.2,
        final_drive_ratio: 4.0,
        ..Default::default()
    };
    let trans = Transmission::new(config);

    assert_eq!(trans.gear_ratio(Gear::Reverse), -3.2);
    assert_eq!(trans.gear_ratio(Gear::Neutral), 0.0);
    assert_eq!(trans.gear_ratio(Gear::Forward(1)), 3.5);
    assert_eq!(trans.gear_ratio(Gear::Forward(4)), 1.0);
    assert_eq!(trans.gear_ratio(Gear::Forward(6)), 0.65);

    // Total ratio: ratio * final drive (4.0)
    assert_eq!(trans.total_ratio(Gear::Forward(1)), 14.0);
    assert_eq!(trans.total_ratio(Gear::Reverse), -12.8);
    assert_eq!(trans.total_ratio(Gear::Neutral), 0.0);
}

#[test]
fn test_manual_shifting() {
    let trans = Transmission::new(TransmissionConfig::default());
    let mut state = TransmissionState::in_neutral();

    assert_eq!(state.current_gear, Gear::Neutral);

    trans.shift_up(&mut state);
    assert_eq!(state.current_gear, Gear::Forward(1));
    assert!(state.is_shifting);

    // Let shift timer expire
    trans.update(&mut state, 3000.0, 8000.0, 0.5, 0.2);
    assert!(!state.is_shifting);

    trans.shift_up(&mut state);
    assert_eq!(state.current_gear, Gear::Forward(2));

    // Shift down back towards neutral and reverse
    trans.update(&mut state, 3000.0, 8000.0, 0.5, 0.2);
    trans.shift_down(&mut state);
    assert_eq!(state.current_gear, Gear::Forward(1));

    trans.update(&mut state, 3000.0, 8000.0, 0.5, 0.2);
    trans.shift_down(&mut state);
    assert_eq!(state.current_gear, Gear::Neutral);

    trans.update(&mut state, 3000.0, 8000.0, 0.5, 0.2);
    trans.shift_down(&mut state);
    assert_eq!(state.current_gear, Gear::Reverse);
}

#[test]
fn test_automatic_shifting_upshift_and_downshift() {
    let config = TransmissionConfig {
        forward_ratios: vec![3.6, 2.2, 1.5, 1.0, 0.8, 0.65],
        shift_cooldown: 0.2,
        shift_duration: 0.05,
        upshift_rpm: 6500.0,
        downshift_rpm: 2500.0,
        is_automatic: true,
        ..Default::default()
    };
    let trans = Transmission::new(config);
    let mut state = TransmissionState::new(); // Starts in 1st gear
    state.cooldown_timer = 1.0;

    // High RPM with throttle => should upshift to 2nd gear
    trans.update(&mut state, 6800.0, 8000.0, 0.8, 0.02);
    assert_eq!(state.current_gear, Gear::Forward(2));
    assert!(state.is_shifting);

    // Finish shift and pass cooldown
    trans.update(&mut state, 4500.0, 8000.0, 0.8, 0.06);
    assert!(!state.is_shifting);
    trans.update(&mut state, 4500.0, 8000.0, 0.8, 0.3); // Cooldown expired

    // Low RPM => should downshift back to 1st gear
    trans.update(&mut state, 2000.0, 8000.0, 0.8, 0.02);
    assert_eq!(state.current_gear, Gear::Forward(1));
}

#[test]
fn test_rev_limiter_safety_upshift() {
    let config = TransmissionConfig {
        forward_ratios: vec![3.5, 2.0, 1.4, 1.0, 0.8, 0.65],
        shift_cooldown: 0.1,
        is_automatic: true,
        ..Default::default()
    };
    let trans = Transmission::new(config);
    let mut state = TransmissionState::new(); // 1st gear
    state.cooldown_timer = 1.0;

    // RPM exceeds 92% of 8000 RPM (7360) even with zero throttle
    trans.update(&mut state, 7600.0, 8000.0, 0.0, 0.02);
    assert_eq!(state.current_gear, Gear::Forward(2));
}
