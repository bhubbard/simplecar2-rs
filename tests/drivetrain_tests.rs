use simplecar2::drivetrain::{Clutch, Engine, EngineConfig, EngineState};

#[test]
fn test_engine_power_curve() {
    let config = EngineConfig {
        idle_rpm: 1000.0,
        peak_power_rpm: 6000.0,
        max_rpm: 8000.0,
        peak_torque: 500.0,
        flywheel_inertia: 0.2,
        friction_drag: 0.05,
        constant_torque: false,
    };
    let engine = Engine::new(config);

    // Below idle: 0.0
    assert_eq!(engine.normalized_power(800.0), 0.0);

    // At idle: 30% baseline torque
    assert!((engine.normalized_power(1000.0) - 0.3).abs() < 1e-4);

    // At peak: 100% torque
    assert!((engine.normalized_power(6000.0) - 1.0).abs() < 1e-4);

    // Mid-way between idle and peak (3500 RPM, t = 0.5)
    // P = 0.3 + 0.7 * (0.5)^2 = 0.3 + 0.175 = 0.475
    assert!((engine.normalized_power(3500.0) - 0.475).abs() < 1e-4);

    // At redline: 0.0 (rev limiter cutoff)
    assert_eq!(engine.normalized_power(8000.0), 0.0);
    assert_eq!(engine.normalized_power(8200.0), 0.0);
}

#[test]
fn test_constant_torque_mode() {
    let config = EngineConfig {
        constant_torque: true,
        ..Default::default()
    };
    let engine = Engine::new(config);

    assert_eq!(engine.normalized_power(500.0), 1.0);
    assert_eq!(engine.normalized_power(4000.0), 1.0);
    assert_eq!(engine.normalized_power(7000.0), 1.0);
}

#[test]
fn test_clutch_torque_transfer() {
    let clutch = Clutch::new(500.0);

    // Equal speeds => zero slip torque
    assert_eq!(clutch.calculate_torque(100.0, 100.0), 0.0);

    // Engine faster than transmission => positive torque driving transmission
    let torque = clutch.calculate_torque(200.0, 100.0);
    assert!(torque > 0.0);
    assert!(torque <= 500.0);

    // Disengaged clutch => zero torque
    let mut disengaged = clutch;
    disengaged.engagement = 0.0;
    assert_eq!(disengaged.calculate_torque(200.0, 100.0), 0.0);
}

#[test]
fn test_engine_state_update() {
    let config = EngineConfig::default();
    let engine = Engine::new(config);
    let mut state = EngineState::new(config.idle_rpm);

    state.throttle = 1.0;
    engine.update(&mut state, 0.0, 0.05);

    // RPM should accelerate upwards under full throttle with zero clutch load
    assert!(state.rpm > config.idle_rpm);
}
