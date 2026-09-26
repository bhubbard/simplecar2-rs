use glam::{Quat, Vec3};
use simplecar2::vehicle::{Vehicle, VehicleConfig, VehicleInputs, WheelRaycastHit};

#[test]
fn test_vehicle_initialization_and_grounded_step() {
    let config = VehicleConfig::default();
    let mut vehicle = Vehicle::new(config);

    let chassis_pos = Vec3::new(0.0, 0.45, 0.0);
    let chassis_rot = Quat::IDENTITY;
    let chassis_vel = Vec3::ZERO;
    let chassis_ang_vel = Vec3::ZERO;

    // Simulate flat ground at y = 0
    let hits = [
        Some(WheelRaycastHit {
            distance: 0.40,
            point: Vec3::new(-0.81, 0.0, 1.35),
            normal: Vec3::Y,
        }),
        Some(WheelRaycastHit {
            distance: 0.40,
            point: Vec3::new(0.81, 0.0, 1.35),
            normal: Vec3::Y,
        }),
        Some(WheelRaycastHit {
            distance: 0.40,
            point: Vec3::new(-0.81, 0.0, -1.30),
            normal: Vec3::Y,
        }),
        Some(WheelRaycastHit {
            distance: 0.40,
            point: Vec3::new(0.81, 0.0, -1.30),
            normal: Vec3::Y,
        }),
    ];

    let inputs = VehicleInputs {
        steer: 0.0,
        throttle: 0.5,
        brake: 0.0,
        handbrake: false,
        clutch_pedal: 0.0,
    };

    let forces = vehicle
        .step(&inputs, hits, chassis_pos, chassis_rot, chassis_vel, chassis_ang_vel, 0.02)
        .expect("Vehicle step failed");

    // Total vertical force should be positive (suspension supporting vehicle mass)
    assert!(forces.total_force.y > 0.0);
    for sf in &forces.suspension_forces {
        assert!(sf.total_normal_force > 0.0);
    }

    // Engine should produce drive torque on rear wheels (RWD layout)
    assert!(vehicle.wheel_states[2].drive_torque > 0.0);
    assert!(vehicle.wheel_states[3].drive_torque > 0.0);
    assert_eq!(vehicle.wheel_states[0].drive_torque, 0.0);
    assert_eq!(vehicle.wheel_states[1].drive_torque, 0.0);
}

#[test]
fn test_vehicle_acceleration_and_braking() {
    let config = VehicleConfig::default();
    let mut vehicle = Vehicle::new(config);

    let chassis_pos = Vec3::new(0.0, 0.45, 0.0);
    let chassis_rot = Quat::IDENTITY;
    let forward_vel = Vec3::new(0.0, 0.0, 10.0); // 10 m/s moving forward
    let chassis_ang_vel = Vec3::ZERO;

    let hits = [
        Some(WheelRaycastHit {
            distance: 0.40,
            point: Vec3::new(-0.81, 0.0, 1.35),
            normal: Vec3::Y,
        }),
        Some(WheelRaycastHit {
            distance: 0.40,
            point: Vec3::new(0.81, 0.0, 1.35),
            normal: Vec3::Y,
        }),
        Some(WheelRaycastHit {
            distance: 0.40,
            point: Vec3::new(-0.81, 0.0, -1.30),
            normal: Vec3::Y,
        }),
        Some(WheelRaycastHit {
            distance: 0.40,
            point: Vec3::new(0.81, 0.0, -1.30),
            normal: Vec3::Y,
        }),
    ];

    let brake_inputs = VehicleInputs {
        steer: 0.0,
        throttle: 0.0,
        brake: 1.0,
        handbrake: false,
        clutch_pedal: 1.0,
    };

    let forces = vehicle
        .step(&brake_inputs, hits, chassis_pos, chassis_rot, forward_vel, chassis_ang_vel, 0.02)
        .expect("Vehicle step failed");

    // Braking force should point backward (opposing forward velocity, Z < 0)
    assert!(forces.total_force.z < 0.0);

    // Brake torque applied to wheels
    for ws in &vehicle.wheel_states {
        assert!(ws.brake_torque > 0.0);
    }
}
