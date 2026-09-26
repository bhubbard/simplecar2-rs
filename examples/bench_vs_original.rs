use std::time::Instant;
use glam::{Quat, Vec3};
use simplecar2::prelude::*;

fn main() {
    println!("============================================================");
    println!("  simplecar2-rs (Rust) vs Godot SimpleCar2 (GDScript) Bench ");
    println!("============================================================");

    // 1. Single-Vehicle Step Latency & 60Hz/120Hz Headroom
    println!("\n--- 1. Full Vehicle Physics Step Latency (4 Wheels, Drivetrain, LSD, Aero) ---");
    {
        let config = VehicleConfig::default();
        let mut vehicle = Vehicle::new(config);

        let chassis_pos = Vec3::new(0.0, 0.45, 0.0);
        let chassis_rot = Quat::IDENTITY;
        let chassis_vel = Vec3::new(0.0, 0.0, 15.0); // 54 km/h cruising
        let chassis_ang_vel = Vec3::ZERO;

        let hits = [
            Some(WheelRaycastHit {
                distance: 0.38,
                point: Vec3::new(-0.81, 0.0, 1.35),
                normal: Vec3::Y,
            }),
            Some(WheelRaycastHit {
                distance: 0.39,
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
            steer: 0.15,
            throttle: 0.8,
            brake: 0.0,
            handbrake: false,
            clutch_pedal: 0.0,
        };

        // Warm up
        for _ in 0..10_000 {
            let _ = vehicle.step(&inputs, hits, chassis_pos, chassis_rot, chassis_vel, chassis_ang_vel, 0.0166);
        }

        let iterations = 1_000_000;
        let start = Instant::now();
        for _ in 0..iterations {
            let _ = vehicle.step(&inputs, hits, chassis_pos, chassis_rot, chassis_vel, chassis_ang_vel, 0.0166);
        }
        let elapsed = start.elapsed();
        let ns_per_step = elapsed.as_nanos() as f64 / iterations as f64;
        let steps_per_sec = iterations as f64 / elapsed.as_secs_f64();

        println!(
            "Iterations: {} | Total Time: {:.2?} | Latency: {:.2} ns/vehicle | {:>10.0} vehicle-ticks/s",
            iterations, elapsed, ns_per_step, steps_per_sec
        );
    }

    // 2. Multi-Vehicle Fleet Simulation (100, 500, 1,000 Concurrent Cars)
    println!("\n--- 2. Multi-Vehicle Fleet Scaling (Concurrent Vehicles at 60 FPS) ---");
    for &fleet_size in &[100, 500, 1000] {
        let config = VehicleConfig::default();
        let mut fleet: Vec<Vehicle> = (0..fleet_size).map(|_| Vehicle::new(config.clone())).collect();

        let chassis_pos = Vec3::new(0.0, 0.45, 0.0);
        let chassis_rot = Quat::IDENTITY;
        let chassis_vel = Vec3::new(0.0, 0.0, 12.0);
        let chassis_ang_vel = Vec3::ZERO;

        let hits = [
            Some(WheelRaycastHit { distance: 0.38, point: Vec3::new(-0.81, 0.0, 1.35), normal: Vec3::Y }),
            Some(WheelRaycastHit { distance: 0.38, point: Vec3::new(0.81, 0.0, 1.35), normal: Vec3::Y }),
            Some(WheelRaycastHit { distance: 0.40, point: Vec3::new(-0.81, 0.0, -1.30), normal: Vec3::Y }),
            Some(WheelRaycastHit { distance: 0.40, point: Vec3::new(0.81, 0.0, -1.30), normal: Vec3::Y }),
        ];

        let inputs = VehicleInputs {
            steer: 0.05,
            throttle: 0.75,
            brake: 0.0,
            handbrake: false,
            clutch_pedal: 0.0,
        };

        let frames = 1000;
        let start = Instant::now();
        for _ in 0..frames {
            for v in &mut fleet {
                let _ = v.step(&inputs, hits, chassis_pos, chassis_rot, chassis_vel, chassis_ang_vel, 0.0166);
            }
        }
        let elapsed = start.elapsed();
        let frame_latency = elapsed / frames as u32;
        let fps_capacity = frames as f64 / elapsed.as_secs_f64();
        let percent_60fps = (frame_latency.as_secs_f64() / 0.016666) * 100.0;

        println!(
            "Fleet Size: {:>4} cars | Fleet Frame: {:>8.2?} | {:>8.0} FPS max | {:>5.2}% of 16.6ms frame budget",
            fleet_size, frame_latency, fps_capacity, percent_60fps
        );
    }

    // 3. Isolated Limited-Slip Differential & Sway Bar Solvers
    println!("\n--- 3. Subsystem Micro-Benchmarks (Differential & Suspension) ---");
    {
        let diff = Differential::limited_slip(60.0, 15.0, 2.5);

        let iterations = 5_000_000;
        let start = Instant::now();
        let mut sum = 0.0;
        for i in 0..iterations {
            let left_vel = 40.0 + (i % 10) as f32 * 0.1;
            let right_vel = 42.0;
            let out = diff.distribute_torque(250.0, left_vel, right_vel, 4000.0, 4200.0);
            sum += out.torque_left + out.torque_right;
        }
        let elapsed = start.elapsed();
        let ns_per_solve = elapsed.as_nanos() as f64 / iterations as f64;
        let solves_per_sec = iterations as f64 / elapsed.as_secs_f64();

        println!(
            "LSD Differential Solves: {} | Time: {:.2?} | {:.2} ns/solve ({:>10.0} solves/s) | Sum: {:.1}",
            iterations, elapsed, ns_per_solve, solves_per_sec, sum
        );
    }

    println!("\n============================================================");
    println!("                      Benchmark Complete                    ");
    println!("============================================================");
}
