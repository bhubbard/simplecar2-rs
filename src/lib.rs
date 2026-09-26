//! # SimpleCar2-rs
//!
//! Pure Rust arcade-sim vehicle dynamics, custom raycast suspension, drivetrain,
//! transmission, differential, and Ackermann steering translated from SimonVutov/SimpleCar2.
//!
//! Designed for high-performance physics simulation in open-world Bevy RPGs and Rust games.
//!
//! ## Subsystems
//! - **Raycast Suspension**: Spring force $F = k \cdot x$, damper force $F = -c \cdot v$,
//!   and anti-roll sway bars connecting left/right wheels to eliminate body roll.
//! - **Drivetrain & Engine**: Power/torque curves (idle, peak, redline), clutch torque transmission,
//!   rev-limiter, and flywheel rotational dynamics.
//! - **Transmission**: Multi-gear ratios (Reverse, Neutral, 1..6+ Forward), final drive,
//!   and automatic transmission shift logic with cooldown and RPM thresholds.
//! - **Differential**: Open, Locked (Spool), and Limited-Slip Differential (LSD) with clutch preload
//!   and torque bias ratios.
//! - **Steering Dynamics**: True Ackermann steering geometry and speed-sensitive steering angle attenuation.
//! - **Vehicle Integration**: Complete 4-wheel chassis solver resolving 3D forces and torques.

pub mod differential;
pub mod drivetrain;
pub mod error;
pub mod steering;
pub mod suspension;
pub mod transmission;
pub mod vehicle;

pub use differential::{Differential, DifferentialOutput, DifferentialType};
pub use drivetrain::{Clutch, Engine, EngineConfig, EngineState};
pub use error::VehicleError;
pub use steering::{SteerAngles, Steering, SteeringConfig, SteeringState};
pub use suspension::{AntiRollBar, Suspension, SuspensionConfig, SuspensionForce, SuspensionState};
pub use transmission::{Gear, Transmission, TransmissionConfig, TransmissionState};
pub use vehicle::{
    AeroConfig, DriveLayout, Vehicle, VehicleConfig, VehicleForces, VehicleInputs, WheelConfig,
    WheelRaycastHit, WheelState,
};

/// Common prelude types for simplecar2.
pub mod prelude {
    pub use crate::differential::{Differential, DifferentialType};
    pub use crate::drivetrain::{Clutch, Engine, EngineConfig, EngineState};
    pub use crate::error::VehicleError;
    pub use crate::steering::{SteerAngles, Steering, SteeringConfig, SteeringState};
    pub use crate::suspension::{AntiRollBar, Suspension, SuspensionConfig, SuspensionState};
    pub use crate::transmission::{Gear, Transmission, TransmissionConfig, TransmissionState};
    pub use crate::vehicle::{
        DriveLayout, Vehicle, VehicleConfig, VehicleForces, VehicleInputs, WheelConfig,
        WheelRaycastHit, WheelState,
    };
}
