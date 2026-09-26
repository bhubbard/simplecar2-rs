//! Internal combustion and electric drivetrain models, torque/RPM curves, and clutch mechanics.
//!
//! # Mechanics & Equations
//! - **Engine Power & Torque Curve**:
//!   Between idle RPM ($RPM_{\text{idle}}$) and peak power RPM ($RPM_{\text{peak}}$),
//!   power builds quadratically:
//!   $$t = \frac{RPM - RPM_{\text{idle}}}{RPM_{\text{peak}} - RPM_{\text{idle}}}$$
//!   $$P(t) = 0.3 + 0.7 \cdot t^2$$
//!
//!   Beyond peak power up to redline ($RPM_{\text{max}}$), power falls linearly:
//!   $$t = \frac{RPM - RPM_{\text{peak}}}{RPM_{\text{max}} - RPM_{\text{peak}}}$$
//!   $$P(t) = 1.0 - 0.9 \cdot t$$
//!
//!   At or above redline ($RPM \ge RPM_{\text{max}}$), the rev limiter cuts fuel injection:
//!   $$P(RPM) = 0$$
//!
//! - **Clutch Dynamics**:
//!   Clutch capacity is proportional to pedal engagement $\gamma \in [0, 1]$:
//!   $$T_{\text{cap}} = \gamma \cdot T_{\text{clutch\_max}}$$
//!   When engaged, the clutch transmits torque between the engine flywheel and transmission input shaft,
//!   coupling their angular velocities via friction.

use std::f32::consts::PI;
use serde::{Deserialize, Serialize};

/// Configuration parameters for an engine.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct EngineConfig {
    /// Engine idle rotation speed in RPM.
    pub idle_rpm: f32,
    /// RPM at which peak engine torque and power are developed.
    pub peak_power_rpm: f32,
    /// Maximum engine speed (redline) in RPM.
    pub max_rpm: f32,
    /// Maximum baseline engine torque in N·m at peak power.
    pub peak_torque: f32,
    /// Rotational inertia of the flywheel and crankshaft in kg·m².
    pub flywheel_inertia: f32,
    /// Internal friction drag coefficient in N·m·s/rad.
    pub friction_drag: f32,
    /// Whether this motor produces constant torque across all RPMs (e.g. electric powertrain).
    pub constant_torque: bool,
}

impl Default for EngineConfig {
    fn default() -> Self {
        Self {
            idle_rpm: 800.0,
            peak_power_rpm: 6_800.0,
            max_rpm: 8_200.0,
            peak_torque: 450.0,
            flywheel_inertia: 0.22,
            friction_drag: 0.05,
            constant_torque: false,
        }
    }
}

/// Dynamic runtime state of the engine.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct EngineState {
    /// Current rotational speed in RPM.
    pub rpm: f32,
    /// Current throttle position in [0.0, 1.0].
    pub throttle: f32,
    /// Current output torque produced at the flywheel in N·m.
    pub output_torque: f32,
    /// Whether the engine rev limiter is actively interrupting fuel supply.
    pub is_rev_limiting: bool,
    /// Engine temperature or thermal factor (optional 1.0 default).
    pub efficiency: f32,
}

impl EngineState {
    /// Creates a newly initialized engine state at idle RPM.
    pub fn new(idle_rpm: f32) -> Self {
        Self {
            rpm: idle_rpm,
            throttle: 0.0,
            output_torque: 0.0,
            is_rev_limiting: false,
            efficiency: 1.0,
        }
    }

    /// Engine angular velocity in radians per second.
    #[inline]
    pub fn angular_velocity(&self) -> f32 {
        self.rpm * (2.0 * PI / 60.0)
    }
}

/// Friction clutch coupling the engine to the transmission input shaft.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Clutch {
    /// Maximum torque capacity when fully clamped in N·m.
    pub max_torque_capacity: f32,
    /// Engagement position: 0.0 (fully disengaged / pedal pressed) to 1.0 (fully engaged).
    pub engagement: f32,
}

impl Clutch {
    /// Creates a new clutch with specified torque capacity.
    pub fn new(max_torque_capacity: f32) -> Self {
        Self {
            max_torque_capacity,
            engagement: 1.0,
        }
    }

    /// Calculates the transmitted torque through the clutch plate.
    ///
    /// # Arguments
    /// - `engine_ang_vel`: Engine crankshaft angular velocity in rad/s.
    /// - `trans_ang_vel`: Transmission input shaft angular velocity in rad/s.
    ///
    /// # Returns
    /// Transmitted torque in N·m (positive pulls engine down and pushes transmission forward).
    pub fn calculate_torque(&self, engine_ang_vel: f32, trans_ang_vel: f32) -> f32 {
        let slip_velocity = engine_ang_vel - trans_ang_vel;
        let capacity = self.max_torque_capacity * self.engagement.clamp(0.0, 1.0);

        // Viscous-friction coupling approaching Coulomb limit smoothly
        let torque = capacity * (slip_velocity * 0.1).tanh();
        torque.clamp(-capacity, capacity)
    }
}

impl Default for Clutch {
    fn default() -> Self {
        Self {
            max_torque_capacity: 900.0, // N·m (well above engine peak torque)
            engagement: 1.0,
        }
    }
}

/// Engine simulation model.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Engine {
    /// Configuration constants.
    pub config: EngineConfig,
}

impl Engine {
    /// Creates a new engine instance.
    pub fn new(config: EngineConfig) -> Self {
        Self { config }
    }

    /// Evaluates the normalized power factor in [0.0, 1.0] as a function of RPM.
    ///
    /// Follows SimpleCar2's realistic torque / power curve with idle pickup,
    /// quadratic peak build, and post-peak linear decline before redline cut.
    pub fn normalized_power(&self, rpm: f32) -> f32 {
        if self.config.constant_torque {
            return 1.0;
        }

        if rpm >= self.config.max_rpm {
            return 0.0; // Rev limiter cut
        }

        if rpm < self.config.idle_rpm {
            return 0.0;
        }

        if rpm < self.config.peak_power_rpm {
            // Power ramps up quadratically from idle (starting at 30% baseline torque)
            let t = (rpm - self.config.idle_rpm) / (self.config.peak_power_rpm - self.config.idle_rpm);
            0.3 + 0.7 * (t * t)
        } else {
            // Power gracefully tapers off past peak towards redline
            let t = (rpm - self.config.peak_power_rpm) / (self.config.max_rpm - self.config.peak_power_rpm);
            1.0 - 0.9 * t
        }
    }

    /// Computes the gross torque generated by combustion at the current state.
    pub fn calculate_gross_torque(&self, state: &EngineState) -> f32 {
        let norm_power = self.normalized_power(state.rpm);
        let throttle = state.throttle.clamp(0.0, 1.0);
        self.config.peak_torque * norm_power * throttle * state.efficiency
    }

    /// Updates the engine state over a time step $dt$.
    ///
    /// # Arguments
    /// - `state`: Mutable reference to the engine state.
    /// - `clutch_load_torque`: Opposing torque exerted by the clutch on the flywheel in N·m.
    /// - `dt`: Delta time in seconds.
    pub fn update(&self, state: &mut EngineState, clutch_load_torque: f32, dt: f32) {
        if dt <= 1e-6 {
            return;
        }

        let gross_torque = self.calculate_gross_torque(state);
        state.output_torque = gross_torque;

        let omega = state.angular_velocity();
        let friction_torque = self.config.friction_drag * omega;

        // Net torque on the engine crankshaft
        let net_torque = gross_torque - clutch_load_torque - friction_torque;

        // Rotational acceleration: alpha = T_net / I
        let angular_accel = net_torque / self.config.flywheel_inertia.max(1e-4);
        let new_omega = (omega + angular_accel * dt).max(0.0);

        // Convert back to RPM
        let new_rpm = new_omega * (60.0 / (2.0 * PI));

        // Idle governor prevents stall when clutch is disengaged or slipping
        state.rpm = new_rpm.max(self.config.idle_rpm);
        state.is_rev_limiting = state.rpm >= self.config.max_rpm;
    }

    /// Directly syncs engine RPM to driveline wheel feedback when fully locked.
    pub fn sync_with_driveline_rpm(&self, state: &mut EngineState, driveline_rpm: f32) {
        state.rpm = driveline_rpm.clamp(self.config.idle_rpm, self.config.max_rpm);
        state.is_rev_limiting = state.rpm >= self.config.max_rpm;
    }
}
