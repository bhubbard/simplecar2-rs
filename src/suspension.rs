//! Raycast suspension system with spring-damper dynamics and anti-roll sway bars.
//!
//! # Mechanics & Equations
//! - **Spring Force**:
//!   $$F_{\text{spring}} = k \cdot x$$
//!   where $k$ is the spring stiffness (N/m) and $x = L_{\text{rest}} - L_{\text{current}}$
//!   is the suspension compression distance.
//!
//! - **Damping Force**:
//!   $$F_{\text{damping}} = -c \cdot v$$
//!   where $c$ is the damping coefficient (N·s/m) and $v = \frac{dx}{dt}$ is the
//!   compression velocity. Positive velocity indicates compression, which damping opposes.
//!
//! - **Anti-Roll Sway Bar**:
//!   Connects left and right wheel suspensions on an axle to resist chassis roll during cornering:
//!   $$\Delta x = x_{\text{left}} - x_{\text{right}}$$
//!   $$F_{\text{arb}} = k_{\text{arb}} \cdot \Delta x$$
//!   The anti-roll force reduces normal force on the outer (more compressed) wheel
//!   and increases it on the inner (less compressed) wheel, stabilizing the vehicle chassis.

use glam::Vec3;
use serde::{Deserialize, Serialize};

/// Configuration parameters for a single wheel's raycast suspension.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SuspensionConfig {
    /// Natural uncompressed length of the suspension spring in meters.
    pub rest_length: f32,
    /// Spring stiffness constant $k$ in Newtons per meter (N/m).
    pub spring_stiffness: f32,
    /// Damper rate $c$ during compression (bump) in N·s/m.
    pub compression_damping: f32,
    /// Damper rate during expansion (rebound) in N·s/m.
    pub rebound_damping: f32,
    /// Maximum allowable suspension compression travel in meters.
    pub max_travel: f32,
    /// Upper clamp limit for total suspension normal force in Newtons.
    pub force_clamp: f32,
}

impl Default for SuspensionConfig {
    fn default() -> Self {
        Self {
            rest_length: 0.45,
            spring_stiffness: 28_000.0,
            compression_damping: 2_500.0,
            rebound_damping: 3_200.0,
            max_travel: 0.35,
            force_clamp: 35_000.0,
        }
    }
}

/// Dynamic runtime state of a suspension strut.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SuspensionState {
    /// Current measured length of the suspension in meters.
    pub current_length: f32,
    /// Measured length from previous physics frame in meters.
    pub previous_length: f32,
    /// Current suspension compression distance $x = L_{\text{rest}} - L_{\text{current}}$ in meters.
    pub compression: f32,
    /// Compression velocity $v = \frac{dx}{dt}$ in m/s. Positive during compression.
    pub compression_velocity: f32,
    /// Whether the wheel is currently contacting a surface.
    pub is_grounded: bool,
    /// Surface contact point in world coordinates, if grounded.
    pub contact_point: Option<Vec3>,
    /// Contact surface normal in world coordinates, if grounded.
    pub contact_normal: Option<Vec3>,
    /// Last calculated normal force in Newtons.
    pub normal_force: f32,
}

impl SuspensionState {
    /// Creates a new default suspension state initialized at rest length.
    pub fn new(rest_length: f32) -> Self {
        Self {
            current_length: rest_length,
            previous_length: rest_length,
            compression: 0.0,
            compression_velocity: 0.0,
            is_grounded: false,
            contact_point: None,
            contact_normal: None,
            normal_force: 0.0,
        }
    }
}

/// Resolved force and vector for suspension action on the chassis.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SuspensionForce {
    /// Elastic spring force component ($F = k \cdot x$) in Newtons.
    pub spring_force: f32,
    /// Viscous damping force component ($F = -c \cdot v$) in Newtons.
    pub damping_force: f32,
    /// Anti-roll sway bar force adjustment in Newtons.
    pub anti_roll_force: f32,
    /// Total net normal force scalar in Newtons.
    pub total_normal_force: f32,
    /// 3D force vector applied to the chassis at the suspension mount point.
    pub force_vector: Vec3,
    /// World position where the force should be applied on the chassis.
    pub application_point: Vec3,
}

/// Anti-roll sway bar connecting left and right wheels on a common axle.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct AntiRollBar {
    /// Torsional stiffness constant $k_{\text{arb}}$ in N/m.
    pub stiffness: f32,
}

impl AntiRollBar {
    /// Creates a new anti-roll bar with specified stiffness.
    pub fn new(stiffness: f32) -> Self {
        Self { stiffness }
    }

    /// Computes the anti-roll force adjustments for left and right wheels.
    ///
    /// Returns `(left_force_adjustment, right_force_adjustment)` in Newtons.
    /// When the left wheel is more compressed than the right wheel ($\Delta x > 0$),
    /// a downward push is applied to the right wheel and an upward resistance
    /// to the left wheel to counteract body roll.
    pub fn calculate_forces(
        &self,
        left_state: &SuspensionState,
        right_state: &SuspensionState,
    ) -> (f32, f32) {
        if !left_state.is_grounded && !right_state.is_grounded {
            return (0.0, 0.0);
        }

        let left_comp = if left_state.is_grounded {
            left_state.compression
        } else {
            0.0
        };
        let right_comp = if right_state.is_grounded {
            right_state.compression
        } else {
            0.0
        };

        let delta_compression = left_comp - right_comp;
        let arb_force = delta_compression * self.stiffness;

        // Left receives -arb_force (opposing excess compression)
        // Right receives +arb_force (adding load to uncompressed side)
        (-arb_force, arb_force)
    }
}

impl Default for AntiRollBar {
    fn default() -> Self {
        Self {
            stiffness: 7_500.0, // N/m typical for road performance car
        }
    }
}

/// Suspension evaluator implementing raycast spring-damper mechanics.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Suspension {
    /// Strut configuration settings.
    pub config: SuspensionConfig,
}

impl Suspension {
    /// Creates a new suspension with given configuration.
    pub fn new(config: SuspensionConfig) -> Self {
        Self { config }
    }

    /// Evaluates the suspension response for one physics time step given raycast hit results.
    ///
    /// # Arguments
    /// - `state`: Mutable reference to the runtime suspension state.
    /// - `ray_hit_distance`: Measured distance along the suspension raycast from mount point to surface.
    /// - `contact_point`: Surface hit point in world space.
    /// - `contact_normal`: Surface normal in world space.
    /// - `anti_roll_force`: Additional force from anti-roll sway bar in Newtons.
    /// - `mount_position`: World position of the suspension top mount on the chassis.
    /// - `suspension_direction`: Normalized vector pointing down along the suspension strut axis in world space.
    /// - `dt`: Physics fixed time step in seconds.
    pub fn evaluate(
        &self,
        state: &mut SuspensionState,
        ray_hit_distance: Option<f32>,
        contact_point: Option<Vec3>,
        contact_normal: Option<Vec3>,
        anti_roll_force: f32,
        mount_position: Vec3,
        suspension_direction: Vec3,
        dt: f32,
    ) -> SuspensionForce {
        let max_distance = self.config.rest_length + self.config.max_travel;

        match ray_hit_distance {
            Some(dist) if dist > 0.0 && dist < max_distance => {
                let clamped_dist = dist.clamp(self.config.rest_length - self.config.max_travel, max_distance);
                state.is_grounded = true;
                state.contact_point = contact_point;
                state.contact_normal = contact_normal;
                state.current_length = clamped_dist;

                // Compression x = rest_length - current_length
                state.compression = (self.config.rest_length - clamped_dist).max(0.0);

                // Compression velocity: rate of change of compression
                // Positive when compressing (current_length < previous_length)
                if dt > 1e-6 {
                    let d_length = state.previous_length - state.current_length;
                    state.compression_velocity = d_length / dt;
                } else {
                    state.compression_velocity = 0.0;
                }
                state.previous_length = state.current_length;

                // Hooke's Law: F_spring = k * x
                let spring_force = self.config.spring_stiffness * state.compression;

                // Damping force: opposes compression velocity F = -c * v
                // In suspension coordinate frame, positive damping adds resistance when compressing
                let damping_coeff = if state.compression_velocity >= 0.0 {
                    self.config.compression_damping
                } else {
                    self.config.rebound_damping
                };
                let damping_force = damping_coeff * state.compression_velocity;

                // Net normal force combining spring, damping, and sway bar
                let raw_normal = spring_force + damping_force + anti_roll_force;
                let normal_force = raw_normal.clamp(0.0, self.config.force_clamp);
                state.normal_force = normal_force;

                // Force acts opposite to suspension direction (upwards against chassis)
                let push_direction = -suspension_direction.normalize_or_zero();
                let force_vector = push_direction * normal_force;

                SuspensionForce {
                    spring_force,
                    damping_force,
                    anti_roll_force,
                    total_normal_force: normal_force,
                    force_vector,
                    application_point: mount_position,
                }
            }
            _ => {
                // Wheel is airborne / extended past travel limit
                state.is_grounded = false;
                state.contact_point = None;
                state.contact_normal = None;
                state.current_length = self.config.rest_length;
                state.previous_length = self.config.rest_length;
                state.compression = 0.0;
                state.compression_velocity = 0.0;
                state.normal_force = 0.0;

                SuspensionForce {
                    spring_force: 0.0,
                    damping_force: 0.0,
                    anti_roll_force: 0.0,
                    total_normal_force: 0.0,
                    force_vector: Vec3::ZERO,
                    application_point: mount_position,
                }
            }
        }
    }
}
