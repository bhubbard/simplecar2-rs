//! Steering dynamics with true Ackermann geometry and speed-sensitive steering reduction.
//!
//! # Mechanics & Equations
//! - **Ackermann Steering Geometry**:
//!   In a turn, inner and outer steered wheels must follow concentric circles to eliminate tire scrub:
//!   $$R = \frac{L}{\tan(\delta)}$$
//!   where $L$ is the wheelbase, $W$ is the track width, and $\delta$ is the nominal central steering angle.
//!   The inner and outer steering angles are:
//!   $$\delta_{\text{inner}} = \arctan\left(\frac{L}{R - \frac{W}{2}}\right) = \arctan\left(\frac{2L \tan(\delta)}{2L - W \tan(\delta)}\right)$$
//!   $$\delta_{\text{outer}} = \arctan\left(\frac{L}{R + \frac{W}{2}}\right) = \arctan\left(\frac{2L \tan(\delta)}{2L + W \tan(\delta)}\right)$$
//!
//! - **Speed-Sensitive Steering Reduction**:
//!   To maintain vehicle stability and prevent rollovers at high speeds, the maximum steering authority
//!   is attenuated by vehicle forward velocity $v$ (m/s):
//!   $$\delta_{\text{effective}} = \frac{\delta_{\text{max}} \cdot \text{input}}{1.0 + v \cdot k_{\text{speed}}}$$
//!   where $k_{\text{speed}}$ is the velocity damping factor.

use serde::{Deserialize, Serialize};

/// Output steering angles for left and right front wheels.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SteerAngles {
    /// Left wheel steer angle in radians (positive = left turn).
    pub left_rad: f32,
    /// Right wheel steer angle in radians (positive = left turn).
    pub right_rad: f32,
    /// Effective nominal steering angle in radians.
    pub center_rad: f32,
}

/// Configuration settings for the steering system.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SteeringConfig {
    /// Vehicle wheelbase $L$ (distance between front and rear axle centers) in meters.
    pub wheelbase: f32,
    /// Front track width $W$ (distance between left and right front wheel centers) in meters.
    pub track_width: f32,
    /// Maximum steering lock angle at zero speed in radians (e.g. 35° = ~0.61 rad).
    pub max_steer_angle_rad: f32,
    /// Speed attenuation coefficient $k_{\text{speed}}$ reducing steering lock as velocity increases.
    pub speed_sensitivity_factor: f32,
    /// Steering response smoothing rate in rad/s.
    pub steer_speed: f32,
}

impl Default for SteeringConfig {
    fn default() -> Self {
        Self {
            wheelbase: 2.65,
            track_width: 1.62,
            max_steer_angle_rad: 35.0_f32.to_radians(),
            speed_sensitivity_factor: 0.03, // Consistent with SimpleCar2 carSpeedFactor
            steer_speed: 12.0,              // Smooth responsive rack speed
        }
    }
}

/// Dynamic runtime state of the steering system.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SteeringState {
    /// Current smoothed steering input in [-1.0, 1.0].
    pub smoothed_input: f32,
    /// Current steering angles applied to wheels.
    pub current_angles: SteerAngles,
}

impl SteeringState {
    /// Creates a newly centered steering state.
    pub fn new() -> Self {
        Self {
            smoothed_input: 0.0,
            current_angles: SteerAngles {
                left_rad: 0.0,
                right_rad: 0.0,
                center_rad: 0.0,
            },
        }
    }
}

impl Default for SteeringState {
    fn default() -> Self {
        Self::new()
    }
}

/// Ackermann steering controller with speed sensitivity.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Steering {
    /// Configuration constants.
    pub config: SteeringConfig,
}

impl Steering {
    /// Creates a new steering controller.
    pub fn new(config: SteeringConfig) -> Self {
        Self { config }
    }

    /// Computes the speed-reduced nominal steering angle for a given raw input and speed.
    ///
    /// # Arguments
    /// - `raw_input`: Steering input in [-1.0, 1.0] (positive = left, negative = right).
    /// - `speed_m_s`: Forward speed magnitude of the vehicle in m/s.
    pub fn speed_attenuated_angle(&self, raw_input: f32, speed_m_s: f32) -> f32 {
        let clamped_input = raw_input.clamp(-1.0, 1.0);
        let speed_factor = (1.0 + speed_m_s * self.config.speed_sensitivity_factor).max(1.0);
        (clamped_input * self.config.max_steer_angle_rad) / speed_factor
    }

    /// Calculates exact Ackermann angles for left and right wheels from a nominal center angle.
    ///
    /// # Arguments
    /// - `delta_center`: Nominal center steering angle in radians (positive = left, negative = right).
    pub fn calculate_ackermann(&self, delta_center: f32) -> SteerAngles {
        let abs_delta = delta_center.abs();
        if abs_delta < 1e-5 {
            return SteerAngles {
                left_rad: 0.0,
                right_rad: 0.0,
                center_rad: 0.0,
            };
        }

        let l = self.config.wheelbase;
        let w = self.config.track_width;
        let tan_delta = abs_delta.tan();

        // Singularity safeguard
        let denom_inner = (2.0 * l - w * tan_delta).max(0.01);
        let denom_outer = 2.0 * l + w * tan_delta;

        let inner_angle = (2.0 * l * tan_delta / denom_inner).atan();
        let outer_angle = (2.0 * l * tan_delta / denom_outer).atan();

        if delta_center > 0.0 {
            // Turning Left: Left is inner, Right is outer
            SteerAngles {
                left_rad: inner_angle,
                right_rad: outer_angle,
                center_rad: delta_center,
            }
        } else {
            // Turning Right: Right is inner, Left is outer
            SteerAngles {
                left_rad: -outer_angle,
                right_rad: -inner_angle,
                center_rad: delta_center,
            }
        }
    }

    /// Updates steering state over a physics time step $dt$.
    ///
    /// # Arguments
    /// - `state`: Mutable reference to steering state.
    /// - `raw_input`: Driver steering input in [-1.0, 1.0].
    /// - `speed_m_s`: Current vehicle speed in m/s.
    /// - `dt`: Physics fixed time step in seconds.
    pub fn update(&self, state: &mut SteeringState, raw_input: f32, speed_m_s: f32, dt: f32) -> SteerAngles {
        let target_input = raw_input.clamp(-1.0, 1.0);

        // Smooth steering input over time
        if dt > 1e-6 {
            let blend = (self.config.steer_speed * dt).clamp(0.0, 1.0);
            state.smoothed_input += (target_input - state.smoothed_input) * blend;
        } else {
            state.smoothed_input = target_input;
        }

        let target_angle = self.speed_attenuated_angle(state.smoothed_input, speed_m_s);
        let angles = self.calculate_ackermann(target_angle);
        state.current_angles = angles;
        angles
    }
}
