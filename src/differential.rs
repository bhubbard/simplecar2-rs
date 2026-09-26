//! Differential torque distribution models: Open, Locked (Spool), and Limited-Slip (LSD).
//!
//! # Mechanics & Equations
//! - **Driveline Kinematic Relationship**:
//!   $$\omega_{\text{diff}} = \frac{\omega_{\text{left}} + \omega_{\text{right}}}{2}$$
//!
//! - **Open Differential**:
//!   Distributes input driveshaft torque equally between left and right driving wheels:
//!   $$T_{\text{left}} = \frac{1}{2} T_{\text{in}}, \quad T_{\text{right}} = \frac{1}{2} T_{\text{in}}$$
//!
//! - **Locked (Spool) Differential**:
//!   Enforces identical wheel speeds ($\omega_{\text{left}} = \omega_{\text{right}}$).
//!   Torque distributes dynamically to each wheel in proportion to available normal force / grip:
//!   $$T_{\text{left}} = T_{\text{in}} \cdot \frac{F_{z,\text{left}}}{F_{z,\text{left}} + F_{z,\text{right}}}$$
//!
//! - **Limited-Slip Differential (LSD)**:
//!   Employs clutch pack preload and speed differential damping to transfer torque away
//!   from the slipping wheel to the gripping wheel:
//!   $$\Delta \omega = \omega_{\text{left}} - \omega_{\text{right}}$$
//!   $$T_{\text{lock}} = \operatorname{clamp}\left(k_{\text{lsd}} \cdot \Delta \omega + \operatorname{sgn}(\Delta \omega) \cdot T_{\text{preload}}, -T_{\text{max}}, T_{\text{max}}\right)$$
//!   $$T_{\text{left}} = \frac{1}{2} T_{\text{in}} - T_{\text{lock}}, \quad T_{\text{right}} = \frac{1}{2} T_{\text{in}} + T_{\text{lock}}$$
//!   Enforces maximum torque bias ratio $B = \frac{T_{\text{high}}}{T_{\text{low}}}$.

use serde::{Deserialize, Serialize};

/// Type of differential mechanism.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum DifferentialType {
    /// Open differential: perfectly even 50/50 torque split.
    Open,
    /// Fully locked (spool) differential: locks axle rotation, divides torque by available traction.
    Locked,
    /// Limited-slip differential (clutch pack / viscous LSD) with preload and torque bias.
    LimitedSlip {
        /// Static clutch preload torque in N·m.
        preload: f32,
        /// Viscous / speed-differential stiffness constant in N·m·s/rad.
        stiffness: f32,
        /// Maximum torque bias ratio $T_{\text{high}} / T_{\text{low}}$ (e.g. 3.0).
        max_bias_ratio: f32,
    },
}

impl Default for DifferentialType {
    fn default() -> Self {
        DifferentialType::LimitedSlip {
            preload: 40.0,
            stiffness: 85.0,
            max_bias_ratio: 3.5,
        }
    }
}

/// Resolved torque split for left and right drive wheels.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct DifferentialOutput {
    /// Torque delivered to the left wheel in N·m.
    pub torque_left: f32,
    /// Torque delivered to the right wheel in N·m.
    pub torque_right: f32,
    /// Average rotational speed of the differential carrier in rad/s.
    pub diff_angular_velocity: f32,
    /// Internal locking torque transferred across the differential in N·m.
    pub lock_torque: f32,
}

/// Differential unit connecting driveshaft to left and right drive wheels.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Differential {
    /// Type and configuration of this differential.
    pub diff_type: DifferentialType,
    /// Final drive gear efficiency (e.g. 0.96).
    pub mechanical_efficiency: f32,
}

impl Differential {
    /// Creates an open differential.
    pub fn open() -> Self {
        Self {
            diff_type: DifferentialType::Open,
            mechanical_efficiency: 0.96,
        }
    }

    /// Creates a locked spool differential.
    pub fn locked() -> Self {
        Self {
            diff_type: DifferentialType::Locked,
            mechanical_efficiency: 0.98,
        }
    }

    /// Creates a limited slip differential.
    pub fn limited_slip(preload: f32, stiffness: f32, max_bias_ratio: f32) -> Self {
        Self {
            diff_type: DifferentialType::LimitedSlip {
                preload,
                stiffness,
                max_bias_ratio: max_bias_ratio.max(1.0),
            },
            mechanical_efficiency: 0.96,
        }
    }

    /// Computes the torque distribution between left and right wheels.
    ///
    /// # Arguments
    /// - `input_torque`: Net driveshaft input torque in N·m.
    /// - `omega_left`: Left wheel angular velocity in rad/s.
    /// - `omega_right`: Right wheel angular velocity in rad/s.
    /// - `normal_force_left`: Normal load on left tire in Newtons.
    /// - `normal_force_right`: Normal load on right tire in Newtons.
    pub fn distribute_torque(
        &self,
        input_torque: f32,
        omega_left: f32,
        omega_right: f32,
        normal_force_left: f32,
        normal_force_right: f32,
    ) -> DifferentialOutput {
        let effective_input = input_torque * self.mechanical_efficiency;
        let avg_omega = 0.5 * (omega_left + omega_right);

        match self.diff_type {
            DifferentialType::Open => {
                let half_torque = 0.5 * effective_input;
                DifferentialOutput {
                    torque_left: half_torque,
                    torque_right: half_torque,
                    diff_angular_velocity: avg_omega,
                    lock_torque: 0.0,
                }
            }
            DifferentialType::Locked => {
                let total_load = normal_force_left + normal_force_right;
                let (t_left, t_right) = if total_load > 1.0 {
                    let frac_left = (normal_force_left / total_load).clamp(0.0, 1.0);
                    let frac_right = 1.0 - frac_left;
                    (effective_input * frac_left, effective_input * frac_right)
                } else {
                    (0.5 * effective_input, 0.5 * effective_input)
                };

                DifferentialOutput {
                    torque_left: t_left,
                    torque_right: t_right,
                    diff_angular_velocity: avg_omega,
                    lock_torque: (t_left - t_right).abs() * 0.5,
                }
            }
            DifferentialType::LimitedSlip {
                preload,
                stiffness,
                max_bias_ratio,
            } => {
                let delta_omega = omega_left - omega_right;
                let half_input = 0.5 * effective_input;

                // Locking torque opposes delta_omega (positive delta means left spins faster,
                // so we remove torque from left and push onto right)
                let dynamic_lock = stiffness * delta_omega;
                let preload_lock = if delta_omega.abs() > 1e-4 {
                    delta_omega.signum() * preload
                } else {
                    0.0
                };
                let raw_lock = dynamic_lock + preload_lock;

                // Max locking torque limited by total input and bias ratio
                let max_lock = half_input.abs() * ((max_bias_ratio - 1.0) / (max_bias_ratio + 1.0));
                let lock_torque = raw_lock.clamp(-max_lock, max_lock);

                let torque_left = half_input - lock_torque;
                let torque_right = half_input + lock_torque;

                DifferentialOutput {
                    torque_left,
                    torque_right,
                    diff_angular_velocity: avg_omega,
                    lock_torque,
                }
            }
        }
    }
}
