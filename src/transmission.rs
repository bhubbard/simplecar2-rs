//! Transmission system supporting manual and automatic shifting, multi-ratio gearboxes, and final drive.
//!
//! # Mechanics & Equations
//! - **Total Gear Ratio**:
//!   $$R_{\text{total}} = R_{\text{gear}} \cdot R_{\text{final}}$$
//!   Torque multiplication from crankshaft to wheels:
//!   $$T_{\text{driveshaft}} = T_{\text{engine}} \cdot R_{\text{total}}$$
//!   Speed reduction:
//!   $$\omega_{\text{driveshaft}} = \frac{\omega_{\text{engine}}}{R_{\text{total}}}$$
//!
//! - **Automatic Shift Logic**:
//!   - **Rev-Limiter Protection**: Forces immediate upshift if $RPM > 0.92 \cdot RPM_{\text{max}}$.
//!   - **Upshift Threshold**: Upshifts when $RPM > RPM_{\text{upshift}}$ and throttle is engaged.
//!   - **Downshift Threshold**: Downshifts when $RPM < RPM_{\text{downshift}}$ to prevent bogging.
//!   - **Shift Delay & Cooldown**: Disengages clutch torque during shifting ($\Delta t_{\text{shift}}$)
//!     and prevents rapid gear cycling with cooldown hysteresis ($\Delta t_{\text{cooldown}}$).

use serde::{Deserialize, Serialize};
use crate::error::VehicleError;

/// Discrete gear selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Gear {
    /// Reverse gear.
    Reverse,
    /// Neutral (disengaged transmission).
    Neutral,
    /// Forward gear 1 through N (1-indexed).
    Forward(u8),
}

impl Gear {
    /// Returns integer index representation: -1 for Reverse, 0 for Neutral, positive for Forward.
    pub fn to_i8(self) -> i8 {
        match self {
            Gear::Reverse => -1,
            Gear::Neutral => 0,
            Gear::Forward(g) => g as i8,
        }
    }

    /// Converts an integer index (-1 = R, 0 = N, 1..=6 = Forward) into a Gear.
    pub fn from_i8(val: i8) -> Result<Self, VehicleError> {
        match val {
            -1 => Ok(Gear::Reverse),
            0 => Ok(Gear::Neutral),
            g if g > 0 => Ok(Gear::Forward(g as u8)),
            other => Err(VehicleError::InvalidGear {
                gear: other,
                min: -1,
                max: 12,
            }),
        }
    }
}

/// Configuration settings for the vehicle transmission.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TransmissionConfig {
    /// Gear ratios for forward gears (index 0 = 1st gear, index 1 = 2nd gear, etc.).
    /// Standard 6-speed ratios provided by default.
    pub forward_ratios: Vec<f32>,
    /// Reverse gear ratio (typically negative or inverted).
    pub reverse_ratio: f32,
    /// Differential final drive ratio (e.g. 3.73).
    pub final_drive_ratio: f32,
    /// Time in seconds required to complete a gear shift (power is cut/attenuated).
    pub shift_duration: f32,
    /// Minimum time in seconds required before another automatic shift can trigger.
    pub shift_cooldown: f32,
    /// Engine RPM threshold to trigger automatic upshift.
    pub upshift_rpm: f32,
    /// Engine RPM threshold to trigger automatic downshift.
    pub downshift_rpm: f32,
    /// Whether automatic gear shifting is enabled.
    pub is_automatic: bool,
}

impl Default for TransmissionConfig {
    fn default() -> Self {
        Self {
            // 6-speed close-ratio sports gearbox
            forward_ratios: vec![3.60, 2.19, 1.52, 1.15, 0.92, 0.74],
            reverse_ratio: -3.40,
            final_drive_ratio: 3.73,
            shift_duration: 0.12,
            shift_cooldown: 0.60,
            upshift_rpm: 6_800.0,
            downshift_rpm: 2_600.0,
            is_automatic: true,
        }
    }
}

/// Dynamic runtime state of the transmission.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct TransmissionState {
    /// Current selected gear.
    pub current_gear: Gear,
    /// Previous gear held before last shift.
    pub previous_gear: Gear,
    /// Whether the transmission is mid-shift.
    pub is_shifting: bool,
    /// Remaining time for the current gear shift transition in seconds.
    pub shift_timer: f32,
    /// Time elapsed since the last completed gear shift in seconds.
    pub cooldown_timer: f32,
}

impl TransmissionState {
    /// Creates a newly initialized transmission in 1st gear.
    pub fn new() -> Self {
        Self {
            current_gear: Gear::Forward(1),
            previous_gear: Gear::Forward(1),
            is_shifting: false,
            shift_timer: 0.0,
            cooldown_timer: 1.0,
        }
    }

    /// Creates a transmission initialized in Neutral.
    pub fn in_neutral() -> Self {
        Self {
            current_gear: Gear::Neutral,
            previous_gear: Gear::Neutral,
            is_shifting: false,
            shift_timer: 0.0,
            cooldown_timer: 1.0,
        }
    }
}

impl Default for TransmissionState {
    fn default() -> Self {
        Self::new()
    }
}

/// Transmission gearbox unit.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Transmission {
    /// Transmission configuration parameters.
    pub config: TransmissionConfig,
}

impl Transmission {
    /// Creates a new transmission.
    pub fn new(config: TransmissionConfig) -> Self {
        Self { config }
    }

    /// Total number of forward gears available.
    #[inline]
    pub fn forward_gear_count(&self) -> usize {
        self.config.forward_ratios.len()
    }

    /// Returns the internal gear ratio for the specified gear (excluding final drive).
    pub fn gear_ratio(&self, gear: Gear) -> f32 {
        match gear {
            Gear::Reverse => self.config.reverse_ratio,
            Gear::Neutral => 0.0,
            Gear::Forward(g) => {
                let idx = (g as usize).saturating_sub(1);
                if idx < self.config.forward_ratios.len() {
                    self.config.forward_ratios[idx]
                } else {
                    0.0
                }
            }
        }
    }

    /// Returns total overall ratio from crankshaft to differential ($R_{\text{gear}} \cdot R_{\text{final}}$).
    pub fn total_ratio(&self, gear: Gear) -> f32 {
        self.gear_ratio(gear) * self.config.final_drive_ratio
    }

    /// Manually commands an upshift to the next higher gear.
    pub fn shift_up(&self, state: &mut TransmissionState) {
        if state.is_shifting {
            return;
        }

        let next_gear = match state.current_gear {
            Gear::Reverse => Gear::Neutral,
            Gear::Neutral => Gear::Forward(1),
            Gear::Forward(g) => {
                if (g as usize) < self.config.forward_ratios.len() {
                    Gear::Forward(g + 1)
                } else {
                    return; // Already in top gear
                }
            }
        };

        self.initiate_shift(state, next_gear);
    }

    /// Manually commands a downshift to the next lower gear.
    pub fn shift_down(&self, state: &mut TransmissionState) {
        if state.is_shifting {
            return;
        }

        let prev_gear = match state.current_gear {
            Gear::Forward(1) => Gear::Neutral,
            Gear::Forward(g) => Gear::Forward(g - 1),
            Gear::Neutral => Gear::Reverse,
            Gear::Reverse => return, // Already in reverse
        };

        self.initiate_shift(state, prev_gear);
    }

    /// Directly selects a target gear, triggering shift timer if different.
    pub fn set_gear(&self, state: &mut TransmissionState, target_gear: Gear) -> Result<(), VehicleError> {
        match target_gear {
            Gear::Forward(g) if (g as usize) > self.config.forward_ratios.len() => {
                return Err(VehicleError::InvalidGear {
                    gear: g as i8,
                    min: -1,
                    max: self.config.forward_ratios.len() as i8,
                });
            }
            _ => {}
        }

        if state.current_gear != target_gear && !state.is_shifting {
            self.initiate_shift(state, target_gear);
        }
        Ok(())
    }

    fn initiate_shift(&self, state: &mut TransmissionState, target_gear: Gear) {
        state.previous_gear = state.current_gear;
        state.current_gear = target_gear;
        state.is_shifting = true;
        state.shift_timer = self.config.shift_duration;
        state.cooldown_timer = 0.0;
    }

    /// Advances timers and executes automatic shifting logic if enabled.
    ///
    /// # Arguments
    /// - `state`: Mutable reference to transmission state.
    /// - `engine_rpm`: Current engine rotational speed.
    /// - `max_rpm`: Engine redline RPM limit.
    /// - `throttle`: Driver throttle input [0.0, 1.0].
    /// - `dt`: Delta time in seconds.
    pub fn update(
        &self,
        state: &mut TransmissionState,
        engine_rpm: f32,
        max_rpm: f32,
        throttle: f32,
        dt: f32,
    ) {
        if dt <= 1e-6 {
            return;
        }

        // Process active shift transition
        if state.is_shifting {
            state.shift_timer -= dt;
            if state.shift_timer <= 0.0 {
                state.is_shifting = false;
                state.shift_timer = 0.0;
            }
            return;
        }

        state.cooldown_timer += dt;

        // Skip automatic logic if disabled or cooldown not met
        if !self.config.is_automatic || state.cooldown_timer < self.config.shift_cooldown {
            return;
        }

        // Automatic shift decision
        match state.current_gear {
            Gear::Forward(g) => {
                let current_idx = g as usize;
                let max_gears = self.config.forward_ratios.len();

                // 1. Redline safety override: shift up immediately
                if engine_rpm > max_rpm * 0.92 && current_idx < max_gears {
                    self.shift_up(state);
                    return;
                }

                // 2. Throttle-driven upshift
                if throttle > 0.15 && engine_rpm > self.config.upshift_rpm && current_idx < max_gears {
                    self.shift_up(state);
                    return;
                }

                // 3. Low-RPM downshift to prevent engine bogging / stall
                if engine_rpm < self.config.downshift_rpm && current_idx > 1 {
                    self.shift_down(state);
                }
            }
            Gear::Neutral => {
                // In automatic mode, engaging throttle in neutral engages 1st gear
                if throttle > 0.05 {
                    self.initiate_shift(state, Gear::Forward(1));
                }
            }
            Gear::Reverse => {
                // Remain in reverse until manually shifted or changed
            }
        }
    }

    /// Returns torque transmission efficiency factor during shifting (0.0 to 1.0).
    /// Disengages torque during gear swaps to simulate clutch interruption.
    #[inline]
    pub fn power_factor(&self, state: &TransmissionState) -> f32 {
        if state.is_shifting {
            0.15 // Small residual torque / synchro drag
        } else {
            1.0
        }
    }
}
