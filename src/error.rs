//! Error types for the SimpleCar2 vehicle dynamics simulation.

use thiserror::Error;

/// Errors that can occur during vehicle physics simulation and configuration.
#[derive(Debug, Error, PartialEq)]
pub enum VehicleError {
    /// Invalid configuration parameters (e.g. negative mass, zero wheelbase).
    #[error("Invalid configuration: {0}")]
    InvalidConfiguration(String),

    /// Invalid gear selection.
    #[error("Invalid gear: requested gear {gear} is outside range [{min}, {max}]")]
    InvalidGear { gear: i8, min: i8, max: i8 },

    /// Floating point anomaly encountered (NaN or infinite value).
    #[error("Numerical anomaly detected: {field} contains non-finite value {value}")]
    NonFiniteNumber { field: &'static str, value: f32 },

    /// Wheel index out of bounds.
    #[error("Wheel index {0} is out of bounds")]
    InvalidWheelIndex(usize),
}
