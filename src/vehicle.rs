//! High-level vehicle physics assembly integrating suspension, steering, drivetrain, and tire friction.

use glam::{Quat, Vec3};
use serde::{Deserialize, Serialize};

use crate::differential::Differential;
use crate::drivetrain::{Clutch, Engine, EngineConfig, EngineState};
use crate::error::VehicleError;
use crate::steering::{Steering, SteeringConfig, SteeringState};
use crate::suspension::{AntiRollBar, Suspension, SuspensionConfig, SuspensionForce, SuspensionState};
use crate::transmission::{Gear, Transmission, TransmissionConfig, TransmissionState};

/// Drive wheel configuration.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum DriveLayout {
    /// Front-Wheel Drive.
    Fwd,
    /// Rear-Wheel Drive.
    Rwd,
    /// All-Wheel Drive with center differential torque split.
    Awd {
        /// Fraction of torque sent to the front axle [0.0, 1.0].
        front_torque_bias: f32,
    },
}

impl Default for DriveLayout {
    fn default() -> Self {
        DriveLayout::Rwd
    }
}

/// Physical parameters for an individual wheel and tire.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct WheelConfig {
    /// Mount offset relative to vehicle chassis origin in meters.
    pub local_position: Vec3,
    /// Wheel radius in meters.
    pub radius: f32,
    /// Wheel mass in kg.
    pub mass: f32,
    /// Static friction coefficient $\mu_s$.
    pub static_friction_coeff: f32,
    /// Kinetic sliding friction coefficient $\mu_k$.
    pub kinetic_friction_coeff: f32,
    /// Longitudinal tire grip stiffness $k_z$.
    pub longitudinal_grip: f32,
    /// Lateral tire grip stiffness $k_x$.
    pub lateral_grip: f32,
    /// Rolling resistance coefficient $C_{\text{rr}}$.
    pub rolling_resistance_coeff: f32,
    /// Maximum brake torque capacity in N·m.
    pub max_brake_torque: f32,
    /// Handbrake application flag for this wheel.
    pub has_handbrake: bool,
}

impl Default for WheelConfig {
    fn default() -> Self {
        Self {
            local_position: Vec3::ZERO,
            radius: 0.34,
            mass: 18.0,
            static_friction_coeff: 1.05,
            kinetic_friction_coeff: 0.75,
            longitudinal_grip: 35.0,
            lateral_grip: 28.0,
            rolling_resistance_coeff: 0.015,
            max_brake_torque: 2_500.0,
            has_handbrake: false,
        }
    }
}

impl WheelConfig {
    /// Calculates rotational inertia of the wheel ($I = \frac{1}{2} m r^2$).
    #[inline]
    pub fn rotational_inertia(&self) -> f32 {
        0.5 * self.mass * self.radius * self.radius
    }
}

/// Dynamic runtime state for a single wheel.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct WheelState {
    /// Current spin angular velocity in rad/s.
    pub angular_velocity: f32,
    /// Current steering angle applied to this wheel in radians.
    pub steer_angle_rad: f32,
    /// Wheel contact point velocity in wheel-local coordinate frame (X = right/lat, Z = forward/long).
    pub local_velocity: Vec3,
    /// Normalized slip ratio / magnitude factor.
    pub slip: f32,
    /// True if tire forces exceed static friction limit (sliding).
    pub is_sliding: bool,
    /// Net tire forces in world space (friction force).
    pub tire_force_world: Vec3,
    /// Drive torque delivered to wheel in N·m.
    pub drive_torque: f32,
    /// Brake torque applied to wheel in N·m.
    pub brake_torque: f32,
}

impl WheelState {
    /// Creates a newly initialized wheel state.
    pub fn new() -> Self {
        Self {
            angular_velocity: 0.0,
            steer_angle_rad: 0.0,
            local_velocity: Vec3::ZERO,
            slip: 0.0,
            is_sliding: false,
            tire_force_world: Vec3::ZERO,
            drive_torque: 0.0,
            brake_torque: 0.0,
        }
    }
}

impl Default for WheelState {
    fn default() -> Self {
        Self::new()
    }
}

/// Vehicle driver control inputs.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct VehicleInputs {
    /// Steering command in [-1.0, 1.0] (positive = left, negative = right).
    pub steer: f32,
    /// Throttle position in [0.0, 1.0].
    pub throttle: f32,
    /// Foot brake pedal in [0.0, 1.0].
    pub brake: f32,
    /// Handbrake flag.
    pub handbrake: bool,
    /// Clutch pedal position in [0.0, 1.0] (0.0 = engaged, 1.0 = fully depressed disengaged).
    pub clutch_pedal: f32,
}

impl Default for VehicleInputs {
    fn default() -> Self {
        Self {
            steer: 0.0,
            throttle: 0.0,
            brake: 0.0,
            handbrake: false,
            clutch_pedal: 0.0,
        }
    }
}

/// Aerodynamic properties of the vehicle body.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct AeroConfig {
    /// Drag coefficient $C_d$ (e.g. 0.31).
    pub drag_coeff: f32,
    /// Frontal area in m² (e.g. 2.1 m²).
    pub frontal_area: f32,
    /// Air density $\rho$ in kg/m³ (1.225 at sea level).
    pub air_density: f32,
    /// Downforce coefficient.
    pub downforce_coeff: f32,
}

impl Default for AeroConfig {
    fn default() -> Self {
        Self {
            drag_coeff: 0.32,
            frontal_area: 2.15,
            air_density: 1.225,
            downforce_coeff: 0.25,
        }
    }
}

/// Raycast query result for a single wheel.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct WheelRaycastHit {
    /// Distance from wheel suspension mount to the surface contact point.
    pub distance: f32,
    /// World position of the contact point.
    pub point: Vec3,
    /// World surface normal vector.
    pub normal: Vec3,
}

/// Complete vehicle chassis configuration.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct VehicleConfig {
    /// Vehicle total mass in kg.
    pub mass: f32,
    /// Wheel configurations: [Front-Left, Front-Right, Rear-Left, Rear-Right].
    pub wheels: [WheelConfig; 4],
    /// Suspension configs for each wheel.
    pub suspensions: [SuspensionConfig; 4],
    /// Front axle anti-roll bar.
    pub front_anti_roll_bar: AntiRollBar,
    /// Rear axle anti-roll bar.
    pub rear_anti_roll_bar: AntiRollBar,
    /// Steering system configuration.
    pub steering: SteeringConfig,
    /// Engine configuration.
    pub engine: EngineConfig,
    /// Transmission configuration.
    pub transmission: TransmissionConfig,
    /// Drive axle layout.
    pub drive_layout: DriveLayout,
    /// Front differential (used if FWD or AWD).
    pub front_differential: Differential,
    /// Rear differential (used if RWD or AWD).
    pub rear_differential: Differential,
    /// Aerodynamic configuration.
    pub aero: AeroConfig,
}

impl Default for VehicleConfig {
    fn default() -> Self {
        let half_track = 0.81;
        let front_z = 1.35;
        let rear_z = -1.30;
        let ride_height = 0.40;

        let base_wheel = WheelConfig::default();
        let mut fl_wheel = base_wheel;
        fl_wheel.local_position = Vec3::new(-half_track, ride_height, front_z);

        let mut fr_wheel = base_wheel;
        fr_wheel.local_position = Vec3::new(half_track, ride_height, front_z);

        let mut rl_wheel = base_wheel;
        rl_wheel.local_position = Vec3::new(-half_track, ride_height, rear_z);
        rl_wheel.has_handbrake = true;

        let mut rr_wheel = base_wheel;
        rr_wheel.local_position = Vec3::new(half_track, ride_height, rear_z);
        rr_wheel.has_handbrake = true;

        Self {
            mass: 1_450.0,
            wheels: [fl_wheel, fr_wheel, rl_wheel, rr_wheel],
            suspensions: [SuspensionConfig::default(); 4],
            front_anti_roll_bar: AntiRollBar::new(8_500.0),
            rear_anti_roll_bar: AntiRollBar::new(6_000.0),
            steering: SteeringConfig::default(),
            engine: EngineConfig::default(),
            transmission: TransmissionConfig::default(),
            drive_layout: DriveLayout::Rwd,
            front_differential: Differential::open(),
            rear_differential: Differential::limited_slip(50.0, 100.0, 3.5),
            aero: AeroConfig::default(),
        }
    }
}

/// Resolved forces and moments to apply to the vehicle rigid body.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct VehicleForces {
    /// Net total force in world coordinates to apply at vehicle center of mass (N).
    pub total_force: Vec3,
    /// Net total torque in world coordinates to apply to vehicle body (N·m).
    pub total_torque: Vec3,
    /// Suspension forces for each wheel [FL, FR, RL, RR].
    pub suspension_forces: [SuspensionForce; 4],
    /// Aerodynamic drag force vector in world coordinates (N).
    pub aero_drag_force: Vec3,
    /// Aerodynamic downforce vector in world coordinates (N).
    pub aero_downforce: Vec3,
}

/// Full vehicle state and subsystem integration.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Vehicle {
    /// Vehicle configuration.
    pub config: VehicleConfig,
    /// Engine subsystem.
    pub engine: Engine,
    /// Engine runtime state.
    pub engine_state: EngineState,
    /// Transmission subsystem.
    pub transmission: Transmission,
    /// Transmission runtime state.
    pub transmission_state: TransmissionState,
    /// Clutch subsystem.
    pub clutch: Clutch,
    /// Steering controller.
    pub steering: Steering,
    /// Steering state.
    pub steering_state: SteeringState,
    /// Suspension struts [FL, FR, RL, RR].
    pub suspensions: [Suspension; 4],
    /// Suspension states [FL, FR, RL, RR].
    pub suspension_states: [SuspensionState; 4],
    /// Wheel runtime states [FL, FR, RL, RR].
    pub wheel_states: [WheelState; 4],
}

impl Vehicle {
    /// Creates a new vehicle from configuration.
    pub fn new(config: VehicleConfig) -> Self {
        let engine = Engine::new(config.engine);
        let engine_state = EngineState::new(config.engine.idle_rpm);
        let transmission = Transmission::new(config.transmission.clone());
        let transmission_state = TransmissionState::new();
        let clutch = Clutch::default();
        let steering = Steering::new(config.steering);
        let steering_state = SteeringState::new();

        let suspensions = [
            Suspension::new(config.suspensions[0]),
            Suspension::new(config.suspensions[1]),
            Suspension::new(config.suspensions[2]),
            Suspension::new(config.suspensions[3]),
        ];

        let suspension_states = [
            SuspensionState::new(config.suspensions[0].rest_length),
            SuspensionState::new(config.suspensions[1].rest_length),
            SuspensionState::new(config.suspensions[2].rest_length),
            SuspensionState::new(config.suspensions[3].rest_length),
        ];

        let wheel_states = [WheelState::new(); 4];

        Self {
            config,
            engine,
            engine_state,
            transmission,
            transmission_state,
            clutch,
            steering,
            steering_state,
            suspensions,
            suspension_states,
            wheel_states,
        }
    }

    /// Computes physics forces and updates all subsystems for one fixed time step.
    ///
    /// # Arguments
    /// - `inputs`: Driver control commands (steer, throttle, brake, handbrake, clutch).
    /// - `raycast_hits`: Surface raycast hit queries for each wheel [FL, FR, RL, RR].
    /// - `chassis_pos`: Vehicle center of mass world position.
    /// - `chassis_rot`: Vehicle chassis world orientation quaternion.
    /// - `chassis_linear_vel`: Vehicle linear velocity vector in world coordinates (m/s).
    /// - `chassis_angular_vel`: Vehicle angular velocity vector in world coordinates (rad/s).
    /// - `dt`: Physics fixed time step in seconds.
    pub fn step(
        &mut self,
        inputs: &VehicleInputs,
        raycast_hits: [Option<WheelRaycastHit>; 4],
        chassis_pos: Vec3,
        chassis_rot: Quat,
        chassis_linear_vel: Vec3,
        chassis_angular_vel: Vec3,
        dt: f32,
    ) -> Result<VehicleForces, VehicleError> {
        let speed = chassis_linear_vel.length();

        // 1. Update Steering with Ackermann geometry & speed reduction
        let steer_angles = self.steering.update(&mut self.steering_state, inputs.steer, speed, dt);
        self.wheel_states[0].steer_angle_rad = steer_angles.left_rad;
        self.wheel_states[1].steer_angle_rad = steer_angles.right_rad;
        self.wheel_states[2].steer_angle_rad = 0.0;
        self.wheel_states[3].steer_angle_rad = 0.0;

        // 2. Anti-Roll Sway Bar Forces
        let (front_arb_l, front_arb_r) = self
            .config
            .front_anti_roll_bar
            .calculate_forces(&self.suspension_states[0], &self.suspension_states[1]);
        let (rear_arb_l, rear_arb_r) = self
            .config
            .rear_anti_roll_bar
            .calculate_forces(&self.suspension_states[2], &self.suspension_states[3]);
        let arb_forces = [front_arb_l, front_arb_r, rear_arb_l, rear_arb_r];

        // 3. Update Suspensions
        let chassis_down = chassis_rot * Vec3::NEG_Y;
        let mut suspension_forces = [SuspensionForce {
            spring_force: 0.0,
            damping_force: 0.0,
            anti_roll_force: 0.0,
            total_normal_force: 0.0,
            force_vector: Vec3::ZERO,
            application_point: Vec3::ZERO,
        }; 4];

        for i in 0..4 {
            let mount_world = chassis_pos + chassis_rot * self.config.wheels[i].local_position;
            let hit_dist = raycast_hits[i].map(|h| h.distance);
            let hit_pt = raycast_hits[i].map(|h| h.point);
            let hit_norm = raycast_hits[i].map(|h| h.normal);

            suspension_forces[i] = self.suspensions[i].evaluate(
                &mut self.suspension_states[i],
                hit_dist,
                hit_pt,
                hit_norm,
                arb_forces[i],
                mount_world,
                chassis_down,
                dt,
            );
        }

        // 4. Update Engine & Transmission
        self.engine_state.throttle = inputs.throttle;
        self.clutch.engagement = (1.0 - inputs.clutch_pedal).clamp(0.0, 1.0);

        self.transmission.update(
            &mut self.transmission_state,
            self.engine_state.rpm,
            self.config.engine.max_rpm,
            inputs.throttle,
            dt,
        );

        let total_ratio = self.transmission.total_ratio(self.transmission_state.current_gear);
        let trans_power_factor = self.transmission.power_factor(&self.transmission_state);
        let _engine_gross_torque = self.engine.calculate_gross_torque(&self.engine_state);

        // Calculate average drive wheel rotation
        let avg_drive_omega = match self.config.drive_layout {
            DriveLayout::Fwd => 0.5 * (self.wheel_states[0].angular_velocity + self.wheel_states[1].angular_velocity),
            DriveLayout::Rwd => 0.5 * (self.wheel_states[2].angular_velocity + self.wheel_states[3].angular_velocity),
            DriveLayout::Awd { .. } => {
                0.25 * (self.wheel_states[0].angular_velocity
                    + self.wheel_states[1].angular_velocity
                    + self.wheel_states[2].angular_velocity
                    + self.wheel_states[3].angular_velocity)
            }
        };

        // Transmission input shaft speed
        let trans_input_omega = avg_drive_omega * total_ratio;

        // Clutch load torque on engine
        let clutch_torque = self
            .clutch
            .calculate_torque(self.engine_state.angular_velocity(), trans_input_omega);
        self.engine.update(&mut self.engine_state, clutch_torque, dt);

        // Torque passed through transmission gearbox to differential
        let driveshaft_torque = if self.transmission_state.current_gear == Gear::Neutral {
            0.0
        } else {
            clutch_torque * total_ratio * trans_power_factor
        };

        // 5. Differential Torque Distribution
        match self.config.drive_layout {
            DriveLayout::Fwd => {
                let diff_out = self.config.front_differential.distribute_torque(
                    driveshaft_torque,
                    self.wheel_states[0].angular_velocity,
                    self.wheel_states[1].angular_velocity,
                    self.suspension_states[0].normal_force,
                    self.suspension_states[1].normal_force,
                );
                self.wheel_states[0].drive_torque = diff_out.torque_left;
                self.wheel_states[1].drive_torque = diff_out.torque_right;
                self.wheel_states[2].drive_torque = 0.0;
                self.wheel_states[3].drive_torque = 0.0;
            }
            DriveLayout::Rwd => {
                let diff_out = self.config.rear_differential.distribute_torque(
                    driveshaft_torque,
                    self.wheel_states[2].angular_velocity,
                    self.wheel_states[3].angular_velocity,
                    self.suspension_states[2].normal_force,
                    self.suspension_states[3].normal_force,
                );
                self.wheel_states[0].drive_torque = 0.0;
                self.wheel_states[1].drive_torque = 0.0;
                self.wheel_states[2].drive_torque = diff_out.torque_left;
                self.wheel_states[3].drive_torque = diff_out.torque_right;
            }
            DriveLayout::Awd { front_torque_bias } => {
                let front_t = driveshaft_torque * front_torque_bias;
                let rear_t = driveshaft_torque * (1.0 - front_torque_bias);

                let diff_f = self.config.front_differential.distribute_torque(
                    front_t,
                    self.wheel_states[0].angular_velocity,
                    self.wheel_states[1].angular_velocity,
                    self.suspension_states[0].normal_force,
                    self.suspension_states[1].normal_force,
                );
                let diff_r = self.config.rear_differential.distribute_torque(
                    rear_t,
                    self.wheel_states[2].angular_velocity,
                    self.wheel_states[3].angular_velocity,
                    self.suspension_states[2].normal_force,
                    self.suspension_states[3].normal_force,
                );

                self.wheel_states[0].drive_torque = diff_f.torque_left;
                self.wheel_states[1].drive_torque = diff_f.torque_right;
                self.wheel_states[2].drive_torque = diff_r.torque_left;
                self.wheel_states[3].drive_torque = diff_r.torque_right;
            }
        }

        // 6. Calculate Wheel Ground Kinetics, Tire Friction, and Torques
        let mut net_force = Vec3::ZERO;
        let mut net_torque = Vec3::ZERO;

        for i in 0..4 {
            let wheel_cfg = &self.config.wheels[i];
            let wheel_state = &mut self.wheel_states[i];
            let sus_state = &self.suspension_states[i];
            let sus_force = &suspension_forces[i];

            // Add suspension push force and moment
            net_force += sus_force.force_vector;
            let arm_to_mount = sus_force.application_point - chassis_pos;
            net_torque += arm_to_mount.cross(sus_force.force_vector);

            if !sus_state.is_grounded {
                // Free airborne wheel deceleration
                let brake_t = inputs.brake * wheel_cfg.max_brake_torque;
                let inertia = wheel_cfg.rotational_inertia();
                let spin_decel = (wheel_state.drive_torque - brake_t) / inertia;
                wheel_state.angular_velocity += spin_decel * dt;
                wheel_state.angular_velocity *= (1.0 - 0.5 * dt).max(0.0);
                wheel_state.slip = 0.0;
                wheel_state.is_sliding = false;
                wheel_state.tire_force_world = Vec3::ZERO;
                continue;
            }

            // Wheel rotation in chassis plane (yaw angle from steering)
            let wheel_local_rot = Quat::from_rotation_y(wheel_state.steer_angle_rad);
            let wheel_world_rot = chassis_rot * wheel_local_rot;

            // Velocity at contact point: V_point = V_chassis + omega_chassis x r
            let contact_pt = sus_state.contact_point.unwrap_or(sus_force.application_point);
            let arm_to_contact = contact_pt - chassis_pos;
            let contact_velocity = chassis_linear_vel + chassis_angular_vel.cross(arm_to_contact);

            // Transform into wheel coordinate frame: X = lateral, Y = normal, Z = longitudinal
            let local_vel = wheel_world_rot.inverse() * contact_velocity;
            wheel_state.local_velocity = local_vel;

            let normal_force = sus_state.normal_force;

            // Longitudinal & lateral slip velocities
            let wheel_linear_speed = wheel_state.angular_velocity * wheel_cfg.radius;
            let long_slip_vel = local_vel.z - wheel_linear_speed;
            let lat_slip_vel = local_vel.x;

            // SimpleCar2 friction formulas
            let raw_long_friction = -wheel_cfg.longitudinal_grip * long_slip_vel;
            let raw_lat_friction = -wheel_cfg.lateral_grip * lat_slip_vel;

            // Friction saturation ellipse
            let friction_vec = Vec3::new(raw_lat_friction, 0.0, raw_long_friction);
            let friction_mag = friction_vec.length();

            let max_static_friction = normal_force * wheel_cfg.static_friction_coeff;
            let max_kinetic_friction = normal_force * wheel_cfg.kinetic_friction_coeff;

            wheel_state.is_sliding = friction_mag > max_static_friction;
            wheel_state.slip = if max_static_friction > 1.0 {
                friction_mag / max_static_friction
            } else {
                0.0
            };

            let clamped_friction = if wheel_state.is_sliding {
                friction_vec.normalize_or_zero() * max_kinetic_friction
            } else {
                friction_vec
            };

            // Transform friction force back to world space
            let tire_world_force = wheel_world_rot * clamped_friction;
            wheel_state.tire_force_world = tire_world_force;

            net_force += tire_world_force;
            net_torque += arm_to_contact.cross(tire_world_force);

            // Wheel rotational dynamics:
            // I * d_omega/dt = T_drive - T_brake - F_long * radius - T_rolling_res
            let mut brake_torque = inputs.brake * wheel_cfg.max_brake_torque;
            if inputs.handbrake && wheel_cfg.has_handbrake {
                brake_torque = brake_torque.max(wheel_cfg.max_brake_torque * 1.5);
            }
            wheel_state.brake_torque = brake_torque;

            let rolling_resistance_t = wheel_cfg.rolling_resistance_coeff
                * normal_force
                * wheel_cfg.radius
                * wheel_state.angular_velocity.signum();

            let tire_reaction_torque = clamped_friction.z * wheel_cfg.radius;

            let brake_opposing = brake_torque * wheel_state.angular_velocity.signum();
            let net_wheel_torque = wheel_state.drive_torque - brake_opposing - tire_reaction_torque - rolling_resistance_t;

            let wheel_inertia = wheel_cfg.rotational_inertia();
            let ang_accel = net_wheel_torque / wheel_inertia;

            wheel_state.angular_velocity += ang_accel * dt;

            // Handbrake lock check
            if inputs.handbrake && wheel_cfg.has_handbrake && wheel_state.angular_velocity.abs() < 5.0 {
                wheel_state.angular_velocity = 0.0;
            }
        }

        // 7. Aerodynamic Drag & Downforce
        let air_rho = self.config.aero.air_density;
        let v_sq = speed * speed;

        let drag_mag = 0.5 * air_rho * v_sq * self.config.aero.drag_coeff * self.config.aero.frontal_area;
        let drag_dir = if speed > 0.01 {
            -chassis_linear_vel.normalize()
        } else {
            Vec3::ZERO
        };
        let aero_drag_force = drag_dir * drag_mag;

        let downforce_mag = 0.5 * air_rho * v_sq * self.config.aero.downforce_coeff * self.config.aero.frontal_area;
        let aero_downforce = chassis_down * downforce_mag;

        net_force += aero_drag_force;
        net_force += aero_downforce;

        Ok(VehicleForces {
            total_force: net_force,
            total_torque: net_torque,
            suspension_forces,
            aero_drag_force,
            aero_downforce,
        })
    }
}
