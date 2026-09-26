# simplecar2-rs

[![Crates.io](https://img.shields.io/badge/crates.io-v0.1.0-orange.svg)](https://crates.io/crates/simplecar2)
[![GitHub Pages](https://img.shields.io/badge/Live%20Demo-GitHub%20Pages-brightgreen?logo=github)](https://bhubbard.github.io/simplecar2-rs/)
[![License](https://img.shields.io/badge/license-MIT%2FApache--2.0-blue.svg)](LICENSE)
[![Rust](https://img.shields.io/badge/rust-2024%20edition-brightgreen.svg)](https://www.rust-lang.org)

🏎️ **[Explore the Live Interactive Vehicle Physics Simulator](https://bhubbard.github.io/simplecar2-rs/)**

Pure Rust arcade-sim vehicle dynamics, custom raycast suspension, drivetrain, transmission, differential, and Ackermann steering translated from Simon Vutov's [`SimpleCar2`](https://github.com/SimonVutov/SimpleCar2).

Engineered specifically for modular game development and open-world Bevy RPGs, with zero engine lock-in (uses [`glam`](https://crates.io/crates/glam) and [`serde`](https://crates.io/crates/serde)).

---

## Features & Mathematical Formulations

### 1. Custom Raycast Suspension & Anti-Roll Bars (`suspension`)
- **Hooke's Law Spring Force**:
  $$F_{\text{spring}} = k \cdot x$$
  where $k$ is the suspension spring stiffness (N/m) and $x = L_{\text{rest}} - L_{\text{measured}}$ is the compression distance.
- **Viscous Damping Force**:
  $$F_{\text{damping}} = -c \cdot v$$
  where $c$ is the damping coefficient (differentiating bump compression vs. rebound expansion) and $v = \frac{dx}{dt}$ is compression velocity.
- **Anti-Roll Sway Bar (ARB)**:
  Connects paired wheels on the same axle to resist lateral chassis body roll during hard cornering:
  $$\Delta x = x_{\text{left}} - x_{\text{right}}$$
  $$F_{\text{arb}} = k_{\text{arb}} \cdot \Delta x$$
  Reduces normal force on the outer loaded wheel and increases normal force on the inner unloaded wheel:
  $$F_{N,\text{left}} = F_{\text{spring}} + F_{\text{damping}} - F_{\text{arb}}$$
  $$F_{N,\text{right}} = F_{\text{spring}} + F_{\text{damping}} + F_{\text{arb}}$$

### 2. Drivetrain & Engine Dynamics (`drivetrain`)
- **Torque & Power Curve**:
  Realistic combustion power curve based on current engine RPM:
  - Below idle ($RPM < RPM_{\text{idle}}$): zero or idle governor compensation.
  - From idle to peak power ($RPM_{\text{idle}} \le RPM < RPM_{\text{peak}}$):
    $$t = \frac{RPM - RPM_{\text{idle}}}{RPM_{\text{peak}} - RPM_{\text{idle}}}$$
    $$P(t) = 0.3 + 0.7 \cdot t^2$$
  - From peak power to redline ($RPM_{\text{peak}} \le RPM < RPM_{\text{max}}$):
    $$t = \frac{RPM - RPM_{\text{peak}}}{RPM_{\text{max}} - RPM_{\text{peak}}}$$
    $$P(t) = 1.0 - 0.9 \cdot t$$
  - At or above redline ($RPM \ge RPM_{\text{max}}$): Rev limiter fuel cut ($P = 0$).
  - Constant torque mode for electric powertrains.
- **Clutch Dynamics**:
  Pedal engagement $\gamma \in [0.0, 1.0]$. Torque capacity:
  $$T_{\text{capacity}} = \gamma \cdot T_{\text{max\_clutch}}$$
  Transmitted torque coupling engine crankshaft and transmission input shaft:
  $$T_{\text{clutch}} = T_{\text{capacity}} \cdot \tanh\left(0.1 \cdot (\omega_{\text{engine}} - \omega_{\text{trans}})\right)$$

### 3. Transmission & Automatic Shift Logic (`transmission`)
- **Gear Ratios**:
  Multi-speed gearbox (Reverse, Neutral, 1..6+ Forward gears) and final drive ratio ($R_{\text{final}}$):
  $$R_{\text{total}} = R_{\text{gear}} \cdot R_{\text{final}}$$
  $$T_{\text{driveshaft}} = T_{\text{engine}} \cdot R_{\text{total}}$$
  $$\omega_{\text{driveshaft}} = \frac{\omega_{\text{engine}}}{R_{\text{total}}}$$
- **Shift Logic**:
  - Automatic upshift triggered when $RPM > RPM_{\text{upshift}}$ and throttle is applied.
  - Automatic downshift triggered when $RPM < RPM_{\text{downshift}}$ to prevent engine bogging.
  - Rev-limiter safety override: forces upshift if $RPM > 0.92 \cdot RPM_{\text{max}}$.
  - Gear change cooldown timer ($\Delta t_{\text{cooldown}}$) prevents gear hunting.
  - Shift transition time ($\Delta t_{\text{shift}}$) disengages drive torque during shifts.

### 4. Differential Torque Distribution (`differential`)
- **Open Differential**:
  Splits input driveshaft torque equally 50/50:
  $$T_{\text{left}} = \frac{1}{2} T_{\text{in}}, \quad T_{\text{right}} = \frac{1}{2} T_{\text{in}}$$
- **Locked (Spool) Differential**:
  Locks axle speeds ($\omega_{\text{left}} = \omega_{\text{right}}$) and distributes torque proportional to instantaneous tire normal load:
  $$T_{\text{left}} = T_{\text{in}} \cdot \frac{F_{z,\text{left}}}{F_{z,\text{left}} + F_{z,\text{right}}}$$
- **Limited-Slip Differential (LSD)**:
  Employs clutch preload ($T_{\text{preload}}$), speed delta stiffness ($k_{\text{lsd}}$), and max torque bias ratio $B = \frac{T_{\text{high}}}{T_{\text{low}}}$:
  $$\Delta \omega = \omega_{\text{left}} - \omega_{\text{right}}$$
  $$T_{\text{lock}} = \operatorname{clamp}\left(k_{\text{lsd}} \cdot \Delta \omega + \operatorname{sgn}(\Delta \omega) \cdot T_{\text{preload}}, -T_{\text{max}}, T_{\text{max}}\right)$$
  $$T_{\text{left}} = \frac{1}{2} T_{\text{in}} - T_{\text{lock}}, \quad T_{\text{right}} = \frac{1}{2} T_{\text{in}} + T_{\text{lock}}$$

### 5. Steering Dynamics (`steering`)
- **Ackermann Steering Geometry**:
  Calculates independent steer angles for inner and outer front wheels to eliminate scrub:
  $$R = \frac{L}{\tan(\delta)}$$
  $$\delta_{\text{inner}} = \arctan\left(\frac{2L \tan(\delta)}{2L - W \tan(\delta)}\right)$$
  $$\delta_{\text{outer}} = \arctan\left(\frac{2L \tan(\delta)}{2L + W \tan(\delta)}\right)$$
  where $L$ is wheelbase and $W$ is front track width.
- **Speed-Sensitive Steering Reduction**:
  Attenuates maximum steering lock as vehicle forward velocity $v$ increases to preserve stability:
  $$\delta_{\text{effective}} = \frac{\delta_{\text{max}} \cdot \text{input}}{1.0 + v \cdot k_{\text{speed}}}$$

---

## Installation

Add to your `Cargo.toml`:

```toml
[dependencies]
simplecar2 = "0.1"
glam = "0.29"
```

---

## Quickstart

```rust
use glam::{Quat, Vec3};
use simplecar2::prelude::*;

fn main() {
    // 1. Configure vehicle chassis, drivetrain, and wheels
    let config = VehicleConfig::default();
    let mut vehicle = Vehicle::new(config);

    // 2. Supply driver inputs
    let inputs = VehicleInputs {
        steer: 0.25,        // Turning slightly left
        throttle: 0.8,     // 80% throttle
        brake: 0.0,
        handbrake: false,
        clutch_pedal: 0.0, // Clutch engaged
    };

    // 3. Provide raycast contact results from your physics world (e.g. Rapier / Bevy XPBD)
    let hits = [
        Some(WheelRaycastHit { distance: 0.40, point: Vec3::new(-0.81, 0.0, 1.35), normal: Vec3::Y }),
        Some(WheelRaycastHit { distance: 0.40, point: Vec3::new(0.81, 0.0, 1.35), normal: Vec3::Y }),
        Some(WheelRaycastHit { distance: 0.40, point: Vec3::new(-0.81, 0.0, -1.30), normal: Vec3::Y }),
        Some(WheelRaycastHit { distance: 0.40, point: Vec3::new(0.81, 0.0, -1.30), normal: Vec3::Y }),
    ];

    let chassis_pos = Vec3::new(0.0, 0.45, 0.0);
    let chassis_rot = Quat::IDENTITY;
    let chassis_linear_vel = Vec3::new(0.0, 0.0, 12.0); // 12 m/s forward
    let chassis_angular_vel = Vec3::ZERO;
    let dt = 1.0 / 60.0;

    // 4. Advance physics step
    let forces = vehicle.step(
        &inputs,
        hits,
        chassis_pos,
        chassis_rot,
        chassis_linear_vel,
        chassis_angular_vel,
        dt,
    ).unwrap();

    println!("Total net force on chassis: {:?}", forces.total_force);
    println!("Total net torque on chassis: {:?}", forces.total_torque);
    println!("Engine RPM: {:.0}", vehicle.engine_state.rpm);
    println!("Current Gear: {:?}", vehicle.transmission_state.current_gear);
}
```

---

## License

Dual-licensed under either of:
- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)
