# Vision-Based Autonomous Quadrotor Landing on a Moving Platform

<p align="center">
  <img src="README%20assets/Cover%20Gif.gif" alt="Autonomous quadrotor landing on a moving platform" />
</p>

<p align="center">
  <img src="https://img.shields.io/badge/MATLAB-R2024b-orange?logo=mathworks&logoColor=white" alt="MATLAB" height="28" />
  <img src="https://img.shields.io/badge/Simulink-R2024b-orange?logo=mathworks&logoColor=white" alt="Simulink" height="28" />
  <img src="https://img.shields.io/badge/CoppeliaSim-EDU-blue" alt="CoppeliaSim" height="28" />
  <img src="https://img.shields.io/badge/Status-Simulation-yellow" alt="Simulation" height="28" />
</p>

**System Design and Simulation Validation**

*Leonardo Sandri · Lorenzo Barbieri · Alessandro Carotenuto*

Full reimplementation of the autonomous landing system described in [Falanga et al. (2017)](https://www.researchgate.net/publication/319244044_Vision-based_Autonomous_Quadrotor_Landing_on_a_Moving_Platform), built in MATLAB/Simulink with CoppeliaSim as a sensor-in-the-loop simulator. The quadrotor detects a visual tag on a moving platform, estimates its state via Kalman filtering, and executes a complete takeoff → search → follow → land sequence autonomously.

---

## Quick Start

### Requirements

- **MATLAB R2024b** (or compatible) + **Simulink**
- **Image Processing Toolbox** (for `imfindcircles`, `bwconncomp`, `imbinarize`)
- **CoppeliaSim** (tested with EDU version) with the Remote API DLL (`remoteApi.dll`) present in `remAPI files/`

### Running the Simulation

1. Open CoppeliaSim and load `UnkownTrajectoryModified.ttt`
2. Start the CoppeliaSim simulation (play button)
3. In MATLAB, run `model_n_control_param.m` and `s_params.m` to load parameters into the workspace
4. Open and run `model_2024b_version.slx` in Simulink
5. The quadrotor will autonomously takeoff, search for the platform, track it, and land

To switch platform trajectory, edit `platform_trajectory.m` and set `trajectory_type` to one of: `'stationary'`, `'linear'`, `'linear_with_ending'`, `'circle'`, `'figure8'`, `'spline'`.

---

## Demo

Simulation videos are available in the `Simulation Videos/` folder:

- `1 - LINEAR PLATFORM TRAJECTORY.mp4`
- `2 - CIRCULAR PLATFORM TRAJECTORY.mp4`
- `3 - FIGURE_8 PLATFORM TRAJECTORY.mp4`

---

## System Architecture

All modules run synchronously at **20 Hz** inside Simulink. CoppeliaSim acts purely as a visualization and camera server, all physics and control computation happens in MATLAB.

<p align="center">
  <img src="README%20assets/Pipeline.png" alt="System pipeline" />
</p>

### Modules

| Module | File | Description |
|--------|------|-------------|
| **Dynamics** | `dynamic_model.m` | 6-DOF rigid body model (Euler angles, 12-state vector) |
| **Position Controller** | `Quadrotor Controllers/position_control.m` | Cascaded PID with gravity compensation and anti-windup |
| **Attitude Controller** | `Quadrotor Controllers/attitude_control.m` | PID + inertia feedforward on roll, pitch, yaw |
| **Vision** | `Coppelia Synchronization and Datastream/coppelia_camera.m` | Tag detection via thresholding + PnP solver; adaptive FOV |
| **Kalman Filter** | `Coppelia Synchronization and Datastream/coppelia_kalman.m` | Constant-velocity KF with attitude gating |
| **State Machine** | `Trajectories/stateManager.m` | Four-state FSM: TAKEOFF → FOLLOW / SEARCH → LANDING |
| **Trajectory Generation** | `Trajectories/` | Polynomial blend + first-order LPF + predictive feedforward |
| **CoppeliaSim Sync** | `Coppelia Synchronization and Datastream/coppelia_sync.m` | Master-slave synchronization via Remote API |

---

## Quadrotor Dynamics

The drone is modeled as a rigid body with state $\mathbf{x} = [x, y, z, \phi, \theta, \psi, \dot{x}, \dot{y}, \dot{z}, p, q, r]$ and control inputs $\mathbf{u} = [T, \tau_\phi, \tau_\theta, \tau_\psi]$:

$$\ddot{x} = -\frac{T}{m}(\cos\psi\sin\theta\cos\phi + \sin\psi\sin\phi)$$

$$\ddot{y} = -\frac{T}{m}(\sin\psi\sin\theta\cos\phi - \cos\psi\sin\phi)$$

$$\ddot{z} = -\frac{T}{m}\cos\theta\cos\phi + g$$

$$\ddot{\phi} = \frac{\tau_\phi}{I_x}, \qquad \ddot{\theta} = \frac{\tau_\theta}{I_y}, \qquad \ddot{\psi} = \frac{\tau_\psi}{I_z}$$

**Physical parameters** (`model_n_control_param.m`):

| Parameter | Value |
|-----------|-------|
| Mass | 1.0 kg |
| $I_x = I_y$ | 0.1 kg·m² |
| $I_z$ | 0.2 kg·m² |
| $g$ | 9.81 m/s² |

---

## Control Architecture

### Position Controller (outer loop)

Computes desired tilt angles and total thrust from position error via PID:

$$T = \frac{m}{\cos\phi\cos\theta}\left(g - K_{p,z}e_z - K_{d,z}\dot{e}_z - K_{i,z}\int e_z\,dt - \ddot{z}_d\right)$$

$$U_x = K_{p,x}e_x + K_{d,x}\dot{e}_x + K_{i,x}\int e_x\,dt + \ddot{x}_d$$

Thrust limits: $T \in [0.1mg,\ 4.0mg]$. Lateral acceleration bounded by max tilt angle ($0.7\ \text{rad} \approx 40^\circ$).

### Attitude Controller (inner loop)

Tracks desired Euler angles with PID + inertia feedforward:

$$\tau_\phi = K_{p,\phi}e_\phi + K_{d,\phi}\dot{e}_\phi + K_{i,\phi}\int e_\phi\,dt + I_x\ddot{\phi}_d$$

Torque saturation: $|\tau_\phi|, |\tau_\theta| \leq 1.0$ N·m, $|\tau_\psi| \leq 0.5$ N·m.

### Tuned Control Gains

| Controller | $K_p$ | $K_d$ | $K_i$ |
|------------|-------|-------|-------|
| Position X, Y | 20, 20 | 15, 15 | 8, 8 |
| Position Z | 30 | 40 | 16 |
| Attitude $\phi$, $\theta$ | 35, 35 | 40, 40 | 40, 40 |
| Attitude $\psi$ | 150 | 70 | 40 |

---

## Vision-Based Platform Detection

The landing target is a **black cross inside a black circle on a white background**, robust to orientation and distance variation.

Detection pipeline (runs in MATLAB at 20 Hz):
1. **Thresholding**, adaptive binarization (sensitivity 0.20)
2. **Connected Components**, find white regions
3. **Largest Quadrangle**, select tag background
4. **Pattern Matching**, detect circle (via `imfindcircles`) or cross (line detection); fallback to quadrangle corners
5. **RANSAC**, reject outlier corners
6. **PnP Solver**, recover 3D platform pose relative to camera, then transform to world frame:

$$\mathbf{p}_{\text{platform,world}} = \mathbf{p}_{\text{quad}} + \mathbf{R}_{\text{quad}} \left( \mathbf{R}_{\text{cam}\to\text{body}} \, \mathbf{p}_{\text{platform,cam}} + \mathbf{d}_{\text{cam}} \right)$$

**Adaptive FOV**: expands from 90° to 135° as the drone descends below 0.6 m, ensuring the tag stays visible during the final approach.

---

## Kalman Filter

Kalman filter estimating platform state $\hat{\mathbf{x}} = [x, y, z, \dot{x}, \dot{y}, \dot{z}]^T$ using a constant-velocity motion model.

**Process noise**: $\sigma_{q,\text{pos}} = 0.1$ m, $\sigma_{q,\text{vel}} = 1.0$ m/s

**Measurement noise**: $\sigma^2_r = 0.05$ m²

**Attitude Gating**: Kalman updates are disabled when $|\phi| > 10^\circ$ or $|\theta| > 10^\circ$ ($2^\circ$ hysteresis). At large tilt angles the downward camera misaligns with the platform, producing noisy corner detections. In prediction-only mode the filter maintains a valid state estimate between detections.

---

## State Machine

Four-state FSM implemented in `stateManager.m`:

```
TAKEOFF ──(alt ≥ 2.5 m AND detect 3s)──► FOLLOW ──(error < 0.8 m for 15s)──► LANDING
   │                                        │
   └──(hover 10s, no detect)──► SEARCH ──(detect 2s)──► FOLLOW
```

| State | Behavior |
|-------|----------|
| **TAKEOFF** | Ascend vertically at 0.75 m/s to 2.5 m; hold position while awaiting detection |
| **FOLLOW** | Track Kalman-estimated platform position (1.5 m above); polynomial transition + LPF + feedforward |
| **SEARCH** | Circular orbit: R = 6 m, $\omega$ = 0.2 rad/s; re-acquires lost platform |
| **LANDING** | Smooth descent at 0.5 m/s; touchdown detected at altitude < 0.05 m |

---

## Trajectory Generation

On-demand generation (no pre-computation):
1. Predict platform position $\tau = 5$ s ahead: $\mathbf{p}_{\text{target}} = \hat{\mathbf{p}} + \hat{\mathbf{v}} \cdot \tau$
2. Polynomial blend ($3x^2 - 2x^3$) from current position to predicted target over $T = 4$ s
3. First-order LPF ($\tau = 5$ s) to smooth the trajectory
4. Predictive feedforward compensates filter lag

---

## Simulation Results

Tested on four platform trajectories (platform speed $\leq 0.3$ m/s):

| Trajectory | Max Error $x$ | Max Error $y$ | Max Error $z$ |
|------------|--------------|--------------|--------------|
| Linear | 0.16 m | 0.06 m | 0.035 m |
| Circular | 0.12 m | 0.39 m | 0.05 m |
| Figure-8 | 0.175 m | 0.23 m | 0.06 m |

- Kalman filter converges within **10–20 s** of first detection
- Roll/pitch errors remain within **±3–6°** during steady tracking
- **~100% landing success** across all tested scenarios

---

## Repository Structure

```
.
├── Vision-Based Autonomous Quadrotor Landing on a Moving Platform/
│   ├── model_2024b_version.slx          # Main Simulink model
│   ├── dynamic_model.m                  # Quadrotor 6-DOF dynamics
│   ├── Quadrotor Controllers/
│   │   ├── position_control.m           # Outer loop PID
│   │   └── attitude_control.m           # Inner loop PID + feedforward
│   ├── Trajectories/
│   │   ├── stateManager.m               # Four-state FSM
│   │   ├── platform_trajectory.m        # Platform motion (linear/circle/fig-8)
│   │   ├── landing_trajectory.m         # Landing descent profile
│   │   ├── hovering_takeoff.m           # Takeoff trajectory
│   │   └── generate_spline_trajectory.m # Polynomial trajectory generator
│   ├── Coppelia Synchronization and Datastream/
│   │   ├── coppelia_sync.m              # Master sync S-function
│   │   ├── coppelia_camera.m            # Vision + PnP detection S-function
│   │   └── coppelia_kalman.m            # Kalman filter S-function
│   ├── Parameter Setting/
│   │   ├── model_n_control_param.m      # Drone + control gains
│   │   └── s_params.m                   # Simulation parameters
│   ├── remAPI files/                    # CoppeliaSim Remote API bindings
│   ├── Visualizations & Plots/          # Plot and visualization scripts
│   └── UnkownTrajectoryModified.ttt     # CoppeliaSim scene file
├── Simulation Videos/
│   ├── 1 - LINEAR PLATFORM TRAJECTORY.mp4
│   ├── 2 - CIRCULAR PLATFORM TRAJECTORY.mp4
│   └── 3 - FIGURE_8 PLATFORM TRAJECTORY.mp4
├── Report - EiR - Sandri, Barbieri, Carotenuto.pdf
└── Vision-Based Autonomous Quadrotor Landing on a Moving Platform.pptx
```

---

## Reference

> Falanga, D., Zanchettin, A., Simovic, A., Delmerico, J., & Scaramuzza, D. (2017). [*Vision-based autonomous quadrotor landing on a moving platform*](https://www.researchgate.net/publication/319244044_Vision-based_Autonomous_Quadrotor_Landing_on_a_Moving_Platform). IEEE International Symposium on Safety, Security and Rescue Robotics (SSRR).
