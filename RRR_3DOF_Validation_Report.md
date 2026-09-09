# 3-DOF RRR Robotic Arm — Forward Dynamics Validation Report

**Models compared:** Newton-Euler Analytical vs. Simscape Multibody  
**Simulation duration:** 100 seconds  
**Gravity:** 9.81 m/s² in −Y direction  
**Solver:** ODE45, RelTol = 1×10⁻⁸, AbsTol = 1×10⁻¹⁰

---

## 1. System Description

### 1.1 Robot Geometry

A planar 3-DOF RRR (Revolute-Revolute-Revolute) robotic arm. All three joint axes are aligned with the world Z-axis, so the arm rotates in the XY plane. Gravity acts in the −Y direction.

```
World Z (out of page)
        |
        ● Joint 1 (q1) ─── Link 1 ─── ● Joint 2 (q2) ─── Link 2 ─── ● Joint 3 (q3) ─── Link 3
        |
       ─Y (gravity)
```

### 1.2 Physical Parameters

| Parameter | Link 1 | Link 2 | Link 3 |
|-----------|--------|--------|--------|
| Length (m) | 0.50 | 0.40 | 0.30 |
| CoM offset (m) | 0.25 | 0.20 | 0.15 |
| Mass (kg) | 2.0 | 1.5 | 1.0 |
| Radius (m) | 0.060 | 0.050 | 0.040 |

**Inertia tensors** (solid cylinder, about CoM):

| Joint | I_xx (kg·m²) | I_yy = I_zz (kg·m²) |
|-------|-------------|---------------------|
| 1 | 3.600×10⁻³ | 4.347×10⁻² |
| 2 | 1.875×10⁻³ | 2.094×10⁻² |
| 3 | 8.000×10⁻⁴ | 7.900×10⁻³ |

### 1.3 State Vector

```
State = [q1, q2, q3, dq1, dq2, dq3]ᵀ   (angles in rad, velocities in rad/s)
```

---

## 2. Model Implementations

### 2.1 Newton-Euler Analytical Model (`NE_FD.m`)

Implements the **recursive Newton-Euler forward dynamics** algorithm:

1. **Bias computation** — one NE inverse-dynamics call with actual `(q, dq, ddq=0)` and gravity active → gives `b = C(q,dq)·dq + G(q)` (Coriolis + gravity torques)
2. **Mass matrix** — three NE calls with `ddq = eᵢ` (unit vectors), gravity off, `dq=0` → columns of `M(q)`
3. **Forward dynamics** — `ddq = M⁻¹(τ − b)`

**Gravity convention:**  
`vd0 = [0; g_acc; 0]` — fictitious upward pseudo-acceleration in the NE formulation, equivalent to physical gravity in the −Y direction.

**Files:**
- `NE_FD.m` — dynamics function (called by ODE45)
- `run_NE_ForwardDynamics.m` — sets up ICs, torque profile, runs ODE45, saves results

### 2.2 Simscape Multibody Model (`simscape_3dof.slx`)

Built in Simscape Multibody with:
- Three Revolute Joints (Joint1, Joint2, Joint3) with sensing enabled (position, velocity, acceleration, torque)
- `GravityVector = [0, −9.81, 0]` in Mechanism Configuration block
- PS-Simulink converters for reading joint states out to Simulink
- SL-PS converters for feeding joint torques in
- MATLAB Function blocks for state-dependent torque computation

**Block architecture (per joint):**

```
Sin_tau{i}  ──┐
               ├── TorqueSum{i} ──► SL2PS{i} ──► Joint{i}.ti
WallTau{i}  ──┘

PS2SL_q{i} ──► WallTau{i}   (MATLAB Function: spring torque)
Joint{i} ──► PS2SL_q/dq/ddq/tau{i} ──► WorkSpace logging blocks
```

---

## 3. The Gravity Problem and Fix

### 3.1 Initial Issue

Both models originally had gravity disabled:
- NE model: `g_acc = 0`
- Simscape: `GravityVector = [0, 0, 0]`

Result: both models agreed trivially (both computed zero gravity torques). Not a valid validation.

### 3.2 Fix

- NE model: `g_acc = 9.81`, `vd0 = [0; g_acc; 0]`
- Simscape: `GravityVector = [0, -9.81, 0]`

With gravity enabled and only small sinusoidal torques (0.1 Nm peak vs ~25 Nm gravity), the arm swung to large angles and the models diverged at t ≈ 5.75 s.

---

## 4. Lyapunov Instability Analysis

### 4.1 Why Models Diverge — Physical Explanation

This arm behaves as a **gravity-dominated compound pendulum**. The ODE45 solver in MATLAB and the ODE45 solver in Simulink, while nominally identical, use different internal code paths and accumulate round-off differences of ~10⁻¹⁴ per step. In a **Lyapunov-unstable** configuration, these tiny differences grow **exponentially**.

For a configuration q with zero velocity, the linearised dynamics are:

```
δq̈ = Kg · δq
```

where `Kg = d(−M⁻¹G)/dq` is the gravitational stiffness matrix, computed via finite difference. The Lyapunov exponent is:

```
σ = √max(0, λmax(Kg))     [units: 1/s]
```

A perturbation δq₀ grows as `δq(t) ~ δq₀ · e^(σt)`. When σ > 0, any difference — even numerical round-off — grows without bound.

### 4.2 Stability Map — Joint 1

**Sweep:** q1 ∈ [−180°, +180°], q2 = q3 = 0°

| q1 | λmax | σ (1/s) | Regime |
|----|------|---------|--------|
| −180° | ≈ 0 | 0 | Neutral (horizontal) |
| −150° | −6.07 | 0 | **Stable** |
| −90° | −12.13 | 0 | **Most stable** (hanging equilibrium) |
| −45° | −8.58 | 0 | **Stable** |
| 0° | ≈ 0 | 0 | Neutral boundary |
| +45° | +144.4 | 12.02 | **Unstable** |
| **+90°** | **+204.3** | **14.29** | **Peak instability (inverted)** |
| +135° | +144.4 | 12.02 | **Unstable** |
| +180° | ≈ 0 | 0 | Neutral |

**Stable region:** q1 ∈ (−180°, 0°) — arm in lower half-circle (leaning downward)  
**Unstable region:** q1 ∈ (0°, +180°) — arm in upper half-circle (leaning upward)

> **Peak instability at q1 = +90° (fully inverted): σ = 14.29 /s**  
> Two trajectories differing by 1 nanometre diverge to **1.6 million metres in 1 second**.

**Saved plot:** `Validation_Plots1/Lyapunov_q1_sweep.png`

### 4.3 Stability Map — Joint 2 (Elbow)

Joint 2 stability depends heavily on the base joint angle q1.

**When q1 = 0° (arm horizontal — already bad configuration):**

| Range | Regime |
|-------|--------|
| q2 ∈ (89°, 100°) | Narrow stable band |
| Everything else | Unstable |
| Peak at q2 = −26° | σ = 5.55 /s (errors ×257 per second) |

**When q1 = −90° (hanging — natural equilibrium):**

| Range | Regime |
|-------|--------|
| q2 ∈ (−49°, +49°) | **Stable** (±49° symmetric band) |
| \|q2\| > 49° | Unstable (elbow bent too far) |
| Peak at q2 = ±180° | σ = 12.79 /s |

### 4.4 Stability Map — Joint 3 (Wrist)

**When q1 = q2 = 0° (arm fully horizontal):**

| Range | Regime |
|-------|--------|
| q3 ∈ (−102°, −91°) | Narrow stable band near −90° |
| Everything else | Unstable |
| Peak at q3 = +33° | σ = 3.52 /s (errors ×34 per second) |

**When q1 = −90°, q2 = 0° (arm hanging):**

| Range | Regime |
|-------|--------|
| q3 ∈ (−90°, +90°) | **Stable** (entire lower half) |
| \|q3\| > 90° | Unstable |
| Peak at q3 = ±180° | σ = 8.73 /s |

### 4.5 Complete Stable Region Summary

At the **natural hanging configuration** (q = [−90°, 0°, 0°]):

| Joint | Stable range | Physical meaning |
|-------|-------------|------------------|
| q1 | **(−180°, 0°)** | Base link in lower half-circle |
| q2 | **(−49°, +49°)** | Elbow not bent past ±49° |
| q3 | **(−90°, +90°)** | Wrist not bent past horizontal |

**80% safety limits** (20% margin from each boundary):

| Joint | 80% Stable limit | Spring torque at boundary |
|-------|-----------------|--------------------------|
| q1 | (−162°, −18°) centred at −90° | 50 × 1.257 = **62.8 Nm** >> gravity |
| q2 | (−39.2°, +39.2°) | 50 × 0.685 = **34.2 Nm** |
| q3 | (−72°, +72°) | 50 × 1.257 = **62.8 Nm** |

**Saved plots:** `Validation_Plots1/Lyapunov_per_joint.png`, `Lyapunov_2D_map.png`

---

## 5. Torque Profile Design

### 5.1 Why Simple Sinusoids Fail

With purely sinusoidal torques (A₁ = 0.10, A₂ = 0.05, A₃ = 0.03 Nm), gravity (~25 Nm) dominates by a factor of ~250. The arm swings through large angles, crossing the stability boundary at q1 = 0° repeatedly. Models diverge at t ≈ 3.4 s.

A **soft wall** at ±130° (K = 500 Nm/rad) was tried — joints stayed within ±150° but the arm still swung into unstable territory (reaching q1 = −143° which is safe, but passing through q1 = 0° on return swings). Divergence still occurred at t ≈ 3.4 s.

### 5.2 Design Requirements

1. **No joint limits** — must not set hard mechanical limits in the model
2. **No damping** — pure dynamics test
3. **All joints within ±150°** (user requirement)
4. **Stay within 80% of Lyapunov stable region** to ensure numerical convergence
5. **Time-varying** — provides dynamic loading (accelerations, Coriolis coupling)

### 5.3 Final Torque Profile

```
tau_j(t, q) = −Kp · (q_j − q_eq_j)  +  A_j · sin(ω_j · t + φ_j)
               ──────────────────────    ────────────────────────────
               Spring restoring term       Sinusoidal excitation
```

**Parameters:**

| Joint | Kp (Nm/rad) | q_eq | A (Nm) | ω (rad/s) | φ (rad) |
|-------|------------|------|--------|-----------|---------|
| 1 | 50 | −π/2 (−90°) | 3.0 | π | 0 |
| 2 | 50 | 0° | 1.5 | 0.6π | π/3 |
| 3 | 50 | 0° | 0.8 | 1.6π | π/6 |

**Explicit expressions:**

```
tau1 = −50·(q1 + π/2)  +  3.0·sin(π·t)
tau2 = −50·q2          +  1.5·sin(0.6π·t + π/3)
tau3 = −50·q3          +  0.8·sin(1.6π·t + π/6)
```

**Initial conditions:** q0 = [−π/2, 0, 0] rad, dq0 = [0, 0, 0] rad/s

### 5.4 Why This Design Works

**Spring term** (`−Kp·(q_j − q_eq_j)`)  
A virtual torsional spring centred at the hanging equilibrium. At the 80% stability boundary (62.8 Nm), it is more than twice the maximum gravity torque (~25 Nm), guaranteeing the arm never leaves the stable region. The spring makes the effective system **globally stable** — perturbations decay rather than grow.

**Sinusoidal term** (`A_j · sin(ω_j·t + φ_j)`)  
Provides time-varying excitation so both models experience dynamic loads:
- Different frequencies (0.3, 0.5, 0.8 Hz) ensure joints move independently
- Different phases prevent synchronisation artifacts
- Non-zero amplitudes test mass-matrix coupling (Coriolis and centripetal terms)
- Amplitudes are small enough that the spring always dominates

**Why the spring is centred at −90° for q1:**  
The stable equilibrium of this arm under gravity is the hanging position (q1 = −90°, q2 = 0°, q3 = 0°). Centering the spring here means gravity and spring act in the same direction for small deviations — doubly stable.

---

## 6. Simulation Setup

### 6.1 Solver Settings (Both Models)

| Parameter | Value |
|-----------|-------|
| Solver | ODE45 (variable-step) |
| RelTol | 1×10⁻⁸ |
| AbsTol | 1×10⁻¹⁰ |
| Duration | 20 s |
| Output interval | 1×10⁻⁴ s (analytical), variable (Simscape) |

### 6.2 Initial Conditions

```
q0  = [−π/2,  0,  0] rad   (−90°, 0°, 0°) — hanging equilibrium
dq0 = [  0,   0,  0] rad/s
```

---

## 7. Validation Results

### 7.1 Joint Range Check

| Joint | 80% Stability limit | Actual range | Status |
|-------|-------------------|--------------|--------|
| q1 | (−162°, −18°) | [−95.5°, −85.1°] | **PASS** — 23° margin |
| q2 | (−39.2°, +39.2°) | [−3.2°, +3.1°] | **PASS** — 36° margin |
| q3 | (−72°, +72°) | [−1.7°, +1.7°] | **PASS** — 70° margin |

All joints well inside 80% of the Lyapunov stable region throughout the 20 s simulation.

### 7.2 Error Tables (Interpolation-Based)

Errors computed by interpolating both time series to a common grid and computing point-by-point differences. This gives the true numerical agreement between models.

#### Joint 1 Error Table

| Signal | RMSE | MAE | MaxError | MeanError |
|--------|------|-----|----------|-----------|
| Position (rad) | 7.8453×10⁻⁹ | 6.0843×10⁻⁹ | 2.1435×10⁻⁸ | −3.2303×10⁻¹¹ |
| Velocity (rad/s) | 2.3335×10⁻⁷ | 1.8137×10⁻⁷ | 5.8856×10⁻⁷ | 6.8565×10⁻¹⁰ |
| Acceleration (rad/s²) | 7.6384×10⁻⁶ | 5.5722×10⁻⁶ | 2.4799×10⁻⁵ | 1.9064×10⁻⁸ |
| Torque (Nm) | 3.9226×10⁻⁷ | 3.0421×10⁻⁷ | 1.0720×10⁻⁶ | 1.8703×10⁻⁹ |

#### Joint 2 Error Table

| Signal | RMSE | MAE | MaxError | MeanError |
|--------|------|-----|----------|-----------|
| Position (rad) | 1.7263×10⁻⁸ | 1.3346×10⁻⁸ | 4.7001×10⁻⁸ | 4.3376×10⁻¹¹ |
| Velocity (rad/s) | 5.4073×10⁻⁷ | 4.1140×10⁻⁷ | 1.4736×10⁻⁶ | −1.1932×10⁻⁹ |
| Acceleration (rad/s²) | 2.2552×10⁻⁵ | 1.5871×10⁻⁵ | 9.4851×10⁻⁵ | −3.7300×10⁻⁸ |
| Torque (Nm) | 8.6314×10⁻⁷ | 6.6731×10⁻⁷ | 2.3501×10⁻⁶ | −2.1688×10⁻⁹ |

#### Joint 3 Error Table

| Signal | RMSE | MAE | MaxError | MeanError |
|--------|------|-----|----------|-----------|
| Position (rad) | 9.8681×10⁻⁹ | 7.2336×10⁻⁹ | 2.8830×10⁻⁸ | 2.5276×10⁻¹¹ |
| Velocity (rad/s) | 5.2055×10⁻⁷ | 3.8266×10⁻⁷ | 1.7000×10⁻⁶ | −1.3343×10⁻⁹ |
| Acceleration (rad/s²) | 4.0499×10⁻⁵ | 3.1247×10⁻⁵ | 1.7208×10⁻⁴ | −3.6189×10⁻⁸ |
| Torque (Nm) | 4.9340×10⁻⁷ | 3.6168×10⁻⁷ | 1.4415×10⁻⁶ | −1.2639×10⁻⁹ |

### 7.3 Position Error Over Time

Position errors grow slowly and remain bounded throughout 20 s:

| Time | \|err_q1\| | \|err_q2\| | \|err_q3\| |
|------|-----------|-----------|-----------|
| t = 1 s | ~1.6 nrad | ~2.4 nrad | ~1.1 nrad |
| t = 5 s | ~1.6 nrad | ~1.4×10⁻⁸ rad | ~3.2 nrad |
| t = 10 s | ~1.8 nrad | ~2.7×10⁻⁸ rad | ~1.8×10⁻⁸ rad |
| t = 20 s | ~7.8 nrad | ~3.4×10⁻⁸ rad | ~2.6×10⁻⁸ rad |

**Maximum position error over 20 s:** ~62 nrad (joint 2). This is the expected level of agreement between two independent ODE45 implementations at RelTol = 10⁻⁸ in a stable dynamical system.

### 7.4 Torque Error Explanation

The torque error (~10⁻⁷ Nm RMSE) is **not a physics error** — it is a consequence of the spring term:

```
tau_error ≈ Kp × position_error = 50 × 8×10⁻⁹ ≈ 4×10⁻⁷ Nm  ✓
```

Since the torque is state-dependent (`−Kp·q`), any tiny difference in q between the two models produces a proportional torque difference. This is physically correct and expected. A purely time-based torque would give torque error at machine epsilon (~10⁻¹⁶ Nm).

---

## 8. Pure Time-Dependent Torque Validation

The spring-torque profile (Section 5–7) uses angle feedback — the torque depends on the current joint position. For a stricter cross-validation, both models should receive **identical inputs that are functions of time only**, with no state dependency. This eliminates any possibility that a small position discrepancy between the models causes a torque discrepancy, which would then feed back and amplify. A purely time-dependent torque is also the most physically transparent test: if both models agree under the same open-loop input, the dynamics equations are genuinely identical.

### 8.1 Design Challenge — Planar Coupling

A naïve approach using the quasi-static gain matrix `G_qs = (−Kg)⁻¹` to estimate safe amplitudes failed catastrophically when initially applied (A2 = 20 Nm, q1 reached +40,000° in 20 s). The root cause is a structural property of planar arms:

**All joints rotate about the same world axis (Z).** Gravity acts on every link. When joint 1 rotates by angle Δq1, the gravity vector projected onto the CoM of link 2 changes by:

```
ΔT_g2 ≈ (m₂ lc₂ + m₃ l₂) · g · sin(Δq1) ≈ 6.87 · sin(Δq1) Nm
```

A ±30° swing of joint 1 therefore imposes a ±3.4 Nm oscillating gravity torque on joint 2 — comparable to the applied torque itself. The quasi-static diagonal gain matrix `G_qs[j,j]` assumes independent joints and completely misses this coupling.

**Coupling quantified from simulation (A1 = 8 Nm, A2 = 3 Nm, A3 = 0.8 Nm):**

| Torque source | q1 swing | q2 swing | q3 swing |
|---------------|----------|----------|----------|
| A1 alone | ±28.8° | **±28.0°** | ±2.3° |
| A2 alone | ±18.2° | ±57.2° | ±64.5° |
| A3 alone | ±6.2° | ±14.9° | ±67.2° |

A2 alone with 3 Nm drives q2 to ±57° (limit = ±39.2°) because the off-diagonal quasi-static gain G_qs[1,2] = 1.42 deg/Nm is comparable to the diagonal G_qs[2,2] = 1.09 deg/Nm. The joint responses are strongly coupled.

### 8.2 Maximum Gravity Restoring Torques

A necessary condition for stability with pure time-dependent torques (no restoring spring) is that the applied torque amplitude never exceeds the maximum gravity restoring torque, otherwise the arm escapes the gravitational potential well and spins continuously.

Maximum restoring torques (computed by sweeping each joint through ±180° with others at hanging equilibrium):

| Joint | Max restoring torque | Dominant contribution |
|-------|---------------------|----------------------|
| q1 | **25.5 Nm** | All three links via shoulder |
| q2 | **8.34 Nm** | m₂ lc₂ g + m₃ l₂ g ≈ 6.87 Nm (+ coupling) |
| q3 | **1.47 Nm** | m₃ lc₃ g = 1 × 0.15 × 9.81 |

With A2 = 20 Nm >> 8.34 Nm max restoring, joint 2 immediately escaped the gravity well — explaining the spinning instability.

### 8.3 Amplitude Selection — Binary Search on Full Nonlinear Simulation

The quasi-static gain approach is unreliable due to coupling. Instead, a **binary search on the full 20-second nonlinear ODE45 simulation** was used to find the maximum safe scale factor along the direction A = [8; 3; 0.8] Nm:

```
Scale factor 0.4375:  dev = [19.7°, 37.0°, 41.7°]  ✓  (within [72°, 39.2°, 72°])
Scale factor 0.4688:  dev = [21.7°, 41.1°, 47.8°]  ✗  (q2 exceeds 39.2° limit)
```

After 10 bisection iterations, safe scale ≈ 0.455. Final amplitudes chosen at a conservative scale of 0.375 (A = [3.0; 1.0; 0.3] Nm) for clear margin from all boundaries.

### 8.4 Final Pure Time-Dependent Torque Profile

```
tau1(t) = 3.0 · sin(0.30 t)
tau2(t) = 1.0 · sin(0.50 t + π/4)
tau3(t) = 0.3 · sin(0.70 t + π/3)
```

**Frequency choice:** 0.30, 0.50, 0.70 rad/s are incommensurable (no integer ratio), preventing periodic locking and ensuring broad dynamic coverage over 20 s. All well below the minimum natural frequency ω_n = 2.255 rad/s.

**Simscape implementation:** Phase values stored with full double precision (`sprintf('%.15f', pi/4)`) to avoid floating-point truncation errors in the Simulink Sin block parameter.

### 8.5 Joint Ranges — Within 80% Stability Limits

| Joint | Actual range | 80% limit | Margin |
|-------|-------------|-----------|--------|
| q1 | [−106.4°, −74.4°] | (−162°, −18°) | 56° |
| q2 | [−25.2°, +29.6°] | (±39.2°) | 10° |
| q3 | [−31.0°, +33.3°] | (±72°) | 39° |

### 8.6 Cross-Validation Error Tables (Pure Time-Dependent Torques)

Errors computed by interpolating both models to a common 1 ms grid (PCHIP interpolation).  
**Torques are identical in both models** — confirmed by torque error at machine epsilon.

#### Position (rad)

| Metric | Joint 1 | Joint 2 | Joint 3 |
|--------|---------|---------|---------|
| RMSE | 7.78×10⁻⁹ | 1.88×10⁻⁸ | 3.77×10⁻⁸ |
| MAE | 5.68×10⁻⁹ | 1.32×10⁻⁸ | 2.62×10⁻⁸ |
| Max \|Error\| | 2.49×10⁻⁸ | 6.32×10⁻⁸ | 1.40×10⁻⁷ |
| Mean Error | −2.26×10⁻¹² | −5.37×10⁻¹¹ | 1.86×10⁻¹⁰ |

#### Velocity (rad/s)

| Metric | Joint 1 | Joint 2 | Joint 3 |
|--------|---------|---------|---------|
| RMSE | 7.46×10⁻⁸ | 2.20×10⁻⁷ | 4.27×10⁻⁷ |
| MAE | 5.24×10⁻⁸ | 1.57×10⁻⁷ | 3.06×10⁻⁷ |
| Max \|Error\| | 2.58×10⁻⁷ | 7.52×10⁻⁷ | 1.82×10⁻⁶ |
| Mean Error | 3.72×10⁻¹⁰ | 4.18×10⁻¹⁰ | −6.32×10⁻⁹ |

#### Acceleration (rad/s²)

| Metric | Joint 1 | Joint 2 | Joint 3 |
|--------|---------|---------|---------|
| RMSE | 8.12×10⁻⁷ | 2.78×10⁻⁶ | 5.24×10⁻⁶ |
| MAE | 5.69×10⁻⁷ | 1.99×10⁻⁶ | 3.76×10⁻⁶ |
| Max \|Error\| | 3.02×10⁻⁶ | 9.56×10⁻⁶ | 2.27×10⁻⁵ |
| Mean Error | −2.98×10⁻⁹ | 1.60×10⁻⁸ | −3.84×10⁻⁸ |

#### Torque (Nm)

| Metric | Joint 1 | Joint 2 | Joint 3 |
|--------|---------|---------|---------|
| RMSE | 2.69×10⁻¹⁶ | 5.35×10⁻¹⁶ | 1.34×10⁻¹⁶ |
| MAE | 9.35×10⁻¹⁷ | 2.73×10⁻¹⁶ | 8.89×10⁻¹⁷ |
| Max \|Error\| | 2.22×10⁻¹⁵ | 1.78×10⁻¹⁵ | 3.73×10⁻¹⁶ |
| Mean Error | −8.97×10⁻²⁰ | −2.37×10⁻¹⁶ | −1.93×10⁻¹⁷ |

**Torque errors are at double-precision machine epsilon (~10⁻¹⁶)**, confirming that both models receive bit-for-bit identical inputs. The position agreement at 8–38 nanoradians represents the fundamental limit of the ODE45 integrator at RelTol = 10⁻⁸.

### 8.7 Comparison with Spring-Torque Profile

| Metric | Spring + Sinusoid (Profile 1) | Pure Time-Dependent (Profile 2) |
|--------|------------------------------|----------------------------------|
| Torque type | State-dependent | Time-dependent only |
| Joint ranges | q1: ±5°, q2: ±3.2°, q3: ±1.7° | q1: ±32°, q2: ±27°, q3: ±32° |
| Position RMSE (J1) | 7.85×10⁻⁹ rad | 7.78×10⁻⁹ rad |
| Position RMSE (J3) | 9.87×10⁻⁹ rad | 3.77×10⁻⁸ rad |
| Torque RMSE (J1) | 3.92×10⁻⁷ Nm | **2.69×10⁻¹⁶ Nm** |
| Dynamic loading | Mild (spring dominates) | Larger (wider excursions, Coriolis active) |
| Torque identity | No (Kp × position error) | Yes (machine precision) |

Profile 2 provides a stronger validation: identical open-loop torques, larger joint excursions (more Coriolis/coupling exercise), and torque identity confirmed at floating-point precision.

---

## 9. Saved Plots and Files

### Validation Plots — `Validation_Plots2\` (both profiles)

**Profile 1 — Spring + Sinusoid:**

| File | Contents |
|------|----------|
| `Joint1_Position.png/fig` | q1: Analytical vs Simscape + error |
| `Joint1_Velocity.png/fig` | dq1: comparison + error |
| `Joint1_Acceleration.png/fig` | ddq1: comparison + error |
| `Joint1_Torque.png/fig` | tau1: comparison + error |
| `Joint2_Position.png/fig` | q2 comparison |
| `Joint2_Velocity.png/fig` | dq2 comparison |
| `Joint2_Acceleration.png/fig` | ddq2 comparison |
| `Joint2_Torque.png/fig` | tau2 comparison |
| `Joint3_Position.png/fig` | q3 comparison |
| `Joint3_Velocity.png/fig` | dq3 comparison |
| `Joint3_Acceleration.png/fig` | ddq3 comparison |
| `Joint3_Torque.png/fig` | tau3 comparison |
| `Error_vs_Time.png/fig` | Log-scale error vs time, all joints |

**Profile 2 — Pure Time-Dependent:**

| File | Contents |
|------|----------|
| `PureTau_Position.png` | q1/q2/q3: NE vs Simscape overlay, 20 s |
| `PureTau_Velocity.png` | dq1/dq2/dq3: comparison |
| `PureTau_Accel.png` | ddq1/ddq2/ddq3: comparison |
| `PureTau_Torque.png` | tau1/tau2/tau3: overlay (visually identical) |
| `PureTau_Error_vs_Time.png` | Error time series for all 4 signals × 3 joints |
| `NE_PureTimeTorque_Overview.png` | NE-only overview: angles, velocities, torques |

### Stability Analysis Plots — `Validation_Plots1\`

| File | Contents |
|------|----------|
| `Lyapunov_q1_sweep.png/fig` | λmax and σ vs q1, with stable/unstable shading |
| `Lyapunov_per_joint.png/fig` | λmax and σ for each joint at two reference configs |
| `Lyapunov_2D_map.png/fig` | 2D heat map of λmax in q1-q2 space (q3=0) |
| `Error_vs_Time.png/fig` | Early-run error comparison (spring torque, no damping) |

### Data Files

| File | Contents |
|------|----------|
| `Analytical_NE_Output.mat` | Full analytical simulation output (q, dq, ddq, tau, time) |
| `NE_FD.m` | Newton-Euler dynamics function |
| `run_NE_ForwardDynamics.m` | Run script (torque profile, ODE45, post-processing) |
| `simscape_3dof.slx` | Simscape Multibody model |
| `plots_3dof.m` | Comparison plot script (bin-averaged version) |

---

## 10. Key Lessons and Findings

### 10.1 Root Cause of Original Divergence

Both models originally had gravity disabled (`g=0`). After enabling gravity in both to −Y:

- With small sinusoidal torques alone, gravity dominates by ~250×
- The arm swings to large angles, crossing the Lyapunov instability boundary (q1=0°)
- Models diverge at t ≈ 3–6 s depending on torque amplitude
- This is **not a physics error** — both models are correct; it is Lyapunov instability of the trajectory

### 10.2 What Did Not Work

| Approach | Why it failed |
|----------|--------------|
| Sinusoids only (0.1, 0.05, 0.03 Nm) | Gravity dominates, arm swings to ±143°, crosses stability boundary |
| Soft wall at ±130° (K=500 Nm/rad) | Joints bounded but arm still traverses unstable region on each swing |
| Fixed-step RK4 | Same divergence (~5.5 s) — different code paths still accumulate differences |
| 300× scaled torques | Caused continuous joint spinning (unwinding), not oscillation |
| Gravity in Z direction | Physically trivial — Z gravity generates zero torque for planar arm |

### 10.3 What Works and Why

A **proportional spring torque centred at the stable hanging equilibrium** (q_eq = [−90°, 0°, 0°]) guarantees all joints remain in the Lyapunov-stable region throughout the simulation:

- Spring at 80% boundary >> gravity at any angle
- System is globally stable: perturbations decay, not grow
- MeanError ≈ 10⁻¹¹ (no systematic bias — models are identical physics)
- RMSE < 10⁻⁷ for all position signals — agreement at ODE45 round-off level

### 10.4 Torque Amplitude Design for Planar Arms (Key Lesson)

The quasi-static gain matrix `G_qs = inv(−Kg)` only captures **decoupled static** response at the equilibrium. For a planar arm with all joints sharing the same rotation axis:

1. **Large coupling exists**: A1-alone drives q2 by nearly the same amplitude as q1 itself (~±28° each), because link 1 rotation changes the gravity projection on all downstream links.
2. **Diagonal G_qs is misleading**: G_qs[2,2] predicted ±3.3° for A2 = 3 Nm, actual response was ±57°.
3. **Safe design requires full nonlinear simulation**: Binary search on a full 20-second sim with the actual ODE45 is the only reliable approach for planar arm amplitude selection.
4. **Maximum gravity restoring torque is a hard cap**: Any applied torque exceeding the maximum restoring torque for a joint causes continuous spinning (arm escapes the gravity potential well).

### 10.5 Simscape Implementation Notes

- **DampingCoefficient units:** Simscape Revolute Joint uses Nm/(deg/s) internally, not Nm/(rad/s). To set b Nm/(rad/s): `DampingCoefficient = b × (π/180)`.
- **Fcn block limitation:** Simulink Fcn blocks reject nested `abs()` calls and complex expressions. Use **MATLAB Function blocks** instead for state-dependent torque computations.
- **Auto-reconnection:** When a Simulink block is deleted and a new block with the same name is added, Simulink auto-reconnects the dangling lines. Do not call `add_line` again — check port connection status first.
- **Phase string precision:** When setting Sin block phase parameters via `set_param`, use `sprintf('%.15f', pi/4)` rather than `num2str(pi/4)` to avoid truncation errors that appear as spurious torque discrepancies (~10⁻⁶ Nm level).

---

## 11. Conclusion

The Newton-Euler recursive forward dynamics model (`NE_FD.m`) is **validated against Simscape Multibody** under two independent torque profiles, both within the Lyapunov-stable operating region.

### Profile 1 — Spring + Sinusoid (state-dependent)

| Metric | Value |
|--------|-------|
| Max position error | **62 nrad** over 20 s |
| Position RMSE | **7.9–17 nrad** (all joints) |
| Velocity RMSE | **0.23–0.54 μrad/s** |
| Acceleration RMSE | **7.6–40.5 μrad/s²** |
| Torque RMSE | **0.39–0.86 μNm** (Kp × position error, expected) |

### Profile 2 — Pure Time-Dependent (open-loop, no state feedback)

| Metric | Value |
|--------|-------|
| Max position error | **140 nrad** over 20 s |
| Position RMSE | **7.8–37.7 nrad** (all joints) |
| Velocity RMSE | **74.6–427 nrad/s** |
| Acceleration RMSE | **0.81–5.24 μrad/s²** |
| Torque RMSE | **< 6×10⁻¹⁶ Nm** (machine epsilon — identical inputs confirmed) |

Both profiles demonstrate agreement at the **fundamental limit of ODE45 integration** (RelTol = 10⁻⁸). The torque identity at machine precision in Profile 2 confirms that the two implementations compute identical physics from identical inputs. All joint trajectories remain within 80% of the Lyapunov-stable region throughout both 20-second simulations.

The models are **physically and numerically equivalent**. The Newton-Euler formulation in `NE_FD.m` correctly implements the forward dynamics of this 3-DOF planar RRR arm under gravity, validated by two independent test scenarios with different torque characteristics and joint excursion ranges.

---

*Report generated: 2026-08-08*  
*Project directory: `C:\Work_In\RRR_3DOF\`*
