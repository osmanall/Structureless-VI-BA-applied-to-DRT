# PPT Plan — What We Did, the Code We Wrote (file + snippet + why)

A slide-by-slide plan. Code slides 3–8 share one layout:
**file path → short snippet → What / Why / Solved.**

---

## Slide 1 — What We Did (overview)
- Applied the paper's **structureless VI-BA** as a refinement step after DRT initialization.
- Wrote **3 new files/components** + reused **1** existing IMU factor, wired into DRT's `process()`.
- Verified each piece numerically, then benchmarked on EuRoC Machine Hall.

Files we touched:
- NEW `include/factor/poseLocalParam.h`
- NEW `include/factor/epipolarFactor.h`
- `src/initMethod/drtVioInit.cpp` (new function `structurelessVIBA()`)
- `src/initMethod/drtLooselyCoupled.cpp` (call site)
- `app/main.cpp` (evaluation metrics)
- reused `include/factor/imuIntegFactor.h`

---

## Slide 2 — What VI-BA Optimizes
- Joint optimization over keyframe states `[position, velocity, orientation, biases]` — **no 3D points**.
- Two constraint types: **IMU factors** (motion) + **epipolar factors** (visual geometry).
- Cost function (paper Eq. 11):
```
min   Σ ‖ IMU factor ‖²   +   Σ  ρ_Huber( ‖ epipolar factor ‖² )
```

---

## Slide 3 — Gravity Alignment
**File:** `src/initMethod/drtVioInit.cpp` — top of `structurelessVIBA()`
```cpp
R_align_ = Utility::g2R(gravity);           // align DRT's tilted frame to gravity-up
for (i) { rotation[i] = R_align_ * rotation[i]; position[i] = R_align_ * position[i]; ... }
// ... optimize ...  then un-align:  rotation[i] = R_align_.transpose() * rotation[i];
```
- **What:** rotate all states into a gravity-aligned frame, then back after solving.
- **Why:** the IMU factor assumes gravity points straight down (−z); DRT leaves the frame tilted.
- **Solved:** made the reused IMU factor valid — without it the IMU residuals are wrong and the solve diverges.

---

## Slide 4 — Pose Parameterization (SO3 × R³)
**File:** `include/factor/poseLocalParam.h` (NEW) — `PoseLocalParameterization::Plus`
```cpp
R_new = R * exp(dphi);      // rotation: TURN it on the manifold (not add)
p_new = p + R * dp;         // position: body-frame update
// ComputeJacobian returns identity  -> factor Jacobians pass through unchanged
```
- **What:** a Ceres `LocalParameterization` that updates a pose block `[log(R), p]`.
- **Why:** rotations live on a curved manifold — you can't optimize them by plain addition.
- **Solved:** correct rotation optimization, *and* matched the IMU factor's convention so we could reuse it unchanged.

---

## Slide 5 — Epipolar Factor (residual)
**File:** `include/factor/epipolarFactor.h` (NEW) — `EpipolarFactor::Evaluate`
```cpp
A = R_j * Rbc * z_j;   B = R_i * Rbc * z_i;    // bearings in world frame   (Eq.14)
t = (p_i + R_i*pbc) - (p_j + R_j*pbc);         // baseline between cameras   (Eq.13)
C = t / ||t||;                                  // scale-invariant
residual = A . (C x B);                         // coplanarity: rays must meet (Eq.13)
```
- **What:** a new analytic Ceres cost function for the visual coplanarity constraint.
- **Why:** it's the paper's core visual measurement — two views of a feature must be geometrically consistent, with **no 3D point**.
- **Solved:** brings vision into the optimization *structurelessly*; it's what refines the trajectory shape.

---

## Slide 6 — Epipolar Factor (Jacobians)
**File:** `include/factor/epipolarFactor.h` (NEW) — same `Evaluate`
```cpp
dr_dA = (Cx*B)^T;  dr_dB = A^T*Cx;  dr_dC = -A^T*Bx;      // Eq.15
dC_dt = (I - C*C^T)/||t||;   dr_dt = dr_dC * dC_dt;        // Eq.18
J_i(0,0) = dr_dB*(-R_i*hat(Rbc*z_i)) + dr_dt*(-R_i*hat(pbc)); // Eq.16-17,19
J_i(0,3) = dr_dt * R_i;                                    // body-frame position
```
- **What:** the analytic derivatives of the residual w.r.t. the two poses (paper Eq. 15–19).
- **Why:** to reuse the IMU factor we must report Jacobians analytically in tangent space (autodiff wouldn't match).
- **Solved:** exact, fast gradients that let Ceres converge; verified against a numerical check.

---

## Slide 7 — The Joint Optimization
**File:** `src/initMethod/drtVioInit.cpp` — `structurelessVIBA()`
```cpp
for (i) { AddParameterBlock(pose[i], PoseLocalParam);  AddParameterBlock(speed_bias[i]); }
SetParameterBlockConstant(pose[0]);                          // fix the gauge
for (i)     Add(ImuIntegFactor, pose[i], sb[i], pose[i+1], sb[i+1]);   // Eq.11 IMU term
for (pairs) Add(EpipolarFactor, pose[i], pose[j]);                     // Eq.11 vision term
Solve(problem);                                             // minimize the cost
```
Called from `src/initMethod/drtLooselyCoupled.cpp`, at the end of `process()`:
```cpp
if (linearAlignment()) { structurelessVIBA(); return true; }
```
- **What:** builds the Ceres problem, adds all factors, solves, writes states back.
- **Why:** to assemble and run the paper's optimization (Eq. 11) on DRT's output.
- **Solved:** turns DRT's rough guess into a refined solution — the whole refinement lives here.

---

## Slide 8 — Reused: IMU Factor
**File:** `include/factor/imuIntegFactor.h` (existing — unchanged)
```cpp
// residual = predicted motion (from state)  -  measured motion (IMU preintegration)
ep = Ri^T (Pj - Pi - Vi*dt - 0.5*g*dt^2) - dP_measured;      // Eq.5-7
```
- **What:** DRT's existing IMU preintegration factor, reused as-is.
- **Why:** provides the metric backbone — scale, gravity, velocity, biases.
- **Solved:** avoided rewriting IMU math; reuse drove our parameterization + analytic-Jacobian choices.

---

## Slide 9 — Verification (making sure it's correct)
- **Epipolar Jacobian:** analytic vs numerical (through the manifold) → matched to machine precision (0.0 error).
- **Cross-check:** residual + rotation Jacobians agreed with an independent reference implementation.
- **IMU-only test:** with vision off, converged to ~zero cost → frame conventions & wiring correct.

---

## Slide 10 — What It Achieved
- Refines DRT's initialization jointly and nonlinearly, using visual geometry DRT ignores.
- Consistent improvement on EuRoC Machine Hall (trajectory / velocity).
- Real-time (~42 ms), no 3D reconstruction.

---

### Build tips
- Slides 3–8 use one template: **file path (small, top) → snippet (monospace box) → What / Why / Solved (three short lines)**.
- Keep snippets to 3–5 lines — the essence, not the full function.
- Slide 9 (verification) shows you wrote *correct* code, not just code.
