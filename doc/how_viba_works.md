# How Structureless VI-BA Works (slide content)

**Subtitle:**
*A joint optimization that refines all keyframe states using IMU and visual geometry —
with no 3D point reconstruction.*

## The core idea
It is a **bundle adjustment** over a sliding window of keyframes. It minimizes the combined
error of two constraint types at once:

```
min   Σ ‖ IMU factor ‖²   +   Σ  ρ_Huber( ‖ epipolar factor ‖² )
```

## The pieces

**1. The state (what it solves for)**
Each keyframe has: **position, velocity, orientation, gyro bias, accel bias.**
→ **No 3D feature points** in the state → *"structureless"* → low-dimensional, fast.

**2. IMU factors** *(between consecutive keyframes)*
Link each keyframe pair to the **measured inertial motion** (preintegrated accelerometer +
gyroscope).
→ provide **scale, gravity, velocity, and biases**.

**3. Epipolar (coplanarity) factors** *(between keyframes sharing a feature)*
For two views of the same feature, the viewing rays and the camera baseline must lie in
**one plane**.
→ enforces **visual geometric consistency** — and it is **scale-invariant**, so no 3D point or
depth is ever needed.

**4. The solve**
A nonlinear least-squares solver (Ceres) adjusts all keyframe states together until the total
weighted error is smallest → **refined initial states**.

## Why "structureless"?
Classic bundle adjustment reconstructs 3D points and reprojects them. Here, vision enters
through **epipolar geometry** instead — so there are **no landmarks to triangulate**, making the
optimization smaller and faster.

## Suggested visual (factor-graph sketch)
```
  KF1 ──IMU── KF2 ──IMU── KF3 ──IMU── KF4
   └────── epipolar (shared feature) ──────┘
```
- **Keyframe states** = position, velocity, orientation, biases
- **IMU factors** connect neighbors
- **Epipolar factors** connect any keyframes seeing the same feature
- Caption: *"no 3D points — vision enters via epipolar geometry."*

(Mirrors the paper's Fig. 2 "structureless" factor graph.)
