# Structureless VI-BA — Code Explained Line by Line

Line-by-line explanation of the three components we wrote, with references to the
equations in Song et al. (2025), arXiv:2502.16598.

---

## 1. `include/factor/poseLocalParam.h` — Pose Parameterization

**Purpose:** tells Ceres how to update a pose (rotation + position) during optimization,
because rotations cannot be updated by plain addition.

```cpp
#ifndef DRT_POSE_LOCAL_PARAM_H
#define DRT_POSE_LOCAL_PARAM_H
```
Include guard — prevents the header being compiled twice.

```cpp
#include <ceres/ceres.h>              // ceres::LocalParameterization base class
#include <Eigen/Core>                 // Eigen vectors / Map
#include "utils/sophusExtUtils.hpp"   // Sophus::SO3d (same rotation type the IMU factor uses)
```

```cpp
class PoseLocalParameterization : public ceres::LocalParameterization {
```
Inherits Ceres's `LocalParameterization` — Ceres calls its methods to update a
parameter block on a manifold.

```cpp
virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const {
```
`Plus` = the update rule. Ceres passes the current pose `x`, a small correction `delta`,
and wants the new pose in `x_plus_delta`. **Corresponds to the paper's ⊞ operator (Eq. 2):
`q ⊞ δ = q ⊗ exp(δ/2)`** — the on-manifold orientation update. We implement the SO3 equivalent.

```cpp
    Eigen::Map<const Eigen::Vector3d> omega(x);       // current rotation vector = log(R)
    Eigen::Map<const Eigen::Vector3d> p(x + 3);       // current position
    Eigen::Map<const Eigen::Vector3d> dphi(delta);    // rotation correction (tangent)
    Eigen::Map<const Eigen::Vector3d> dp(delta + 3);  // position correction (tangent)
```
`Map` reinterprets the raw 6-number arrays as Eigen 3-vectors without copying. The pose
block is `[ω, p]` where `ω = log(R)` is the axis-angle form of the rotation (the tangent
vector `δ` in Eq. 2).

```cpp
    Sophus::SO3d R     = Sophus::SO3d::exp(omega);       // rebuild rotation from ω
    Sophus::SO3d R_new = R * Sophus::SO3d::exp(dphi);    // RIGHT-multiply: R·exp(δφ)
```
`exp(ω)` maps the vector back to a rotation. `R * exp(dphi)` is the **manifold update —
"turn," not "add."** SO3 form of the paper's `q ⊗ exp(δ/2)` (Eq. 2); the ½ in the quaternion
form is absorbed by SO3's `exp`.

```cpp
    Eigen::Map<Eigen::Vector3d> omega_new(x_plus_delta);
    Eigen::Map<Eigen::Vector3d> p_new(x_plus_delta + 3);
    omega_new = R_new.log();     // store new rotation back as a 3-vector
    p_new     = p + R * dp;      // position update in the BODY frame
```
`R_new.log()` converts the updated rotation back to `ω`. `p + R*dp` updates position — the
`R*` makes the correction **body-frame**, matching the convention baked into the reused
`ImuIntegFactor`'s Jacobians (an implementation-consistency requirement, not a paper equation).

```cpp
    return true;
}
```

```cpp
virtual bool ComputeJacobian(const double* /*x*/, double* jacobian) const {
    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> J(jacobian);
    J.setIdentity();
    return true;
}
```
Returns the **6×6 identity** ("VINS-Mono trick"): the factors report their Jacobians already
in tangent space, so identity makes Ceres pass them through unchanged — this is what lets us
reuse `ImuIntegFactor` without editing it.

```cpp
    virtual int GlobalSize() const { return 6; }   // stored pose = [ω(3), p(3)]
    virtual int LocalSize()  const { return 6; }   // correction  = [δφ(3), δp(3)]
};
```
Both 6 — rotation has 3 real DOF, so no over-parameterization.

---

## 2. `include/factor/epipolarFactor.h` — Epipolar Factor

**Purpose:** the visual constraint (paper **Eq. 13–19**). Two cameras seeing the same feature
must be geometrically consistent (coplanar rays).

```cpp
class EpipolarFactor : public ceres::SizedCostFunction<1, 6, 6> {
```
`<1, 6, 6>` = residual is **1 scalar**, depending on **two pose blocks** (`pose_i`, `pose_j`)
of size 6 each.

```cpp
EpipolarFactor(const Eigen::Vector3d& z_i, const Eigen::Vector3d& z_j,
               const Eigen::Matrix3d& Rbc, const Eigen::Vector3d& pbc, double weight = 1.0)
    : z_i_(z_i), z_j_(z_j), Rbc_(Rbc), pbc_(pbc), weight_(weight) {}
```
Stores the constants: the two bearings `z_i, z_j` (= `zⁿ = h_d⁻¹(u)` in **Eq. 12**), the
extrinsics `Rbc = ᴵ_C R`, `pbc = ᴵp_C`, and the weight (`Σ_C^{-1/2}`).

```cpp
Eigen::Map<const Eigen::Vector3d> omega_i(parameters[0]);
Eigen::Map<const Eigen::Vector3d> p_i(parameters[0] + 3);
Eigen::Map<const Eigen::Vector3d> omega_j(parameters[1]);
Eigen::Map<const Eigen::Vector3d> p_j(parameters[1] + 3);
Eigen::Matrix3d R_i = Sophus::SO3d::exp(omega_i).matrix();
Eigen::Matrix3d R_j = Sophus::SO3d::exp(omega_j).matrix();
```
Unpack the two poses and rebuild rotation matrices from their `ω` form.

```cpp
Eigen::Vector3d A = R_j * Rbc_ * z_j_;   // Eq.14:  A = ᴳ_Ij R · ᴵ_C R · zⁿ_j
Eigen::Vector3d B = R_i * Rbc_ * z_i_;   // Eq.14:  B = ᴳ_Ii R · ᴵ_C R · zⁿ_i
```
**Eq. 14.** Rotate each bearing (camera → IMU via `Rbc` → world via `R`). `A`, `B` are the
two feature rays in the global frame.

```cpp
Eigen::Vector3d t = (p_i + R_i * pbc_) - (p_j + R_j * pbc_);  // Eq.13: t = ᴳp_Ci − ᴳp_Cj
double t_norm = t.norm();
```
**Eq. 13.** `p_C = p_I + R·pbc` is each camera center; `t` is the baseline between them.

```cpp
if (t_norm < 1e-4) {   // degenerate (near-zero baseline) guard
    residuals[0] = 0.0;
    if (jacobians) { /* setZero both blocks */ }
    return true;
}
Eigen::Vector3d C = t / t_norm;   // Eq.14: C = t / ||t||
```
Guard against a vanishing baseline (would blow up `C`). Otherwise normalize `t` (**Eq. 14**) —
this is what makes the constraint **scale-invariant**.

```cpp
residuals[0] = weight_ * A.dot(C.cross(B));   // Eq.13/14:  r = Aᵀ [C]× B
```
**Eq. 13 / 14 — the residual.** `A·(C×B)` is the scalar triple product = coplanarity error.
Zero when rays and baseline are coplanar. `weight_` = `Σ_C^{-1/2}`.

```cpp
if (!jacobians) return true;   // Ceres sometimes wants only the residual
```

```cpp
Eigen::Matrix3d Cx = Sophus::SO3d::hat(C);   // [C]×
Eigen::Matrix3d Bx = Sophus::SO3d::hat(B);   // [B]×
Eigen::RowVector3d dr_dA = (Cx * B).transpose();     // Eq.15:  ∂r/∂A = ([C]×B)ᵀ
Eigen::RowVector3d dr_dB = A.transpose() * Cx;       // Eq.15:  ∂r/∂B = Aᵀ[C]×
Eigen::RowVector3d dr_dC = -A.transpose() * Bx;      // Eq.15:  ∂r/∂C = −Aᵀ[B]×
```
**Eq. 15** — derivatives of the scalar `r` w.r.t. the intermediates `A`, `B`, `C`.

```cpp
Eigen::Matrix3d dC_dt = (Eigen::Matrix3d::Identity() - C * C.transpose()) / t_norm;  // Eq.18
Eigen::RowVector3d dr_dt = dr_dC * dC_dt;   // chain: ∂r/∂t = ∂r/∂C · ∂C/∂t
```
**Eq. 18** — derivative of the normalization (`∂C/∂t = (I − CCᵀ)/‖t‖`), pre-combined into
`∂r/∂t`.

```cpp
if (jacobians[0]) {                                       // pose_i
    Eigen::Map<Eigen::Matrix<double,1,6,Eigen::RowMajor>> J(jacobians[0]);
    J.block<1,3>(0,0) = dr_dB * (-R_i * hat(Rbc_*z_i_))    // Eq.16: ∂B/∂φ_i
                      + dr_dt * (-R_i * hat(pbc_));         // Eq.17: ∂t/∂φ_i
    J.block<1,3>(0,3) = dr_dt * R_i;                        // Eq.17: ∂t/∂p_i (·R = body-frame)
    J *= weight_;
}
```
**Eq. 19 (chain rule) for keyframe i.** Rotation columns combine `∂r/∂B·∂B/∂φ` (Eq. 16) and
`∂r/∂t·∂t/∂φ` (Eq. 17). Position columns use `∂t/∂p_i = I` (Eq. 17); the extra `·R_i` converts
to the body-frame tangent (matching the parameterization).

```cpp
if (jacobians[1]) {                                       // pose_j
    Eigen::Map<Eigen::Matrix<double,1,6,Eigen::RowMajor>> J(jacobians[1]);
    J.block<1,3>(0,0) = dr_dA * (-R_j * hat(Rbc_*z_j_))    // Eq.16: ∂A/∂φ_j
                      + dr_dt * ( R_j * hat(pbc_));         // Eq.17: ∂t/∂φ_j (+ sign)
    J.block<1,3>(0,3) = -dr_dt * R_j;                      // Eq.17: ∂t/∂p_j = −I, ·R_j (body)
    J *= weight_;
}
```
Mirror for keyframe j: uses `dr_dA` (because `A`, not `B`, depends on `R_j`); the `t`-terms
flip sign because j enters `t` with a minus.

---

## 3. `src/initMethod/drtVioInit.cpp` — `structurelessVIBA()`

**Purpose:** builds and solves the joint optimization (paper **Eq. 11**) over the state
(**Eq. 10**).

```cpp
R_align_ = Utility::g2R(gravity);
for (int i = 0; i < (int)rotation.size(); ++i) {
    rotation[i] = R_align_ * rotation[i];
    position[i] = R_align_ * position[i];
    velocity[i] = R_align_ * velocity[i];
}
gravity = R_align_ * gravity;
```
**Gravity alignment** into the gravity-aligned global frame `{G}` (paper Notation §III).
The IMU factor assumes gravity along −z; `g2R` rotates every state so gravity points up-z.

```cpp
const int N = (int)rotation.size();
std::vector<std::array<double, 6>> pose(N);       // [ log(R), p ]
std::vector<std::array<double, 9>> speed_bias(N); // [ v, bg, ba ]
for (int i = 0; i < N; ++i) {
    Eigen::Quaterniond q(rotation[i]); q.normalize();
    Eigen::Map<Eigen::Vector3d>(pose[i].data())           = Sophus::SO3d(q).log();
    Eigen::Map<Eigen::Vector3d>(pose[i].data() + 3)       = position[i];
    Eigen::Map<Eigen::Vector3d>(speed_bias[i].data())     = velocity[i];
    Eigen::Map<Eigen::Vector3d>(speed_bias[i].data() + 3) = biasg;
    Eigen::Map<Eigen::Vector3d>(speed_bias[i].data() + 6) = biasa;
}
```
**Builds the state (paper Eq. 10):** per keyframe `[ᴳp, ᴳv, ᴳ_Ik q, b_a, b_g]`. Seeds each
block from DRT's solution (quaternion→normalize→log for a safe matrix→ω conversion).

```cpp
ceres::Problem problem;
for (int i = 0; i < N; ++i) {
    problem.AddParameterBlock(pose[i].data(), 6, new PoseLocalParameterization());
    problem.AddParameterBlock(speed_bias[i].data(), 9);
}
problem.SetParameterBlockConstant(pose[0].data());   // fix the gauge (KF0)
```
Registers the unknowns; attaches our parameterization to the pose blocks; pins keyframe-0 to
remove the unobservable global position+yaw gauge.

```cpp
for (int i = 0; i < N - 1; ++i) {
    auto* imu_factor = new vio::ImuIntegFactor(&imu_meas[i]);
    problem.AddResidualBlock(imu_factor, nullptr,
                             pose[i].data(),     speed_bias[i].data(),
                             pose[i + 1].data(), speed_bias[i + 1].data());
}
```
**IMU term of Eq. 11** (`Σ‖r_I‖²`). One reused `ImuIntegFactor` (paper **Eq. 5–7**) between each
consecutive keyframe pair; `nullptr` = no robust loss on IMU.

```cpp
ceres::LossFunction* vis_loss = new ceres::HuberLoss(1.0);
const double vis_weight = 20.0;   // = Σ_C^{-1/2}
for (const auto& kv : SFMConstruct) {
    const auto& obs = kv.second.obs;
    if (obs.size() < 2) continue;
    for (auto it_i = obs.begin(); it_i != obs.end(); ++it_i) {
        auto it_j = std::next(it_i);
        for (; it_j != obs.end(); ++it_j) {
            int i = time_frameid2_int_frameid.at(it_i->first);
            int j = time_frameid2_int_frameid.at(it_j->first);
            if ((position[i] - position[j]).norm() < 0.1) continue;   // skip small baseline
            auto* ef = new EpipolarFactor(it_i->second.normalpoint,
                                          it_j->second.normalpoint, Rbc_, pbc_, vis_weight);
            problem.AddResidualBlock(ef, vis_loss, pose[i].data(), pose[j].data());
        }
    }
}
```
**Visual term of Eq. 11** (`Σ ρ_Hub(‖r^n‖²)`). Iterates every tracked feature; for each
**co-visible keyframe pair** `(i,j)` (paper's `K^l`), adds an `EpipolarFactor` with a **Huber**
robust loss (paper's `ρ_Hub`). The `< 0.1` line skips degenerate small-baseline pairs.

```cpp
ceres::Solver::Options options;
options.linear_solver_type = ceres::DENSE_SCHUR;
options.trust_region_strategy_type = ceres::DOGLEG;
options.max_num_iterations = 50;
ceres::Solver::Summary summary;
ceres::Solve(options, &problem, &summary);   // solves Eq. 11
```
Runs the nonlinear least-squares solve — minimizes the combined cost of **Eq. 11**.

```cpp
for (int i = 0; i < N; ++i) {
    Eigen::Vector3d w = Eigen::Map<const Eigen::Vector3d>(pose[i].data());
    rotation[i] = Sophus::SO3d::exp(w).matrix();
    position[i] = Eigen::Map<const Eigen::Vector3d>(pose[i].data() + 3);
    velocity[i] = Eigen::Map<const Eigen::Vector3d>(speed_bias[i].data());
}
biasg = Eigen::Map<const Eigen::Vector3d>(speed_bias[0].data() + 3);
biasa = Eigen::Map<const Eigen::Vector3d>(speed_bias[0].data() + 6);
```
**Write-back:** unpack the optimized blocks into the public state (`exp(ω)`→rotation matrix, etc.).

```cpp
Eigen::Matrix3d Rt = R_align_.transpose();
for (int i = 0; i < (int)rotation.size(); ++i) {
    rotation[i] = Rt * rotation[i];
    position[i] = Rt * position[i];
    velocity[i] = Rt * velocity[i];
}
gravity = Rt * gravity;
return true;
```
**Un-align:** rotate everything back to DRT's original frame (inverse of the gravity alignment),
so downstream evaluation is in the expected frame.

---

## Equation map (quick reference)

| Code | Paper equation |
|---|---|
| `Plus` (R·exp) | Eq. 2 (⊞ operator) |
| epipolar residual `Aᵀ[C]×B` | Eq. 13–14 |
| `dr_dA / dr_dB / dr_dC` | Eq. 15 |
| `∂A,∂B/∂R`, `∂t/∂R`, `∂t/∂p` | Eq. 16–17 |
| `dC_dt` | Eq. 18 |
| Jacobian assembly | Eq. 19 |
| state blocks | Eq. 10 |
| IMU + epipolar cost | Eq. 11 |
| IMU factor (reused) | Eq. 5–7 |
