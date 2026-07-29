# Structureless VI-BA — Project Handoff / Context

Paste this into a new chat to continue. It captures goal, state, files, results, decisions, and next steps.

## Goal & deliverables
Apply Song et al. (2025, arXiv:2502.16598) **structureless VI-BA** to the **DRT-l** initializer, then:
1. Apply the paper's VI-BA to DRT ✅ (done)
2. Compare to original DRT — metrics: **gravity direction error, gyro bias error, trajectory RMSE** (+ scale, velocity, runtime) ✅ (done for EuRoC Machine Hall)
3. Integrate into **VINS-Fusion**, compare vs original VINS-Fusion ⏳ (not started — the big remaining piece)
4. Runtime analysis ⏳
Timeline: ~1.5 months. Paper PDF at `~/Downloads/VI-BA_paper.pdf`.

## Repos
- `~/drt-vio-init` — working repo (has VI-BA), git branch `structureless-viba` (5 commits made).
- `~/drt-vio-baseline` — pristine ORIGINAL DRT (algorithm untouched), only build/eval/config changes. Used as the baseline.

## Files we wrote / changed (in ~/drt-vio-init)
- `include/factor/poseLocalParam.h` (NEW) — `PoseLocalParameterization` (SO3×R³ manifold update).
- `include/factor/epipolarFactor.h` (NEW) — `EpipolarFactor` (coplanarity residual + analytic Jacobians, paper Eq.13-19).
- `src/initMethod/drtVioInit.cpp` — new `structurelessVIBA()` (gravity-align → build blocks → IMU + epipolar factors → solve → writeback → un-align).
- `src/initMethod/drtLooselyCoupled.cpp` — calls `structurelessVIBA()` at end of `process()`.
- `app/main.cpp` — added posyaw ATE + `solve_time` metrics (baseline repo has these too).
- Reused UNCHANGED: `include/factor/imuIntegFactor.h` (`ImuIntegFactor`).
- Docs: `doc/code_explained.md`, `doc/how_viba_works.md`, `doc/ppt_plan.md`, this file.

## Current config / knobs
- `vis_weight = 20.0` in `structurelessVIBA()` (best trajectory; sweep showed plateau ~20-50).
- Small-baseline filter: skip epipolar pair if `||position[i]-position[j]|| < 0.1` (fixes ill-conditioning).
- In-factor guard: `if (t_norm < 1e-4) residual/jac = 0`.
- `max_num_iterations = 50` — does NOT converge (NO_CONVERGENCE) but result is usable; forcing convergence (fixing accel bias) made accuracy worse, so we accept it.
- gyro bias `bg` is FREE (kept in the state, per paper Eq.10).

## Results — DRT vs VI-BA (EuRoC Machine Hall MH01-05, averaged)
| Metric | DRT | VI-BA | change |
|---|---|---|---|
| Trajectory ATE (umeyama) | 0.1754 | 0.1193 | −32% |
| Velocity RMSE | 0.1270 | 0.1084 | −15% |
| Trajectory ATE (posyaw) | 0.3977 | 0.3972 | flat |
| Scale error | 0.5481 | 0.1569 | −71% (confounded — see below) |
| Gyro bias error (%) | 1.4235 | 2.0705 | +45% (regresses) |
| Solve time (ms) | 2.28 | 41.80 | overhead |

Per-sequence tables and raw values: `~/drt-vio-init/result/results1-5viba.txt`, `~/drt-vio-baseline/result/results1-5og.txt`.

## Key findings & decisions (IMPORTANT)
- **VI-BA improves trajectory + velocity** — exactly the states DRT constrains linearly. Matches the paper's stated purpose (refine what DRT's decoupling+linearization compromised).
- **umeyama vs posyaw:** umeyama alignment removes scale (shows the SHAPE gain, −32%); posyaw keeps scale (realistic, but flat). VI-BA fixes shape, NOT size — epipolar is scale-invariant and scale is IMU-limited.
- **Scale metric (−71%) is CONFOUNDED** with shape (it's umeyama's fitted scale). Do NOT claim a real 71% size fix — posyaw (real size) is flat.
- **Gyro bias regresses** (+45%). Root cause: VI-BA re-optimizes bg jointly, looser than DRT's dedicated tightly-coupled estimator. Vision actually HELPS bg (IMU-only alone is worst); it just can't fully recover DRT's value. NOT a code bug (code verified). Bias & trajectory TRADE OFF: fixing/anchoring bg preserves bias but wrecks the trajectory gain (they're coupled via the epipolar factor's rotation dependence). Decision: keep bg free, report regression honestly. A prior/fix = beyond the paper's Eq.11.
- **Gravity NOT optimized** — not in paper's VI-BA state (Eq.10); correctly unchanged. Not a bug.
- **Absolute numbers differ from the paper** because the paper standardizes the front-end to VINS-Mono's feature tracker + parallax keyframe selection and uses posyaw ATE; we use the repo's own front-end. Decision: report RELATIVE improvement in our consistent setup, not paper-exact reproduction. (Our DRT-l scale ~13% vs paper ~2.5% is the main gap, traced to the front-end.)
- **Runtime** differs from paper due to hardware (our DRT ~2ms vs paper 8.6ms). Only same-machine DRT-vs-VIBA comparison is meaningful.

## Verification done
- Epipolar Jacobian: **manual manifold numerical check PASSED** (machine precision). NOTE: `ceres::GradientChecker` gives a FALSE fail here — it's incompatible with our identity-`ComputeJacobian` trick (compares ambient vs our tangent Jacobians). Always verify tangent-reporting factors with a manual manifold-perturbation check.
- Cross-checked residual + rotation Jacobians against an external reference implementation.
- IMU-only test converged to ~1e-13 cost → frame conventions & wiring correct.

## Gotchas / conventions
- Gravity: `Utility::g2R(gravity)` aligns DRT's (up-pointing, VINS convention) gravity to +z; the reused `ImuIntegFactor` uses `g=(0,0,-9.81)` (down = −z). Consistent because up=+z ⇒ down=−z.
- Pose parameterization: rotation `R·exp(δφ)` (RIGHT), position `p + R·δp` (BODY-frame). Both required to match `ImuIntegFactor`'s Jacobian convention (verified via the factor's own commented self-check).
- Epipolar factor position Jacobian is chained through `R` (body-frame) — paper Eq.17 is world-frame, adapted to match.
- Datasets extracted at `~/Downloads/machine_hall/MH_0X_.../mav0`. Config `data_path` must point at a sequence's `mav0/`. The multi-run scripts `sed`-edit `config/euroc.yaml`'s `data_path` per sequence (and leave it at MH05 at the end — watch out when running a single sequence afterward).
- Build: TBB made optional in CMakeLists (not installed). No node/pip/python-pptx on this machine (LibreOffice only).

## Multi-sequence run recipe (per repo)
Edit `config/euroc.yaml` `data_path` to a sequence's mav0, then from `build/`: `./run_euroc drtLoosely <label>` → writes `result/drtLoosely_<label>.txt`. Metrics via awk on that file (umeyama=`pose_error`, scale=`scale_error`, velo=`velo_error`, gyro=`biasg_error`, gravity=1st num of `gravity_error`) and on the stdout log (posyaw=`ate_posyaw`, runtime=`solve_time`).

## User preferences
- Do NOT add `Co-Authored-By` trailer to git commits.
- Wants short, list-style progress reports.
- Explanations: simple, concrete, step-by-step.

## NEXT STEPS
1. Decide gyro bias handling: keep free + report honestly (recommended), or add a soft `bg` prior (labeled "beyond paper") for a compromise.
2. (optional) run more sequences / TUM-VI for a fuller table.
3. **VINS-Fusion integration** — the main remaining deliverable: port the epipolar factor + parameterization into VINS-Fusion's init, compare vs original VINS-Fusion, runtime.
4. Finalize the PPT/report (plans in `doc/ppt_plan.md`).
