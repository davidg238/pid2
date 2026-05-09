# PID/PI Controller Cleanup & Test Suite — Design

**Date:** 2026-05-08
**Status:** Approved (pending implementation plan)

## Background

The `pid2` package ships two controllers:

- `Controller` (`src/controller.toit`) — original implementation. Position-form PID: `output = kp·e + ki·∫e dt + kd·de/dt`. The integral accumulator is unbounded; the output is clamped on the way out but the accumulator is never constrained → classic integral windup.
- `PI-Controller` (`src/pi-controller.toit`) — velocity (incremental) form PI: `out_n = out_{n-1} + Δproportional + Δintegral`, then clamp. Because `out_{n-1}` is the *clamped* previous output, the integral cannot wind up beyond saturation. Includes operator faceplate (auto/manual + manual SP and manual output), bumpless transfer, and SP velocity limiting.

The user observed misbehavior when tuning `PI-Controller` against a tank-level simulation, and initially suspected the algorithm was specialized to integrating processes (and not safe for self-regulating processes). On review, the velocity-form algorithm is sound and works for both plant types — but four real bugs and one initialization quirk in `PI-Controller`, plus the windup bug in `Controller`, plausibly explain the observed tuning frustration.

## Goals

1. Fix bugs in `PI-Controller` so it behaves as documented on first iteration and across mode transitions.
2. Add anti-windup to `Controller` (preserving its existing API).
3. Add a deterministic, simulation-based test suite that validates both controllers against representative plant models — one self-regulating, one integrating.
4. Tidy up the tank example so it matches its README intent.

## Non-Goals

- New controller variants (the user explicitly asked for no skinny version).
- Tuning helpers / autotune.
- Hardware-in-the-loop tests.
- API redesign of `PI-Controller`'s operator surface.

## Detailed Design

### 1. `PI-Controller` bug fixes (`src/pi-controller.toit`)

#### 1.1 Lazy initialization of last-values

**Problem.** Field declarations hardcode `pv-last_ := 50.0`, `dev-last_ := 0.0`, `out-last_ := 50.0`, `sp-last_ := 50.0`, and the constructor sets `time-last_ = Time.monotonic-us`. If the actual first PV is, e.g., 30.0 with SP-manual at 50.0, the first `update` sees `p1 = dev - dev-last_ = -20 - 0 = -20` and produces a large proportional kick. Similarly, any delay between construction and the first `update` inflates the first `dT` → outsized first integral increment.

**Fix.** Use a sentinel (`time-last_ /int? := null`) and detect first-call inside `update`:

- On first call: set `time-last_ = time-now`, `pv-last_ = pv`, `dev-last_ = pv - sp_`, `out-last_` to the clamped starting output (the manual output if `auto == false`, else the existing default starting output — see §1.5), and return that output without applying the delta-based proportional term or the integral increment for this iteration.
- On subsequent calls: behave as today.

#### 1.2 Re-prime SP velocity-limiter on manual→auto transition

**Problem.** In manual mode, `sp_ = pv` and the SP velocity-limiter is *not* called. The limiter's `last_` is `null` until first updated. When the operator flips to auto, the very first `sp-limiter.update s1 dT` returns `s1` (i.e., `sp-manual`) directly because `last_ == null` — no ramp. `sp_` jumps from `pv` to `sp-manual` in one step and the proportional channel sees the jump as `p1`, producing an output kick. The advertised "velocity-limited SP changes / bumpless transfer" only works on subsequent SP edits, not on the auto re-engage.

**Fix.** Expose a public `prime val/float` method on `Velocity-Limit` that sets `last_ = val` (the field stays private). In `PI-Controller.update`, when `not auto`, call `sp-limiter.prime pv`. On the next auto iteration, the limiter ramps from `pv` toward `sp-manual` at the configured velocity.

#### 1.3 `ti` typed as `float`

**Problem.** Constructor and `tune` both type `ti/int`, so fractional integral times (e.g., `ti=0.5s`) are unrepresentable.

**Fix.** Change to `ti/float` in both, default `1.0`. Drop the `* 1.0` conversion in the body.

#### 1.4 Optional debug logging

**Problem.** The `report_` line in `update` is commented-out source code, requiring an edit to enable.

**Fix.** Add `--debug/bool=false` to the constructor; store as `debug /bool`. Replace the commented call with `if debug: report_ p1 p2 p3 proportional integral`. Add a `debug=` setter so it can be toggled at runtime.

#### 1.5 Default starting output

**Problem.** `out_ /float := 50.0`, `out-last_ /float := 50.0` bake in a 50% midrange assumption. Combined with §1.1, the first call returns 50.0 regardless of `out-min`/`out-max`.

**Fix.** On the first `update` (when limits have already been set by the caller), initialize `out-last_` and `out_` to the midpoint of `[out-min, out-max]` if `auto`, or to the clamped `out-manual` if `not auto`. This makes the controller usable when limits are not the default 0–100.

### 2. `Controller` anti-windup (`src/controller.toit`)

**Approach.** Conditional integration. Same `update error/float elapsed/Duration -> float` signature.

```
elapsed_s = elapsed.in_ns / 1e9
derivative_error = (error - last_error_) / elapsed_s
unclamped = kp*error + ki*integral_error_ + kd*derivative_error
clamped = clamp(unclamped, min, max)

# Direction the integrator's next increment would push the output:
#   sign(ki * error). Freeze the integrator only when output is saturated
#   AND the next increment would push further into that saturation.
push = ki * error
saturated_high = (unclamped > max) and (push > 0)
saturated_low  = (unclamped < min) and (push < 0)
if not (saturated_high or saturated_low):
    integral_error_ += error * elapsed_s

last_error_ = error
return clamped
```

The `ki * error` test (rather than `error` alone) is the correct gate: it handles both direct-acting (`ki > 0`) and reverse-acting (`ki < 0`) configurations. The integrator can still unwind out of a saturation it caused (e.g., saturated high, then error reverses, integrator decreases — allowed).

### 3. Test Suite

New top-level `tests/` directory. Each test is a runnable Toit program that exits non-zero on failure (using the standard `expect` library).

#### 3.1 Plant models (`tests/plant-models.toit`)

A small library of deterministic plant simulations. No `random`. Time-stepped using a caller-provided `dt` in seconds.

- **`First-Order-Plant`** — `dy/dt = (K·u − y) / τ`, with optional pure dead time implemented as a delay buffer. Self-regulating; flow-loop analogue.
  - Constructor: `--gain/float=1.0 --tau/float=10.0 --dead-time/float=0.0 --initial/float=0.0`.
  - Method: `step u/float dt/float -> float` (returns new output).
- **`Integrating-Plant`** — `dy/dt = K·(u − u_bias)`, with optional dead time. Level-loop analogue.
  - Constructor: `--gain/float=1.0 --u-bias/float=50.0 --dead-time/float=0.0 --initial/float=50.0`.
  - Method: `step u/float dt/float -> float`.

Both plants support an additive disturbance via a `--disturbance/float=0.0` field that the test mutates between iterations.

#### 3.2 Test scenarios

One file per scenario family. Each closes the loop at a fixed `dt` (e.g., 100 ms simulated; tests use a logical clock, not wall-clock sleeps).

| File | Scenario | Pass criteria |
|---|---|---|
| `tests/test-pi-step.toit` | SP step on `First-Order-Plant` and `Integrating-Plant` | First-iteration output change ≤ ε (no kick); PV settles within ±2% of SP within configured horizon |
| `tests/test-pi-windup.toit` | Step SP large enough to saturate output for ≥ N seconds, then reduce SP | After de-saturation, no overshoot beyond a fixed bound (e.g., 5% of the SP delta); recovery time ≤ horizon |
| `tests/test-pi-bumpless.toit` | Run in auto, switch to manual, edit `out-manual`, switch back to auto | On each transition, `\|out_n − out_{n-1}\| ≤ ε`; on manual→auto, `sp_` ramps from `pv` |
| `tests/test-pi-velocity-limit.toit` | Configure `sp-limiter.limit = L`; step `sp-manual` by ΔSP in auto | Per-step SP change is bounded by `L·dt`; total ramp time ≈ `ΔSP / L` |
| `tests/test-pi-disturbance.toit` | Constant SP, step the plant disturbance after settling | PV returns within ±2% of SP within horizon |
| `tests/test-controller-windup.toit` | Same windup scenario for `Controller` | Demonstrates anti-windup works: post-saturation overshoot bounded |

Tests run via Toit's standard test invocation (resolved during implementation; see Open Questions).

### 4. Tank example tidy-up (`examples/tank-control.toit`)

- Remove unused top-level `sp := 60.0` (the live setpoint lives on `controller.sp-manual`).
- Set `controller.sp-manual = 60.0` after construction so the example's intended setpoint matches its variable name.
- Add a commented `// controller.debug = true` line as a hint that debug logging is now available without source edits.

## Files Touched

**Modified:**
- `src/pi-controller.toit` — bugs §1.1–§1.5
- `src/controller.toit` — anti-windup §2
- `examples/tank-control.toit` — §4

**New:**
- `tests/plant-models.toit`
- `tests/test-pi-step.toit`
- `tests/test-pi-windup.toit`
- `tests/test-pi-bumpless.toit`
- `tests/test-pi-velocity-limit.toit`
- `tests/test-pi-disturbance.toit`
- `tests/test-controller-windup.toit`

**Untouched:**
- `src/pid2.toit` (re-export only)
- `examples/tank-faceplate.toit`
- `package.yaml`, `README.md` (README updated only if any externally-visible API change lands; the planned changes are additive or behavior-preserving except the `ti` int→float widening, which is source-compatible)

## Risks & Open Questions

- **`ti` type widening** is technically a signature change (`int` → `float`). Toit literal `100` still parses as an `int`, so callers passing `--ti=100` may need `--ti=100.0`. This shows up in `examples/tank-control.toit:38`. Acceptable for an unreleased package; will note in the implementation plan.
- **Toit test runner.** The exact incantation to run the test files (`toit test`, `jag run`, etc.) will be resolved during implementation by consulting `toit-exe` and `toit-package` skills. Tests are written as runnable mains regardless.
- **Anti-windup choice for `Controller`.** Conditional integration is simpler than back-calculation tracking and adequate for the typical clamp-on-output-only case here. If the user later wants a tracking-time-constant `Kt` for smoother behavior near saturation, that's a future change.

## Acceptance

- All seven test files compile and pass.
- `examples/tank-control.toit` runs without changes to its UDP/operator interface.
- No regression in `PI-Controller` external API except the documented `ti` widening.
