# Control theory background

This document explains the engineering choices behind `pid2`'s two
controllers: why both forms exist, where each one fails badly without
care, and what we did about it.

It is not a tutorial on PID. For that, the [Wikipedia article on PID
controllers](https://en.wikipedia.org/wiki/PID_controller) is a fine
starting point, and `PI-Controller.pdf` in the repo root has a longer
treatment in the operator-interface idiom this package adopts.

---

## 1. Two ways to write the same controller

A PID controller computes:

```
                              t
u(t) = kp · e(t) + (kp/ti) · ∫ e(τ) dτ + kp · td · de/dt
                              0
```

where `e = sp - pv` is the error between setpoint and process variable
and `u` is the output sent to the actuator.

There are two common ways to express this in code:

### Positional form

Compute `u(t)` directly each iteration. Carry the running integral
`∫e dτ` across calls; on each call, add `e · dt` to it.

```
u_n = kp · e_n + ki · I_n + kd · (e_n - e_{n-1}) / dt
I_n = I_{n-1} + e_n · dt
```

This is what `Controller` does. It is the textbook form, and it maps
directly to the math.

### Velocity (incremental) form

Compute the *change* in output each iteration, and add it to the
previous output. This drops the `kp · e_n` term (because `(kp · e_n) -
(kp · e_{n-1}) = kp · Δe`) and turns the integral into a simple
addition:

```
Δu_n = kp · (e_n - e_{n-1}) + (kp/ti) · e_n · dt
u_n  = u_{n-1} + Δu_n
```

This is what `PI-Controller` does (with `kd = 0`, hence "PI"). The form
has two practical attractions:

1. **Bumpless transfer.** When the operator switches the loop from
   manual to auto, the controller can pick up wherever the manual
   output happened to be — there is no hidden integrator state that
   would re-impose a different output at the moment of switch.
2. **Structural windup limiting.** The output is its own integrator. If
   the actuator saturates, `u_{n-1}` stays at the saturation value and
   the next `Δu_n` is small (because `Δe` and `e · dt / ti` are both
   small), so the controller doesn't accumulate phantom integral
   demand.

The velocity form has its own surprises, covered below.

---

## 2. Integrator windup

Both forms suffer if you implement them naively.

In a PID-controlled loop, when the actuator saturates against a physical
limit (a valve fully open, a motor at max RPM), the controller can no
longer affect the process. If error persists during saturation, the
positional form keeps adding `e · dt` to its integral, so by the time
the disturbance clears and the process can finally respond, the
integral is huge — the output stays pinned at the limit for far longer
than necessary, and the process overshoots the setpoint badly before
the integrator unwinds. This is **windup**.

### Windup in `Controller` (positional)

The pre-cleanup `Controller` accumulated integral on every call and
only clamped the *output*. The integrator was free to grow without
bound while the output was saturated. The fix used here is
**conditional integration**:

```toit
push := ki * error
saturated-high := unclamped > max and push > 0.0
saturated-low := unclamped < min and push < 0.0
if not (saturated-high or saturated-low):
  integral-error_ += error * elapsed-s
```

Read: only update the integral if doing so wouldn't push the output
*further* in the direction it is already pinned. The product `ki * error`
captures the sign of the integrator's would-be next contribution, so this
works for both direct- and reverse-acting tunings. The integrator can
still *reduce* in magnitude during saturation — exactly what you want
when the disturbance starts to recede.

`tests/controller-windup-test.toit` documents the pre/post behavior:
without the fix, a 50% disturbance saturating the loop for 3 s causes a
recovery overshoot far beyond the setpoint; with the fix, overshoot
stays under 20 % and the loop settles cleanly.

### Windup in `PI-Controller` (velocity)

The velocity form does not accumulate a separate integrator state, so
"classic" windup doesn't apply. Once `u_{n-1}` is pinned at `out-max`,
subsequent `Δu_n` values are small and the controller doesn't run away.

That said, two related defects in the original `update` would produce
their own kinds of bumps, both fixed in the cleanup:

1. **First-iteration kick.** The "previous" values (`pv-last_`,
   `dev-last_`, `out-last_`) were initialised to defaults like `50.0`.
   The very first call to `update` then computed `Δe = e_1 - 0` (or
   similar), which is enormous and produced a large phantom kick.

   Fix: **lazy initialization**. The first call to `update` primes the
   internal state from the actual current `pv` and returns a neutral
   "starting" output (the midpoint of `out-min..out-max` in auto mode,
   or the clamped manual output in manual mode). Subsequent iterations
   then compute differences against real previous values.

2. **Manual → auto bump.** While the loop ran in manual, the SP
   velocity-limiter (which exists to slow large operator SP changes)
   carried whatever target it had last seen. Re-engaging auto then
   ramped the internal SP from that stale value toward the new
   `sp-manual`, often producing a step.

   Fix: **prime the limiter from PV during manual**. While `auto =
   false`, the controller forces the limiter's anchor to follow the
   current `pv`, so when auto re-engages, the internal SP is already at
   PV and the limiter ramps toward `sp-manual` at the configured rate
   instead of jumping.

`tests/pi-init-test.toit` documents these.

---

## 3. SP velocity limiting

The `Velocity-Limit` helper inside `PI-Controller` exists because
operators occasionally enter setpoint changes that, taken at face
value, would demand the actuator instantaneously go to its limit. On a
real plant this is undesirable: it stresses equipment, wastes
disturbance budget, and produces overshoot that the operator then has
to nurse back. The limiter ramps the *internal* SP from its previous
value toward `sp-manual` at no more than `sp-limiter.limit` units per
second, so the controller sees a smooth move regardless of how
abruptly the operator changes the target.

The limit is per-second; e.g. `controller.sp-limiter.limit = 1.0` means
"don't move the internal SP by more than 1 unit per second of wall
time." `tests/pi-velocity-limit-test.toit` confirms a 50-unit step at
`limit=10/s` takes ~5 s to traverse.

---

## 4. Auto / manual mode

`PI-Controller` has two modes:

- **Auto** — the controller is in charge. The output it computes is
  what the actuator sees.
- **Manual** — the operator is in charge. `out-manual` is what the
  actuator sees; the controller is along for the ride, with its
  internal state kept consistent with the actual situation so that
  re-engaging auto is bumpless.

While in manual the controller still runs `update` each iteration: it
keeps last-values current, anchors the SP limiter to PV (see fix #2
above), and tracks `out-manual` as if it were its own output. This
means the operator can sit in manual indefinitely, then flip to auto,
and the controller picks up cleanly from the current operating point.

---

## 5. The `ti` API change

`PI-Controller --ti=` was previously typed as `int` (integer seconds).
It is now `float`, so fractional integral times like `ti=0.5` work.
Existing call sites that passed an integer literal (`--ti=100`) need to
be updated to a float literal (`--ti=100.0`). This is the only
caller-visible breaking change in the cleanup.

---

## 6. Choosing between the two controllers

| Need | Use |
|---|---|
| Embedded loop, no operator interface, you compute error yourself | `Controller` |
| Need derivative term (D) | `Controller` (PI-Controller is PI-only) |
| Operator faceplate, manual mode, SP limiting, bumpless transfer | `PI-Controller` |
| Self-regulating process (flow loop, temperature loop) | Either — both have the relevant fixes |
| Integrating process (level loop, position loop) | Either — both have the relevant fixes |

The two examples in `examples/` drive the same simulated tank from each
controller:

- [`examples/tank-control.toit`](../examples/tank-control.toit) — full
  faceplate-driven `PI-Controller` setup with UDP-controlled SP, mode,
  and manual output.
- [`examples/level-controller-pid.toit`](../examples/level-controller-pid.toit)
  — single-process positional `Controller` on the same tank, with
  hardcoded SP and stdout reporting. Useful for seeing the API
  difference and the steady-state behavior side-by-side.
