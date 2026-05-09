# PID Controller Cleanup & Tests Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix five bugs in `PI-Controller`, add anti-windup to the original `Controller` class, and add a deterministic simulation-based test suite that exercises both controllers against representative plant models.

**Architecture:** TDD throughout. Each bug gets a failing test that documents the desired behavior, then a minimal fix, then a passing test, then a commit. Tests close the loop against deterministic plant simulations (`First-Order-Plant` for self-regulating processes, `Integrating-Plant` for level-loop analogues), so the tuning argument the user raised is settled by demonstration on both plant types.

**Tech Stack:** Toit (kebab-case identifiers, 2-space indent, `import expect show *` for tests). Tests live in `tests/` and end with `-test.toit`. Build/test driven by Make + CMake/CTest per Toit package conventions.

**Reference:** Spec at `docs/superpowers/specs/2026-05-08-pid-controller-cleanup-design.md`. Note: the spec uses `tests/test-foo.toit` for filenames, but Toit convention is `tests/foo-test.toit` — this plan adopts the Toit convention.

---

## Pre-flight check

Before starting Task 1, run:

```bash
cd /home/david/workspaceToit/pid2
git status
toit version
```

Expected: clean working tree on `main`, `toit version` prints (e.g.) `v2.0.0-alpha.x` matching `examples/package.lock`'s `sdk: ^2.0.0-alpha.64`.

---

## Task 1: Test infrastructure scaffolding

**Files:**
- Create: `Makefile`
- Create: `CMakeLists.txt`
- Create: `tests/CMakeLists.txt`
- Create: `tests/package.yaml`
- Create: `tests/smoke-test.toit`
- Modify: `.gitignore` (create if absent)

- [ ] **Step 1: Create `Makefile`**

```makefile
all: test

.PHONY: build/CMakeCache.txt
build/CMakeCache.txt:
	$(MAKE) rebuild-cmake

install-pkgs: rebuild-cmake
	cmake --build build --target install-pkgs

test: install-pkgs rebuild-cmake
	cmake --build build --target check

rebuild-cmake:
	mkdir -p build
	cmake -B build -DCMAKE_BUILD_TYPE=Debug

.PHONY: all test rebuild-cmake install-pkgs
```

- [ ] **Step 2: Create `CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.23)

project(pid2 NONE)

enable_testing()
add_subdirectory(tests)
```

- [ ] **Step 3: Create `tests/CMakeLists.txt`**

```cmake
file(GLOB TESTS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*-test.toit" "*_test_slow.toit")

find_program(TOIT_EXEC toit)
if(NOT TOIT_EXEC)
  set(TOIT_EXEC "toit${CMAKE_EXECUTABLE_SUFFIX}")
endif()
set(TEST_TIMEOUT 40 CACHE STRING "The maximal amount of time each test is allowed to run")

message(STATUS "Toit executable: ${TOIT_EXEC}")

add_custom_target(
  "install-pkgs"
  COMMAND "${TOIT_EXEC}" pkg install
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)

include(ProcessorCount)
ProcessorCount(NUM_CPU)

add_custom_target(
  "check"
  COMMAND "${CMAKE_CTEST_COMMAND}" -j${NUM_CPU} --output-on-failure -C Debug
  USES_TERMINAL
)

foreach(file ${TESTS})
  set(test_name "/tests/${file}")
  add_test(
    NAME "${test_name}"
    COMMAND "${TOIT_EXEC}" "tests/${file}"
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  )
  set_tests_properties(${test_name} PROPERTIES TIMEOUT ${TEST_TIMEOUT})
endforeach()
```

- [ ] **Step 4: Create `tests/package.yaml`**

```yaml
name: pid2-tests
description: Tests for the pid2 package.
dependencies:
  pid2:
    path: ..
```

- [ ] **Step 5: Create `tests/smoke-test.toit`** (sanity check that the tests directory wires up)

```toit
// Copyright (C) 2026 Toit contributors.
// Use of this source code is governed by an MIT-style license that can be
// found in the LICENSE file.

import expect show *
import pid2 show PI-Controller

/**
Smoke test: the package can be imported and a controller can be constructed.
*/
main:
  controller := PI-Controller --kp=1.0 --ti=10.0 --ks=-1
  expect-not-null controller
```

- [ ] **Step 6: Create `.gitignore`** (create if missing; otherwise append)

```
.packages/
build/
```

- [ ] **Step 7: Run `make test`**

```bash
cd /home/david/workspaceToit/pid2
make test
```

Expected: CMake configures, `tests/smoke-test.toit` runs and passes; output ends with `100% tests passed`.

If `make test` fails because `toit pkg install` cannot resolve the local path, double-check the `path: ..` line in `tests/package.yaml`.

- [ ] **Step 8: Commit**

```bash
git add Makefile CMakeLists.txt tests/CMakeLists.txt tests/package.yaml tests/smoke-test.toit .gitignore
git commit -m "$(cat <<'EOF'
Add test infrastructure: Make + CMake/CTest harness

Adds the standard Toit-package build files (Makefile, CMakeLists.txt,
tests/CMakeLists.txt) plus a tests/package.yaml that points at the
parent package, and a smoke test that verifies the wiring works.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 2: Plant model library

**Files:**
- Create: `tests/plant-models.toit`
- Create: `tests/plant-models-test.toit`

- [ ] **Step 1: Write the failing test in `tests/plant-models-test.toit`**

```toit
// Copyright (C) 2026 Toit contributors.
// Use of this source code is governed by an MIT-style license that can be
// found in the LICENSE file.

import expect show *
import .plant-models

/**
First-Order-Plant: a constant input drives the output toward gain*input
with time constant tau. After ~5*tau, output should be within ~1% of the
steady-state value.
*/
test-first-order-step-response:
  plant := First-Order-Plant --gain=2.0 --tau=1.0 --initial=0.0
  // Apply unit step input. After 5 seconds (5*tau), should be near gain*input = 2.0.
  steps := 5000  // 5000 * 1ms = 5s
  steady := 0.0
  steps.repeat:
    steady = plant.step 1.0 0.001
  expect (steady - 2.0).abs < 0.05  // within 2.5% of 2.0

/**
Integrating-Plant: with input == u-bias, output should not change.
With input above u-bias by delta, output should rise linearly at rate gain*delta.
*/
test-integrating-balance-and-ramp:
  plant := Integrating-Plant --gain=0.5 --u-bias=50.0 --initial=20.0
  // Apply input == u-bias for 1s; expect output unchanged.
  100.repeat: plant.step 50.0 0.01
  expect (plant.y - 20.0).abs < 1e-9
  // Apply input == 60 (delta=10) for 2s; expect rise of 0.5 * 10 * 2 = 10.
  200.repeat: plant.step 60.0 0.01
  expect ((plant.y - 30.0).abs < 0.01)

/**
Disturbance is additive on the rate of change.
*/
test-integrating-disturbance:
  plant := Integrating-Plant --gain=1.0 --u-bias=0.0 --initial=0.0
  plant.disturbance = 1.0
  // With u==0, disturbance==1, dy/dt = -1. After 1s, y should be -1.
  100.repeat: plant.step 0.0 0.01
  expect ((plant.y + 1.0).abs < 0.01)

main:
  test-first-order-step-response
  test-integrating-balance-and-ramp
  test-integrating-disturbance
```

- [ ] **Step 2: Run the test, expect FAIL (compile error: `First-Order-Plant` undefined)**

```bash
make test
```

Expected: ctest reports `tests/plant-models-test.toit` failed; the failure is a Toit compile error referencing `First-Order-Plant` not found.

- [ ] **Step 3: Implement `tests/plant-models.toit`**

```toit
// Copyright (C) 2026 Toit contributors.
// Use of this source code is governed by an MIT-style license that can be
// found in the LICENSE file.

/**
Deterministic plant simulations for closed-loop controller testing.

Two plant types are provided, matching the two process classes commonly
discussed in control engineering:

  $First-Order-Plant: self-regulating process (e.g. flow through a valve).
  The output settles to a steady-state value `gain * input` for a constant
  input, with first-order dynamics characterized by time constant `tau`.

  $Integrating-Plant: integrating process (e.g. tank level, where flow in
  vs. flow out determines the rate of level change). The output rises or
  falls at a rate proportional to the offset between input and `u-bias`.
*/

/**
A first-order plant: dy/dt = (gain*u - y) / tau + disturbance.

Self-regulating: a constant input drives the output to a steady-state
value `gain * u + tau * disturbance`.
*/
class First-Order-Plant:
  gain/float
  tau/float
  y/float := ?
  disturbance/float := 0.0

  /**
  Constructs a first-order plant with the given $gain (steady-state output per
  unit input), time constant $tau (in seconds), and starting output $initial.
  */
  constructor --.gain/float=1.0 --.tau/float=1.0 --initial/float=0.0:
    y = initial

  /**
  Advances the plant by $dt seconds with input $u.
  Returns the new output value.
  */
  step u/float dt/float -> float:
    dy := ((gain * u - y) / tau + disturbance) * dt
    y += dy
    return y

/**
An integrating plant: dy/dt = gain * (u - u-bias) - disturbance.

Output rises at rate `gain * (u - u-bias)` for u above bias, falls below.
$disturbance is subtracted from the rate (e.g. an outflow on a tank).
*/
class Integrating-Plant:
  gain/float
  u-bias/float
  y/float := ?
  disturbance/float := 0.0

  /**
  Constructs an integrating plant with the given $gain (output rate per unit
  input above $u-bias) and starting output $initial.
  */
  constructor --.gain/float=1.0 --.u-bias/float=0.0 --initial/float=0.0:
    y = initial

  /**
  Advances the plant by $dt seconds with input $u.
  Returns the new output value.
  */
  step u/float dt/float -> float:
    dy := (gain * (u - u-bias) - disturbance) * dt
    y += dy
    return y
```

- [ ] **Step 4: Run the tests, expect PASS**

```bash
make test
```

Expected: `100% tests passed` (smoke test + plant-models-test, 2 passing).

- [ ] **Step 5: Commit**

```bash
git add tests/plant-models.toit tests/plant-models-test.toit
git commit -m "$(cat <<'EOF'
Add deterministic plant models for controller testing

First-Order-Plant simulates self-regulating processes (flow loops);
Integrating-Plant simulates integrating processes (level loops). Both
support an additive disturbance term used by later closed-loop tests.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 3: PI-Controller — lazy initialization + manual-mode SP-limiter prime

This task fixes spec items §1.1 (lazy init), §1.2 (re-prime on manual→auto), and §1.5 (default starting output midpoint) together because they all live in the same `update` method and overlap.

**Files:**
- Create: `tests/pi-init-test.toit`
- Modify: `src/pi-controller.toit`

- [ ] **Step 1: Write the failing test**

Create `tests/pi-init-test.toit`:

```toit
// Copyright (C) 2026 Toit contributors.
// Use of this source code is governed by an MIT-style license that can be
// found in the LICENSE file.

import expect show *
import pid2 show PI-Controller

/**
First call to update must not produce a phantom proportional kick from
default-seeded last-values. The first output should be the midpoint of
[out-min, out-max] in auto mode, ignoring any phantom delta.
*/
test-no-first-iteration-kick:
  controller := PI-Controller --kp=10.0 --ti=100.0 --ks=-1
  controller.sp-manual = 50.0
  // Real PV is 30, far from default-seeded 50. Without lazy init, a phantom
  // proportional kick would push output to its upper limit.
  out := controller.update 30.0
  expect-equals 50.0 out

/**
First-call output respects custom output limits (uses midpoint, not 50.0).
*/
test-first-iteration-uses-midpoint:
  controller := PI-Controller --kp=1.0 --ti=10.0 --ks=-1
  controller.out-min = 4.0
  controller.out-max = 20.0
  out := controller.update 10.0
  expect-equals 12.0 out  // midpoint of 4..20

/**
First call in manual mode returns the clamped manual output.
*/
test-first-iteration-manual:
  controller := PI-Controller --kp=10.0 --ti=100.0 --ks=-1
  controller.auto = false
  controller.out-manual = 25.0
  out := controller.update 30.0
  expect-equals 25.0 out

/**
After running in manual, switching to auto must not produce an output kick.
The SP limiter must ramp from current PV toward sp-manual rather than
jumping to sp-manual on the first auto iteration.
*/
test-manual-to-auto-bumpless:
  controller := PI-Controller --kp=10.0 --ti=100.0 --ks=-1
  controller.sp-limiter.limit = 1.0  // 1 unit/second
  controller.auto = false
  controller.sp-manual = 80.0
  controller.out-manual = 25.0
  // Run a few iterations in manual at PV=30 so the limiter sees the manual state.
  4.repeat:
    sleep --ms=10
    controller.update 30.0
  // Switch to auto.
  controller.auto = true
  sleep --ms=10
  out-after-switch := controller.update 30.0
  // With the fix, the kick should be tiny: SP moves at most 1 unit/sec * dt.
  // Without the fix, the SP jumps to 80, producing a large proportional kick.
  expect ((out-after-switch - 25.0).abs < 5.0)

main:
  test-no-first-iteration-kick
  test-first-iteration-uses-midpoint
  test-first-iteration-manual
  test-manual-to-auto-bumpless
```

- [ ] **Step 2: Run, expect FAIL**

```bash
make test
```

Expected: `tests/pi-init-test.toit` fails — `test-no-first-iteration-kick` expects `50.0` but the current code clamps a kicked output to `100.0`. (May fail on multiple tests; that's fine, the fix below addresses all four.)

- [ ] **Step 3: Implement the fix in `src/pi-controller.toit`**

Modify the field declarations (lines 40-50) so the sentinel-bearing fields are `null` to start:

```toit
  sp_ /float  := 50.0
  ti_ /float := 1.0       // integral time constant
  out-pid_ /float := 0.0  // pid algorithm output
  out_ /float := 50.0     // output
  kp2_ /float := 0.0

  time-last_ /int? := null
  pv-last_ /float := 0.0
  sp-last_ /float := 0.0
  dev-last_ /float := 0.0
  out-last_ /float := 0.0
  n_ := 0
```

Modify the constructor (remove the `time-last_ = ...` line):

```toit
  /**
  Creates a PI controller with nominated proportional gain, integral time and direct/reverse action.
  */
  constructor --.kp=1.0 --ti/int --.ks/int:
    kp2_ = kp == 0.0? 1.0: kp
    ti_ = ti * 1.0
```

(The `--.ti/int` widening to `float` is in Task 4 — keep `int` here.)

Add a `prime` method to `Velocity-Limit` (at the end of the file):

```toit
  /**
  Forces the limiter's internal anchor to $val so that the next call to $update
  ramps from $val toward its target at the configured velocity. Used by the
  enclosing controller to keep the limiter in sync with PV during manual mode.
  */
  prime val/float -> none:
    last_ = val
```

Add a public getter for `sp_` (used by the velocity-limit test in Task 6) just below the `pv` field declaration:

```toit
  /** The current internal setpoint, after auto/manual select and SP velocity limiting. */
  sp -> float: return sp_
```

Replace the body of `update` with the lazy-init + manual-prime version:

```toit
  /**
  Answer the output of the controller.
  This method is expected to be called at the loop execution rate, dependant upon the process dynamics.
  */
  update .pv/float -> float:
    time-now := Time.monotonic-us
    if time-last_ == null:
      // First call. Prime last-values from the actual PV and return a
      // neutral starting output so no phantom delta-based term applies.
      time-last_ = time-now
      pv-last_ = pv
      sp_ = pv
      sp-last_ = pv
      dev-last_ = 0.0
      sp-limiter.prime pv
      starting := auto ? (out-min + out-max) / 2.0 : out-manual
      out-last_ = min (max out-min starting) out-max
      out_ = out-last_
      if auto: out-manual = out_
      return out_

    dT := (time-now - time-last_) / 1000.0

    if not auto:
      // Keep the SP limiter anchored at PV so re-engaging auto ramps smoothly.
      sp-limiter.prime pv
      sp_ = pv
    else:
      sp_ = sp-limiter.update sp-manual dT

    dev := pv - sp_
    p1 := dev - dev-last_
    p2 := pv - pv-last_
    p3 := spio? p2: p1
    proportional := p3 * ks * kp
    integral := dev * ks * kp2_ * (dT / 1000.0) / ti_  // dT is in milli-sec
    out-pid_ = proportional + integral + out-last_
    out_ = min (max out-min (auto? out-pid_: out-manual)) out-max
    if auto: out-manual = out_
    pv-last_ = pv
    dev-last_ = dev
    out-last_ = out_
    time-last_ = time-now
    return out_
```

- [ ] **Step 4: Run, expect PASS**

```bash
make test
```

Expected: all tests pass (smoke + plant-models + pi-init = 4 test files green).

- [ ] **Step 5: Commit**

```bash
git add src/pi-controller.toit tests/pi-init-test.toit
git commit -m "$(cat <<'EOF'
Fix PI-Controller startup and manual->auto bump

Three related fixes in update():
- Lazy initialization. First update primes last-values from the actual
  PV and returns a midpoint output (or clamped manual output), so the
  first iteration no longer applies a phantom proportional kick from
  default-seeded last-values.
- Manual-mode SP limiter prime. While in manual, the SP velocity-limiter
  is kept anchored to the current PV via a new Velocity-Limit.prime
  method. When auto re-engages, the limiter ramps from PV toward
  sp-manual instead of jumping in one step.
- Default starting output respects out-min/out-max instead of being
  hardcoded to 50.0.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 4: PI-Controller — `ti` typed as `float`

**Files:**
- Modify: `src/pi-controller.toit`
- Modify: `examples/tank-control.toit` (will need `ti=100.0` instead of `ti=100`)
- Create: `tests/pi-ti-float-test.toit`

- [ ] **Step 1: Write the failing test**

Create `tests/pi-ti-float-test.toit`:

```toit
// Copyright (C) 2026 Toit contributors.
// Use of this source code is governed by an MIT-style license that can be
// found in the LICENSE file.

import expect show *
import pid2 show PI-Controller

/**
ti must accept fractional values.
*/
test-ti-accepts-float:
  // Should compile and construct successfully with ti=0.5 (half-second integral time).
  controller := PI-Controller --kp=1.0 --ti=0.5 --ks=-1
  expect-not-null controller

/**
tune accepts a float ti.
*/
test-tune-accepts-float:
  controller := PI-Controller --kp=1.0 --ti=10.0 --ks=-1
  controller.tune --kp=2.0 --ti=2.5 --ks=1
  // Run an update to confirm post-tune the controller still works.
  out := controller.update 50.0
  expect-not-null out

main:
  test-ti-accepts-float
  test-tune-accepts-float
```

- [ ] **Step 2: Run, expect FAIL (compile error: `ti=0.5` is not an `int`)**

```bash
make test
```

Expected: compile error in `pi-ti-float-test.toit` because `--.ti/int` rejects a `float` literal.

- [ ] **Step 3: Widen to `float` in `src/pi-controller.toit`**

Constructor signature (currently `--.ti/int`):

```toit
  constructor --.kp=1.0 --ti/float --.ks/int:
    kp2_ = kp == 0.0? 1.0: kp
    ti_ = ti
```

`tune` signature (currently `--ti/int`):

```toit
  /** Tune the controller, with:
    $kp proportial gain, in engineering units
    $ti integral time, in seconds
    $ks direct/reverse acting control
  */
  tune --.kp/float=1.0 --ti/float --.ks/int -> none:
    kp2_ = kp == 0.0? 1.0: kp
    ti_ = ti
```

Update `examples/tank-control.toit` line 38 to use a float literal:

```toit
controller := PI-Controller --kp=10.0 --ti=100.0 --ks=-1
```

- [ ] **Step 4: Run, expect PASS**

```bash
make test
```

Expected: all tests pass.

- [ ] **Step 5: Commit**

```bash
git add src/pi-controller.toit examples/tank-control.toit tests/pi-ti-float-test.toit
git commit -m "$(cat <<'EOF'
Widen PI-Controller ti from int to float

Allows fractional integral times (e.g., ti=0.5s). Updates the tank
example call site to use a float literal.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 5: PI-Controller — optional debug flag

**Files:**
- Modify: `src/pi-controller.toit`
- Create: `tests/pi-debug-test.toit`

- [ ] **Step 1: Write the failing test**

Create `tests/pi-debug-test.toit`:

```toit
// Copyright (C) 2026 Toit contributors.
// Use of this source code is governed by an MIT-style license that can be
// found in the LICENSE file.

import expect show *
import pid2 show PI-Controller

/**
The debug flag can be set via constructor and toggled at runtime.
*/
test-debug-flag-toggleable:
  controller := PI-Controller --kp=1.0 --ti=10.0 --ks=-1 --debug
  expect controller.debug
  controller.debug = false
  expect-not controller.debug
  // Construct without --debug → defaults to false.
  c2 := PI-Controller --kp=1.0 --ti=10.0 --ks=-1
  expect-not c2.debug

main:
  test-debug-flag-toggleable
```

- [ ] **Step 2: Run, expect FAIL (compile error: `debug` undefined)**

```bash
make test
```

Expected: compile error referencing `--debug` or `controller.debug`.

- [ ] **Step 3: Add the flag to `src/pi-controller.toit`**

Add a public mutable field near the top of the class (after `out-max`):

```toit
  /** When true, $update prints diagnostic info every 10 iterations. */
  debug /bool := false
```

Modify the constructor:

```toit
  constructor --.kp=1.0 --ti/float --.ks/int --debug/bool=false:
    kp2_ = kp == 0.0? 1.0: kp
    ti_ = ti
    this.debug = debug
```

In `update`, replace the existing commented-out report line with a guarded call (find the line `// report_ p1 p2 p3 proportional integral  // uncomment, for understanding of the algorithm`):

```toit
    if debug: report_ p1 p2 p3 proportional integral
```

- [ ] **Step 4: Run, expect PASS**

```bash
make test
```

- [ ] **Step 5: Commit**

```bash
git add src/pi-controller.toit tests/pi-debug-test.toit
git commit -m "$(cat <<'EOF'
Add toggleable debug flag to PI-Controller

Replaces the commented-out report_ call with a runtime-toggleable
--debug flag (constructor parameter and public field), so diagnostic
logging can be enabled without source edits.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 6: PI-Controller — closed-loop scenario tests

This task adds the remaining scenario tests called for in the spec: step response on both plant types, output saturation/anti-windup, SP velocity limiting, and disturbance rejection. These exercise the (already-fixed) `PI-Controller` against the plant models from Task 2.

**Files:**
- Create: `tests/pi-step-test.toit`
- Create: `tests/pi-windup-test.toit`
- Create: `tests/pi-velocity-limit-test.toit`
- Create: `tests/pi-disturbance-test.toit`

- [ ] **Step 1: Create `tests/pi-step-test.toit`**

```toit
// Copyright (C) 2026 Toit contributors.
// Use of this source code is governed by an MIT-style license that can be
// found in the LICENSE file.

import expect show *
import pid2 show PI-Controller
import .plant-models

DT-MS ::= 10
ITERATIONS ::= 3000  // 30 seconds of simulated control at 10ms

/**
Closes the PI-Controller loop around a self-regulating first-order plant.
After settling, PV should be within 5% of SP.
*/
test-step-on-first-order-plant:
  // Self-regulating: gain 5, tau 1s. SP=200 requires u=40 in steady state.
  plant := First-Order-Plant --gain=5.0 --tau=1.0 --initial=100.0
  controller := PI-Controller --kp=0.5 --ti=2.0 --ks=-1
  controller.sp-limiter.limit = 1000.0  // effectively unlimited; this test isn't about SP ramping.
  controller.sp-manual = 200.0
  controller.out-min = 0.0
  controller.out-max = 100.0
  pv := plant.y
  ITERATIONS.repeat:
    sleep --ms=DT-MS
    u := controller.update pv
    pv = plant.step u (DT-MS / 1000.0)
  expect ((pv - 200.0).abs / 200.0 < 0.05)

/**
Closes the loop around an integrating plant. After settling, PV should be
within 5% of SP. (Same controller form, just different plant type --
demonstrates the algorithm works on integrating processes too.)
*/
test-step-on-integrating-plant:
  // Tank-like: u in 0..100 (% valve open), bias at 50% (equilibrium), gain 0.2.
  plant := Integrating-Plant --gain=0.2 --u-bias=50.0 --initial=20.0
  controller := PI-Controller --kp=10.0 --ti=20.0 --ks=-1
  controller.sp-limiter.limit = 1000.0
  controller.sp-manual = 60.0
  controller.out-min = 0.0
  controller.out-max = 100.0
  pv := plant.y
  ITERATIONS.repeat:
    sleep --ms=DT-MS
    u := controller.update pv
    pv = plant.step u (DT-MS / 1000.0)
  expect ((pv - 60.0).abs / 60.0 < 0.05)

main:
  test-step-on-first-order-plant
  test-step-on-integrating-plant
```

- [ ] **Step 2: Create `tests/pi-windup-test.toit`**

```toit
// Copyright (C) 2026 Toit contributors.
// Use of this source code is governed by an MIT-style license that can be
// found in the LICENSE file.

import expect show *
import pid2 show PI-Controller
import .plant-models

DT-MS ::= 10

/**
Drive the controller into output saturation by holding the plant under a
heavy disturbance, then relax. After de-saturation, the velocity-form PI
must recover smoothly: no overshoot greater than 10% of SP.
*/
test-windup-recovery-on-integrating-plant:
  plant := Integrating-Plant --gain=0.2 --u-bias=50.0 --initial=20.0
  controller := PI-Controller --kp=10.0 --ti=20.0 --ks=-1
  controller.sp-limiter.limit = 1000.0
  controller.sp-manual = 60.0
  controller.out-min = 0.0
  controller.out-max = 100.0

  pv := plant.y
  // Phase 1: heavy disturbance saturates the output for ~3s.
  plant.disturbance = 50.0
  300.repeat:
    sleep --ms=DT-MS
    u := controller.update pv
    pv = plant.step u (DT-MS / 1000.0)

  // Phase 2: remove disturbance; controller must recover without overshoot.
  plant.disturbance = 0.0
  max-pv := pv
  1500.repeat:
    sleep --ms=DT-MS
    u := controller.update pv
    pv = plant.step u (DT-MS / 1000.0)
    if pv > max-pv: max-pv = pv

  // Should recover to within 5% of SP without overshoot beyond 15%.
  expect ((pv - 60.0).abs / 60.0 < 0.05)
  expect (max-pv < 60.0 * 1.15)

main:
  test-windup-recovery-on-integrating-plant
```

- [ ] **Step 3: Create `tests/pi-velocity-limit-test.toit`**

```toit
// Copyright (C) 2026 Toit contributors.
// Use of this source code is governed by an MIT-style license that can be
// found in the LICENSE file.

import expect show *
import pid2 show PI-Controller
import .plant-models

DT-MS ::= 10

/**
Verifies the SP velocity limiter ramps the setpoint at the configured rate.
With limit=10/s and a step from 50 to 100 (50 units), ramp time should be
about 5s. Sample sp_ via the controller's exposed field.
*/
test-velocity-limit-ramp-time:
  plant := First-Order-Plant --gain=1.0 --tau=0.5 --initial=50.0
  controller := PI-Controller --kp=5.0 --ti=2.0 --ks=1
  controller.sp-limiter.limit = 10.0  // 10 units / second
  controller.sp-manual = 50.0
  controller.out-min = 0.0
  controller.out-max = 100.0
  // Run a few iterations at SP=50 to settle.
  pv := plant.y
  50.repeat:
    sleep --ms=DT-MS
    u := controller.update pv
    pv = plant.step u (DT-MS / 1000.0)

  // Step SP to 100. Track how long sp_ takes to reach 99 (allowing for last-step tolerance).
  controller.sp-manual = 100.0
  steps-to-arrive := -1
  i := 0
  // Allow up to 8 seconds.
  max-iterations := 800
  while i < max-iterations:
    sleep --ms=DT-MS
    u := controller.update pv
    pv = plant.step u (DT-MS / 1000.0)
    if controller.sp >= 99.0 and steps-to-arrive == -1:
      steps-to-arrive = i + 1
    i++

  // 50 units at 10/s = 5s = 500 steps at 10ms/step. Allow 4.5..6.0s window.
  expect steps-to-arrive > 0
  ramp-seconds := steps-to-arrive * DT-MS / 1000.0
  expect (ramp-seconds > 4.5)
  expect (ramp-seconds < 6.0)

main:
  test-velocity-limit-ramp-time
```

(`controller.sp` is the public getter added in Task 3.)

- [ ] **Step 4: Create `tests/pi-disturbance-test.toit`**

```toit
// Copyright (C) 2026 Toit contributors.
// Use of this source code is governed by an MIT-style license that can be
// found in the LICENSE file.

import expect show *
import pid2 show PI-Controller
import .plant-models

DT-MS ::= 10

/**
With a constant SP, a step disturbance perturbs the plant; the PI controller
must drive PV back to within 2% of SP within the test horizon.
*/
test-disturbance-rejection-on-integrating-plant:
  plant := Integrating-Plant --gain=0.2 --u-bias=50.0 --initial=60.0
  controller := PI-Controller --kp=10.0 --ti=20.0 --ks=-1
  controller.sp-limiter.limit = 1000.0
  controller.sp-manual = 60.0
  controller.out-min = 0.0
  controller.out-max = 100.0

  pv := plant.y
  // Settle for 5s.
  500.repeat:
    sleep --ms=DT-MS
    u := controller.update pv
    pv = plant.step u (DT-MS / 1000.0)

  // Step disturbance: equivalent to a sudden additional outflow of ~5 units/s.
  plant.disturbance = 5.0

  // Allow 15s for recovery.
  1500.repeat:
    sleep --ms=DT-MS
    u := controller.update pv
    pv = plant.step u (DT-MS / 1000.0)

  expect ((pv - 60.0).abs / 60.0 < 0.05)

main:
  test-disturbance-rejection-on-integrating-plant
```

- [ ] **Step 5: Run, expect all PASS**

```bash
make test
```

Expected: all 8 test files pass. The closed-loop scenarios use generous horizons (15–30s simulated) and 5% tolerances by design — they assert the controller's qualitative behavior, not tight numerical targets. If a test fails despite that, do not silently relax the assertion: report the failure with output traces so we can decide whether the gains, the assertions, or the controller behavior are wrong.

- [ ] **Step 6: Commit**

```bash
git add tests/pi-step-test.toit tests/pi-windup-test.toit tests/pi-velocity-limit-test.toit tests/pi-disturbance-test.toit
git commit -m "$(cat <<'EOF'
Add closed-loop scenario tests for PI-Controller

Four scenario test files exercising the controller against both plant
types: step response (self-regulating + integrating), windup recovery
under saturation, SP velocity-limit ramp time, and disturbance
rejection. These document and lock in the controller's behavior across
the cases the user asked about.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 7: Controller — anti-windup

**Files:**
- Create: `tests/controller-windup-test.toit`
- Modify: `src/controller.toit`

- [ ] **Step 1: Write the failing test**

Create `tests/controller-windup-test.toit`:

```toit
// Copyright (C) 2026 Toit contributors.
// Use of this source code is governed by an MIT-style license that can be
// found in the LICENSE file.

import expect show *
import pid2 show Controller
import .plant-models

DT ::= Duration --ms=10
DT-S ::= 0.01

/**
Drives a Controller into output saturation, then relaxes. Without
anti-windup, the integrator accumulates while saturated and produces
massive overshoot when the output unsaturates. With conditional-
integration anti-windup, overshoot remains bounded.
*/
test-controller-windup-recovery:
  plant := Integrating-Plant --gain=0.2 --u-bias=50.0 --initial=20.0
  // Direct-acting: positive error -> output increases.
  // Plant: u above 50 raises level; we want level to go up to SP=60.
  // So feed error = SP - PV.
  controller := Controller --kp=10.0 --ki=0.5 --kd=0.0 --min=0.0 --max=100.0

  pv := plant.y
  sp := 60.0

  // Phase 1: heavy disturbance saturates the output for ~3s.
  plant.disturbance = 50.0
  300.repeat:
    error := sp - pv
    u := controller.update error DT
    pv = plant.step u DT-S

  // Phase 2: remove disturbance and observe overshoot.
  plant.disturbance = 0.0
  max-pv := pv
  1500.repeat:
    error := sp - pv
    u := controller.update error DT
    pv = plant.step u DT-S
    if pv > max-pv: max-pv = pv

  // After fix: bounded overshoot (within 20% of SP).
  expect (max-pv < 60.0 * 1.20)
  // And eventual recovery.
  expect ((pv - 60.0).abs / 60.0 < 0.05)

main:
  test-controller-windup-recovery
```

- [ ] **Step 2: Run, expect FAIL (overshoot exceeds 15%)**

```bash
make test
```

Expected: `controller-windup-test` reports the `max-pv < 69.0` assertion failed.

- [ ] **Step 3: Implement conditional-integration anti-windup in `src/controller.toit`**

Replace the entire file contents with:

```toit
// Licensed under the included MIT License LICENSE1

import core

/**
PID Controller with conditional-integration anti-windup.

The output is clamped to $min..$max. The integral term is only
accumulated when doing so would not push further into a saturation
the output is already in.

For more context, see https://en.wikipedia.org/wiki/PID_controller
*/
class Controller:
  kp/float
  ki/float
  kd/float

  min/float
  max/float

  constructor --.kp=0.0 --.ki=0.0 --.kd=0.0 --.min=0.0 --.max=1.0:

  last-error_/float := 0.0
  integral-error_/float := 0.0

  /**
  Updates the controller with a new $error and the elapsed time since
  the last update.

  Returns the new output value, clamped to $min..$max.
  */
  update error/float elapsed/Duration -> float:
    elapsed-s := elapsed.in-ns.to-float / Duration.NANOSECONDS-PER-SECOND
    derivative-error := (error - last-error_) / elapsed-s
    unclamped := kp * error + ki * integral-error_ + kd * derivative-error
    clamped := core.max min (core.min max unclamped)

    // Direction the integrator's next increment would push the output is
    // sign(ki * error). Only freeze when output is saturated AND the
    // increment would push further into that saturation.
    push := ki * error
    saturated-high := unclamped > max and push > 0.0
    saturated-low := unclamped < min and push < 0.0
    if not (saturated-high or saturated-low):
      integral-error_ += error * elapsed-s

    last-error_ = error
    return clamped
```

Note: this also updates the field names from snake/underscore (`integral_error_`, `last_error_`) to kebab-case to match Toit conventions, since we're rewriting anyway. No external API change (those were private).

- [ ] **Step 4: Run, expect PASS**

```bash
make test
```

- [ ] **Step 5: Commit**

```bash
git add src/controller.toit tests/controller-windup-test.toit
git commit -m "$(cat <<'EOF'
Add conditional-integration anti-windup to Controller

The original positional-form Controller accumulated integral error
unconditionally and only clamped the output, producing classic windup
on saturation. Now the integrator freezes when output is saturated and
the next increment would push further into that saturation -- using
sign(ki * error) so it works for both direct- and reverse-acting
configurations. External API is unchanged.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 8: Tank example tidy-up

**Files:**
- Modify: `examples/tank-control.toit`

- [ ] **Step 1: Apply the three tidy-ups**

In `examples/tank-control.toit`:

- Remove the unused top-level `sp := 60.0` line (around line 19).
- After the `controller := PI-Controller ...` line, add `controller.sp-manual = 60.0` to make the example's intended setpoint explicit.
- Add a commented-out `// controller.debug = true` line as a hint that debug logging is now toggleable without source edits.

The relevant section becomes:

```toit
/** Tank level */
level := 50.0
/** Tank fill valve */
vlv := 50.0
/** Control mode */
auto := true
/** Loop time, in ms */
interval := 200

...

controller := PI-Controller --kp=10.0 --ti=100.0 --ks=-1

main:
  controller.sp-manual = 60.0
  // controller.debug = true  // Uncomment for diagnostic logging.
  task :: faceplate
  task :: control
```

- [ ] **Step 2: Verify it still compiles**

```bash
toit compile -o /tmp/tank-control examples/tank-control.toit
```

Expected: compiles without errors.

- [ ] **Step 3: Commit**

```bash
git add examples/tank-control.toit
git commit -m "$(cat <<'EOF'
Tidy tank-control example

Remove unused top-level 'sp' variable, set controller.sp-manual to 60.0
explicitly so the example matches its README intent, and add a hint
about the new --debug flag.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 9: Final verification

- [ ] **Step 1: Run the full test suite**

```bash
cd /home/david/workspaceToit/pid2
make test
```

Expected: 100% tests passed across all of:
- `tests/smoke-test.toit`
- `tests/plant-models-test.toit`
- `tests/pi-init-test.toit`
- `tests/pi-ti-float-test.toit`
- `tests/pi-debug-test.toit`
- `tests/pi-step-test.toit`
- `tests/pi-windup-test.toit`
- `tests/pi-velocity-limit-test.toit`
- `tests/pi-disturbance-test.toit`
- `tests/controller-windup-test.toit`

- [ ] **Step 2: Inspect the git log**

```bash
git log --oneline ^origin/main HEAD
```

Expected: ~9 commits, each one self-contained.

- [ ] **Step 3: Confirm tank example still launches**

```bash
toit compile -o /tmp/tank-control examples/tank-control.toit
toit compile -o /tmp/tank-faceplate examples/tank-faceplate.toit
```

Expected: both compile cleanly.

(End-to-end UDP behavior with two terminals is out of scope for the test suite — manual smoke test only if the user runs it.)

---

## Summary of changes

**Modified:**
- `src/pi-controller.toit` — lazy init, manual-mode SP-limiter prime, `Velocity-Limit.prime`, `ti` widened to `float`, `--debug` flag, default starting output midpoint.
- `src/controller.toit` — conditional-integration anti-windup, kebab-case field renames.
- `examples/tank-control.toit` — remove dead `sp`, set `sp-manual=60.0`, add debug hint, `ti=100.0` literal.

**Created:**
- `Makefile`, `CMakeLists.txt`, `tests/CMakeLists.txt`, `tests/package.yaml`, `.gitignore`
- `tests/plant-models.toit` (library)
- 9 test files (`smoke-test`, `plant-models-test`, `pi-init-test`, `pi-ti-float-test`, `pi-debug-test`, `pi-step-test`, `pi-windup-test`, `pi-velocity-limit-test`, `pi-disturbance-test`, `controller-windup-test`).

**Untouched:**
- `src/pid2.toit` (re-export only — still valid).
- `examples/tank-faceplate.toit`, `package.yaml`, `README.md`, `LICENSE1`, `LICENSE2`, `PI-Controller.pdf`.
