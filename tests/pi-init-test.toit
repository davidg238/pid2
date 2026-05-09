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
