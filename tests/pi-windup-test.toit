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
must recover smoothly: no overshoot greater than 15% of SP.
*/
test-windup-recovery-on-integrating-plant:
  plant := Integrating-Plant --gain=0.2 --u-bias=50.0 --initial=20.0
  // kp=5, ti=2: see pi-step-test for tuning rationale.
  controller := PI-Controller --kp=5.0 --ti=2.0 --ks=-1
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
  // After heavy disturbance, pv drops well below 0 (plant unclamped). Recovery
  // is bounded by the saturated valve flow rate (gain * (out-max - u-bias) =
  // 0.2 * 50 = 10 units/s), so allow ~25s.
  plant.disturbance = 0.0
  max-pv := pv
  2500.repeat:
    sleep --ms=DT-MS
    u := controller.update pv
    pv = plant.step u (DT-MS / 1000.0)
    if pv > max-pv: max-pv = pv

  // Should recover to within 5% of SP without overshoot beyond 15%.
  expect ((pv - 60.0).abs / 60.0 < 0.05)
  expect (max-pv < 60.0 * 1.15)

main:
  test-windup-recovery-on-integrating-plant
