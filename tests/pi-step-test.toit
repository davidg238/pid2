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
  // kp=5, ti=2: aggressive enough that the velocity-form integral builds u
  // back toward bias before the proportional term throttles it off (the
  // plan's original kp=10 ti=20 was too sluggish for this plant).
  plant := Integrating-Plant --gain=0.2 --u-bias=50.0 --initial=20.0
  controller := PI-Controller --kp=5.0 --ti=2.0 --ks=-1
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
