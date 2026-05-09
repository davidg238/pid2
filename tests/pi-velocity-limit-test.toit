// Licensed under the included MIT License LICENSE2

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
