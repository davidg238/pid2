// Licensed under the included MIT License LICENSE2

import expect show *
import pid2 show PI-Controller
import .plant-models

DT-MS ::= 10

/**
With a constant SP, a step disturbance perturbs the plant; the PI controller
must drive PV back to within 5% of SP within the test horizon.
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
