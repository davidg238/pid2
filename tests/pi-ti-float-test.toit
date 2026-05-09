// Licensed under the included MIT License LICENSE2

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
