// Licensed under the included MIT License LICENSE2

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
  expect (steady - 2.0).abs < 0.02  // within 1% of 2.0

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
