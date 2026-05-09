// Licensed under the included MIT License LICENSE2

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
  2500.repeat:
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
