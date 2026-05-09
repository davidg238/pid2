// Licensed under the included MIT License LICENSE2

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
