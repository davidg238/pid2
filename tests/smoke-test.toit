// Licensed under the included MIT License LICENSE2

import expect show *
import pid2 show PI-Controller

/**
Smoke test: the package can be imported and a controller can be constructed.
*/
main:
  controller := PI-Controller --kp=1.0 --ti=10.0 --ks=-1
  expect-not-null controller
