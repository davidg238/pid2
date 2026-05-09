// Copyright (C) 2026 Toit contributors.
// Use of this source code is governed by an MIT-style license that can be
// found in the LICENSE file.

import expect show *
import pid2 show PI-Controller

/**
Smoke test: the package can be imported and a controller can be constructed.
*/
main:
  controller := PI-Controller --kp=1.0 --ti=10 --ks=-1
  expect-not-null controller
