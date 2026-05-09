// Licensed under the included MIT License LICENSE2

/**
The positional `Controller` driving the same simulated tank as
`tank-control.toit`. Use this for side-by-side comparison with the
velocity-form `PI-Controller`: same plant, different controller form.

Run with:  `jag run -d host examples/level-controller-pid.toit`

The controller computes its own error from a hardcoded SP and a measured
`level`. There is no operator faceplate; periodic stdout reporting takes
its place.
*/

import pid2 show Controller

/** Tank level (gallons, 0..100) */
level := 50.0
/** Tank fill valve position (%, 0..100) */
vlv := 40.0
/** Tank level setpoint (gallons) */
sp := 60.0
/** Loop time, in ms */
interval := 200

/**
Simulated tank dynamics, identical to `tank-control.toit`. The block
takes the previous tank level and the fill-valve position (0..100);
inflow is valve/5 gpm and outflow is a random 4..12 gpm representing
downstream demand.
*/
tank := :: | last/float in/float |
  loss := random 4 12
  new := last + (in / 5.0 - loss) * (interval / 60_000.0)
  min (max 0.0 new) 100.0  // clamp to 0..100 gal

/**
Positional Controller. Tuning notes:
  - kp gives an immediate response to error.
  - ki is small because the loop period is in seconds (200 ms) and the
    integrator accumulates `error * elapsed_s`; a large ki would build
    up too quickly.
  - kd=0: derivative on a noisy random-outflow process would amplify
    noise without helping.
  - min/max clamp the valve to its physical range.
*/
controller := Controller --kp=2.0 --ki=0.5 --kd=0.0 --min=0.0 --max=100.0

main:
  print "Step   Time(s)  Level    Valve    Error"
  step := 0
  while step < 500:  // 500 * 200 ms = 100 s
    level = tank.call level vlv
    error := sp - level
    vlv = controller.update error (Duration --ms=interval)
    if step % 25 == 0:
      seconds := step * interval / 1000.0
      print "$(%4d step)   $(%5.1f seconds)   $(%5.2f level)   $(%5.2f vlv)   $(%6.2f error)"
    sleep --ms=interval
    step++
  print "Final level: $(%.2f level)  (SP=$(%.2f sp), error=$(%.2f sp - level))"
