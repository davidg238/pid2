// Licensed under the included MIT License LICENSE1

import core

/**
PID Controller with conditional-integration anti-windup.

The output is clamped to $min..$max. The integral term is only
accumulated when doing so would not push further into a saturation
the output is already in.

For more context, see https://en.wikipedia.org/wiki/PID_controller
*/
class Controller:
  kp/float
  ki/float
  kd/float

  min/float
  max/float

  constructor --.kp=0.0 --.ki=0.0 --.kd=0.0 --.min=0.0 --.max=1.0:

  last-error_/float := 0.0
  integral-error_/float := 0.0

  /**
  Updates the controller with a new $error and the elapsed time since
  the last update.

  Returns the new output value, clamped to $min..$max.
  */
  update error/float elapsed/Duration -> float:
    elapsed-s := elapsed.in-ns.to-float / Duration.NANOSECONDS-PER-SECOND
    derivative-error := (error - last-error_) / elapsed-s
    unclamped := kp * error + ki * integral-error_ + kd * derivative-error
    clamped := core.max min (core.min max unclamped)

    // Direction the integrator's next increment would push the output is
    // sign(ki * error). Only freeze when output is saturated AND the
    // increment would push further into that saturation.
    push := ki * error
    saturated-high := unclamped > max and push > 0.0
    saturated-low := unclamped < min and push < 0.0
    if not (saturated-high or saturated-low):
      integral-error_ += error * elapsed-s

    last-error_ = error
    return clamped
