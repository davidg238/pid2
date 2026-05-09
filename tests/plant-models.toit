// Licensed under the included MIT License LICENSE2

/**
Deterministic plant simulations for closed-loop controller testing.

Two plant types are provided, matching the two process classes commonly
discussed in control engineering:

  $First-Order-Plant: self-regulating process (e.g. flow through a valve).
  The output settles to a steady-state value `gain * input` for a constant
  input, with first-order dynamics characterized by time constant `tau`.

  $Integrating-Plant: integrating process (e.g. tank level, where flow in
  vs. flow out determines the rate of level change). The output rises or
  falls at a rate proportional to the offset between input and `u-bias`.

The two classes use opposite sign conventions for `disturbance`: it is
additive on the rate for $First-Order-Plant (e.g. a heat-load disturbance)
and subtractive on the rate for $Integrating-Plant (e.g. an outflow on a
tank). Each class's docstring describes the convention used.
*/

/**
A first-order plant: dy/dt = (gain*u - y) / tau + disturbance.

Self-regulating: a constant input drives the output to a steady-state
value `gain * u + tau * disturbance`.
*/
class First-Order-Plant:
  gain/float
  tau/float
  y/float := ?
  /** Added to the rate of change (e.g. a load disturbance). */
  disturbance/float := 0.0

  /**
  Constructs a first-order plant with the given $gain (steady-state output per
  unit input), time constant $tau (in seconds), and starting output $initial.
  */
  constructor --.gain/float=1.0 --.tau/float=1.0 --initial/float=0.0:
    y = initial

  /**
  Advances the plant by $dt seconds with input $u.
  Returns the new output value.
  */
  step u/float dt/float -> float:
    dy := ((gain * u - y) / tau + disturbance) * dt
    y += dy
    return y

/**
An integrating plant: dy/dt = gain * (u - u-bias) - disturbance.

Output rises at rate `gain * (u - u-bias)` for u above bias, falls below.
$disturbance is subtracted from the rate (e.g. an outflow on a tank).
*/
class Integrating-Plant:
  gain/float
  u-bias/float
  y/float := ?
  /** Subtracted from the rate of change (e.g. an outflow on a tank). */
  disturbance/float := 0.0

  /**
  Constructs an integrating plant with the given $gain (output rate per unit
  input above $u-bias) and starting output $initial.
  */
  constructor --.gain/float=1.0 --.u-bias/float=0.0 --initial/float=0.0:
    y = initial

  /**
  Advances the plant by $dt seconds with input $u.
  Returns the new output value.
  */
  step u/float dt/float -> float:
    dy := (gain * (u - u-bias) - disturbance) * dt
    y += dy
    return y
