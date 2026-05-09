// Licensed under the included MIT License LICENSE2

  /**
  A PI controller, with a faceplate input for manual operation of setpoint, auto/manual mode and manual output.
  Refer to PI-Controller.pdf for an explanation of function and usage.
  */

class PI-Controller:

  /** setpoint, by operator */
  sp-manual  := 50.0
  /** auto/manual mode, by operator */
  auto /bool := true
  /** manual output, by operator */
  out-manual /float  := 50.0
  
  /** SP velocity limiting, the preferred method to minimize impact of SP changes */
  sp-limiter := Velocity-Limit --limit=1.0

  /** loop process variable */
  pv /float  := 50.0
  /** The current internal setpoint, after auto/manual select and SP velocity limiting. */
  sp -> float: return sp_
  /**
  Integral only action, on setpoint change.
  Used in older systems, only works where significant loop integral action is available,
  otherwise SP changes will be very sluggish.
  Disabled by default.
  */
  spio /bool := false
  /** Set as 1 for direct, -1 for reverse action (reverse: error = SP - PV) */
  ks /int    := -1
  /** Proportional gain term.
  If kp=0, controller action is integral only.
  */
  kp /float  := 1.0
  /** The minimum limit of the control output */
  out-min := 0.0
  /** The maximum limit of the control output */
  out-max := 100.0

  /** When true, $update prints diagnostic info every 10 iterations. */
  debug /bool := false

  sp_ /float  := 50.0
  ti_ /float := 1.0       // integral time constant
  out-pid_ /float := 0.0  // pid algorithm output
  out_ /float := 50.0     // output
  kp2_ /float := 0.0

  time-last_ /int? := null
  pv-last_ /float := 0.0
  sp-last_ /float := 0.0
  dev-last_ /float := 0.0
  out-last_ /float := 0.0
  n_ := 0

  /**
  Creates a PI controller with nominated proportional gain, integral time and direct/reverse action.
  */
  constructor --.kp=1.0 --ti/float --.ks/int --debug/bool=false:
    kp2_ = kp == 0.0? 1.0: kp
    ti_ = ti
    this.debug = debug

  /** 
  Answer the output of the controller.  
  This method is expected to be called at the loop execution rate, dependant upon the process dynamics.
  */
  update .pv/float -> float:
    time-now := Time.monotonic-us
    if time-last_ == null:
      // First call. Prime last-values from the actual PV and return a
      // neutral starting output so no phantom delta-based term applies.
      time-last_ = time-now
      pv-last_ = pv
      sp_ = pv
      sp-last_ = pv
      dev-last_ = 0.0
      sp-limiter.prime pv
      starting := auto ? (out-min + out-max) / 2.0 : out-manual
      out-last_ = min (max out-min starting) out-max
      out_ = out-last_
      if auto: out-manual = out_
      return out_

    dT := (time-now - time-last_) / 1000.0

    if not auto:
      // Keep the SP limiter anchored at PV so re-engaging auto ramps smoothly.
      sp-limiter.prime pv
      sp_ = pv
    else:
      sp_ = sp-limiter.update sp-manual dT

    dev := pv - sp_
    p1 := dev - dev-last_
    p2 := pv - pv-last_
    p3 := spio? p2: p1
    proportional := p3 * ks * kp
    integral := dev * ks * kp2_ * (dT / 1000.0) / ti_ //since dT is in milli-sec
    out-pid_ = proportional + integral + out-last_
    out_ = min (max out-min (auto? out-pid_: out-manual)) out-max
    if auto: out-manual = out_
    if debug: report_ p1 p2 p3 proportional integral
    pv-last_ = pv
    dev-last_ = dev
    out-last_ = out_
    time-last_ = time-now
    return out_

  /** Tune the controller, with:
    $kp proportial gain, in engineering units
    $ti integral time, in seconds
    $ks direct/reverse acting control
  */
  tune --.kp/float=1.0 --ti/float --.ks/int -> none:
    kp2_ = kp == 0.0? 1.0: kp
    ti_ = ti


  report_ p1 p2 p3 proportional integral -> none:
    if n_ % 10 == 0: 
      mode := auto? "A": "M"
      sp-mode := spio? "I": "RL"
      print "pv $(%.1f pv) sp $(%.1f sp_) out $(%.1f out_) | P $(%.3f proportional) I $(%.3f integral) | $mode  $sp-mode sp-m $(%.0f sp-manual) out-m $(%.1f out-manual)" 
    n_ += 1

/**
A module used in control schemes to limit the rate of change of a variable.
*/
class Velocity-Limit:

  /** Limit the rate of change of the value to +/- limit per second */
  limit /float := 2.0  
  last_ /float? := null

  constructor --.limit/float=2.0:

  /** Answer the limited value of $val.
  $dT must be set with the loop execution time, in milli-seconds. */
  update val/float dT/float -> float:
    if last_ == null:
      last_ = val
      return val
    velocity := (val - last_) * 1000.0 / dT
    if velocity.abs > limit:
      velocity = (velocity > 0)? limit: -limit
    last_ += (velocity * dT / 1000.0)
    return last_

  /**
  Forces the limiter's internal anchor to $val so that the next call to $update
  ramps from $val toward its target at the configured velocity. Used by the
  enclosing controller to keep the limiter in sync with PV during manual mode.
  */
  prime val/float -> none:
    last_ = val
