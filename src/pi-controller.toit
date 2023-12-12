// Licensed under the included MIT License LICENSE2

class PI-Controller:

  // Refer to PI-Controller.pdf for an explanation of function and usage.
  // The following are the 3 inputs set by the human operator
  sp-manual  := 50.0        // setpoint, set by the operator
  auto /bool := true        // auto/manual mode
  out-manual /float  := 50.0 // operator output, used in manual
  
  // SP velocity limiting, the preferred method to minimize impact of SP changes
  sp-limiter := Velocity-Limit --limit=1.0


  pv /float  := 50.0        // process variable
  sp /float  := 50.0        // setpoint, internal
  /*
  Integral only action, on setpoint change.
  Used in older systems, only works where significant loop integral action is available,
  otherwise SP changes will be very sluggish.
  */
  spio /bool := false       // default, disabled

  out-pid /float := 0.0     // pid algorithm output
  out /float := 50.0        // output
  ks /int    := -1          // direct 1, reverse -1 (reverse: error = SP - PV)
  kp /float  := 1.0         // proportional gain.  If kp=0, integral only controller
  ti_ /float := 1.0         // integral time constant
  kp2 /float := 0.0
  dT /float? := null     // time interval in milli-sec

  /// The manadatory limits of the control output action
  out-min := 0.0            // minimum output limit
  out-max := 100.0          // maximum output limit

  time-now /int? := 0
  time-last /int? := 0
  pv-last /float := 50.0
  sp-last /float := 50.0
  dev-last /float := 0.0
  out-last /float := 50.0
  n := 0

  constructor --.kp=1.0 --ti/int --.ks/int:
    time-last = Time.monotonic-us
    kp2 = kp == 0.0? 1.0: kp
    ti_ = ti * 1.0

  update .pv/float -> float:
    time-now = Time.monotonic-us
    dT = (time-now - time-last) / 1000.0

    s1 := auto? sp-manual: pv
    sp = not auto? s1: (sp-limiter.update s1 dT)

    dev := pv - sp
    p1 := dev - dev-last
    p2 := pv - pv-last
    p3 := spio? p2: p1
    proportional := p3 * ks * kp
    integral := dev * ks * kp2 * (dT / 1000.0) / ti_ //since dT is in milli-sec
    out-pid = proportional + integral + out-last
    out = min (max out-min (auto? out-pid: out-manual)) out-max
    if auto: out-manual = out
    report_ p1 p2 p3 proportional integral  // uncomment, for understanding of the algorithm
    pv-last = pv
    dev-last = dev
    out-last = out
    time-last = time-now
    return out

  tune --.kp=1.0 --ti/int --.ks/int -> none:
    kp2 = kp == 0.0? 1.0: kp
    ti_ = ti * 1.0

  report_ p1 p2 p3 proportional integral -> none:
    if n % 10 == 0: // adjust this depending on the tick interval and how often you want to report
      mode := auto? "A": "M"
      sp-mode := spio? "I": "RL"
      print "pv $(%.1f pv) sp $(%.1f sp) out $(%.1f out) | P $(%.3f proportional) I $(%.3f integral) | $mode  $sp-mode sp-m $(%.0f sp-manual) out-m $(%.1f out-manual)" //  a/m $(auto) op_co $(op_co)
    n += 1

class Velocity-Limit:

  limit /float := 2.0  // Limit the rate of change to +/- limit per second
  dT /float? := null
  last /float? := null

  constructor --.limit/float=2.0:

  update val/float .dT/float -> float:
    if last == null:
      last = val
      return val

    velocity := (val - last) * 1000.0 / dT 
    if velocity.abs > limit:
      velocity = (velocity > 0)? limit: -limit
    last += (velocity * dT / 1000.0)
    return last
