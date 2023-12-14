// Licensed under the included MIT License LICENSE2

/**
A simple tank level simulation, to demonstrate usage of the PI-Controller.
  Run with:  `jag run -d host tank-control.toit`

Use tank-faceplate.toit to control the simulation.
*/

import pid2 show PI-Controller

import system
import net
import net.udp

/** Tank level */
level := 50.0
/** Tank level setpoint */
sp := 60.0
/** Tank fill valve */
vlv := 50.0
/** Control mode */
auto := true
/** Loop time, in ms */
interval := 200 

/** 
Answers the simulated tank level, 0-100 gallons
  last, the last tank level
  in, inflow in gpm  (since the fill valve (0-100%) results in 0-20gpm flow)
  loss, random system perturbation, outflow in gpm
*/  
tank := :: | last in|
  loss := random 4 12  
  new := last + (in/5 - loss)*(interval/60_000.0)
  min (max 0.0 new) 100.0  // clamp tank level to 0-100gal

controller := PI-Controller --kp=10.0 --ti=100 --ks=-1

main:
  task :: faceplate
  task :: control

/** Task to accept operater inputs from the `tank-faceplate` process */
faceplate:

  network := net.open
  socket := network.udp-open --port=12345

  while true:
    result := socket.receive.data.to-string
    print "Operator: $result"
    list := result.split " "
    if list.size == 1:
      cmd := list[0]
      if cmd == "a":
        controller.auto = true
      else if cmd == "m":
        controller.auto = false
    else if list.size == 2:
      cmd := list[0]
      val := float.parse list[1]
      if cmd == "sp":
        controller.sp-manual = val
      else if cmd == "out":
        controller.out-manual = val

/** The control loop, executed every $interval ms*/
control:
  while true:
    level = tank.call level vlv
    vlv = controller.update level
    sleep --ms=interval