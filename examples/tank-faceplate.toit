// Licensed under the included MIT License LICENSE2

/**
A simple control Faceplate, to control the tank level simulation.
  Run with `jag run -d host tank-faceplate.toit`
*/

import host.pipe

import system
import net
import net.udp

in := pipe.stdin

main:

  print """
  Usage:
    command

  Available commands:
    a - auto mode
    m - manual mode
    sp <value> - setpoint
    out <value> - manual output
  """

  network := net.open
  socket := network.udp-open

  while true:
    payload := in.read.to-string.trim.to-byte-array
    datagram := udp.Datagram
      payload
      net.SocketAddress
        net.IpAddress.parse "127.0.0.1"
        12345
    socket.send datagram
