# pid2

PID controllers for closed-loop process control on Toit.

This package provides two controllers, intended for two different audiences:

- **`Controller`** — a textbook positional PID. Takes an `error` and an
  elapsed `Duration` per call, returns a clamped output. Suitable when the
  caller already has a setpoint, can compute its own error, and wants the
  smallest possible API surface. Now includes conditional-integration
  anti-windup so the integrator no longer accumulates while the output is
  pinned against `min` / `max`.

- **`PI-Controller`** — a velocity-form PI with the operator-facing
  features needed for a real industrial loop: an internal setpoint with
  manual override, auto / manual mode with bumpless transfer, an SP
  velocity limiter, and toggleable diagnostic logging. Takes the raw
  process variable per call and manages everything else internally.

For background on positional vs. velocity form, the windup mechanism, and
the rationale for the operator-mode features, see
[`docs/control-theory.md`](docs/control-theory.md).

## Examples

- [`examples/tank-control.toit`](examples/tank-control.toit) — `PI-Controller`
  driving a simulated tank level loop, with a UDP-driven operator faceplate
  ([`tank-faceplate.toit`](examples/tank-faceplate.toit)) for SP, mode, and
  manual-output changes.
- [`examples/level-controller-pid.toit`](examples/level-controller-pid.toit)
  — the positional `Controller` driving the same tank simulation, for
  side-by-side comparison.

## License

All code in this package is MIT-licensed under [`LICENSE2`](LICENSE2)
(Copyright Ekorau LLC), with one exception: the original positional
controller in [`src/controller.toit`](src/controller.toit) is MIT-licensed
under [`LICENSE1`](LICENSE1). Each source file declares its applicable
license in a one-line header comment.

## API notes

- `PI-Controller --ti=` is now `float` (previously `int`). Pass `ti=10.0`
  rather than `ti=10`. Same applies to `tune --ti=`.
- `PI-Controller` exposes a `debug` field (and `--debug` constructor flag)
  to enable periodic diagnostic prints without source edits.
- `PI-Controller.sp` is a public read-only getter for the current internal
  setpoint after auto / manual selection and SP velocity limiting — useful
  for instrumentation and tests.
