# AGENTS.md

Project: `botcli`

## Purpose

`botcli` is a persistent C++ daemon around `libwallaby` for KIPR/Wombat-style robot hardware.
It is the sole process that touches hardware and exposes sensor, motor, and servo operations over a Unix domain socket.
Camera planned, deferred.

External callers (likely Go) connect to the socket and speak the JSON protocol directly.
There is no CLI client.

## Current decisions

- Build natively on the Raspberry Pi where `libwallaby` is installed.
- Do not spend effort on cross-compilation yet.
- Prefer Meson unless it creates unnecessary complications; CMake is acceptable only if Meson becomes awkward.
- Use `third_party/picojson.h` for JSON.
- Use `third_party/lest.hpp` later for tests.
- Third-party project READMEs in `third_party/*.md` are local reference for those single-header dependencies.
- `libwallaby/` intentionally untracked, local reference only.
- `build-deploy.sh` expected compile path during development.
- Daemon starts with `botcli [--socket <path>] [--motor-timeout-ms <ms>]`, stays foreground for supervision.
- `--socket` defaults to `$XDG_RUNTIME_DIR/botcli.sock` when set, otherwise `/tmp/botcli.sock`.
- Reject non-integral, negative, or zero motor timeout values from `--motor-timeout-ms`; valid timeout values are positive integer milliseconds.
- Daemon protocol is newline-delimited UTF-8 JSON.
Requests have optional `id`, required string `command`, required object `params`; use `{}` for no args.
- Request IDs are strings, 1-128 characters, ASCII letters, digits, `.`, `_`, `:`, and `-` only.
Reject unknown top-level fields, unknown param fields, and non-integral or non-number values for integer parameters.
- Do not implement `motor set` as one detached child process per request.
`mav` stops when the calling process exits, and per-request background setters would race with `motor off`.
- `motor get` means `gmpc(port)`, returns motor position counter in ticks.
`motor off` uses passive `off()` / `ao()` semantics, not active braking.
- If motor timeout configured, every `motor.set`, including velocity `0`, calls `mav(port, velocity)` and refreshes port deadline.
Dispatcher must also wait for timeout deadlines and stop expired ports through same serialized hardware path.
Without timeout, set motor runs until applicable `motor off` or daemon shutdown.
- Daemon should serialize all hardware calls through one dispatcher thread or equivalent global hardware mutex, and stop all motors on clean shutdown and SIGINT/SIGTERM.

## Socket protocol

Each request and response is exactly one JSON object plus newline on the Unix socket.

Success:

```json
{"ok":true,"id":"req-42","command":"analog","result":{"port":0,"value":812}}
```

When port omitted for analog, digital, motor get, or servo reader commands, `result.values` contains all values in port order.

```json
{"ok":true,"command":"analog","result":{"values":[812,790,0,0,1023,511]}}
```

Axis-specific accelerometer, gyroscope, magnetometer responses:

```json
{"ok":true,"command":"accel","result":{"axis":"x","value":-14}}
```

When axis omitted:

```json
{"ok":true,"command":"accel","result":{"x":-14,"y":3,"z":1021}}
```

Writer responses include requested write params in `result`:

```json
{"ok":true,"command":"servo.set_enabled","result":{"port":1,"enabled":1}}
{"ok":true,"command":"servo.set","result":{"port":1,"position":1200}}
```

Error:

```json
{"ok":false,"id":"req-42","error":{"code":"invalid_port","message":"analog port must be between 0 and 5"}}
```

If `id` is absent from the request, omit `id` from the response.

Stable error codes include `invalid_port`, `invalid_velocity`, `invalid_position`, `invalid_id`, `invalid_axis`, `missing_argument`, `unknown_command`, `daemon_timeout`, `daemon_protocol_error`, and `daemon_shutting_down`.

Daemon stderr is for startup failures, accept-loop diagnostics, and crashes — not normal request errors.

## Validation

Validate arguments at the socket boundary before calling `libwallaby`.

- Analog ports: `0-5`
- Digital ports: `0-9`
- Motor ports: `0-3`
- Motor velocity: `-1500-1500`
- Servo ports: `0-3`
- Servo enabled values: `0` or `1`
- Servo positions: `0-2047`

## Servo commands

Daemon protocol commands:

- `servo.get_enabled` — optional `port`; returns all four when omitted
- `servo.get` — optional `port`; returns all four when omitted
- `servo.set_enabled` — required `port` and `enabled`
- `servo.set` — required `port` and `position`

Writer commands acknowledge the write request in `result`; they do not read back hardware state.

## Initial scope

Support accelerometer, gyroscope, magnetometer, side button, analog, digital, motor, and servo daemon commands.

Defer camera commands until later.
