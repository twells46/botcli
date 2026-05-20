# AGENTS.md

Project: `botcli`

## Purpose

`botcli` small C++ command-line wrapper around `libwallaby` for KIPR/Wombat-style robot hardware.
Expose sensor, motor, servo functions as shell commands.
Camera planned, deferred.

CLI for humans at shell, more importantly stable subprocess interface for another program, likely Go.

## Current decisions

- Build natively on the Raspberry Pi where `libwallaby` is installed.
- Do not spend effort on cross-compilation yet.
- Prefer Meson unless it creates unnecessary complications; CMake is acceptable only if Meson becomes awkward.
- Use `third_party/CLI11.hpp` for CLI parsing wherever possible.
- Use `third_party/picojson.h` for JSON output.
- Use `third_party/lest.hpp` later for tests.
- Third-party project READMEs in `third_party/*.md` are local reference for those single-header dependencies.
- `libwallaby/` intentionally untracked, local reference only.
- `build-deploy.sh` expected compile path during development.
- Plan full `botcli` daemon as persistent owner of `libwallaby`; initially route only motor commands through it.
Other commands may stay one-off direct `libwallaby` calls until reason to migrate.
- Daemon starts explicitly with `botcli [--socket <path>] daemon [--motor-timeout-ms <ms>]`, stays foreground for supervision.
No motor command autostart.
- `--socket` is a global option.
Daemon-backed motor commands use socket path precedence `--socket` > default.
Motor timeout precedence is `--motor-timeout-ms` > indefinite timeout.
- If `botcli motor ...` cannot connect to running daemon, return `daemon_unavailable`.
Other daemon transport errors use stable codes such as `daemon_timeout` and `daemon_protocol_error`.
Daemon-backed CLI clients use internal fixed response timeout, no public option initially.
- Daemon protocol is newline-delimited UTF-8 JSON.
Requests have optional `id`, required string `command`, required object `params`; use `{}` for no args.
Initially support `motor.set`, `motor.off`, `motor.get`, and `motor.clear`.
- Daemon request IDs follow same rules as public `--id`.
Reject unknown top-level fields, unknown param fields, and non-integral or non-number values for integer parameters.
- Do not implement `motor set` as one detached child process per request.
`mav` stops when the calling process exits, and per-request background setters would race with `motor off`.
- `motor get` means `gmpc(port)`, returns motor position counter in ticks.
`motor off` uses passive `off()` / `ao()` semantics, not active braking.
- If motor timeout configured, every `motor.set`, including velocity `0`, calls `mav(port, velocity)` and refreshes port deadline.
Dispatcher must also wait for timeout deadlines and stop expired ports through same serialized hardware path.
Without timeout, set motor runs until applicable `motor off` or daemon shutdown.
- Reject non-integral, negative, or zero motor timeout values from `--motor-timeout-ms`; valid timeout values are positive integer milliseconds.
- Daemon should serialize motor hardware calls through one dispatcher thread or equivalent global hardware mutex, and stop all motors on clean shutdown and SIGINT/SIGTERM.

## Output contract

Default success output should be raw, parseable stdout:

```text
<value>\n
```

For accelerometer, gyroscope, magnetometer commands with no axis arg, default success output all three axis values:

```text
<x> <y> <z>\n
```

For analog, digital, servo reader commands with no port arg, default success output all port values in port order as space-separated values plus newline.
Analog returns six values for ports `0-5`; digital returns ten values for ports `0-9`; servo reader commands return four values for ports `0-3`.

Writer commands with no hardware value should ack success:

```text
ok\n
```

Means request executed, not value read back from hardware.

Do not add labels or decorative text in default mode.

Default errors should go to stderr with a nonzero exit status:

```text
error: <message>
```

Support global JSON mode:

```text
botcli [--json] [--id <request-id>] <command> ...
```

In JSON mode, every normal response goes to stdout as exactly one JSON object plus newline.
Includes success and expected error responses.

```json
{"ok":true,"id":"req-42","command":"analog","result":{"port":0,"value":812}}
```

When port omitted for analog, digital, or servo reader commands, return all port values in `values` array inside `result`.
Array index is port number:

```json
{"ok":true,"command":"analog","result":{"values":[812,790,0,0,1023,511]}}
```

Axis-specific accelerometer, gyroscope, magnetometer JSON responses use:

```json
{"ok":true,"command":"accel","result":{"axis":"x","value":-14}}
```

When axis omitted for those commands, return all axes:

```json
{"ok":true,"command":"accel","result":{"x":-14,"y":3,"z":1021}}
```

Writer command JSON responses include requested write params in `result`, with `ok:true` as ack:

```json
{"ok":true,"command":"servo.set_enabled","result":{"port":1,"enabled":1}}
{"ok":true,"command":"servo.set","result":{"port":1,"position":1200}}
```

JSON errors use a nonzero exit status:

```json
{"ok":false,"id":"req-42","error":{"code":"invalid_port","message":"analog port must be between 0 and 5"}}
```

Reserve stderr in JSON mode for failures outside normal response contract: crashes, dynamic loader failures, unexpected diagnostics before `botcli` can emit JSON.

If `--id` is absent, omit the `id` field.
`--id` is only valid with `--json`; using `--id` without `--json` is an error.

Request IDs are strings, not numeric-only IDs.
Valid request IDs are 1-128 characters and may contain only ASCII letters, digits, `.`, `_`, `:`, and `-`.

JSON error codes stable machine-readable strings, such as `invalid_port`, `invalid_velocity`, `invalid_position`, `invalid_id`, `missing_argument`, `unknown_command`, or `wallaby_error`.

## Validation

Validate arguments in `botcli` before calling `libwallaby`.

- Analog ports: `0-5`
- Digital ports: `0-9`
- Motor ports: `0-3`
- Motor velocity: `-1500-1500`
- Servo ports: `0-3`
- Servo enabled values: `0` or `1`
- Servo positions: `0-2047`

## Servo commands

Servo reader commands follow same omitted-port pattern as analog and digital reads:

- `botcli servo get_enabled [port]`
- `botcli servo get [port]`

When `port` omitted, return all four values in port order.

Servo writer commands require explicit port:

- `botcli servo set_enabled <port> <enabled>`
- `botcli servo set <port> <position>`

On success, servo writer commands print `ok` in default mode.
In JSON mode, return `ok:true` and include requested `port` plus `enabled` or `position` value in `result`.
This acknowledges write request, not hardware readback.

Do not treat an omitted port as a bulk write for servo writer commands.
Bulk servo writes possible future extension, but must be explicit if added later, such as `botcli servo set_enabled all <enabled>` or `botcli servo set all <position>`.

## Initial scope

Implement README accelerometer, gyroscope, magnetometer, side button, analog, digital, motor, servo commands first.

Defer camera commands until later.
