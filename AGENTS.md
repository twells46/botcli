# AGENTS.md

Project: `botcli`

## Purpose

`botcli` is a small C++ command-line wrapper around `libwallaby` for KIPR/Wombat-style robot hardware. It exposes sensor, motor, and servo functions as shell commands. Camera support is planned but deferred.

The CLI is meant for humans at a shell and, more importantly, as a stable subprocess interface for another program, likely written in Go.

## Current decisions

- Build natively on the Raspberry Pi where `libwallaby` is installed.
- Do not spend effort on cross-compilation yet.
- Prefer Meson unless it creates unnecessary complications; CMake is acceptable only if Meson becomes awkward.
- Use `third_party/CLI11.hpp` for parsing.
- Use `third_party/picojson.h` for JSON output.
- Use `third_party/lest.hpp` later for tests.
- The third-party project READMEs in `third_party/*.md` are local reference material for those single-header dependencies.
- `libwallaby/` is intentionally untracked and present only as local reference material.
- `build-deploy.sh` is the expected compile path during development.

## Output contract

Default success output should be raw, parseable stdout:

```text
<value>\n
```

For accelerometer, gyroscope, and magnetometer commands with no axis argument, default success output should be all three axis values as:

```text
<x> <y> <z>\n
```

Do not add labels or decorative text in default mode.

Default errors should go to stderr with a nonzero exit status:

```text
error: <message>
```

Support global JSON mode:

```text
botcli [--json] [--id <request-id>] <command> ...
```

In JSON mode, every normal response goes to stdout as exactly one JSON object followed by a newline. This includes both success and expected error responses.

```json
{"ok":true,"id":"req-42","command":"analog","result":{"port":0,"value":812}}
```

Axis-specific accelerometer, gyroscope, and magnetometer JSON responses should use:

```json
{"ok":true,"command":"accel","result":{"axis":"x","value":-14}}
```

When the axis is omitted for those commands, return all axes:

```json
{"ok":true,"command":"accel","result":{"x":-14,"y":3,"z":1021}}
```

JSON errors use a nonzero exit status:

```json
{"ok":false,"id":"req-42","error":{"code":"invalid_port","message":"analog port must be between 0 and 9"}}
```

Reserve stderr in JSON mode for failures outside the normal response contract, such as crashes, dynamic loader failures, or unexpected diagnostics before `botcli` can emit JSON.

If `--id` is absent, omit the `id` field. `--id` is only valid with `--json`; using `--id` without `--json` is an error.

Request IDs are strings, not numeric-only IDs. Valid request IDs are 1-128 characters and may contain only ASCII letters, digits, `.`, `_`, `:`, and `-`.

JSON error codes should be stable machine-readable strings, such as `invalid_port`, `invalid_velocity`, `invalid_position`, `invalid_id`, `missing_argument`, `unknown_command`, or `wallaby_error`.

## Validation

Validate arguments in `botcli` before calling `libwallaby`.

- Analog and digital ports: `0-9`
- Motor ports: `0-3`
- Motor velocity: `-1500-1500`
- Servo ports: `0-3`
- Servo enabled values: `0` or `1`
- Servo positions: `0-2047`

## Initial scope

Implement the README's accelerometer, gyroscope, magnetometer, side button, analog, digital, motor, and servo commands first.

Defer camera commands until later.
