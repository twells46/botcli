# botcli

Small C++ CLI around [libwallaby](https://github.com/kipr/libwallaby) for KIPR/Wombat-style robot hardware.
It exposes sensor, motor, and servo commands for humans at a shell and for subprocess callers.

Current state:

- Native Raspberry Pi build with Meson and installed `libwallaby`.
- One foreground daemon owns hardware access.
- CLI hardware commands connect to the daemon over a Unix domain socket.
- Camera commands are deferred.

Daemon internals detailed in [docs/daemon_arch.md](docs/daemon_arch.md).

## Build

`botcli` is intended to build on the Raspberry Pi where `libwallaby` is installed.
Cross-compilation is out of scope for now.

```sh
meson setup build
meson compile -C build
```

During development, `build-deploy.sh` copies the project to the Pi and compiles it there.

Third-party single-header dependencies are vendored in `third_party/`:

- `CLI11.hpp` for CLI parsing
- `picojson.h` for JSON
- `lest.hpp` for future tests

`libwallaby/` may exist locally as an intentionally untracked reference checkout, but it is not project source.

## Run

Start the daemon first:

```sh
botcli daemon
```

Client commands do not autostart the daemon.

Then run commands from another shell:

```sh
botcli analog 0
botcli motor set 0 600
botcli motor off 0
```

Global options:

```text
botcli [--json] [--id <request-id>] [--socket <path>] <command> ...
```

`--socket` defaults to `$XDG_RUNTIME_DIR/botcli.sock` when `XDG_RUNTIME_DIR` is set, otherwise `/tmp/botcli.sock`.
The same option is used by the daemon and by client commands.

Motor timeout is configured on the daemon:

```sh
botcli daemon --motor-timeout-ms 500
```

Timeout values must be positive integer milliseconds.
With no timeout, motor velocity commands remain active until `motor off` or daemon shutdown.

## Commands

Sensor reads:

```text
botcli accel [x|y|z]
botcli gyro [x|y|z]
botcli magneto [x|y|z]
botcli side_btn
botcli analog [0-5]
botcli digital [0-9]
```

Motor commands:

```text
botcli motor set <0-3> <-1500-1500>
botcli motor off [0-3]
botcli motor get [0-3]
botcli motor clear <0-3>
```

`motor get` reads `gmpc(port)` position counter ticks.
`motor off` uses passive `off()` / `ao()` semantics, not active braking.
If `motor off` has no port, it stops all motors.

Servo commands:

```text
botcli servo get_enabled [0-3]
botcli servo get [0-3]
botcli servo set_enabled <0-3> <0|1>
botcli servo set <0-3> <0-2047>
```

Servo reader commands return all four ports when port is omitted.
Servo writer commands require an explicit port and acknowledge the write request; they do not read back hardware state.

## Output

Default success output is raw stdout plus newline.

```text
botcli analog 0
812

botcli analog
812 790 0 0 1023 511

botcli accel x
-14

botcli accel
-14 3 1021

botcli servo set 2 1200
ok
```

Writer commands with no hardware value print:

```text
ok
```

Default errors go to stderr with nonzero exit status:

```text
error: analog port must be between 0 and 5
```

If the daemon is not reachable, client commands return `daemon_unavailable`.
Other daemon transport failures use stable codes such as `daemon_timeout` and `daemon_protocol_error`.

## JSON Mode

In JSON mode, daemon success, daemon error, and daemon transport error responses are exactly one JSON object on stdout plus newline.
JSON errors still use nonzero exit status.
CLI parse errors come from CLI11 and may use stderr diagnostics before a daemon request exists.

```sh
botcli --json --id req-42 analog 0
```

```json
{"ok":true,"id":"req-42","command":"analog","result":{"port":0,"value":812}}
```

When a port is omitted for analog, digital, motor get, or servo reader commands, `result.values` contains all values in port order.

```json
{"ok":true,"command":"analog","result":{"values":[812,790,0,0,1023,511]}}
```

Axis-specific accelerometer, gyroscope, and magnetometer responses use:

```json
{"ok":true,"command":"accel","result":{"axis":"x","value":-14}}
```

When axis is omitted:

```json
{"ok":true,"command":"accel","result":{"x":-14,"y":3,"z":1021}}
```

Writer JSON responses include requested write parameters:

```json
{"ok":true,"command":"servo.set_enabled","result":{"port":1,"enabled":1}}
{"ok":true,"command":"servo.set","result":{"port":1,"position":1200}}
{"ok":true,"command":"motor.set","result":{"port":0,"velocity":600}}
```

JSON error shape:

```json
{"ok":false,"id":"req-42","error":{"code":"invalid_port","message":"analog port must be between 0 and 5"}}
```

Stderr is reserved in JSON mode for failures outside the normal response contract: crashes, dynamic loader failures, or diagnostics before `botcli` can emit JSON.

If `--id` is absent, `id` is omitted.
`--id` is only valid with `--json`.
Request IDs are strings, 1-128 characters, and may contain only ASCII letters, digits, `.`, `_`, `:`, and `-`.

Stable error codes include `invalid_port`, `invalid_velocity`, `invalid_position`, `invalid_id`, `missing_argument`, `unknown_command`, `daemon_unavailable`, `daemon_timeout`, `daemon_protocol_error`, and `wallaby_error`.

## Validation

`botcli` validates arguments before sending daemon requests, and the daemon validates again at the socket boundary.

- Analog ports: `0-5`
- Digital ports: `0-9`
- Motor ports: `0-3`
- Motor velocity: `-1500-1500`
- Servo ports: `0-3`
- Servo enabled values: `0` or `1`
- Servo positions: `0-2047`

## Deferred

Camera commands are planned later.
Possible future syntax:

```text
botcli camera load_config <file>
botcli camera open
botcli camera close
botcli camera get width
botcli camera get height
botcli camera get channels
botcli camera get objects <channel>
botcli camera get obj <channel> <object> <data|confidence|area|bbox|centroid|center>
```
