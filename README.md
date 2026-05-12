# botcli

Provide a simple CLI wrapper around [libwallaby](https://github.com/kipr/libwallaby) to expose sensor, motor, and servo values.

The primary consumer is expected to be another program, but the CLI should remain readable and useful when run by a human at a shell.

## Build and dependencies

`botcli` is intended to build natively on a Raspberry Pi with `libwallaby` already installed. For now, cross-compilation is out of scope.

- Prefer Meson for the build system unless it creates avoidable complexity.
- Use `third_party/CLI11.hpp` for command-line parsing.
- Use `third_party/picojson.h` for JSON output.
- Use `third_party/lest.hpp` for future tests.
- The third-party project READMEs in `third_party/*.md` are local reference material for those single-header dependencies.
- `libwallaby/` may exist locally as an intentionally untracked reference checkout, but it is not project source.
- `build-deploy.sh` is the expected path for copying the project to the Raspberry Pi and compiling it there.

Here is an example compilation command for a program depending on `libwallaby`:

```sh
gcc -I"/home/kipr/Documents/KISS/Default User/Wombat Factory Test/include" -I"/usr/local/include/include" -Wall "/home/kipr/Documents/KISS/Default User/Wombat Factory Test/src/main.c" "/home/kipr/harrogate/apps/compiler/compilation-environments/c/_init_helper.c" -L"/usr/local/lib" -lkipr -lm -o "/home/kipr/Documents/KISS/Default User/Wombat Factory Test/bin/botball_user_program" -lz -lpthread
```

The most important is `-lkipr`.
The others may or may not be necessary for this project.

## Output contract

By default, successful commands print only the result value to stdout, followed by a newline. This keeps the CLI easy to parse from other programs.

Examples:

```sh
botcli analog 0
812

botcli analog
812 790 0 0 1023 511

botcli accel x
-14

botcli accel
-14 3 1021

botcli servo get_enabled 2
1

botcli servo set 2 1200
ok
```

Writer commands that do not return a hardware value should print `ok` on success. This acknowledges that the request executed without implying a readback of the written value.

Errors print a human-readable message to stderr and exit with a nonzero status.

```sh
botcli analog 12
error: analog port must be between 0 and 5
```

For structured consumers, `botcli` should support global JSON output and an optional request ID:

```text
botcli [--json] [--id <request-id>] <command> ...
```

In JSON mode, every normal response is written to stdout as exactly one JSON object followed by a newline. This includes both success and expected error responses.

```json
{"ok":true,"id":"req-42","command":"analog","result":{"port":0,"value":812}}
```

When the port is omitted for analog, digital, or servo reader commands, default output includes all port values separated by spaces. JSON responses keep `result` as an object and return a `values` array whose index is the port number. Analog arrays are length 6 for ports `0-5`; digital arrays are length 10 for ports `0-9`; servo reader arrays are length 4 for ports `0-3`.

```json
{"ok":true,"command":"analog","result":{"values":[812,790,0,0,1023,511]}}
```

Axis-specific accelerometer, gyroscope, and magnetometer JSON responses include the requested axis and value:

```json
{"ok":true,"command":"accel","result":{"axis":"x","value":-14}}
```

When the axis is omitted for accelerometer, gyroscope, or magnetometer commands, JSON responses include all three axes:

```json
{"ok":true,"command":"accel","result":{"x":-14,"y":3,"z":1021}}
```

Writer command JSON responses should include the requested write parameters in `result`, while still treating `ok:true` as the acknowledgement:

```json
{"ok":true,"command":"servo.set_enabled","result":{"port":1,"enabled":1}}
{"ok":true,"command":"servo.set","result":{"port":1,"position":1200}}
```

JSON error responses still use a nonzero exit status:

```json
{"ok":false,"id":"req-42","error":{"code":"invalid_port","message":"analog port must be between 0 and 5"}}
```

Stderr is reserved for failures outside the normal JSON response contract, such as crashes, dynamic loader failures, or unexpected diagnostics before `botcli` can emit JSON.

If `--id` is not supplied, omit the `id` field. `--id` is only valid with `--json`; using `--id` without `--json` is an error. The request ID exists for future batching, streaming, or daemon-style use; callers that invoke one process per command already know which output belongs to which command.

Request IDs are strings, not numeric-only IDs. Valid request IDs are 1-128 characters and may contain only ASCII letters, digits, `.`, `_`, `:`, and `-`.

JSON error codes should be stable machine-readable strings, such as `invalid_port`, `invalid_velocity`, `invalid_position`, `invalid_id`, `missing_argument`, `unknown_command`, or `wallaby_error`.

## Validation

`botcli` should validate command arguments before calling `libwallaby`.

- Digital ports: `0-9`
- Analog ports: `0-5`
- Analog sensor values: `0-4095`
- Motor ports: `0-3`
- Motor velocity: `-1500-1500`
- Servo ports: `0-3`
- Servo enabled values: `0` or `1`
- Servo positions: `0-2047`

## Functionality

Each section is a different subsystem.
First we list the functions from libwallaby to expose, then the planned CLI syntax.

## Accelerometer

**libwallaby**:

- `accel_x()`
- `accel_y()`
- `accel_z()`

**botcli**:

`botcli accel x|y|z` = `botcli accel <axis>`

`botcli accel` returns all three axes as `x y z`.

## Gyroscope

**libwallaby**

- `gyro_x()`
- `gyro_y()`
- `gyro_z()`

**botcli**

`botcli gyro x|y|z` = `botcli gyro <axis>`

`botcli gyro` returns all three axes as `x y z`.

## Magnetometer

**libwallaby**

- `magneto_x()`
- `magneto_y()`
- `magneto_z()`

**botcli**

`botcli magneto x|y|z` = `botcli magneto <axis>`

`botcli magneto` returns all three axes as `x y z`.

## Button

Note: this function reads the value from the side button.

**libwallaby**

`side_button()`

**botcli**

`botcli side_btn`

## Analog

**libwallaby**

`analog(int port)`

**botcli**

`botcli analog [0-5]` = `botcli analog <port>`

`botcli analog` returns all six analog ports as space-separated values in port order.

## Digital

**libwallaby**

`digital(int port)`

**botcli**

`botcli digital [0-9]` = `botcli digital <port>`

`botcli digital` returns all ten digital ports as space-separated values in port order.

## Motors

**libwallaby**

- `gmpc(int motor)`
- `cmpc(int motor)`
- `mav(int motor, int velocity)`
- `motor(int motor, int percent)` * Omitted for more orthogonal syntax. Callers can proxy with `mav` later if required.
- `off(int motor)`
- `ao()`

**botcli**

`botcli motor set [0-3] [-1500-1500]` = `botcli motor set <port> <velocity>`

`botcli motor get [0-3]` = `botcli motor get <port>` [1]

`botcli motor clear [0-3]` = `botcli motor clear <port>`

`botcli motor off [0-3]` = `botcli motor off <port>` [2]

[1] Get and clear cover `gmpc` and `cmpc`.
Shortnames for `get_motor_position_counter` and `clear_motor_position_counter`.

[2] Port 0-3 optional. If port unspecified, run `ao()`

## Servos

**libwallaby**

- `set_servo_enabled(int port, int enabled)`
- `get_servo_enabled(int port)`
- `get_servo_position(int port)`
- `set_servo_position(int port, int position)`

**botcli**

`botcli servo set_enabled [0-3] (0|1)` = `botcli servo set_enabled <port> <enabled>`

`botcli servo get_enabled [0-3]` = `botcli servo get_enabled <port>`

`botcli servo get_enabled` returns all four enabled values as space-separated values in port order.

`botcli servo set [0-3] [0-2047]` = `botcli servo set <port> <position>`

`botcli servo get [0-3]` = `botcli servo get <port>`

`botcli servo get` returns all four servo positions as space-separated values in port order.

Servo writer commands require an explicit port. Omitting the port is only an "all ports" shortcut for reader commands.

On success, servo writer commands print `ok` in default mode. In JSON mode, they return `ok:true` and include the requested `port` plus `enabled` or `position` value in `result`. This is an acknowledgement of the write request, not a hardware readback.

Bulk servo writes are a possible future extension, but are not part of the initial command set. If added later, they should be explicit, such as `botcli servo set_enabled all <enabled>` or `botcli servo set all <position>`, and may use the libwallaby helpers `enable_servos()` and `disable_servos()` where appropriate.

## Camera

Deferred until after the initial sensor, motor, and servo commands.

**libwallaby**

- `camera_load_config(const char *name)`: Relative to /etc/botui/channels
- `camera_open()`
- `camera_close()`
- `get_camera_width()`
- `get_camera_height()`
- `camera_update()`
- `get_channel_count()`
- `get_object_count(int channel)`
- `get_object_data(int channel, int object)`
- `get_object_confidence (int channel, int object)`
- `get_object_area (int channel, int object)`
- `get_object_bbox (int channel, int object)`
- `get_object_centroid (int channel, int object)`
- `get_object_center (int channel, int object)`

**botcli**

`botcli camera load_config <file>`
`botcli camera open`
`botcli camera close`
`botcli camera get width`
`botcli camera get height`
`botcli camera get channels`
`botcli camera get objects [0,int_max]` = `botcli camera get objects <channel>`
`botcli camera get obj [0,int_max] [0,int_max]  (data|confidence|area|bbox|centroid|center)` = `botcli camera get obj <channel> <object> <metric>`
