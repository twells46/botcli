# botcli

Provide a simple CLI wrapper around [libwallaby](https://github.com/kipr/libwallaby) to expose sensor, motor, and servo values.

The primary consumer is expected to be another program, but the CLI should remain readable and useful when run by a human at a shell.

## Build and dependencies

`botcli` is intended to build natively on a Raspberry Pi with `libwallaby` already installed. For now, cross-compilation is out of scope.

- Prefer Meson for the build system unless it creates avoidable complexity.
- Use `third_party/CLI11.hpp` for command-line parsing wherever possible.
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

## Daemon plan

Motor velocity commands need a persistent owner because the `libwallaby` `mav(int motor, int velocity)` effect stops when the calling program terminates. `botcli motor set` should therefore not fork a separate detached process per request. That would make later `motor off` commands race with or be undone by the still-running setter process.

The planned design is a full `botcli` daemon that owns `libwallaby` behind a local IPC interface. For the first daemon-backed implementation, only motor commands should route through the daemon:

```text
botcli motor set <port> <velocity>  -> client request to daemon
botcli motor off [port]             -> client request to daemon
botcli motor get <port>             -> client request to daemon
botcli motor clear <port>           -> client request to daemon
```

The daemon does not autostart. If `botcli motor ...` cannot connect to a running daemon, it should fail with a `daemon_unavailable` error. In default mode that error goes to stderr with a nonzero exit status; in JSON mode it is emitted as a normal JSON error response.

Run the daemon explicitly:

```text
botcli [--socket <path>] daemon [--motor-timeout-ms <ms>]
```

The daemon should stay in the foreground so it can be supervised by a parent process or by `systemd` later. Logs and diagnostics should go to stderr or a log file, not to stdout or into daemon protocol responses.

All existing non-motor commands may remain one-off commands that call `libwallaby` directly for now. The daemon protocol should still be designed so analog, digital, sensor, servo, and future camera commands can move behind the same daemon later without changing the public CLI contract. Once motors are daemon-backed, all motor reads and writes should go through the daemon so one process owns the motor hardware state consistently.

For the first daemon-backed implementation, that single-owner rule applies to motor functions only. Non-motor commands may still call `libwallaby` directly, with the accepted limitation that this does not yet make the daemon the exclusive owner of every lower-level hardware access path.

The public output contract does not change. Default mode still prints raw values or `ok`; JSON mode still prints exactly one JSON object for normal responses and expected errors. Internally, daemon-backed writer commands acknowledge that the daemon accepted and applied the requested state, not that a one-off process performed a complete hardware action before exiting.

### Socket and protocol

Use a Unix domain stream socket. The default socket path should be under `$XDG_RUNTIME_DIR` when available and otherwise under `/tmp`, for example:

```text
$XDG_RUNTIME_DIR/botcli.sock
/tmp/botcli-<uid>.sock
```

Allow an override through `BOTCLI_SOCKET` or a global `--socket <path>` option so tests and multiple robot users can avoid colliding. Socket path precedence is:

```text
--socket <path> > BOTCLI_SOCKET > default socket path
```

The same precedence applies to the daemon and to daemon-backed `botcli motor ...` client commands. `--socket` is a global option, so both `botcli --socket <path> daemon ...` and `botcli --socket <path> motor ...` use the same override mechanism. The daemon should create the socket with owner-only access, remove its socket file on clean shutdown, and handle a stale socket file at startup by verifying whether a daemon is actually listening before unlinking it.

Use newline-delimited JSON for the daemon protocol rather than a plain text mirror of CLI syntax. Plain text such as `motor off 0` is easy for a human but becomes an ad hoc parser as soon as request IDs, structured errors, optional fields, future batching, or camera data are needed. JSON already matches the public `--json` contract, is straightforward for a Go client, and lets the CLI stay as the human-facing syntax adapter.

Each request and response should be exactly one UTF-8 JSON object followed by `\n`. The daemon should accept only structured daemon requests; it should not parse shell-style command lines from the socket.

Daemon requests use this shape:

```json
{"id":"req-42","command":"motor.set","params":{"port":0,"velocity":600}}
```

Top-level request fields:

- `id`: optional request ID using the same rules as public `--id`: 1-128 characters containing only ASCII letters, digits, `.`, `_`, `:`, and `-`.
- `command`: required string command name.
- `params`: required object. Use `{}` for commands with no arguments.

For the first daemon-backed implementation, `command` must be one of `motor.set`, `motor.off`, `motor.get`, or `motor.clear`. Do not include a protocol version field yet; add one later only if the daemon protocol needs a second incompatible shape.

Reject unknown top-level fields and unknown `params` fields. Ports, velocities, and future numeric scalar fields must be JSON numbers with integral values. Do not accept strings, booleans, `null`, arrays, objects, or fractional numbers where a scalar integer is expected.

Example requests:

```json
{"id":"req-42","command":"motor.set","params":{"port":0,"velocity":600}}
{"id":"req-43","command":"motor.off","params":{"port":0}}
{"id":"req-44","command":"motor.off","params":{}}
{"id":"req-45","command":"motor.get","params":{"port":0}}
{"id":"req-46","command":"motor.clear","params":{"port":0}}
```

Example responses:

```json
{"ok":true,"id":"req-42","command":"motor.set","result":{"port":0,"velocity":600}}
{"ok":true,"id":"req-43","command":"motor.off","result":{"port":0}}
{"ok":true,"id":"req-44","command":"motor.off","result":{"ports":[0,1,2,3]}}
{"ok":true,"id":"req-45","command":"motor.get","result":{"port":0,"value":1234}}
{"ok":true,"id":"req-46","command":"motor.clear","result":{"port":0}}
{"ok":false,"id":"req-42","error":{"code":"invalid_velocity","message":"motor velocity must be between -1500 and 1500"}}
```

The daemon should validate requests itself even if the `botcli` client has already validated command-line arguments. The client-side validation is for fast, nice CLI errors; the daemon is the trust boundary for every process that can connect to the socket.

The daemon and client should use stable errors for daemon transport failures. Missing socket, refused connection, or another failure to connect should become `daemon_unavailable`; a request that waits too long for a daemon response should become `daemon_timeout`; malformed daemon responses should become `daemon_protocol_error`.

Daemon-backed CLI clients should use an internal fixed response timeout, initially without a public CLI or environment option. This timeout only bounds how long a `botcli motor ...` subprocess waits for the daemon response after it has sent a request; it is separate from motor timeout, which controls how long the daemon keeps a motor command active. The initial value can be a small conservative default such as 2 seconds. If callers later need tuning for slow hardware or supervision environments, expose a public timeout option then.

### Motor supervision

The daemon should be the single owner of motor hardware calls, but the first implementation does not need to model physical motor state. It should be an ordered command dispatcher: `motor set` calls `mav(port, velocity)`, `motor off <port>` calls `off(port)`, `motor off` with no port calls `ao()`, `motor get <port>` calls `gmpc(port)`, and `motor clear <port>` calls `cmpc(port)`. `motor get` returns the motor position counter in ticks, not current velocity or the last requested velocity. `motor off` uses the `libwallaby` `off()` / `ao()` passive stop behavior; it is not active braking.

If the daemon tracks anything initially, it should be minimal desired command state only, such as the last requested velocity and optional timeout deadline for each port. That state is useful for a future heartbeat, status command, or ownership/lease rule; it is not authoritative hardware state and should not be used to answer `motor get`. `motor get` should always read the motor position counter through `gmpc(port)`.

Do not create one long-lived thread per motor command. `mav()` is not a blocking operation that needs a worker thread; in `libwallaby` it sets speed mode and writes the goal velocity register, then returns. The reason a one-shot `botcli motor set` stops on process exit is `libwallaby` cleanup: the motor module registers `ao()` as a cleanup function, and the `Platform` cleanup path runs on process teardown or SIGINT/SIGTERM. Destroying or cancelling a thread would not run that process-level cleanup, and destroying a joinable C++ `std::thread` would terminate the process. A thread that simply returns after calling `mav()` would leave the motor command active as long as the daemon process remains alive.

Serialize all hardware operations through one dispatcher thread or one global hardware mutex. Per-motor mutexes are not sufficient because `libwallaby` motor functions update shared motor mode and direction registers with read-modify-write sequences. A single dispatcher also makes command ordering deterministic when multiple clients send commands at nearly the same time:

```text
client socket readers -> validated daemon request queue -> single hardware dispatcher -> response
```

The dispatcher should apply each request completely before the next request reaches `libwallaby`. This means `motor set 0 600` followed by `motor off 0` has one clear winner based on daemon receive order, not on thread scheduling.

### Request queue and ordering

Use a single producer-consumer queue as the first implementation. The queue should be owned by a small daemon runtime object and protected by a mutex plus condition variable. Socket handler threads are producers; the hardware dispatcher is the only consumer and the only thread that calls motor functions.

The main thread should own process-level lifecycle: create the socket, start the dispatcher, accept client connections, handle shutdown signals, and join worker threads during shutdown. It does not need to call `libwallaby` directly. Keeping hardware calls in the dispatcher avoids accidental future calls from accept or signal handling code.

The daemon should handle SIGINT/SIGTERM deliberately rather than relying on `libwallaby` cleanup from an arbitrary signal path. Prefer blocking shutdown signals in worker threads, receiving them in the main thread, then requesting dispatcher shutdown so `ao()` is called from the dispatcher before exit.

Each complete parsed request becomes one work item:

```text
sequence
optional request id
command
validated parameters
response completion handle
```

Assign `sequence` while holding the queue mutex, immediately before pushing the work item. That sequence number defines daemon order. A request is considered received when the daemon has read a full newline-delimited JSON object, parsed it, validated it enough to identify the command shape, and pushed it onto the queue. Global request ordering is based on queue insertion after parsing a complete JSON line, not on client process start time, socket accept time, or partial byte arrival. Malformed JSON and basic protocol errors can be answered directly by the socket handler without entering the hardware queue because they do not touch hardware.

For the first version, support one outstanding queued request per client connection: the socket handler reads one request, validates and queues it, waits for that request's response, writes the response line, then reads the next request. This preserves request order on each connection without implementing response reordering or pipelining. The CLI client will normally open a connection, send one request, read one response, and close. A future Go client can keep a connection open and send repeated commands, still one at a time.

The daemon should include basic limits so partial or hostile clients cannot consume resources indefinitely. Exact values can be chosen later, but the protocol should define limits for maximum concurrent clients, maximum request line size, and request read timeout.

The dispatcher loop should look conceptually like this:

```text
while running:
  wait until queue is non-empty, a motor timeout deadline expires, or shutdown is requested
  pop the oldest work item
  apply exactly one libwallaby operation or small operation group
  update optional desired command state, if that feature is enabled
  complete the work item with one JSON response
```

When motor timeout is configured, the dispatcher wait must use the nearest active motor deadline as a wake-up time. If the deadline expires before another work item arrives, the dispatcher should stop the expired port with `off(port)` and clear that port's desired command state. If a client work item and one or more timeout deadlines are ready at the same time, the dispatcher should handle them in one deterministic order and keep all resulting hardware calls serialized through the same dispatcher path.

`motor.off` with no port is one operation group: call `ao()` before completing the response. If desired command state is being tracked, clear all ports after `ao()`. `motor.set`, `motor.get`, `motor.clear`, and `motor.off <port>` each map to one motor operation; `motor.set` and `motor.off <port>` may also update optional desired command state.

On shutdown, stop accepting new clients, reject or drain queued requests, then call `ao()` from the dispatcher before it exits. Prefer draining already queued requests only if shutdown is graceful and quick; for SIGINT/SIGTERM, stopping motors takes priority over serving pending client responses. When possible, rejected clients should receive an error such as `daemon_shutting_down`. SIGINT/SIGTERM shutdown may close sockets without a response after motors are stopped.

The daemon should stop all motors on clean shutdown and on SIGINT/SIGTERM. Motor timeout is optional and defaults to indefinite. Configure it with `--motor-timeout-ms <ms>` or `BOTCLI_MOTOR_TIMEOUT`; precedence is:

```text
--motor-timeout-ms <ms> > BOTCLI_MOTOR_TIMEOUT > indefinite timeout
```

Parse command-line options with CLI11 wherever possible. The daemon should reject non-integral, negative, or zero motor timeout values from either `--motor-timeout-ms` or `BOTCLI_MOTOR_TIMEOUT`; valid timeout values are positive integer milliseconds. Invalid command-line timeout values should produce a normal CLI parse/validation error. Invalid environment timeout values should produce a stable machine-readable error code such as `invalid_motor_timeout` in JSON mode and the corresponding default-mode error on stderr.

When a timeout is configured, every `motor.set` call, including `motor set <port> 0`, calls `mav(port, velocity)` and refreshes that port's deadline. The motor should keep running until the daemon receives `motor off` for that port, receives `motor off` for all ports, or the timeout expires, whichever comes first. If no timeout is configured, the motor should keep running until the daemon receives an applicable `motor off` or the daemon shuts down.

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
