# Daemon Architecture

`botcli` is a foreground daemon and the single process that touches `libwallaby`.
External clients connect over a Unix domain socket, send newline-delimited JSON requests, and read newline-delimited JSON responses.

## Startup

Run:

```text
botcli [--socket <path>] [--motor-timeout-ms <ms>]
```

The daemon:

- Creates a Unix domain stream socket.
- Uses `$XDG_RUNTIME_DIR/botcli.sock` by default when available, otherwise `/tmp/botcli.sock`.
- Rejects socket paths too long for `sockaddr_un::sun_path`.
- Refuses startup if another daemon is listening at the socket path.
- Unlinks stale socket files before binding.
- Sets socket permissions to owner read/write.
- Stays in the foreground for supervision.
- Uses stderr for startup and accept-loop diagnostics.
- Removes the socket file on clean shutdown.

`SIGINT` and `SIGTERM` request shutdown.
On shutdown, the dispatcher calls `ao()` before exit.

## Protocol

Daemon protocol is newline-delimited UTF-8 JSON.
Each request and response is exactly one JSON object followed by `\n`.

Request shape:

```json
{"id":"req-42","command":"motor.set","params":{"port":0,"velocity":600}}
```

Fields:

- `id`: optional request ID; 1-128 characters, ASCII letters, digits, `.`, `_`, `:`, and `-` only
- `command`: required string
- `params`: required object; use `{}` for no arguments

Unknown top-level fields and unknown `params` fields are rejected.
Integer params must be JSON numbers with integral values.
Strings, booleans, `null`, arrays, objects, and fractional numbers are invalid for integer params.

Current daemon commands:

```text
accel
analog
digital
gyro
magneto
side_btn
motor.set
motor.off
motor.get
motor.clear
servo.get_enabled
servo.get
servo.set_enabled
servo.set
```

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

When `id` is absent from the request, omit `id` from the response.

Transport and protocol error codes:

- `daemon_timeout`: daemon timed out reading a request line
- `daemon_protocol_error`: malformed JSON, unknown fields, oversized line
- `daemon_shutting_down`: request rejected during shutdown

Stable validation error codes include `invalid_port`, `invalid_velocity`, `invalid_position`, `invalid_id`, `invalid_axis`, `missing_argument`, and `unknown_command`.

## Response shapes

Success responses have `ok:true`, optional `id`, `command`, and a `result` object.

Single-value reads:

```json
{"ok":true,"command":"analog","result":{"port":0,"value":812}}
```

Bulk reads (port omitted):

```json
{"ok":true,"command":"analog","result":{"values":[812,790,0,0,1023,511]}}
```

Axis-specific sensor reads:

```json
{"ok":true,"command":"accel","result":{"axis":"x","value":-14}}
```

All axes (axis omitted):

```json
{"ok":true,"command":"accel","result":{"x":-14,"y":3,"z":1021}}
```

Writer acknowledgements include requested parameters:

```json
{"ok":true,"command":"servo.set_enabled","result":{"port":1,"enabled":1}}
{"ok":true,"command":"servo.set","result":{"port":1,"position":1200}}
{"ok":true,"command":"motor.set","result":{"port":0,"velocity":600}}
```

## Dispatch

Hardware access is serialized through one dispatcher thread.
Accepted client requests are parsed and validated, then queued as work items.
The dispatcher is the only thread that calls `libwallaby`.

Current implementation handles accepted socket connections inline in the accept loop.
Long-lived concurrent clients need a future socket-handler thread or polling loop before they can share the daemon fairly.

Dispatcher flow:

```text
client request -> parse/validate -> queue -> dispatcher -> libwallaby -> response
```

Queue order defines command order.
Protocol errors that do not touch hardware are answered before entering the queue.

Request line limit is 64 KiB.
Daemon request read timeout is 2 seconds per request line.

## Motor Semantics

Motor commands map to `libwallaby` calls:

```text
motor.set <port> <velocity> -> mav(port, velocity)
motor.off <port>            -> off(port)
motor.off                   -> ao()
motor.get <port>            -> gmpc(port)
motor.clear <port>          -> cmpc(port)
```

`motor get` returns position counter ticks.
It does not return current velocity or last requested velocity.

`motor off` uses passive `off()` / `ao()` behavior.
It is not active braking.

`mav()` is not treated as a blocking operation.
The daemon persists so `libwallaby` cleanup does not run after every `motor set`.
No detached per-request motor setter process exists.

## Motor Timeout

`--motor-timeout-ms <ms>` configures optional motor supervision.
Valid values are positive integer milliseconds.
Default is no timeout.

With timeout configured:

- Every `motor.set`, including velocity `0`, calls `mav(port, velocity)`.
- Every `motor.set` refreshes that port deadline.
- Expired ports are stopped with `off(port)` on the dispatcher thread.
- `motor.off <port>` clears that port deadline.
- `motor.off` clears all deadlines after `ao()`.

The dispatcher waits for queued work or the nearest motor deadline, whichever comes first.
Expired timeout stops and client work both pass through the same serialized hardware path.

## Validation

The daemon validates all request parameters at the socket boundary.

- Analog ports: `0-5`
- Digital ports: `0-9`
- Motor ports: `0-3`
- Motor velocity: `-1500-1500`
- Servo ports: `0-3`
- Servo enabled values: `0` or `1`
- Servo positions: `0-2047`

Servo reader commands accept an omitted `port` and return all four values in `result.values`.
Servo writer commands require an explicit `port`.
