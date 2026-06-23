# botcli

Persistent C++ daemon around [libwallaby](https://github.com/kipr/libwallaby) for KIPR/Wombat-style robot hardware.
It is the sole process that touches hardware and exposes sensor, motor, and servo operations over a Unix domain socket.

Current state:

- Native Raspberry Pi build with Meson and installed `libwallaby`.
- Foreground daemon owns all `libwallaby` access.
- External callers (e.g. Go) connect to the socket and speak the JSON protocol.
- Camera commands are deferred.

Protocol and internals are documented in [docs/daemon_arch.md](docs/daemon_arch.md).

## Build

`botcli` is intended to build on the Raspberry Pi where `libwallaby` is installed.
Cross-compilation is out of scope for now.

```sh
meson setup build
meson compile -C build
```

During development, `build-deploy.sh` copies the project to the Pi and compiles it there.

Third-party single-header dependencies are vendored in `third_party/`:

- `picojson.h` for JSON
- `lest.hpp` for future tests

`libwallaby/` may exist locally as an intentionally untracked reference checkout, but it is not project source.

## Run

Start the daemon:

```sh
botcli
```

Optional startup flags:

```text
botcli [--socket <path>] [--motor-timeout-ms <ms>]
```

`--socket` defaults to `$XDG_RUNTIME_DIR/botcli.sock` when `XDG_RUNTIME_DIR` is set, otherwise `/tmp/botcli.sock`.

Motor timeout is configured at startup:

```sh
botcli --motor-timeout-ms 500
```

Timeout values must be positive integer milliseconds.
With no timeout, motor velocity commands remain active until `motor.off` or daemon shutdown.

The daemon stays in the foreground until `SIGINT` or `SIGTERM`.

## Client interface

There is no CLI client.
Callers connect to the Unix socket and send newline-delimited JSON requests.
Each request receives one JSON response line.

Example:

```sh
echo '{"command":"analog","params":{"port":0}}' | socat - UNIX-CONNECT:/tmp/botcli.sock
```

See [docs/daemon_arch.md](docs/daemon_arch.md) for the full protocol, supported commands, and response shapes.

## Deferred

Camera commands are planned later.
