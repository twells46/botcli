#!/usr/bin/env bash

PI=tom@devpi.lan
SRC="$(pwd)"
PROJ=botcli

rsync -az --exclude build/ --exclude .git/ --exclude libwallaby "${SRC}/" "${PI}:~/${PROJ}/"
# $PROJ should expand client-side
# shellcheck disable=SC2029
ssh "${PI}" "cd ~/${PROJ} && meson setup build && meson compile -C build"
