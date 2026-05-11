#!/usr/bin/env bash

PI=tom@devpi.lan
SRC=$(pwd)
PROJ=botcli

rsync -az --exclude build/ --exclude .git/ --exclude libwallaby "$SRC/" "$PI:~/botcli/"
ssh "$PI" "cd ~/botcli && meson setup build && meson compile -C build"
