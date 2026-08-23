#!/bin/sh
set -eu

repo_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
test_binary="${TMPDIR:-/tmp}/kodenchan_uplink_protocol_test.$$"

trap 'rm -f "$test_binary"' EXIT HUP INT TERM

cc -std=c11 -Wall -Wextra -Werror -pedantic \
  -I"$repo_dir/include" \
  "$repo_dir/test/host/test_uplink_protocol.c" \
  "$repo_dir/src/protocol/ac_packet_v6.c" \
  "$repo_dir/src/protocol/crc16_ccitt.c" \
  "$repo_dir/src/protocol/rover_up_general.c" \
  "$repo_dir/src/protocol/uplink_stream_parser.c" \
  -o "$test_binary"

"$test_binary"
