#!/bin/sh
# Download real SENT sensor captures (logic-analyzer CSV exports) published
# by the rusEFI project as decoder test vectors. They are not redistributed
# here; they land in tests/data/ which is git-ignored.
set -e
dir="$(dirname "$0")/../tests/data"
mkdir -p "$dir"
base=https://raw.githubusercontent.com/rusefi/rusefi/master/unit_tests/tests/sent/resources
for f in SENT-fuel-pressure ford-sent-closed ford-sent-idle mb_A_000_905_31_01-ambient \
         opel-throttle-idle opel-throttle-move vag_04e.906.051; do
  [ -s "$dir/$f.csv" ] || curl -sfL -o "$dir/$f.csv" "$base/$f.csv" && echo "  $f.csv"
done
