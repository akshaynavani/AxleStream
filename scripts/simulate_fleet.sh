#!/usr/bin/env bash
# Generate telemetry for a small simulated fleet: one axlestream process per vehicle, each
# with its own scenario seed and vehicle id, all writing into the same output directory.
#
#   scripts/simulate_fleet.sh [VEHICLES] [SECONDS] [OUT_DIR]
#
# Vehicles run in parallel in realtime, so wall time ~= SECONDS and latency figures are
# representative. FAST=1 generates data as fast as possible instead (lossless, but frames
# queue behind each other so end-to-end latency then includes queueing delay).
# FAULT_RATE injects CAN checksum errors into odd-numbered vehicles, e.g. FAULT_RATE=0.01.
set -euo pipefail

VEHICLES="${1:-4}"
SECONDS_PER_VEHICLE="${2:-120}"
OUT_DIR="${3:-telemetry_out}"
BIN="${AXLE_BIN:-build/axlestream}"
FAULT_RATE="${FAULT_RATE:-0.005}"
MODE_FLAGS=()
if [[ "${FAST:-0}" == "1" ]]; then MODE_FLAGS+=(--fast); fi

if [[ ! -x "$BIN" ]]; then
  echo "axlestream binary not found at $BIN (build first: cmake -B build && cmake --build build -j)" >&2
  exit 1
fi

mkdir -p "$OUT_DIR"
pids=()
for i in $(seq 1 "$VEHICLES"); do
  vid=$(printf "veh-%03d" "$i")
  fault=0
  if (( i % 2 == 1 )); then fault="$FAULT_RATE"; fi
  "$BIN" run --source sim --quiet ${MODE_FLAGS[@]+"${MODE_FLAGS[@]}"} \
    --duration "$SECONDS_PER_VEHICLE" \
    --seed "$((1000 + i))" \
    --vehicle-id "$vid" \
    --can-fault-rate "$fault" \
    --out "$OUT_DIR" > "$OUT_DIR/$vid.log" 2>&1 &
  pid=$!
  pids+=("$pid")
  echo "started $vid (pid $pid, can fault rate $fault)"
done

status=0
for pid in "${pids[@]}"; do
  wait "$pid" || status=1
done

echo "fleet simulation done -> $OUT_DIR"
find "$OUT_DIR/frames" -name '*.jsonl' | wc -l | xargs echo "jsonl files:"
exit "$status"
