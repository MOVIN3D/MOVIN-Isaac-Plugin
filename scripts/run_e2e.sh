#!/usr/bin/env bash
# End-to-end test of examples/mocap_to_isaaclab.py in Isaac Lab.
#
# Runs every input mode (bvh, live, replay) with both skeleton presets, all view
# modes and both robots. Live mode is fed by scripts/fake_movin_studio.py, its
# --record output is then replayed, so no MOVIN Studio or mocap rig is needed.
# A scenario passes when the app reaches --max_frames, exits with status 0 and
# logs no Python traceback, crash or retarget error.
#
# Usage (from an environment where Isaac Lab is installed):
#   scripts/run_e2e.sh              # headless
#   scripts/run_e2e.sh --gui        # windowed, saves a viewport screenshot per scenario
#   scripts/run_e2e.sh live         # only scenarios whose name matches the regex
#
# Env: PYTHON (default: python), E2E_OUT (default: build/e2e), E2E_PORT (default: 11235)
set -u

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
PYTHON=${PYTHON:-python}
PORT=${E2E_PORT:-11235}
GUI=0
FILTER=""
for arg in "$@"; do
  case $arg in
    --gui) GUI=1 ;;
    -h|--help) sed -n '2,17p' "$0"; exit 0 ;;
    *) FILTER=$arg ;;
  esac
done

MODE=$([[ $GUI == 1 ]] && echo gui || echo headless)
OUT=${E2E_OUT:-$ROOT/build/e2e}/$MODE
LOGS=$OUT/logs; REC=$OUT/recordings; SHOTS=$OUT/shots
mkdir -p "$LOGS" "$REC" "$SHOTS"
: > "$OUT/summary.txt"
export OMNI_KIT_ACCEPT_EULA=${OMNI_KIT_ACCEPT_EULA:-YES} PYTHONUNBUFFERED=1
cd "$ROOT"

failures=0
run() {  # name, bvh to stream (or -), max_frames, args...
  local name=$1 stream_bvh=$2 frames=$3; shift 3
  [[ -n "$FILTER" && ! "$name" =~ $FILTER ]] && return
  local log=$LOGS/$name.log sender=""
  if [[ "$stream_bvh" != "-" ]]; then
    "$PYTHON" scripts/fake_movin_studio.py "$stream_bvh" --port "$PORT" > "$LOGS/$name.stream.log" 2>&1 &
    sender=$!
  fi
  local cmd=("$PYTHON" examples/mocap_to_isaaclab.py --headless)
  if [[ $GUI == 1 ]]; then
    cmd=(env CAPTURE_PREFIX="$SHOTS/$name" CAPTURE_STEPS=$(( frames * 2 / 3 ))
         "$PYTHON" scripts/capture_viewport.py examples/mocap_to_isaaclab.py)
  fi

  local t0=$SECONDS
  timeout 600 "${cmd[@]}" --debug --max_frames "$frames" "$@" > "$log" 2>&1
  local rc=$? secs=$(( SECONDS - t0 ))
  if [[ -n "$sender" ]]; then kill "$sender" 2>/dev/null; wait "$sender" 2>/dev/null; fi

  local tracebacks crashes retarget_errors preset fps ik status=PASS
  tracebacks=$(grep -c "^Traceback (most recent call last)" "$log")
  crashes=$(grep -c "A crash has occurred" "$log")
  retarget_errors=$(grep -c "Retarget error" "$log")
  preset=$(grep -oP "Skeleton preset: \K\S+" "$log" | head -1)
  fps=$(grep -oP "Render FPS: \K[0-9.]+" "$log" | tail -1)
  ik=$(grep -oP "IK error: \K[0-9.]+" "$log" | tail -1)
  if [[ $rc -ne 0 || $tracebacks -ne 0 || $crashes -ne 0 || $retarget_errors -ne 0 ]]; then
    status=FAIL
  elif [[ "$name" == *print_joints* ]]; then
    grep -q "Isaac Lab Joint Order" "$log" || status=FAIL
  else
    grep -q "Reached --max_frames" "$log" || status=FAIL
  fi
  [[ $status == FAIL ]] && failures=$((failures + 1))
  printf "%-4s %-30s rc=%-3s %4ss  preset=%-12s fps=%-6s ik_err=%s\n" \
    "$status" "$name" "$rc" "$secs" "${preset:--}" "${fps:--}" "${ik:--}" | tee -a "$OUT/summary.txt"
}

V3=data/test_V3.bvh
LEGACY=data/Locomotion.bvh

# BVH playback
run bvh_v3_skeleton               -   300 --mode bvh --bvh_file $V3
run bvh_v3_mesh_skeleton_g1       -   300 --mode bvh --bvh_file $V3 --view_mode mesh_skeleton --robot unitree_g1
run bvh_v3_mesh                   -   300 --mode bvh --bvh_file $V3 --view_mode mesh
run bvh_legacy_mesh_skeleton      -   300 --mode bvh --bvh_file $LEGACY --view_mode mesh_skeleton
run bvh_legacy_g1_hands_robot_only -  300 --mode bvh --bvh_file $LEGACY --robot unitree_g1_with_hands --robot_view robot_only --forward_mode isaac_world
run bvh_v3_print_joints           -   10  --mode bvh --bvh_file $V3 --print_joints
# Live from the fake MOVIN Studio stream, recorded for the replay scenarios
run live_v3_g1_overlay_record     $V3     600 --mode live --port "$PORT" --view_mode mesh_skeleton --robot unitree_g1 --robot_view overlay --record "$REC/live_v3.pkl"
run live_legacy_record            $LEGACY 600 --mode live --port "$PORT" --preset movinman --view_mode mesh_skeleton --record "$REC/live_legacy.pkl"
# Replay of those recordings
run replay_v3_mesh_skeleton_g1    -   300 --mode replay --recording "$REC/live_v3.pkl" --view_mode mesh_skeleton --robot unitree_g1
run replay_legacy_skeleton        -   300 --mode replay --recording "$REC/live_legacy.pkl"

echo "Logs: $LOGS$([[ $GUI == 1 ]] && echo "  Screenshots: $SHOTS")"
[[ $failures -eq 0 ]] && echo "All scenarios passed." || echo "$failures scenario(s) failed."
exit $(( failures > 0 ))
