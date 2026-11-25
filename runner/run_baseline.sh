#!/usr/bin/env bash
# Launch one executor-baseline authoring session (docs/EXECUTOR_BASELINE.md, EB4).
#
# Usage: runner/run_baseline.sh [baseline|consult] [task_id]
#
# Like the labmate's setup: Claude Code --print in a container (docker/authoring/),
# the prompt as the first user message, MCP servers `kit` (the Kit bridge: run_python,
# files, lab__* verbs) and `platform` (check_delivery, evaluate_planner, submit_work,
# move_robot, robot_health), /app read-write, /app/evaluations and /data read-only.
#
#   executor/workspace/baseline/<session>/
#       app/            -> /app        artifact/ (starts as executor/planner_templates/baseline),
#                                     evaluations/ (written by the platform), report.md
#       data/           -> /data       the staged task view (tools/stage_task_data.py)
#       session/        -> /session    mcp.json, prompt.md
#   records/authoring/<session>/       transcript.jsonl, platform.log, session.json, evals.jsonl,
#                                      snapshots/, agent_config.json   (never the agent's)
#
# Needs: the cell up (Isaac playing with the command server and the bridge's lab_tools,
# sim_moveit_bringup), ANTHROPIC_API_KEY, the image (docker build ... -t cell-authoring),
# and docker access. MODEL=..., WALL_S=... override config/cell.yaml baseline.
set -euo pipefail
cd "$(dirname "$0")/.."
REPO="$(pwd)"
condition="${1:-baseline}"
task_id="${2:-fmb_assembly_2}"
case "$condition" in baseline|consult) ;; *) echo "condition must be baseline or consult" >&2; exit 1 ;; esac
[ "${SHELL_ONLY:-0}" = 1 ] || : "${ANTHROPIC_API_KEY:?ANTHROPIC_API_KEY must be set for the container}"

cfg() { python3 -c "import yaml,sys; print(yaml.safe_load(open('config/cell.yaml'))['baseline'][sys.argv[1]])" "$1"; }
IMAGE="$(cfg image)"; PLATFORM_MCP="$(cfg platform_mcp)"; KIT_PORT="$(cfg kit_bridge_port)"
MODEL="${MODEL:-$(cfg model)}"; WALL_S="${WALL_S:-$(cfg wall_s)}"; BUDGET="$(cfg budget_episodes)"; RO="$(cfg read_only_root)"

session="${task_id}-${condition}-$(date -u +%Y%m%dT%H%M%SZ)"
root="$REPO/executor/workspace/baseline/$session"
records="$REPO/records/authoring/$session"
mkdir -p "$root/app/evaluations" "$root/session" "$records"
cp -rL executor/planner_templates/baseline "$root/app/artifact"
python3 tools/make_task_data.py --check || { echo "task_data/ is stale: python3 tools/make_task_data.py" >&2; exit 1; }
python3 tools/stage_task_data.py "$root/data" --task "$task_id" > "$records/staged.json"
# The planner side's config, as the container sees it at /opt/cell/repo/config (no WatchDog files).
mkdir -p "$root/cellroot/config/platforms" "$root/cellroot/config/cameras" "$root/cellroot/config/parts"
cp config/cell.yaml "$root/cellroot/config/"
cp config/platforms/sim.yaml "$root/cellroot/config/platforms/"
cp config/cameras/sim.yaml "$root/cellroot/config/cameras/"
cp config/parts/executor_parts.yaml "$root/cellroot/config/parts/"

# The prompt: platform + assembly (+ consult) layers (EB5), budget substituted.
layers=(executor/prompts/baseline/platform.md executor/prompts/baseline/assembly.md)
[ "$condition" = consult ] && layers+=(executor/prompts/baseline/consult.md)
for f in "${layers[@]}"; do [ -f "$f" ] || { echo "$f missing (EB5)" >&2; exit 1; }; done
python3 - "$BUDGET" "$task_id" "${layers[@]}" > "$root/session/prompt.md" <<'PY'
import sys
budget, task_id, files = sys.argv[1], sys.argv[2], sys.argv[3:]
text = "\n\n".join(open(f).read().strip() for f in files) + "\n"
print(text.replace("{evaluation_budget_episodes}", budget).replace("{task_id}", task_id), end="")
PY
cat > "$root/session/mcp.json" <<JSON
{"mcpServers": {
  "kit": {"command": "kit-mcp", "env": {"OMNI_KIT_MCP_PORT": "$KIT_PORT", "KIT_MCP_NAMESPACE": "lab"}},
  "platform": {"type": "http", "url": "http://$PLATFORM_MCP/mcp"}
}}
JSON

# The platform MCP server, host side, for this session only.
source /opt/ros/jazzy/setup.bash; source ros2_ws/install/setup.bash
set +u; source ros2_ws/.venv/bin/activate; set -u
PLATFORM_SESSION_DIR="$root/app" PLATFORM_RECORD_DIR="$records" PLATFORM_CONDITION="$condition" \
PLATFORM_TASK="$root/data/tasks/$task_id/task.yaml" PLATFORM_BUDGET="$BUDGET" CELL_REPO_ROOT="$REPO" \
    python executor/platform_mcp/server.py --http "$PLATFORM_MCP" > "$records/platform.log" 2>&1 &
platform_pid=$!
trap 'kill $platform_pid 2>/dev/null || true' EXIT
for _ in $(seq 50); do (echo > "/dev/tcp/${PLATFORM_MCP%:*}/${PLATFORM_MCP#*:}") 2>/dev/null && break; sleep 0.2; done

python3 - "$records/agent_config.json" <<PY
import json, sys
json.dump({"session": "$session", "condition": "$condition", "task": "$task_id", "model": "$MODEL", "image": "$IMAGE",
           "wall_s": $WALL_S, "budget_episodes": $BUDGET, "platform_mcp": "$PLATFORM_MCP", "kit_bridge_port": $KIT_PORT},
          open(sys.argv[1], "w"), indent=2)
PY

ro_args=()
if [ "$RO" = "True" ]; then
  ro_args=(--read-only --tmpfs /tmp --tmpfs /home/agent/.claude-session --tmpfs /home/agent/.cache --tmpfs /home/agent/.local/state)
fi
echo "session $session: /app = $root/app; transcript -> $records/transcript.jsonl" >&2
set +e
docker_args=(--rm --network host --cap-drop ALL --security-opt no-new-privileges "${ro_args[@]}"
    -v "$root/app:/app:rw" -v "$root/app/evaluations:/app/evaluations:ro" -v "$root/data:/data:ro"
    -v "$root/session:/session:ro" -v "$root/cellroot:/opt/cell/repo:ro"
    -v "$REPO/ros2_ws/install/cell_planner_msgs:/opt/cell/cell_planner_msgs:ro"
    -v "$REPO/ros2_ws/install/cell_platform_interfaces:/opt/cell/cell_platform_interfaces:ro"
    -v "$REPO/ros2_ws/install/cell_backend_sim:/opt/cell/cell_backend_sim:ro"
    -v "$REPO/ros2_ws/install/isaac_trajectory_bridge:/opt/cell/isaac_trajectory_bridge:ro"
    -v "$(ls -d "$REPO"/ros2_ws/install/robot_backend/lib/python3*/site-packages/robot_backend/utils):/opt/cell/py/robot_backend/utils:ro"
    -e ANTHROPIC_API_KEY -e MODEL="$MODEL" -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" -e CELL_TASK_DATA=/data)
if [ "${SHELL_ONLY:-0}" = 1 ]; then       # a shell in the container with every mount, no agent (checks)
    docker run -it "${docker_args[@]}" "$IMAGE" shell
    exit 0
fi
timeout --signal=INT "$WALL_S" docker run "${docker_args[@]}" "$IMAGE" > "$records/transcript.jsonl"
rc=$?
set -e
echo "{\"container_exit\": $rc}" > "$records/exit.json"
echo "session $session ended (exit $rc)" >&2
