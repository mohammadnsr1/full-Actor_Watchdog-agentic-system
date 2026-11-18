#!/usr/bin/env bash
# Launch an executor AUTHORING session: a Claude Code session that writes a
# planner (docs/PLANNER_CONTRACT.md) and evaluates it on the cell.
#
# Usage: runner/run_authoring.sh [platform] [task]
#   platform  sim (default) | hardware
#   task      default config/tasks/fmb_assembly_2.yaml
#
# Creates executor/workspace/<session>/ with planner/ copied from
# executor/planner_template, then starts claude with ONLY the
# executor-authoring MCP server (--strict-mcp-config) and the restricted
# settings config/settings/executor.<platform>.settings.json. MODEL and EFFORT
# default to that settings file; override with MODEL=... EFFORT=... as for
# run_watchdog.sh. Feedback (what the agent learns from an episode) follows
# config/cell.yaml executor_feedback; AUTHORING_FEEDBACK=checker|watchdog|both
# overrides it for one session.
#
# The cell must be up (Isaac playing with the command server, the cell bringup)
# before evaluate_planner is called; with feedback watchdog|both every episode
# also starts a WatchDog session (claude -p), which costs tokens per episode.
set -euo pipefail
cd "$(dirname "$0")/.."
REPO="$(pwd)"

platform="${1:-sim}"
task="${2:-config/tasks/fmb_assembly_2.yaml}"
case "$platform" in sim|hardware) ;; *) echo "platform must be sim or hardware" >&2; exit 1 ;; esac
config="config/mcp/executor.${platform}.mcp.json"
settings="config/settings/executor.${platform}.settings.json"
assembly="$(basename "$task" .yaml)"

session="${assembly}-$(date -u +%Y%m%dT%H%M%SZ)"
session_dir="$REPO/executor/workspace/$session"
mkdir -p "$session_dir"
cp -r executor/planner_template "$session_dir/planner"
rm -rf "$session_dir/planner/__pycache__" "$session_dir/planner/.pytest_cache"

MODEL="${MODEL:-$(python3 -c "import json; print(json.load(open('$settings')).get('model',''))")}"
EFFORT="${EFFORT:-$(python3 -c "import json; print(json.load(open('$settings')).get('effortLevel','medium'))")}"
FEEDBACK="${AUTHORING_FEEDBACK:-$(python3 -c "import yaml; print(yaml.safe_load(open('config/cell.yaml')).get('executor_feedback','watchdog'))")}"

python3 - "$session_dir" "$platform" "$task" "$MODEL" "$EFFORT" "$FEEDBACK" <<'PYEOF'
import datetime, json, subprocess, sys
d, platform, task, model, effort, feedback = sys.argv[1:7]
try:
    cli = subprocess.run(["claude", "--version"], capture_output=True, text=True, timeout=15).stdout.strip()
except Exception as exc:
    cli = f"unavailable: {exc}"
json.dump({"role": "executor_authoring", "platform": platform, "task": task, "model": model, "effort": effort,
           "feedback": feedback, "claude_cli_version": cli,
           "launched_wall_time_iso": datetime.datetime.now(datetime.timezone.utc).isoformat()},
          open(f"{d}/agent_config.json", "w"), indent=2)
PYEOF

export AUTHORING_SESSION_DIR="$session_dir" AUTHORING_TASK="$task" AUTHORING_ASSEMBLY="$assembly" CELL_PLATFORM="$platform"
[ -n "${AUTHORING_FEEDBACK:-}" ] && export AUTHORING_FEEDBACK
echo "authoring session $session ($platform, task $task, model=$MODEL effort=$EFFORT feedback=$FEEDBACK)" >&2
echo "editable planner: $session_dir/planner" >&2
exec claude --mcp-config "$config" --strict-mcp-config \
  --model "$MODEL" --effort "$EFFORT" \
  --settings "$settings" \
  "You are writing a planner for a robot cell. Call get_task first and follow its how_to: edit the planner in editable_planner_dir, evaluate it with evaluate_planner, read the feedback, iterate, and call submit_work when it is as good as you can make it."
