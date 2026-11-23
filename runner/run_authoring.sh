#!/usr/bin/env bash
# Launch an executor AUTHORING session: a Claude Code session that writes a
# planner (docs/PLANNER_CONTRACT.md) and evaluates it on the cell.
#
# Usage: runner/run_authoring.sh [platform] [condition] [task]
#   platform   sim (default) | hardware
#   condition  plan_given (default) | plan_free
#   task       the FULL task file, default config/tasks/fmb_assembly_2.yaml
#
# The condition is a study variable:
#   plan_given  the agent gets the full task file (instruction, step plan,
#               steps with grasp and insertion offsets) and starts from
#               executor/planner_templates/plan_given, which executes the steps.
#   plan_free   the agent gets <task>.plan_free.yaml (goal, constraints and
#               object descriptions only; tools/derive_plan_free_task.py) plus
#               the part data, and starts from executor/planner_templates/plan_free,
#               a skeleton with the contract plumbing and no assembly logic.
#               Its settings deny reading every place the plan is written.
#
# Creates executor/workspace/<condition>/<session>/planner/ (the editable
# planner, a dereferenced copy of the template) and records/authoring/<session>/
# (agent_config.json, session.json, evals.jsonl: out of the agent's reach,
# since they carry the checker outcome). Then starts claude with ONLY the
# executor-authoring MCP server (--strict-mcp-config) and the condition's
# settings, config/settings/executor.<platform>[.plan_free].settings.json.
# MODEL and EFFORT default to that settings file; override with MODEL=...
# EFFORT=... as for run_watchdog.sh. Feedback follows config/cell.yaml
# executor_feedback; AUTHORING_FEEDBACK=checker|watchdog|both overrides it.
#
# The cell must be up (Isaac playing with the command server, the cell bringup)
# before evaluate_planner is called; with feedback watchdog|both every episode
# also starts a WatchDog session (claude -p), which costs tokens per episode.
set -euo pipefail
cd "$(dirname "$0")/.."
REPO="$(pwd)"

platform="${1:-sim}"
condition="${2:-plan_given}"
full_task="${3:-config/tasks/fmb_assembly_2.yaml}"
case "$platform" in sim|hardware) ;; *) echo "platform must be sim or hardware" >&2; exit 1 ;; esac
case "$condition" in
  plan_given) task="$full_task"; settings="config/settings/executor.${platform}.settings.json" ;;
  plan_free)  task="${full_task%.yaml}.plan_free.yaml"; settings="config/settings/executor.${platform}.plan_free.settings.json"
              [ -f "$task" ] || { echo "$task missing: python3 tools/derive_plan_free_task.py $full_task" >&2; exit 1; } ;;
  *) echo "condition must be plan_given or plan_free" >&2; exit 1 ;;
esac
config="config/mcp/executor.${platform}.mcp.json"
template="executor/planner_templates/${condition}"
assembly="$(python3 -c "import yaml,sys; print(yaml.safe_load(open(sys.argv[1]))['name'])" "$full_task")"

session="${assembly}-${condition}-$(date -u +%Y%m%dT%H%M%SZ)"
session_dir="$REPO/executor/workspace/$condition/$session"
record_dir="$REPO/records/authoring/$session"
mkdir -p "$session_dir" "$record_dir"
cp -rL "$template" "$session_dir/planner"
rm -rf "$session_dir/planner/__pycache__" "$session_dir/planner/.pytest_cache"

MODEL="${MODEL:-$(python3 -c "import json; print(json.load(open('$settings')).get('model',''))")}"
EFFORT="${EFFORT:-$(python3 -c "import json; print(json.load(open('$settings')).get('effortLevel','medium'))")}"
FEEDBACK="${AUTHORING_FEEDBACK:-$(python3 -c "import yaml; print(yaml.safe_load(open('config/cell.yaml')).get('executor_feedback','watchdog'))")}"

python3 - "$record_dir" "$platform" "$condition" "$task" "$template" "$settings" "$MODEL" "$EFFORT" "$FEEDBACK" <<'PYEOF'
import datetime, json, subprocess, sys
d, platform, condition, task, template, settings, model, effort, feedback = sys.argv[1:10]
try:
    cli = subprocess.run(["claude", "--version"], capture_output=True, text=True, timeout=15).stdout.strip()
except Exception as exc:
    cli = f"unavailable: {exc}"
json.dump({"role": "executor_authoring", "platform": platform, "condition": condition, "task": task, "template": template,
           "settings": settings, "model": model, "effort": effort, "feedback": feedback, "claude_cli_version": cli,
           "launched_wall_time_iso": datetime.datetime.now(datetime.timezone.utc).isoformat()},
          open(f"{d}/agent_config.json", "w"), indent=2)
PYEOF

export AUTHORING_SESSION_DIR="$session_dir" AUTHORING_RECORD_DIR="$record_dir" AUTHORING_TASK="$task" \
       AUTHORING_ASSEMBLY="$assembly" AUTHORING_CONDITION="$condition" CELL_PLATFORM="$platform"
[ -n "${AUTHORING_FEEDBACK:-}" ] && export AUTHORING_FEEDBACK
echo "authoring session $session ($platform, $condition, task $task, model=$MODEL effort=$EFFORT feedback=$FEEDBACK)" >&2
echo "editable planner: $session_dir/planner   records: $record_dir" >&2
exec claude --mcp-config "$config" --strict-mcp-config \
  --model "$MODEL" --effort "$EFFORT" \
  --settings "$settings" \
  "You are writing a planner for a robot cell. Call get_task first and follow its how_to: edit the planner in editable_planner_dir, evaluate it with evaluate_planner, read the feedback, iterate, and call submit_work when it is as good as you can make it."
