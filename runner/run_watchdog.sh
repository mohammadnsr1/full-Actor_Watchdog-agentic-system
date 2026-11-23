#!/usr/bin/env bash
# Launch a WatchDog Claude Code session by hand (the plan-blind session).
#
# Usage: runner/run_watchdog.sh [plan_blind] [2]
#
# Episodes normally start their own WatchDog session (runner/episode_runner,
# --watchdog plan_blind) with WATCHDOG_SESSION_DIR=records/<episode>/watchdog;
# this script is for a manual session against a running cell.
#
# Model and effort are NOT arguments: they live in watchdog.settings.json as
# "model" and "effortLevel". Both are echoed below so the launch line records
# what the run used.
#
# Always passes --strict-mcp-config and --settings watchdog.settings.json, and
# submits the bootstrap instruction as the session's first message. The task
# is fetched in two parts because the combined get_rendered_task result
# exceeds the CLI's MCP result-size cap.
#
# The v1 S3 session (step-bearing task file) was removed in Milestone 7.
set -euo pipefail
cd "$(dirname "$0")/.."

scenario="${1:-plan_blind}"
assembly="${2:-2}"
case "$scenario" in
  plan_blind) config="config/mcp/watchdog.plan_blind.mcp.json" ;;
  *) echo "scenario must be plan_blind (got: $scenario)" >&2; exit 1 ;;
esac
if [ "$assembly" != "2" ]; then
  echo "only assembly 2 is available in this repo (got: $assembly)" >&2
  exit 1
fi
settings="config/settings/watchdog.settings.json"

# MODEL and EFFORT come from the environment, defaulting to the values in
# watchdog.settings.json ("model" / "effortLevel"), and are passed as explicit
# --model/--effort flags -- the highest-precedence form the installed claude
# CLI supports, so nothing in the user's global settings can silently override
# what this launch line says. The study varies ONLY the WatchDog's effort;
# override per run with e.g.:  EFFORT=high runner/run_watchdog.sh
MODEL="${MODEL:-$(python3 -c "import json; print(json.load(open('$settings')).get('model','claude-sonnet-5'))")}"
EFFORT="${EFFORT:-$(python3 -c "import json; print(json.load(open('$settings')).get('effortLevel','low'))")}"

# Put the launch parameters in the run record. The manifest is written by the
# bringup before any session exists, so the launcher is the first place that
# knows which model and effort this role actually ran with.
session_dir=$(cat ~/.watchdog/current_session 2>/dev/null || true)
if [ -n "$session_dir" ] && [ -d "$session_dir" ]; then
  python3 - "$session_dir" "$scenario" "$assembly" "$MODEL" "$EFFORT" <<'PYEOF'
import datetime, json, subprocess, sys
session_dir, scenario, assembly, model, effort = sys.argv[1:6]
try:
    cli = subprocess.run(["claude", "--version"], capture_output=True,
                         text=True, timeout=15).stdout.strip()
except Exception as exc:
    cli = f"unavailable: {exc}"
record = {
    "role": "watchdog", "scenario": scenario, "assembly": assembly,
    "model": model, "effort": effort, "claude_cli_version": cli,
    "launched_wall_time_iso": datetime.datetime.now(datetime.timezone.utc).isoformat(),
}
path = f"{session_dir}/agent_config_watchdog.json"
with open(path, "w") as handle:
    json.dump(record, handle, indent=2)
print(f"wrote {path}", file=sys.stderr)
PYEOF
else
  echo "WARNING: ~/.watchdog/current_session does not name a directory;" \
       "agent_config_watchdog.json not written (bring the cell up first)" >&2
fi

echo "launching watchdog session: $config  (model=$MODEL effort=$EFFORT)" >&2
exec claude --mcp-config "$config" --strict-mcp-config \
  --model "$MODEL" --effort "$EFFORT" \
  --settings "$settings" \
  'Call get_rendered_task(part="system_prompt") and then get_rendered_task(part="task"). Load system_prompt as your standing instructions and task as your opening task, then begin your role.'
