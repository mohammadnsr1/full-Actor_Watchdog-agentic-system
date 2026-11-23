import json
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from runner.jsonl import JsonlLog, StderrPump, read_jsonl  # noqa: E402


def test_log_appends_one_object_per_line(tmp_path):
    log = JsonlLog(tmp_path / "a" / "runner.jsonl")
    log.write("stage", stage="reset", ok=True)
    log.write("log", msg="hello")
    lines = (tmp_path / "a" / "runner.jsonl").read_text().splitlines()
    assert [json.loads(x)["event"] for x in lines] == ["stage", "log"]
    assert json.loads(lines[0])["stage"] == "reset" and "wall_time_iso" in json.loads(lines[0])


def test_stderr_pump_wraps_lines(tmp_path):
    pump = StderrPump(tmp_path / "p.stderr.jsonl", "bag")
    proc = subprocess.Popen([sys.executable, "-c", "import sys; sys.stderr.write('one\\ntwo\\n')"], stderr=pump.fileno())
    pump.started()
    proc.wait()
    pump.close()
    recs = read_jsonl(tmp_path / "p.stderr.jsonl")
    assert [(r["source"], r["line"]) for r in recs] == [("bag", "one"), ("bag", "two")]


def test_read_skips_a_torn_line(tmp_path):
    p = tmp_path / "x.jsonl"
    p.write_text('{"a": 1}\n{"a": 2}\n{"a"')
    assert read_jsonl(p) == [{"a": 1}, {"a": 2}]
    assert read_jsonl(p, tail=2) == [{"a": 2}]
