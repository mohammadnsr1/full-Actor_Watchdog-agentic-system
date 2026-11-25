"""The baseline prompt layers cite only docs that exist, and carry the placeholders run_baseline.sh fills."""
import re
from pathlib import Path

HERE = Path(__file__).resolve().parent


def test_cited_docs_exist_and_placeholders_present():
    text = "".join((HERE / f).read_text() for f in ("platform.md", "assembly.md"))
    for doc in set(re.findall(r"docs/([a-z-]+\.md)", text)) | set(
            re.findall(r"docs/([a-z-]+\.md)", "".join(p.read_text() for p in (HERE / "docs").glob("*.md")))):
        assert (HERE / "docs" / doc).is_file(), doc
    assert "{evaluation_budget_episodes}" in text and "{task_id}" in text


def test_no_watchdog_in_the_baseline_text():
    text = "".join(p.read_text() for p in [HERE / "platform.md", HERE / "assembly.md", *(HERE / "docs").glob("*.md")])
    assert "watchdog" not in text.lower() and "consult" not in text.lower()
