"""capture-log against a stand-in for the programmer tool (tests/fake_cli.py)."""
import csv
import os
import subprocess
import sys
from pathlib import Path

import pytest

FAKE_CLI = Path(__file__).with_name("fake_cli.py")
# the firmware's header, as telemetry_print_csv_header() prints it
TELEMETRY_C = Path(__file__).parents[2] / "Core" / "Src" / "telemetry.c"


def firmware_header():
    """Rebuild the header from the C string literals in telemetry.c, so the
    tests follow the firmware if its columns change."""
    src = TELEMETRY_C.read_text()
    body = src[src.index("void telemetry_print_csv_header(void)"):]
    body = body[:body.index(");")]
    parts = []
    for line in body.splitlines():
        if '"' in line:
            parts.append(line[line.index('"') + 1:line.rindex('"')])
    return "".join(parts).replace("\\r\\n", "")


def run(tmp_path, *args, **fake):
    env = dict(os.environ, FAKE_HEADER=firmware_header(),
               **{f"FAKE_{k.upper()}": str(v) for k, v in fake.items()})
    out = tmp_path / "log.csv"
    proc = subprocess.run(
        [sys.executable, "-m", "flight_log_tools.capture_log", "-o", str(out),
         "--idle-timeout", "5", *args],
        env=env, capture_output=True, text=True, timeout=60)
    return proc, out


def live(tmp_path, **fake):
    return run(tmp_path, "--cli", str(FAKE_CLI), **fake)


def rows(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def test_header_matches_firmware():
    header = firmware_header()
    assert header.startswith("page,timestamp_ms,")
    assert len(header.split(",")) == len(set(header.split(",")))


def test_clean_dump_saved_in_order(tmp_path):
    proc, out = live(tmp_path, records=500)
    assert proc.returncode == 0, proc.stderr
    got = rows(out)
    assert len(got) == 500
    assert list(got[0]) == firmware_header().split(",")
    t = [int(r["timestamp_ms"]) for r in got]
    assert t == sorted(t)
    assert "end of log received" in proc.stderr


def test_gap_reboot_and_bad_row_reported(tmp_path):
    proc, out = live(tmp_path, records=300, gap_at=100, reboot_at=200, bad_row=1)
    assert proc.returncode == 2
    assert len(rows(out)) == 300
    assert "1 gaps longer than" in proc.stderr
    assert "power-up or reboot" in proc.stderr
    assert "1 malformed rows" in proc.stderr
    # logged time is summed across the reboot: records 0-199 span
    # 199 x 20 ms + the 200 ms gap = 4.18 s, records 200-299 span 99 x 20 ms
    # = 1.98 s, so 6.16 s
    assert "6.2 s of log" in proc.stderr


def test_missing_end_marker_is_flagged(tmp_path):
    proc, out = live(tmp_path, records=50, end=0)
    assert proc.returncode == 2
    assert len(rows(out)) == 50
    assert "before the end-of-log marker" in proc.stderr


def test_from_file_matches_live(tmp_path):
    proc, out = live(tmp_path, records=200)
    assert proc.returncode == 0, proc.stderr
    raw = out.with_suffix(".raw.txt")
    again = tmp_path / "again"
    again.mkdir()
    proc2, out2 = run(again, "--from-file", str(raw))
    assert proc2.returncode == 0, proc2.stderr
    assert out2.read_text() == out.read_text()


@pytest.mark.parametrize("text", ["", "boot chatter only\n"])
def test_no_header_is_an_error(tmp_path, text):
    saved = tmp_path / "saved.txt"
    saved.write_text(text)
    proc, _ = run(tmp_path, "--from-file", str(saved))
    assert proc.returncode == 1
    assert "No CSV header" in proc.stderr


def test_progress_uses_log_size(tmp_path):
    proc, _ = live(tmp_path, records=460)            # 20 pages
    assert proc.returncode == 0, proc.stderr
    assert "log on the board: 20 pages, up to 460 records" in proc.stderr
    assert "100.0%  page 20/20" in proc.stderr


def test_dump_shorter_than_log_size_is_flagged(tmp_path):
    proc, _ = live(tmp_path, records=230, pages=40)  # rows stop at page 10 of 40
    assert proc.returncode == 2
    assert "the log has 40 pages but the last row received is from page 10" in proc.stderr


def test_old_firmware_without_log_size_still_works(tmp_path):
    proc, out = live(tmp_path, records=100, info=0)
    assert proc.returncode == 0, proc.stderr
    assert len(rows(out)) == 100
    assert "log size not reported" in proc.stderr
