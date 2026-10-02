"""capture-log against a stand-in for the programmer tool (tests/fake_cli.py)."""
import base64
import binascii
import csv
import os
import re
import struct
import subprocess
import sys
from pathlib import Path

import pytest

from flight_log_tools import capture_log

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


# --- the base64 dump (telemetry_print_b64) ---

TELEMETRY_H = TELEMETRY_C.parents[1] / "Inc" / "telemetry.h"


def firmware_record_format():
    """struct format of TelemetryRecord_t, worked out from telemetry.h with
    the ARM EABI's natural alignment, to check capture_log.RECORD against."""
    src = TELEMETRY_H.read_text()
    body = src[:src.index("} TelemetryRecord_t;")]
    body = body[body.rindex("typedef struct {") + len("typedef struct {"):]
    types = {"uint32_t": ("I", 4), "float": ("f", 4), "uint16_t": ("H", 2),
             "uint8_t": ("B", 1), "int8_t": ("b", 1)}
    fmt, offset, biggest = "<", 0, 1
    for decl in body.split(";"):
        decl = re.sub(r"/\*.*?\*/", "", decl, flags=re.S).split()
        if not decl:
            continue
        code, size = types[decl[0]]
        count = int(re.search(r"\[(\d+)\]", decl[1]).group(1)) if "[" in decl[1] else 1
        pad = -offset % size
        fmt += "x" * pad + code * count
        offset += pad + size * count
        biggest = max(biggest, size)
    fmt += "x" * (-offset % biggest)
    return fmt


def expand(fmt):
    """'<I3bx' -> '<Ibbbx', so formats can be compared"""
    return re.sub(r"(\d+)(\D)", lambda m: m.group(2) * int(m.group(1)), fmt)


def test_decoder_columns_match_firmware():
    assert capture_log.COLUMNS == firmware_header()


def test_decoder_record_layout_matches_firmware():
    assert expand(capture_log.RECORD.format) == firmware_record_format()
    assert capture_log.RECORD.size == 88
    assert "sizeof(TelemetryRecord_t) == 88" in TELEMETRY_H.read_text()


def test_decode_row_formats_like_printf():
    rec = capture_log.RECORD.pack(
        123456, 1, -2, 3,
        -998.94, 47.34, 7.8125, 0.0, -70.0, -630.0,
        33.15, 1002.22, 92.24, 2,
        6925, 10285,
        -0.037, -0.0, 1.5e-4, 2.9, 9,
        0.25, 3048.04, 0x0102, 4, 3)
    data = struct.pack("<I", 383) + rec
    line = "@" + base64.b64encode(data).decode() + "*%04X" % binascii.crc_hqx(data, 0xFFFF)
    assert capture_log.decode_b64(line) == (
        "383,123456,1,-2,3,-998.94,47.34,7.81,0.00,-70.00,-630.00,"
        "33.15,1002.22,92.24,2,BURNOUT,6925,10285,"
        "-0.037,-0.000,0.000,2.90,9,0.2500,3048.0,258,4,3")


@pytest.mark.parametrize("damage", [
    lambda l: l[:20] + ("B" if l[20] == "A" else "A") + l[21:],   # a changed character
    lambda l: l[:20] + l[21:],                                    # a lost character
    lambda l: l.split("*")[0],                                    # no CRC
    lambda l: l[:-1] + "G",                                       # CRC not hex
])
def test_damaged_line_is_rejected(damage):
    data = bytes(range(92))
    line = "@" + base64.b64encode(data).decode() + "*%04X" % binascii.crc_hqx(data, 0xFFFF)
    capture_log.decode_b64(line)   # the undamaged line decodes
    with pytest.raises(ValueError):
        capture_log.decode_b64(damage(line))


def test_b64_dump_decoded(tmp_path):
    proc, out = live(tmp_path, records=300, format="b64")
    assert proc.returncode == 0, proc.stderr
    got = rows(out)
    assert list(got[0]) == firmware_header().split(",")
    assert [int(r["timestamp_ms"]) for r in got] == [1000 + 20 * i for i in range(300)]
    assert [int(r["page"]) for r in got] == [i // 23 for i in range(300)]
    assert got[0]["imu_ax_mg"] == "0.00" and got[0]["fsm_state_name"] == "READY"


def test_b64_damaged_line_counted_as_malformed(tmp_path):
    proc, out = live(tmp_path, records=300, format="b64", corrupt_at=50)
    assert proc.returncode == 2
    assert len(rows(out)) == 299
    assert "1 malformed rows" in proc.stderr


def test_b64_record_size_mismatch_is_an_error(tmp_path):
    proc, out = live(tmp_path, records=50, format="b64", record_bytes=92)
    assert proc.returncode == 2
    assert "update RECORD" in proc.stderr
    assert len(rows(out)) == 0


def test_b64_from_file_matches_live(tmp_path):
    proc, out = live(tmp_path, records=200, format="b64")
    assert proc.returncode == 0, proc.stderr
    again = tmp_path / "again"
    again.mkdir()
    proc2, out2 = run(again, "--from-file", str(out.with_suffix(".raw.txt")))
    assert proc2.returncode == 0, proc2.stderr
    assert out2.read_text() == out.read_text()


def test_find_cli_runs():
    """find_cli() is only used without --cli, which the other tests always pass"""
    found = capture_log.find_cli()
    assert found is None or Path(found).name == "STM32_Programmer_CLI"


MAIN_C = TELEMETRY_C.with_name("main.c")


def test_default_clock_matches_dump_firmware():
    m = re.search(r"#define DUMP_HCLK_MHZ (\d+)", MAIN_C.read_text())
    assert m and int(m.group(1)) == capture_log.DEFAULT_HCLK_MHZ


def test_clock_line_reported(tmp_path):
    proc, _ = live(tmp_path, records=30, format="b64")
    assert proc.returncode == 0, proc.stderr
    assert "board at 128 MHz, SWO at 2000 kHz" in proc.stderr


# --- --last-minutes ---

def _is_dump_build(elf):
    try:
        capture_log.symbol_address(elf, capture_log.REQUEST_SYMBOL)
        return True
    except (OSError, LookupError):
        return False


# the Debug ELF is a dump build only while config.h has LOG_DUMP_AT_BOOT 1
needs_elf = pytest.mark.skipif(not _is_dump_build(capture_log.DEFAULT_ELF),
                               reason="needs a LOG_DUMP_AT_BOOT 1 Debug build")


def test_request_magic_matches_firmware():
    m = re.search(r"#define DUMP_REQUEST_MAGIC (0x[0-9A-Fa-f]+)u", MAIN_C.read_text())
    assert m and int(m.group(1), 16) == capture_log.REQUEST_MAGIC
    assert re.search(r"volatile uint32_t dump_request\[2\];", MAIN_C.read_text())


def test_first_page_arithmetic():
    # 0.5 min = 1500 records = 65.2 pages: 1500 // 23 + 2 = 67 pages back
    assert capture_log.records_for_minutes(0.5) == 1500
    assert capture_log.expected_first_page(100, 1500) == 33
    assert capture_log.expected_first_page(50, 1500) == 0      # more than the log: all of it


@needs_elf
def test_last_minutes_dumps_only_the_end(tmp_path):
    w32 = tmp_path / "w32.txt"
    proc, out = run(tmp_path, "--cli", str(FAKE_CLI), "--last-minutes", "0.5",
                    records=2300, format="b64", w32_log=w32)        # 100 pages
    assert proc.returncode == 0, proc.stderr
    address = capture_log.symbol_address(capture_log.DEFAULT_ELF, "dump_request")
    assert w32.read_text().split() == [f"{address:#x}", "0xd0a9d0a9", "1500"]
    got = rows(out)
    assert int(got[0]["page"]) == 33 and int(got[-1]["page"]) == 99
    assert len(got) == (100 - 33) * 23
    assert "dumping pages 33-99, the newest 67 of 100" in proc.stderr
    assert "100.0%  page 100/100" in proc.stderr


@needs_elf
def test_request_ignored_by_board_warns(tmp_path):
    proc, out = run(tmp_path, "--cli", str(FAKE_CLI), "--last-minutes", "0.5",
                    records=2300, format="b64", w32_log=tmp_path / "w32.txt", ignore_request=1)
    assert proc.returncode == 0, proc.stderr
    assert "but the board starts at page 0" in proc.stderr
    assert len(rows(out)) == 2300


def test_no_request_without_last_minutes(tmp_path):
    w32 = tmp_path / "w32.txt"
    proc, out = live(tmp_path, records=230, format="b64", w32_log=w32)
    assert proc.returncode == 0, proc.stderr
    assert not w32.exists()
    assert len(rows(out)) == 230


def test_last_minutes_needs_the_elf(tmp_path):
    proc, _ = run(tmp_path, "--cli", str(FAKE_CLI), "--last-minutes", "1",
                  "--elf", str(tmp_path / "missing.elf"))
    assert proc.returncode != 0
    assert "missing.elf not found" in proc.stderr
