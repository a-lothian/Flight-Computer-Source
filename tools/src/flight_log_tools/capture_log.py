"""
Collect the flight computer's NAND log as it streams out over SWO, and save
it as a CSV file.

The firmware side: build Debug with LOG_DUMP_AT_BOOT 1 (config.h) and flash
it. At boot it waits for a debugger to turn SWO on, prints the log's size and
the CSV header, then one line per record, then "# end of log", and stops.
Each record line is the record's raw bytes in base64 (telemetry_print_b64 in
telemetry.c), which this script decodes back into the CSV row. Plain CSV rows
(telemetry_print_csv, older firmware) are accepted too.

This script runs STM32CubeProgrammer's command line tool (bundled with
STM32CubeIDE), which connects over the ST-Link without halting or resetting
the board and turns SWO on. That starts the dump. CubeIDE must not have a
debug session open, since only one program can use the ST-Link at a time.

    cd tools
    uv run capture-log                       # -> log_<date>_<time>.csv
    uv run capture-log -o flight1.csv
    uv run capture-log --last-minutes 10     # only the newest 10 minutes
    uv run capture-log --from-file swv.txt   # text saved from the SWV console

Everything received is also written, unparsed, to <output>.raw.txt, so
nothing is lost if a line can't be parsed.
"""

import argparse
import base64
import binascii
import glob
import math
import os
import re
import select
import shutil
import struct
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

HEADER_START = "page,timestamp_ms,"
LOG_INFO = re.compile(r"# log pages=(\d+) records_per_page=(\d+)")
FORMAT_INFO = re.compile(r"# format=b64 record_bytes=(\d+)")
CLOCK_INFO = re.compile(r"# clock hclk_mhz=(\d+) clock_up=(-?\d+) swo_khz=(\d+)")
# the dump firmware's core clock (DUMP_HCLK_MHZ in main.c): the programmer
# times SWO from it, so it must match
DEFAULT_HCLK_MHZ = 128
FIRST_PAGE_INFO = re.compile(r"# dump first_page=(\d+)")
# --last-minutes: written to dump_request in main.c over SWD
REQUEST_SYMBOL = "dump_request"
REQUEST_MAGIC = 0xD0A9D0A9            # DUMP_REQUEST_MAGIC in main.c
RECORDS_PER_PAGE = 23
# the Debug build, which is the one that dumps
DEFAULT_ELF = Path(__file__).resolve().parents[3] / "Debug" / "flight-computer-testing.elf"


def symbol_address(elf_path, name):
    """Address of a global variable in the firmware ELF."""
    from elftools.elf.elffile import ELFFile
    with open(elf_path, "rb") as f:
        found = ELFFile(f).get_section_by_name(".symtab").get_symbol_by_name(name)
    if not found:
        raise LookupError(f"{name} not in {elf_path}: is it a LOG_DUMP_AT_BOOT 1 Debug build?")
    return found[0]["st_value"]


def records_for_minutes(minutes):
    return math.ceil(minutes * 60_000 / LOOP_MS)


def expected_first_page(pages_total, records):
    """The page the firmware starts at for a request of `records` (main.c)."""
    pages = records // RECORDS_PER_PAGE + 2
    return pages_total - pages if pages < pages_total else 0
END_MARK = "# end of log"
LOOP_MS = 20                 # MIN_CONTROL_LOOP_PERIOD_MS in config.h
GAP_MS = int(LOOP_MS * 1.5)  # a step longer than this means lost records
ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")

# TelemetryRecord_t (telemetry.h) as it is on the flash: little-endian, with
# the compiler's padding (x). Change it with the struct.
RECORD = struct.Struct("<I3bx9fB3xII4fB3x2fHBB")
# telemetry_print_b64 sends the page number, then the record
PACKET = struct.Struct("<I" + RECORD.format[1:])
# FSM_StateName() in fsm.c
FSM_NAMES = ["READY", "THRUST", "BURNOUT", "APOGEE", "SECOND_CHUTE", "LANDED"]
# the columns, in the order csv_row() writes them
COLUMNS = ("page,timestamp_ms,h3lis_x,h3lis_y,h3lis_z,"
           "imu_ax_mg,imu_ay_mg,imu_az_mg,imu_gx_mdps,imu_gy_mdps,imu_gz_mdps,"
           "temperature_C,pressure_hPa,altitude_m,fsm_state,fsm_state_name,"
           "loop_us,loop_us_max,"
           "fusion_altitude_m,fusion_velocity_mps,fusion_accel_vert_mps2,fusion_tilt_deg,fusion_flags,"
           "ab_cmd,ab_pred_apogee_m,ab_faults,ab_mode,ab_boot")


def decode_b64(line):
    """'@<base64>*<CRC hex>' -> the CSV row telemetry_print_csv would print.
    Raises ValueError if the line is damaged."""
    body, sep, crc = line[1:].partition("*")
    if not sep or len(crc) != 4:
        raise ValueError("no CRC")
    data = base64.b64decode(body, validate=True)
    if len(data) != PACKET.size:
        raise ValueError(f"{len(data)} bytes, expected {PACKET.size}")
    if binascii.crc_hqx(data, 0xFFFF) != int(crc, 16):
        raise ValueError("CRC mismatch")
    return csv_row(PACKET.unpack(data))


def csv_row(v):
    """Format the unpacked page and record exactly as telemetry_print_csv's
    printf does. A float printed with %.2f comes out the same in C and Python,
    except a NaN with its sign bit set, which newlib prints as -nan."""
    (page, t, hx, hy, hz, ax, ay, az, gx, gy, gz, temp, prs, alt, state,
     loop_us, loop_max, f_alt, f_vel, f_acc, f_tilt, f_flags,
     ab_cmd, ab_apogee, ab_faults, ab_mode, ab_boot) = v

    def f(x, places):
        if math.isnan(x):
            return "-nan" if math.copysign(1.0, x) < 0 else "nan"
        return f"{x:.{places}f}"

    name = FSM_NAMES[state] if state < len(FSM_NAMES) else "UNKNOWN"
    return ",".join([
        str(page), str(t), str(hx), str(hy), str(hz),
        f(ax, 2), f(ay, 2), f(az, 2), f(gx, 2), f(gy, 2), f(gz, 2),
        f(temp, 2), f(prs, 2), f(alt, 2), str(state), name,
        str(loop_us), str(loop_max),
        f(f_alt, 3), f(f_vel, 3), f(f_acc, 3), f(f_tilt, 2), str(f_flags),
        f(ab_cmd, 4), f(ab_apogee, 1), str(ab_faults), str(ab_mode), str(ab_boot)])


CLI_GLOB = ("/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/"
            "com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.*/tools/bin/STM32_Programmer_CLI")


def find_cli():
    found = sorted(glob.glob(CLI_GLOB)) or [shutil.which("STM32_Programmer_CLI")]
    return found[-1]


def write_request(cli, address, records):
    """Write the --last-minutes request into dump_request, in a CLI run of its
    own: given -w32 and -swv together, the CLI writes but never starts SWV.
    Hot plug, so the board isn't halted or reset and keeps waiting for SWO."""
    cmd = [cli, "-c", "port=SWD", "mode=HOTPLUG",
           "-w32", f"0x{address:08X}", f"0x{REQUEST_MAGIC:08X}", f"0x{records:08X}"]
    print("running:", " ".join(cmd), file=sys.stderr)
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, errors="replace", timeout=60)
    except subprocess.TimeoutExpired:
        sys.exit("the request write timed out - is the ST-Link connected?")
    out = ANSI.sub("", proc.stdout + proc.stderr)
    if proc.returncode != 0 or "download complete" not in out:
        tail = "\n".join(out.strip().splitlines()[-6:])
        sys.exit(f"couldn't write the request to the board:\n{tail}")


def lines_from_cli(cli, hclk_mhz, swv_log, idle_s):
    """Run the CubeProgrammer CLI and yield each line it prints.

    It runs on a pseudo-terminal, because a program writing into a plain pipe
    usually holds its output back in large chunks instead of sending each line
    as it arrives. Stops after idle_s seconds with nothing new."""
    import pty

    cmd = [cli, "-c", "port=SWD", "mode=HOTPLUG",
           "-swv", f"freq={hclk_mhz}", "portnumber=0", str(swv_log), "-RA"]
    print("running:", " ".join(cmd), file=sys.stderr)
    master, slave = pty.openpty()
    proc = subprocess.Popen(cmd, stdin=slave, stdout=slave, stderr=slave)
    os.close(slave)
    pending = b""
    last_data = time.monotonic()
    try:
        while True:
            ready, _, _ = select.select([master], [], [], 0.5)
            if ready:
                try:
                    chunk = os.read(master, 65536)
                except OSError:      # the CLI exited and closed its terminal
                    chunk = b""
                if not chunk:
                    break
                last_data = time.monotonic()
                pending += chunk
                *done, pending = pending.split(b"\n")
                for raw in done:
                    yield raw.decode(errors="replace")
            elif proc.poll() is not None or time.monotonic() - last_data > idle_s:
                break
        if pending:
            yield pending.decode(errors="replace")
    finally:
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()
        os.close(master)


def lines_from_file(path):
    with open(path, errors="replace") as f:
        for line in f:
            yield line


class Collector:
    """Turns the stream of console lines into CSV rows, and checks them."""

    def __init__(self, out_path, request_records=None):
        self.request_records = request_records   # --last-minutes, as records
        self.first_page = 0                      # where the firmware starts the dump
        self.out = open(out_path, "w", newline="")
        self.raw = open(out_path.with_suffix(".raw.txt"), "w")
        self.columns = 0
        self.rows = 0
        self.bad = []          # lines that looked like rows but didn't parse
        self.gaps = []         # (previous timestamp, timestamp)
        self.backwards = []    # timestamp went down: the board rebooted
        self.first_t = self.last_t = None
        self.span_ms = 0       # logged time, summed over reboots
        self.ended = False
        self.pages_total = None    # from the firmware's "# log pages=" line
        self.b64 = False           # "# format=b64" line seen, and the layout matches
        self.fatal = None          # why the stream can't be decoded
        self.last_page = None
        self.started = None        # time the first row arrived
        self.shown = 0.0           # time progress was last shown
        self.tty = sys.stderr.isatty()

    def feed(self, line):
        """Handle one console line. Returns False once the log has ended."""
        self.raw.write(line if line.endswith("\n") else line + "\n")
        line = ANSI.sub("", line).strip()

        if not self.columns:
            # anything before the header is boot messages or CLI chatter,
            # except the log size the firmware prints just before it
            info = LOG_INFO.search(line)
            if info:
                self.pages_total = int(info.group(1))
                per_page = int(info.group(2))
                print(f"log on the board: {self.pages_total} pages, up to "
                      f"{self.pages_total * per_page} records", file=sys.stderr)
            clock = CLOCK_INFO.search(line)
            if clock:
                print(f"board at {clock.group(1)} MHz, SWO at {clock.group(3)} kHz", file=sys.stderr)
                if clock.group(2) != "0":
                    print("WARNING: the board couldn't raise its clock for the dump", file=sys.stderr)
            first = FIRST_PAGE_INFO.search(line)
            if first:
                self.first_page = int(first.group(1))
                self.check_first_page()
            fmt = FORMAT_INFO.search(line)
            if fmt:
                size = int(fmt.group(1))
                if size == RECORD.size:
                    self.b64 = True
                else:
                    self.fatal = (f"the firmware's record is {size} bytes, but this tool "
                                  f"decodes {RECORD.size}: TelemetryRecord_t changed, so "
                                  "update RECORD and csv_row() in capture_log.py")
                    print("ERROR: " + self.fatal, file=sys.stderr)
            at = line.find(HEADER_START)
            if at >= 0:
                header = line[at:]
                if self.b64 and header != COLUMNS:
                    self.b64 = False
                    self.fatal = ("the firmware's CSV columns differ from the ones this tool "
                                  "decodes: update COLUMNS and csv_row() in capture_log.py")
                    print("ERROR: " + self.fatal, file=sys.stderr)
                self.columns = header.count(",") + 1
                self.out.write(header + "\n")
                print(f"header found, {self.columns} columns", file=sys.stderr)
            return True

        if line.startswith(END_MARK):
            self.ended = True
            return False

        if line.startswith("@"):
            if not self.b64:
                self.bad.append(line)
                return True
            try:
                line = decode_b64(line)
            except ValueError:   # includes bad base64 (binascii.Error)
                self.bad.append(line)
                return True

        fields = line.split(",")
        if not line or not fields[0].isdigit():
            return True        # CLI chatter between rows
        if len(fields) != self.columns or not fields[1].isdigit():
            self.bad.append(line)
            return True

        t = int(fields[1])
        if self.last_t is not None:
            if t < self.last_t:
                self.backwards.append((self.last_t, t))
                self.span_ms += self.last_t - self.first_t
                self.first_t = t
            elif t - self.last_t > GAP_MS:
                self.gaps.append((self.last_t, t))
        if self.first_t is None:
            self.first_t = t
        self.last_t = t

        self.out.write(line + "\n")
        self.rows += 1
        self.last_page = int(fields[0])
        if self.rows % 1000 == 0:
            self.out.flush()
            self.raw.flush()
        self.progress()
        return True

    def progress(self, final=False):
        """Pages done out of the total, with rate and time left. Redrawn in
        place on a terminal, otherwise a line every 10%."""
        now = time.monotonic()
        if self.started is None:
            self.started = now
        if not final and now - self.shown < 0.5:
            return
        if self.pages_total and self.pages_total > self.first_page:
            done = min(self.last_page + 1, self.pages_total)
            frac = (done - self.first_page) / (self.pages_total - self.first_page)
            elapsed = now - self.started
            left = elapsed / frac * (1 - frac) if frac > 0 else 0.0
            bar = "#" * int(frac * 30)
            text = f"[{bar:<30}] {frac * 100:5.1f}%  page {done}/{self.pages_total}"
            if elapsed >= 2.0:     # too little to go on before that
                text += (f"  {self.rows / elapsed:6.0f} rec/s  "
                         f"{int(left) // 60}m{int(left) % 60:02d}s left")
            step = int(frac * 10)
        else:
            text = f"{self.rows} records (log size not reported)"
            step = self.rows // 5000
        if self.tty:
            print("\r" + text, end="\n" if final else "", file=sys.stderr, flush=True)
        elif final or step != getattr(self, "_step", None):
            print(text, file=sys.stderr)
            self._step = step
        self.shown = now

    def check_first_page(self):
        """Say what's being dumped, and whether the board took the request."""
        if self.pages_total is None:
            return
        if self.request_records is None:
            if self.first_page:
                print(f"NOTE: the board starts the dump at page {self.first_page}, "
                      "not the start of the log", file=sys.stderr)
            return
        want = expected_first_page(self.pages_total, self.request_records)
        if self.first_page == want:
            print(f"dumping pages {self.first_page}-{self.pages_total - 1}, the newest "
                  f"{self.pages_total - self.first_page} of {self.pages_total}", file=sys.stderr)
        else:
            print(f"WARNING: asked for the dump to start at page {want}, but the board "
                  f"starts at page {self.first_page}. Is the firmware on the board built "
                  "from the ELF used (--elf)? Rebuild and reflash Debug.", file=sys.stderr)

    def close(self):
        self.out.close()
        self.raw.close()

    def report(self, out_path):
        if self.rows:
            self.progress(final=True)
        print(file=sys.stderr)
        if not self.columns:
            print("No CSV header received. Is LOG_DUMP_AT_BOOT 1 flashed, and the board "
                  f"powered? Everything received is in {out_path.with_suffix('.raw.txt')}. "
                  f"If it's garbled, the board isn't at {DEFAULT_HCLK_MHZ} MHz: dump firmware "
                  "older than the clock change (or one whose clock change failed) needs "
                  "--hclk-mhz 8", file=sys.stderr)
            return 1
        if self.fatal:
            print("ERROR: " + self.fatal, file=sys.stderr)
        span = (self.span_ms + self.last_t - self.first_t) / 1000 if self.rows else 0.0
        print(f"{self.rows} records, {span:.1f} s of log, saved to {out_path}", file=sys.stderr)
        print("end of log received" if self.ended else
              "WARNING: stream stopped before the end-of-log marker - the CSV may be incomplete",
              file=sys.stderr)
        short = (self.pages_total is not None and self.rows
                 and self.last_page + 1 < self.pages_total)
        if short:
            print(f"WARNING: the log has {self.pages_total} pages but the last row received "
                  f"is from page {self.last_page + 1}", file=sys.stderr)
        if self.gaps:
            worst = max(b - a for a, b in self.gaps)
            print(f"WARNING: {len(self.gaps)} gaps longer than {GAP_MS} ms between records "
                  f"(worst {worst} ms), first at t = {self.gaps[0][0]} ms", file=sys.stderr)
        if self.backwards:
            print(f"NOTE: timestamp went back to the start {len(self.backwards)} time(s) - "
                  "a power-up or reboot, each one adds to the log", file=sys.stderr)
        if self.bad:
            print(f"WARNING: {len(self.bad)} malformed rows skipped (kept in the .raw.txt), "
                  f"first: {self.bad[0][:120]!r}", file=sys.stderr)
        return 0 if self.ended and not self.bad and not short and not self.fatal else 2


def main():
    ap = argparse.ArgumentParser(prog="capture-log", description=__doc__.strip().split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-o", "--output", type=Path,
                    default=Path(datetime.now().strftime("log_%Y%m%d_%H%M%S.csv")))
    ap.add_argument("--from-file", type=Path,
                    help="parse saved console text instead of capturing from the board")
    ap.add_argument("--cli", help="path to STM32_Programmer_CLI (found automatically)")
    ap.add_argument("--hclk-mhz", type=int, default=DEFAULT_HCLK_MHZ,
                    help=f"the board's core clock during the dump, MHz (default {DEFAULT_HCLK_MHZ}, "
                         "as DUMP_HCLK_MHZ in main.c; 8 for older dump firmware)")
    ap.add_argument("--last-minutes", type=float,
                    help="dump only the newest this-many minutes of log (default: all of it)")
    ap.add_argument("--elf", type=Path, default=DEFAULT_ELF,
                    help="the flashed Debug build, to find dump_request in for --last-minutes "
                         f"(default {DEFAULT_ELF.relative_to(DEFAULT_ELF.parents[1])})")
    ap.add_argument("--idle-timeout", type=float, default=15.0,
                    help="stop after this many seconds with no data (default 15)")
    args = ap.parse_args()

    request = None
    if args.last_minutes is not None:
        if args.last_minutes <= 0:
            sys.exit("--last-minutes must be more than 0")
        if args.from_file:
            sys.exit("--last-minutes asks the board for less of the log, so it can't be "
                     "used with --from-file")
        if not args.elf.exists():
            sys.exit(f"{args.elf} not found - build Debug, or pass the flashed build with --elf")
        try:
            address = symbol_address(args.elf, REQUEST_SYMBOL)
        except LookupError as e:
            sys.exit(str(e))
        records = records_for_minutes(args.last_minutes)
        request = (address, records)
        built = datetime.fromtimestamp(args.elf.stat().st_mtime).strftime("%Y-%m-%d %H:%M")
        print(f"asking for the newest {args.last_minutes:g} min ({records} records), "
              f"using {args.elf.name} built {built}", file=sys.stderr)

    if args.from_file:
        lines = lines_from_file(args.from_file)
    else:
        cli = args.cli or find_cli()
        if not cli or not Path(cli).exists():
            sys.exit("STM32_Programmer_CLI not found - pass its path with --cli")
        if request:
            # the firmware reads it once SWO is on, so it must go first
            write_request(cli, *request)
        lines = lines_from_cli(cli, args.hclk_mhz,
                               args.output.with_suffix(".swv.log"), args.idle_timeout)

    col = Collector(args.output, request[1] if request else None)
    try:
        for line in lines:
            if not col.feed(line):
                break
    except KeyboardInterrupt:
        print("\nstopped by Ctrl-C", file=sys.stderr)
    finally:
        if hasattr(lines, "close"):
            lines.close()     # stops the CLI
        col.close()
    sys.exit(col.report(args.output))


if __name__ == "__main__":
    main()
