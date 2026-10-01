"""
Collect the flight computer's NAND log as it streams out over SWO, and save
it as a CSV file.

The firmware side: build Debug with LOG_DUMP_AT_BOOT 1 (config.h) and flash
it. At boot it waits for a debugger to turn SWO on, prints the log as CSV
(a header line, one row per record, then "# end of log"), and stops.

This script runs STM32CubeProgrammer's command line tool (bundled with
STM32CubeIDE), which connects over the ST-Link without halting or resetting
the board and turns SWO on. That starts the dump. CubeIDE must not have a
debug session open, since only one program can use the ST-Link at a time.

    cd tools
    uv run capture-log                       # -> log_<date>_<time>.csv
    uv run capture-log -o flight1.csv
    uv run capture-log --from-file swv.txt   # text saved from the SWV console

Everything received is also written, unparsed, to <output>.raw.txt, so
nothing is lost if a line can't be parsed.
"""

import argparse
import glob
import os
import re
import select
import shutil
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

HEADER_START = "page,timestamp_ms,"
LOG_INFO = re.compile(r"# log pages=(\d+) records_per_page=(\d+)")
END_MARK = "# end of log"
LOOP_MS = 20                 # MIN_CONTROL_LOOP_PERIOD_MS in config.h
GAP_MS = int(LOOP_MS * 1.5)  # a step longer than this means lost records
ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")

CLI_GLOB = ("/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/"
            "com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.*/tools/bin/STM32_Programmer_CLI")


def find_cli():
    found = sorted(glob.glob(CLI_GLOB)) or [shutil.which("STM32_Programmer_CLI")]
    return found[-1]


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

    def __init__(self, out_path):
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
            at = line.find(HEADER_START)
            if at >= 0:
                header = line[at:]
                self.columns = header.count(",") + 1
                self.out.write(header + "\n")
                print(f"header found, {self.columns} columns", file=sys.stderr)
            return True

        if line.startswith(END_MARK):
            self.ended = True
            return False

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
        if self.pages_total:
            done = min(self.last_page + 1, self.pages_total)
            frac = done / self.pages_total
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

    def close(self):
        self.out.close()
        self.raw.close()

    def report(self, out_path):
        if self.rows:
            self.progress(final=True)
        print(file=sys.stderr)
        if not self.columns:
            print("No CSV header received. Is LOG_DUMP_AT_BOOT 1 flashed, and the board "
                  f"powered? Everything received is in {out_path.with_suffix('.raw.txt')}",
                  file=sys.stderr)
            return 1
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
        return 0 if self.ended and not self.bad and not short else 2


def main():
    ap = argparse.ArgumentParser(prog="capture-log", description=__doc__.strip().split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-o", "--output", type=Path,
                    default=Path(datetime.now().strftime("log_%Y%m%d_%H%M%S.csv")))
    ap.add_argument("--from-file", type=Path,
                    help="parse saved console text instead of capturing from the board")
    ap.add_argument("--cli", help="path to STM32_Programmer_CLI (found automatically)")
    ap.add_argument("--hclk-mhz", type=int, default=8, help="core clock, MHz (default 8)")
    ap.add_argument("--idle-timeout", type=float, default=15.0,
                    help="stop after this many seconds with no data (default 15)")
    args = ap.parse_args()

    if args.from_file:
        lines = lines_from_file(args.from_file)
    else:
        cli = args.cli or find_cli()
        if not cli or not Path(cli).exists():
            sys.exit("STM32_Programmer_CLI not found - pass its path with --cli")
        lines = lines_from_cli(cli, args.hclk_mhz,
                               args.output.with_suffix(".swv.log"), args.idle_timeout)

    col = Collector(args.output)
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
