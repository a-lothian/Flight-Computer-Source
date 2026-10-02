# flight-log-tools

PC-side tools for the flight computer, managed with [uv](https://docs.astral.sh/uv/).
Run everything from this `tools/` folder.

## capture-log

Collects the NAND log as the firmware streams it out over SWO, and saves it as CSV.

The firmware sends each record as its raw bytes in base64 with a CRC
(`telemetry_print_b64` in `telemetry.c`), and this script turns them back into
the CSV row the firmware's `printf` would have produced. Formatting floats with
`printf` at 8 MHz took most of the dump time. A line that fails its CRC is
counted as malformed and kept in the `.raw.txt`. If `TelemetryRecord_t` changes,
update `RECORD`, `COLUMNS` and `csv_row()` in `capture_log.py` to match (the
tests check them against `telemetry.h` and `telemetry.c`). Plain CSV rows from
older firmware are still accepted.

The dump firmware raises its clock from 8 to 128 MHz before it starts sending
(`Dump_Clock_Up` in `main.c`), and the programmer has to be told that clock to
time SWO, so `--hclk-mhz` defaults to 128. Dump firmware from before that change
needs `--hclk-mhz 8`; a wrong value shows up as garbage or no header at all.
The script prints the clock and SWO rate the board reports.

1. In `Core/Inc/config.h` set `LOG_DUMP_AT_BOOT 1`, build Debug and flash it.
   End the CubeIDE debug session afterwards: only one program can use the ST-Link.
2. With the board powered and the ST-Link plugged in:

   ```
   uv run capture-log -o flight1.csv
   ```

   The firmware waits for SWO to be switched on, reports the log's size, dumps the
   log, then stops, so the log on the flash is kept. The script shows progress and
   time left, stops at the end-of-log marker and prints a summary (record count,
   gaps, reboots, malformed rows, and whether every page arrived). Everything
   received is also saved, unparsed, to `flight1.raw.txt`.
3. Set `LOG_DUMP_AT_BOOT` back to 0. Release builds refuse to build with it set.

To dump only the end of the log, which is much quicker when the board sat on the
pad for a long time:

```
uv run capture-log --last-minutes 10 -o flight1.csv
```

The script writes the request into the board's RAM over the debugger (the
`dump_request` variable in `main.c`, found in the Debug build's ELF) just before
it switches SWO on, and the board starts the dump that many records (50 a second)
before the end, rounded out to whole pages. It's a record count, so time the board
was off doesn't count, and records logged after landing do. Flash the same Debug
build the script reads (`Debug/flight-computer-testing.elf`, or pass `--elf`): the
board reports where it started, and the script warns if that isn't what it asked for.

Every power-up adds to the log on the chip, so a reboot mid-flight continues the
log instead of writing over it. Bench sessions add to it too: to start a fresh log,
dump it first, then flash once with `LOG_ERASE_AT_BOOT 1` (it erases the log and
stops), and set it back to 0.

Text copied from CubeIDE's SWV console, or the `.swv.log` the programmer writes,
can be turned into the same CSV with `uv run capture-log --from-file saved.txt`.

It uses STM32CubeProgrammer's command line tool from the STM32CubeIDE install
(found automatically on macOS, or pass `--cli PATH`). macOS and Linux only: it
runs the tool on a pseudo-terminal, which Windows doesn't have.

## Tests

```
uv run pytest
```

These run `capture-log` against a stand-in for the programmer tool, so no board is needed.
