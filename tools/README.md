# flight-log-tools

PC-side tools for the flight computer, managed with [uv](https://docs.astral.sh/uv/).
Run everything from this `tools/` folder.

## capture-log

Collects the NAND log as the firmware streams it out over SWO, and saves it as CSV.

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
