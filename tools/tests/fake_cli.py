#!/usr/bin/env python3
"""Stands in for STM32_Programmer_CLI: prints some chatter, then a dump in
odd-sized chunks, then hangs like the real tool does. The dump is described
by environment variables so each test can shape it."""
import os
import sys
import time

header = os.environ["FAKE_HEADER"]
n = int(os.environ.get("FAKE_RECORDS", "100"))
gap_at = int(os.environ.get("FAKE_GAP_AT", "-1"))
reboot_at = int(os.environ.get("FAKE_REBOOT_AT", "-1"))
bad_row = os.environ.get("FAKE_BAD_ROW") == "1"
end = os.environ.get("FAKE_END", "1") == "1"

w = sys.stdout.buffer
w.write(b"\x1b[32m  STM32CubeProgrammer\x1b[0m\r\nST-LINK SN  : 123\r\nSWV reception started\r\n")
w.flush()
cols = header.count(",") + 1
pages = int(os.environ.get("FAKE_PAGES", str((n + 22) // 23)))
text = "Hello World\r\n\r\n"
if os.environ.get("FAKE_INFO", "1") == "1":
    text += f"# log pages={pages} records_per_page=23\r\n"
text += header + "\r\n"
t = 1000
for i in range(n):
    if i == gap_at:
        t += 200
    if i == reboot_at:
        t = 500
    row = [str(i // 23), str(t)] + ["0"] * (cols - 2)
    text += ",".join(row) + "\r\n"
    t += 20
if bad_row:
    text += "12,3,bad row\r\n"
if end:
    text += "# end of log\r\n"
data = text.encode()
for k in range(0, len(data), 777):
    w.write(data[k:k + 777])
    w.flush()
    time.sleep(0.001)
time.sleep(60)
