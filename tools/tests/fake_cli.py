#!/usr/bin/env python3
"""Stands in for STM32_Programmer_CLI: prints some chatter, then a dump in
odd-sized chunks, then hangs like the real tool does. The dump is described
by environment variables so each test can shape it."""
import base64
import binascii
import os
import struct
import sys
import time

header = os.environ["FAKE_HEADER"]
n = int(os.environ.get("FAKE_RECORDS", "100"))
gap_at = int(os.environ.get("FAKE_GAP_AT", "-1"))
reboot_at = int(os.environ.get("FAKE_REBOOT_AT", "-1"))
bad_row = os.environ.get("FAKE_BAD_ROW") == "1"
end = os.environ.get("FAKE_END", "1") == "1"
b64 = os.environ.get("FAKE_FORMAT", "csv") == "b64"   # rows as telemetry_print_b64 sends them
record_bytes = int(os.environ.get("FAKE_RECORD_BYTES", "88"))
corrupt_at = int(os.environ.get("FAKE_CORRUPT_AT", "-1"))
ignore_request = os.environ.get("FAKE_IGNORE_REQUEST") == "1"   # firmware without dump_request

# a -w32 run writes dump_request and exits, like the real CLI; the board's
# RAM is the FAKE_W32_LOG file. -w32 together with -swv starts no SWV, also
# like the real CLI
if "-w32" in sys.argv:
    i = sys.argv.index("-w32")
    address, magic, records = (int(v, 16) for v in sys.argv[i + 1:i + 4])
    with open(os.environ["FAKE_W32_LOG"], "w") as f:
        f.write(f"{address:#x} {magic:#x} {records}\n")
    sys.stdout.write("Write 32-bit data\r\n32-bit data download complete\r\n")
    sys.exit(0)
first_page = 0
magic = None
w32_log = os.environ.get("FAKE_W32_LOG")
if w32_log and os.path.exists(w32_log):
    _, magic, records = open(w32_log).read().split()
    magic, records = int(magic, 16), int(records)

w = sys.stdout.buffer
w.write(b"\x1b[32m  STM32CubeProgrammer\x1b[0m\r\nST-LINK SN  : 123\r\nSWV reception started\r\n")
w.flush()
cols = header.count(",") + 1
pages = int(os.environ.get("FAKE_PAGES", str((n + 22) // 23)))
text = "Hello World\r\n\r\n"
if os.environ.get("FAKE_INFO", "1") == "1":
    text += f"# log pages={pages} records_per_page=23\r\n"
if magic == 0xD0A9D0A9 and not ignore_request:
    pages_wanted = records // 23 + 2
    first_page = pages - pages_wanted if pages_wanted < pages else 0
if b64:
    text += "# clock hclk_mhz=128 clock_up=0 swo_khz=2000\r\n"
    text += f"# format=b64 record_bytes={record_bytes}\r\n"
    text += f"# dump first_page={first_page}\r\n"
text += header + "\r\n"
t = 1000
for i in range(n):
    if i // 23 < first_page:
        t += 20
        continue
    if i == gap_at:
        t += 200
    if i == reboot_at:
        t = 500
    if b64:
        # page, then a record holding only the timestamp
        data = struct.pack("<II", i // 23, t) + bytes(record_bytes - 4)
        line = "@" + base64.b64encode(data).decode() + "*%04X" % binascii.crc_hqx(data, 0xFFFF)
        if i == corrupt_at:
            line = line[:10] + ("B" if line[10] == "A" else "A") + line[11:]
        text += line + "\r\n"
    else:
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
