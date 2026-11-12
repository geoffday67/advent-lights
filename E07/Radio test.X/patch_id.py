#!/usr/bin/env python3
"""
patch_id.py - Make a per-tree copy of the receiver firmware.

Writes RETLW <id> (0x34nn) into program-flash word TREE_ID_ADDR of the hex file
that MPLAB builds, and saves the result as tree_NN.hex. The receiver firmware
reads that word at start-up (rx_main.c, tree_id_read()).

Usage:
    python3 patch_id.py ID [-i INPUT.hex] [-o OUTPUT.hex]

Defaults:
    INPUT   dist/RX/production/Radio_test.X.production.hex (next to this script)
    OUTPUT  tree_hex/tree_NN.hex (next to this script)
"""
import argparse
import os
import sys

TREE_ID_ADDR = 0x1FFF           # Word address; must match rx_main.c
RETLW = 0x34                    # High byte of a RETLW instruction (14-bit 0x34kk)
HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_IN = os.path.join(HERE, "dist", "RX", "production", "Radio_test.X.production.hex")


def checksum(data):
    return (-sum(data)) & 0xFF


def parse(line, lineno):
    line = line.strip()
    if not line.startswith(":"):
        sys.exit(f"Line {lineno}: not an Intel HEX record")
    raw = bytes.fromhex(line[1:])
    if len(raw) < 5 or len(raw) != raw[0] + 5:
        sys.exit(f"Line {lineno}: bad record length")
    if sum(raw) & 0xFF:
        sys.exit(f"Line {lineno}: bad checksum")
    return raw[0], (raw[1] << 8) | raw[2], raw[3], bytearray(raw[4:-1])


def record(addr, rtype, data):
    body = bytes([len(data), addr >> 8, addr & 0xFF, rtype]) + bytes(data)
    return ":" + (body + bytes([checksum(body)])).hex().upper()


def main():
    ap = argparse.ArgumentParser(description="Patch the tree ID into the receiver hex file.")
    ap.add_argument("id", type=int, help="tree ID, 1-254")
    ap.add_argument("-i", "--input", default=DEFAULT_IN, help="hex file built by MPLAB")
    ap.add_argument("-o", "--output", help="output hex file (default tree_hex/tree_NN.hex)")
    args = ap.parse_args()

    if not 1 <= args.id <= 254:
        sys.exit("ID must be 1-254 (0xFF means 'not set')")
    out = args.output or os.path.join(HERE, "tree_hex", f"tree_{args.id:02d}.hex")

    with open(args.input) as f:
        lines = [l for l in f.read().splitlines() if l.strip()]

    byte_addr = TREE_ID_ADDR * 2            # Hex files hold 2 bytes per word, low byte first
    new_word = bytes([args.id, RETLW])
    upper = 0                               # Extended linear address (record type 04)
    patched = False
    out_lines = []

    for n, line in enumerate(lines, 1):
        count, addr, rtype, data = parse(line, n)
        if rtype == 0x04:
            upper = (data[0] << 8) | data[1]
        elif rtype == 0x01:                 # EOF: insert a record if the word wasn't in the file
            if not patched:
                if upper != 0:
                    out_lines.append(record(0, 0x04, b"\x00\x00"))
                out_lines.append(record(byte_addr, 0x00, new_word))
                patched = True
        elif rtype == 0x00 and upper == 0:
            start = addr
            end = addr + count
            if start <= byte_addr and byte_addr + 1 < end:
                i = byte_addr - start
                old = data[i] | (data[i + 1] << 8)
                if (old >> 8) != RETLW and old != 0x3FFF:
                    sys.exit(f"Word 0x{TREE_ID_ADDR:04X} holds 0x{old:04X}, not the ID placeholder; "
                             "check TREE_ID_ADDR matches rx_main.c")
                data[i:i + 2] = new_word
                line = record(addr, 0x00, data)
                patched = True
            elif start <= byte_addr < end or start <= byte_addr + 1 < end:
                sys.exit("ID word is split across records; not handled")
        out_lines.append(line)

    if not patched:
        sys.exit("No EOF record found in input")

    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    with open(out, "w") as f:
        f.write("\n".join(out_lines) + "\n")
    print(f"Tree {args.id}: wrote {out}")


if __name__ == "__main__":
    main()
