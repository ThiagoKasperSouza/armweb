#!/usr/bin/env python3
"""Print a file with line numbers and visible indentation.

Useful when a file has mixed tabs/spaces and Python reports
"unindent does not match any outer indentation level" -- the offending line
is not obvious from the traceback alone.
"""
import sys


def show(path, lo=1, hi=10 ** 6):
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        lines = fh.read().split("\n")
    for i, line in enumerate(lines, 1):
        if i < lo or i > hi:
            continue
        stripped = line.lstrip(" \t")
        indent = line[: len(line) - len(stripped)]
        width = sum(4 if c == "\t" else 1 for c in indent)
        flags = []
        if "\t" in indent:
            flags.append("TAB")
        if indent.startswith(" ") and " " * width != indent:
            flags.append("MIXED")
        tag = (" [" + ",".join(flags) + "]") if flags else ""
        print(f"{i:4} w={width:3} {indent!r}{stripped[:64]}{tag}")


if __name__ == "__main__":
    p = sys.argv[1]
    lo = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    hi = int(sys.argv[3]) if len(sys.argv) > 3 else 10 ** 6
    show(p, lo, hi)