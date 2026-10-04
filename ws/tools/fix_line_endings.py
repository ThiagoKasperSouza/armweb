"""Normalise CRLF -> LF in a file, without relying on shell escape handling.

`tr -d "\r"` and `sed 's/\r$//'` are both shell-quoting hazards: depending on
how the command reaches the shell, \\r becomes the literal letter 'r' and
every 'r' in the file is deleted. Doing it in Python sidesteps the problem
entirely because the escape is unambiguous.

Usable directly:

    python3 fix_line_endings.py FILE_OR_DIR [FILE_OR_DIR ...]
"""
import os
import sys


def normalise(path):
    """Rewrite `path` with CRLF and lone CR turned into LF.

    Returns True when the file changed.
    """
    with open(path, "rb") as fh:
        data = fh.read()
    out = data.replace(b"\r\n", b"\n").replace(b"\r", b"\n")
    if out != data:
        with open(path, "wb") as fh:
            fh.write(out)
        return True
    return False


def main(argv):
    changed = []
    for target in argv:
        if os.path.isfile(target):
            if normalise(target):
                changed.append(target)
        elif os.path.isdir(target):
            for root, dirs, files in os.walk(target):
                dirs[:] = [d for d in dirs
                           if d not in ("install", "build", "log",
                                        "node_modules", ".git")]
                for name in files:
                    p = os.path.join(root, name)
                    if normalise(p):
                        changed.append(p)
    print(f"normalised {len(changed)} file(s)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))