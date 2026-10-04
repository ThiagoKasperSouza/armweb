#!/usr/bin/env bash
# Mirror the WSL project (the source of truth) back to the Windows editor dir.
# This is the REVERSE of sync.sh: edits are now made in WSL via the \\wsl
# paths, and the Windows copy must be refreshed so the editor sees them.
#
# CRLF: the Windows side is fine either way, but strip on the way in so the
# two trees stay byte-comparable apart from line endings.
# Use `sed 's/\r$//'`, NEVER `tr -d "\r"` (that deletes every letter 'r').
set +e

SRC=/home/thiag/armweb
DST=/mnt/c/Users/thiag/AppData/Local/Cline

lfnorm() {
  sed 's/\r$//' < "$1" > "$1.lf" && mv "$1.lf" "$1"
}

mkdir -p "$DST"

for f in Makefile Dockerfile docker-compose.yml README.md; do
  if [ -f "$SRC/$f" ]; then
    cp -f "$SRC/$f" "$DST/$f" 2>/dev/null
    echo "mirrored $f"
  fi
done

for d in ws/tools ws/assets scripts web; do
  [ -d "$SRC/$d" ] || continue
  mkdir -p "$DST/$d"
  # Only mirror tracked source, never build output or caches.
  (cd "$SRC" && find "$d" -type f \
      ! -path '*/__pycache__/*' ! -name '*.pyc' ! -name '*.pyo' \
      ! -name '*.log' -print0) 2>/dev/null |
  while IFS= read -r -d '' f; do
    mkdir -p "$DST/$(dirname "$f")"
    cp -f "$SRC/$f" "$DST/$f" 2>/dev/null && echo "mirrored $f"
  done
done

echo "mirror complete: $SRC -> $DST"