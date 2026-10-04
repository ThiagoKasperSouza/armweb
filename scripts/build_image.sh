#!/usr/bin/env bash
# Build the armweb image. Writes to build.log so it can be polled.
cd /home/thiag/armweb || exit 1
exec >> /home/thiag/armweb/build.log 2>&1
echo "=== build started $(date -Is) ==="
docker build -t armweb:latest .
rc=$?
echo "=== build finished rc=$rc at $(date -Is) ==="
exit $rc