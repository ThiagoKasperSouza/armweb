#!/usr/bin/env bash
# Remove colcon build artifacts. They are created as root inside the
# container, so removal has to happen as root too.
cd "$(dirname "$0")/.."

docker compose stop sim >/dev/null 2>&1 || true

if docker image inspect armweb:latest >/dev/null 2>&1; then
  docker run --rm -v "$PWD/ws:/ws" armweb:latest bash -lc \
    'rm -rf /ws/install /ws/build /ws/log && echo "workspace build artifacts removed"'
else
  rm -rf ws/install ws/build ws/log
  echo "workspace build artifacts removed (no image needed)"
fi

# Make the tree user-writable again for the host user
docker run --rm -v "$PWD:/proj" armweb:latest bash -lc \
  'chown -R 1000:1000 /proj 2>/dev/null || true' 2>/dev/null || true

echo "done"