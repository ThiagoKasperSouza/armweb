#!/usr/bin/env bash
# Serve the viewer locally and confirm all assets are reachable.
cd /home/thiag/armweb || exit 1
python3 -m http.server 8099 >/dev/null 2>&1 &
SRV=$!
sleep 2
for p in web/viewer.html \
         data/gltf/fr3_rest.glb data/gltf/fr3_animated.glb \
         data/gltf/arm_rest.glb data/gltf/arm_posed.glb data/gltf/arm_animated.glb; do
  [ -f "$p" ] || { echo "  $p -> MISSING"; continue; }
  code=$(curl -s -o /dev/null -w '%{http_code}' "http://localhost:8099/$p")
  size=$(curl -s "http://localhost:8099/$p" | wc -c)
  echo "  $p -> HTTP $code, ${size} bytes"
done
kill $SRV 2>/dev/null
echo done