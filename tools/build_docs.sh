#!/bin/sh
# Build the three PDFs and copy them to app/assets/docs (shown by the app's Documentation button).
set -e
cd "$(dirname "$0")/.."
for d in physics:bbp_physics_model control:bbp_control_library vision:bbp_vision_library; do
  dir=${d%%:*}; name=${d#*:}
  (cd "docs/$dir" && latexmk -pdf -interaction=nonstopmode "$name.tex" >/dev/null 2>&1 || true)
  test -f "docs/$dir/build/$name.pdf" || { echo "failed: $name (see docs/$dir/build/$name.log)"; exit 1; }
  grep -q "undefined" "docs/$dir/build/$name.log" && echo "warning: undefined references in $name"
  cp "docs/$dir/build/$name.pdf" app/assets/docs/
  echo "built $name.pdf"
done
