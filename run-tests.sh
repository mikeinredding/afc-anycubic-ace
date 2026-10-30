#!/usr/bin/env bash
# Runs the ACE Pro tests inside an AFC checkout (they use AFC's test mocks).
#
#   ./run-tests.sh                 unit tests (clones AFC into .deps/ if needed)
#   KLIPPER_PATH=~/klipper ./run-tests.sh   also run the real-reactor scenarios
#
#   AFC_PATH=/path/to/AFC-Klipper-Add-On    use an existing AFC checkout
#   AFC_REPO / AFC_BRANCH                   what to clone (default: AFCProject main,
#                                           what Simple AF installs)
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
AFC_REPO="${AFC_REPO:-https://github.com/AFCProject/AFC-Klipper-Add-On.git}"
AFC_BRANCH="${AFC_BRANCH:-main}"
AFC_PATH="${AFC_PATH:-$ROOT/.deps/AFC-Klipper-Add-On-$AFC_BRANCH}"

if [ ! -f "$AFC_PATH/extras/AFC.py" ]; then
  echo "==> cloning AFC ($AFC_BRANCH) into $AFC_PATH"
  git clone -q --depth 1 -b "$AFC_BRANCH" "$AFC_REPO" "$AFC_PATH"
fi

# Overlay this repo's files into the AFC checkout as symlinks
for f in "$ROOT"/extras/*.py; do ln -sf "$f" "$AFC_PATH/extras/$(basename "$f")"; done
for f in "$ROOT"/tests/*.py; do ln -sf "$f" "$AFC_PATH/tests/$(basename "$f")"; done
mkdir -p "$AFC_PATH/templates"
for f in "$ROOT"/templates/AFC_ACEPRO*.cfg; do
  [ -e "$f" ] && ln -sf "$f" "$AFC_PATH/templates/$(basename "$f")"
done

cd "$AFC_PATH"
tests=()
for f in "$ROOT"/tests/test_AFC_ACEPRO_*.py; do tests+=("tests/$(basename "$f")"); done
python3 -m pytest -q -p no:cacheprovider "${tests[@]}" "$@"
