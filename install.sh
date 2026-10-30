#!/bin/sh
# Links the Anycubic ACE Pro unit for AFC into Klipper's extras.
# POSIX sh so it runs on K1-series BusyBox as well as on a Pi.
#
#   ./install.sh                 link extras (auto-detects Klipper)
#   ./install.sh -k <klipper>    use this Klipper/Kalico checkout
#   ./install.sh -t              also copy templates/AFC_ACEPRO_1.cfg into your AFC config dir
#   ./install.sh -u              remove the links
#
# Requires AFC (AFC-Klipper-Add-On) to be installed already.
set -e

REPO_DIR=$(cd "$(dirname "$0")" && pwd)
KLIPPER_DIR=""
COPY_TEMPLATE=0
UNINSTALL=0
FILES="AFC_ACEPRO.py AFC_ACEPRO_lane.py AFC_ACEPRO_protocol.py"

while getopts "k:tuh" opt; do
  case "$opt" in
    k) KLIPPER_DIR="$OPTARG" ;;
    t) COPY_TEMPLATE=1 ;;
    u) UNINSTALL=1 ;;
    *) sed -n '2,10p' "$0"; exit 0 ;;
  esac
done

if [ -z "$KLIPPER_DIR" ]; then
  for d in /usr/data/klipper "$HOME/klipper" "$HOME/kalico"; do
    if [ -f "$d/klippy/klippy.py" ]; then KLIPPER_DIR="$d"; break; fi
  done
fi
if [ -z "$KLIPPER_DIR" ] || [ ! -d "$KLIPPER_DIR/klippy/extras" ]; then
  echo "Could not find Klipper. Pass it with -k /path/to/klipper" >&2
  exit 1
fi
EXTRAS="$KLIPPER_DIR/klippy/extras"

if [ "$UNINSTALL" -eq 1 ]; then
  for f in $FILES; do
    if [ -L "$EXTRAS/$f" ]; then rm -f "$EXTRAS/$f"; echo "removed $EXTRAS/$f"; fi
  done
  echo "Remove your [AFC_ACEPRO] / [AFC_ACEPRO_lane] sections, then restart Klipper."
  exit 0
fi

if [ ! -e "$EXTRAS/AFC.py" ]; then
  echo "AFC is not installed in $EXTRAS (AFC.py missing). Install AFC-Klipper-Add-On first." >&2
  exit 1
fi

for f in $FILES; do
  ln -sf "$REPO_DIR/extras/$f" "$EXTRAS/$f"
  echo "linked $EXTRAS/$f"
done

# Keep the links out of Klipper's dirty-tree check
EXCLUDE="$KLIPPER_DIR/.git/info/exclude"
if [ -f "$EXCLUDE" ]; then
  for f in $FILES; do
    grep -qxF "klippy/extras/$f" "$EXCLUDE" || echo "klippy/extras/$f" >> "$EXCLUDE"
  done
fi

if [ "$COPY_TEMPLATE" -eq 1 ]; then
  for d in /usr/data/printer_data/config/AFC "$HOME/printer_data/config/AFC"; do
    if [ -d "$d" ]; then
      if [ -e "$d/AFC_ACEPRO_1.cfg" ]; then
        echo "$d/AFC_ACEPRO_1.cfg already exists, not overwriting"
      else
        cp "$REPO_DIR/templates/AFC_ACEPRO_1.cfg" "$d/AFC_ACEPRO_1.cfg"
        echo "copied template to $d/AFC_ACEPRO_1.cfg (edit serial and bowden_length first!)"
      fi
      break
    fi
  done
fi

echo
echo "Done. Edit your ACE config (see docs/ACEPRO.md), then restart the Klipper service."
