# AFC Anycubic ACE Pro

[AFC](https://github.com/AFCProject/AFC-Klipper-Add-On) unit type for the
first-generation **Anycubic ACE Pro**. It drives the ACE over its own USB link
from mainline Klipper / Kalico, with no Anycubic software.

Built for a **K1 Max with the Creality K1-series CFS upgrade kit** (as the extruder
and cutter) **running [Simple AF](https://github.com/pellcorp/creality)**, but nothing
here is K1-specific beyond the example config.

> **Status: experimental, not yet tested on a printer.** Everything below runs
> against a simulated ACE on Klipper's real event loop; follow the staged test plan
> in [docs/ACEPRO.md](docs/ACEPRO.md) and stay at the printer for the first sessions.

## What you get

- `[AFC_ACEPRO <name>]` — one ACE: USB link, status polling (which is also the
  ACE's keepalive), slot presence → AFC lanes, dryer, feed assist
- `[AFC_ACEPRO_lane <name>]` — one slot (1–4): full tool load, verified cut and unload
- Lanes show up in Mainsail, Fluidd and GrumpyScreen's AFC panel like any other AFC lane
- Anycubic RFID tags set the lane's colour and material on insert
- Survives the ACE's habit of dropping off USB: reopens the by-id path and re-arms
  feed assist
- A failed load pulls its filament back so it doesn't block the splitter
- `AFC_ACEPRO_STATUS`, `AFC_ACEPRO_CALIBRATE`, `AFC_ACEPRO_RAW` and a trace file for
  bring-up
- Serial I/O never blocks Klipper, and the keepalive never waits on gcode

## Install

AFC must already be installed. On Simple AF, follow its FAQ:
[How to install AFC on Simple AF](https://pellcorp.github.io/creality-wiki/faq/#how-to-install-afc-on-simple-af).

```sh
cd /usr/data               # ~ on a Pi
git clone https://github.com/mikeinredding/afc-anycubic-ace.git
cd afc-anycubic-ace
./install.sh -t            # links extras into Klipper, copies the example config
```

Then edit `AFC/AFC_ACEPRO_1.cfg` and `AFC/AFC_Hardware.cfg` as described in
[docs/ACEPRO.md](docs/ACEPRO.md) and restart the Klipper service
(`supervisorctl restart klipper` on Simple AF). `./install.sh -u` removes the links.

### Updates through Moonraker

```ini
[update_manager afc-anycubic-ace]
type: git_repo
path: /usr/data/afc-anycubic-ace        # ~/afc-anycubic-ace on a Pi
origin: https://github.com/mikeinredding/afc-anycubic-ace.git
primary_branch: main
managed_services: klipper
```

## Layout

The folders mirror AFC's own (`extras/`, `templates/`, `docs/`, `tests/`), the same
way [afc-creality-cfs](https://github.com/mikeinredding/afc-creality-cfs) does.
Names (`AFC_ACEPRO*`) don't collide with AFC's own `AFC_ACE` work on the ACE_series
branch or with ACEPROK1Max's `[ace]`, but only one of them may own the ACE's port.

## Tests

```sh
./run-tests.sh                              # clones AFC main into .deps/ for its test mocks
KLIPPER_PATH=~/klipper ./run-tests.sh       # also runs the real-reactor scenarios
```

CI runs AFC `main`, `DEV` and the ACE_series branch against Klipper and Kalico.

## Credits

Written by Jean-Claude Van Code (Claude, Anthropic's AI) together with
[@mikeinredding](https://github.com/mikeinredding).

Protocol knowledge from printers-for-people/ACEResearch, utkabobr/DuckACE,
BlackFrogKok/BunnyACE, szkrisz/ACEPROSV08, swilsonnc/ACEPROK1Max, Kobra-S1/ACEPRO and
the AFC ACE_series branch. Thanks to [Pellcorp](https://github.com/pellcorp) for
[Simple AF](https://github.com/pellcorp/creality).

GPL-3.0, same as AFC and Klipper.
