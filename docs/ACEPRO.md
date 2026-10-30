# Anycubic ACE Pro support (in testing)

An AFC unit type for the first-generation **Anycubic ACE Pro**, driven from
mainline Klipper / Kalico with no Anycubic software. Written for a **K1 Max with
the Creality K1-series CFS upgrade kit** (the kit's extruder, sensor and cutter,
not the CFS box) running Simple AF. Apart from the example config and macros
nothing here is K1-specific. The ACE 2 Pro speaks a different protocol (RS-485)
and is not supported.

## How it works

| File | Role |
|---|---|
| `extras/AFC_ACEPRO_protocol.py` | Framing, CRC-16, frame parser, status helpers. No Klipper imports. |
| `extras/AFC_ACEPRO.py` | `[AFC_ACEPRO <name>]` unit. Owns the USB serial link, polls the ACE, mirrors slot presence and RFID tags into lanes, runs feed / unwind / feed assist. |
| `extras/AFC_ACEPRO_lane.py` | `[AFC_ACEPRO_lane <name>]` lane. `AFC_ACEPRO_TOOL_LOAD` / `AFC_ACEPRO_TOOL_UNLOAD` run as AFC's `custom_load_cmd` / `custom_unload_cmd`. |
| `templates/AFC_ACEPRO_1.cfg` | Example config for one ACE on a K1 Max. |

**The link.** The ACE is a USB CDC serial device that takes JSON-RPC requests in
binary frames (`FF AA`, length, JSON, CRC-16, `FE`). It handles one request at a
time, and if it gets no complete frame for about 3 s it resets its USB link: the
device disappears and comes back, sometimes as a different `ttyACM`. So:

- All serial I/O is non-blocking on Klipper's reactor (`register_fd` + completions);
  a slow or silent ACE cannot stall heater or MCU timers.
- Exactly one request is in flight. Replies are matched by id; a late reply to a
  request that already timed out is recognised and dropped.
- The unit polls `get_status` every `poll_interval` (1 s, max 2 s). That poll is the
  keepalive, so it runs on its own timer that never waits on gcode: anything that
  can block (a runout's `PAUSE`, setting a lane's colour) is handed to a separate
  event timer.
- Use the `/dev/serial/by-id/...` path. After a USB reset the unit notices the node
  is gone, reopens the same path, and turns feed assist back on for the loaded lane.

**Presence.** Each slot's `status` (`ready` / `empty`) is the lane's spool sensor.
The ACE reports other slots as `empty` for a moment while it is busy, so readings
taken while it is busy or moving don't count, and a change must be seen
`presence_confirm` (2) polls in a row. Pulling a spool mid-print pauses, or hands
off to the lane's `runout_lane` if one is set.

**Load** (per attempt): wait for the ACE to report ready → `feed_filament`
`bowden_length - approach_length` at `feed_speed` → feed slowly at
`approach_speed` until the toolhead sensor triggers (watched every 20 ms; the stop
is confirmed against the status, and never sent within 300 ms of the feed's start,
which the ACE would ignore) → hand-off: the ACE pushes `grab_length` while the
extruder turns the same distance, so the gears take the filament → feed assist on →
the extruder pushes the rest of `tool_stn`. A feed that stalls (`feed_error`), never
starts or never reaches the sensor fails the attempt and is **pulled back out**, so
it doesn't block the splitter for the other lanes: with a splitter sensor, until it
clears plus `retreat_margin`; without one, by the estimated fed distance.

**Unload:** `AFC_CUT` (verified against `cutter_sensor_pin` if set) → park / form
tip per AFC settings → feed assist off → the extruder retracts out of the gears
(`unload_retract_steps`; on the CFS kit the cutter is 10 mm below the gears) → the
ACE unwinds `bowden_length + unload_extra`, leaving the tip staged just past its
outlet. The toolhead sensor must clear on the way, and the splitter sensor if you
configured one. Unwinding too far pulls the tip back past the slot sensor and the
slot then reads empty, which is why `unload_extra` defaults to 0.

**Feed assist.** While a lane is loaded the ACE feeds along with the extruder. Only
one slot can assist at a time; the unit stops it before any move and re-arms it if
the ACE forgets it (after a USB reset). `AFC_ACEPRO_FEED_ASSIST LANE=<lane> ENABLE=0`
turns it off for a lane.

**RFID.** The ACE reads Anycubic tags itself. When a spool goes in and the ACE
identifies its tag, the lane gets the tag's colour and material through AFC's own
`SET_COLOR` / `SET_MATERIAL` (skipped if the lane has a Spoolman spool).
`rfid_apply: False` turns it off.

## Hardware

- **USB.** The ACE Pro's 4-pin printer plug carries native USB. Wire **D+, D− and
  GND only — never 5 V**: the ACE has its own supply. Check the pinout of your
  cable with a multimeter before plugging anything in; the
  [Kobra-S1/ACEPRO](https://github.com/Kobra-S1/ACEPRO) and
  [ACEPROK1Max](https://github.com/swilsonnc/ACEPROK1Max) READMEs show the plug.
  Keep the cable short (1 m or less) and good quality: a flaky cable shows up as
  timeouts and reopens in `AFC_ACEPRO_STATUS`.
- **Splitter.** The four ACE outlets go through a 4-in-1 PTFE splitter into one tube
  to the toolhead. Smooth inside matters more than anything else.
- **Splitter sensor (optional).** The K1 Max's stock runout sensor (`PC15`) can sit
  on the shared tube after the splitter as the `[AFC_hub]` switch. Loads then refuse
  to start while something is stuck there, and unloads must clear it.
- **Toolhead.** The CFS upgrade kit's extruder: filament sensor above the gears
  (`^!nozzle_mcu:PA10` → `pin_tool_start`), cutter below them, and the cutter's
  end-of-travel switch (`nozzle_mcu:PA8` → `cutter_sensor_pin`). Flash the kit's
  **non-CFS** toolhead firmware.

> Pin note: ACEPROK1Max treats `PA8` as a second filament sensor at the cutter.
> On the K1 Max kit measured for [afc-creality-cfs](https://github.com/mikeinredding/afc-creality-cfs)
> it is the cutter's end-of-travel switch (it fires at Y306 with the blade pushed in).
> Check yours: with `cutter_sensor_pin` set, `AFC_ACEPRO_STATUS` should show the
> cutter sensor change only when the cutter arm is pressed, not when filament passes.

## Setup on a K1 Max with Simple AF

1. Install AFC with the steps in Simple AF's FAQ,
   [How to install AFC on Simple AF](https://pellcorp.github.io/creality-wiki/faq/#how-to-install-afc-on-simple-af),
   cloned into `/usr/data`. AFC's installer has no ACE option; see
   [AFC installer choices](#afc-installer-choices) below. Works with AFCProject
   `main` (what that FAQ installs) and `DEV`; CI tests both.
2. Install this repo, also under `/usr/data`:
   ```sh
   cd /usr/data
   git clone https://github.com/mikeinredding/afc-anycubic-ace.git
   cd afc-anycubic-ace && ./install.sh -t
   ```
   It finds Klipper at `/usr/data/klipper`, links the ACE files into its extras,
   and copies `AFC_ACEPRO_1.cfg` into `/usr/data/printer_data/config/AFC/`.
3. Nothing else may own the ACE's port: no ACEPROK1Max `[ace]` section or
   `ace.py`, no ACE_series `[AFC_ACE]`.
4. Edit `AFC/AFC_ACEPRO_1.cfg`:
   - `serial:` — `ls /dev/serial/by-id/`, pick the `ANYCUBIC` device.
   - `cutter_sensor_pin:` — uncomment once you've checked it (see the pin note).
   - `[AFC_hub ACE_1] switch_pin:` — `virtual`, or `!PC15` for the stock runout
     sensor on the shared tube (comment out the stock `[filament_switch_sensor
     filament_sensor]` that uses it).
   - `bowden_length:` — after step 4 of the test plan.

   And in `AFC/AFC_Hardware.cfg`, under `[AFC_extruder extruder]`:
   - `pin_tool_start: ^!nozzle_mcu:PA10`
   - `tool_stn: 60` (sensor to nozzle; measure it), `tool_stn_unload: 0`,
     `tool_load_speed: 10`.
5. In GrumpyScreen's `grumpyscreen.ini` set `[mmu] backend: afc` for the lane panel.

Code updates need a **Klipper service restart** (`supervisorctl restart klipper` on
Simple AF): `FIRMWARE_RESTART` keeps the already-loaded Python code.

### AFC installer choices

Install AFC as a 4-lane BoxTurtle to get AFC's shared files, then remove the
BoxTurtle unit file. In the **I. Install New System** menu:

| Option | Set to | Why |
|---|---|---|
| T. Installation Type | `BoxTurtle (4-Lane)` | Simplest unit file to remove afterwards |
| 1. Add AFC includes | `True` | Adds `[include AFC/*.cfg]` to printer.cfg |
| 2. Tip-forming | `False` | The kit cuts |
| 3. Toolhead cutter | `True` | `AFC_CUT` with your lever position |
| 4. Hub cutter | `False` | |
| 5. Kick macro | `False` | |
| 6. Park macro | `True` | Park over the purge chute before loads |
| 7. Poop macro | `True` | Purge after a colour change |
| 8. Wipe macro | `False` until you configure one | |
| 9. Toolhead sensor or ramming | `Sensor` | |
| A. Toolhead sensor pin | `^!nozzle_mcu:PA10` | Written to `AFC_Hardware.cfg` |
| B. Buffer type | `None` | The ACE has its own buffer |

Then **I** to install and **Q** to quit, and before restarting Klipper:

```sh
cd /usr/data/printer_data/config/AFC
rm AFC_Turtle_1.cfg            # BoxTurtle unit: its [mcu Turtle_1] would stop Klipper starting
```

### Lines to change in AFC's own files

Edit the line that is there instead of adding a second copy (Klipper uses the
last one). Line numbers are for a fresh install (AFC `main` / `DEV`, October 2026).
The toolhead values are the ones measured on a K1 Max with the CFS kit; measure
your own cutter position.

`AFC/AFC.cfg`, section `[AFC]`:

| Line | Option | Set to |
|---|---|---|
| 84 | `tool_cut` | `True` |
| 86 | `tool_cut_cmd` | `AFC_CUT` |
| 89 | `park` | `True` |
| 90 | `park_cmd` | `AFC_ACEPRO_PARK` (in the template) |
| 93 | `poop` | `True` |
| 97 | `kick` | `False` |
| 101 | `wipe` | `False` (or your own wipe macro) |
| 113 | `post_load_macro` | uncomment: `AFC_ACEPRO_LEAVE_PURGE` |
| 116 | `post_unload_macro` | uncomment: `AFC_ACEPRO_LEAVE_PURGE` |

`AFC/AFC_Macro_Vars.cfg`:

| Line | Option | K1 Max (CFS kit) |
|---|---|---|
| 35 | `_AFC_CUT_TIP_VARS` `variable_pin_loc_xy` | `38, 300` (measure yours) |
| 48 | `variable_cut_direction` | `"back"` |
| 60 | `variable_cut_move_dist` | `7` |
| 86 | `variable_retract_length` | `0` |
| 92 | `variable_cut_count` | `1` |
| 101 | `variable_pushback_length` | `0` |
| 164 | `_AFC_POOP_VARS` `variable_purge_loc_xy` | `185, 305.5` |
| 165 | `variable_purge_spd` | `6` |
| 166 | `variable_z_purge_move` | `False` |
| 191 | `variable_purge_length` | `140` (tune down later) |
| 272 | `_AFC_PARK_VARS` `variable_park_loc_xy` | `185, 305.5` |

`printer.cfg`: `[stepper_y] position_max` from `306` to `307.5`, or the cut ends
exactly at the cutter switch's trigger point.

To find the cutter position: `G28`, `G1 X38 Y290 F6000`, then step toward the back
0.5 mm at a time until the cutter arm touches the pin (`pin_loc_y`); keep going and
run `AFC_ACEPRO_STATUS UNIT=ACE_1` after each step until the cutter sensor reads
triggered. Set `cut_move_dist` so the cut ends about 1 mm past that point.

## Test plan — do these in order, at the printer

Each step should pass before the next. `AFC_ACEPRO_RAW` lets you poke the ACE
directly, e.g. `AFC_ACEPRO_RAW UNIT=ACE_1 METHOD=get_status`.

1. **Link, read-only.** `AFC_ACEPRO_STATUS UNIT=ACE_1`: model and firmware, the
   right slots `ready`, `timeouts 0`. Leave it a few minutes: `reopened 0x` means
   the keepalive holds. Pull a spool: the lane shows empty within ~2 s.
2. **Toolhead sensor.** Push filament in by hand above the extruder; AFC must show
   the toolhead sensor triggered.
3. **Cutter alone.** Heat, load filament by hand, `AFC_CUT`. The lever must actuate
   and the filament must be cut.
4. **Measure the tube.** Toolhead empty: `AFC_ACEPRO_CALIBRATE LANE=lane1`. It feeds
   in 50 mm steps until the toolhead sensor triggers, reports the length, and unwinds
   it again. Put it in `bowden_length` and restart Klipper.
5. **Load slot 1.** Nozzle hot: `AFC_ACEPRO_TOOL_LOAD LANE=lane1`. Filament should
   reach the nozzle; `AFC_ACEPRO_STATUS` shows feed assist on slot 1.
6. **Unload slot 1.** `AFC_ACEPRO_TOOL_UNLOAD LANE=lane1`: cut, out of the gears,
   back to the ACE. The slot must still read `ready` afterwards.
7. **Slot switch.** Load 2, unload, load 4.
8. **Through AFC.** `CHANGE_TOOL LANE=lane2`, then `T0` / `T1` from the console.
9. **A short two-colour print**, then a runout test (pull a spool mid-print).

**Recovering from a failed load.** A failed attempt is pulled back automatically;
if something is still in the tube, `AFC_ACEPRO_EJECT LANE=lane1` unwinds
`eject_length`, then `RESET_FAILURE`. Once the extruder has the filament, use
`AFC_ACEPRO_TOOL_UNLOAD` instead. The lane's `ace_last_load` status field records
each phase's result and how far it fed.

Mark each step in the trace: `AFC_ACEPRO_TRACE UNIT=ACE_1 MARK="step 5 load 1"`.

## Commands

| Command | |
|---|---|
| `AFC_ACEPRO_STATUS UNIT=` | ACE state, slots, tags, dryer, feed assist, link counters |
| `AFC_ACEPRO_TOOL_LOAD LANE=` / `AFC_ACEPRO_TOOL_UNLOAD LANE=` | what AFC runs for a tool change; usable by hand |
| `AFC_ACEPRO_CALIBRATE LANE=` | measure `bowden_length` |
| `AFC_ACEPRO_EJECT LANE=` | unwind `eject_length` (filament not in the toolhead) |
| `AFC_ACEPRO_FEED_ASSIST LANE= ENABLE=0\|1` | feed assist for a lane |
| `AFC_ACEPRO_DRY UNIT= [TEMP=] [DURATION=]` / `AFC_ACEPRO_DRY_STOP UNIT=` | dryer (minutes; capped at `max_dryer_temperature`) |
| `AFC_ACEPRO_RECONNECT UNIT=` | close and reopen the link |
| `AFC_ACEPRO_TRACE UNIT= [ENABLE=] [MARK=]` | trace control |
| `AFC_ACEPRO_RAW UNIT= METHOD= [PARAMS=<json>]` | one raw request, bring-up only |

## Recording a trace

With `trace_file` set (the template turns it on), the unit records a JSON-lines file
of requests and replies (with latency), timeouts, port opens/closes, presence
changes, tags, every move (length, speed, result, estimated distance, how long),
each load and unload phase, cuts, and failures. The 1 s status polls are left out
unless `trace_polls: True`. Writes happen on a background thread; the file rotates
at `trace_max_size`. Download `acepro_trace.jsonl` from Mainsail/Fluidd's Logs and
attach it with `klippy.log` when reporting a problem.

## Known unknowns

- **Feed assist reporting.** The V1 status has no per-slot "assisting" flag; the
  unit tracks which slot it turned on and re-arms it after a USB reset.
- **Infinite spool.** When a spool runs out, its tail is still in the tube. With a
  `runout_lane` AFC unloads (cut + unwind) and loads the next lane; if the tail is
  already past the ACE's rollers the unwind can't pull it back and the unload stops
  with the toolhead sensor still triggered. Pause-on-runout (the default) is safe.
- **Distances are estimates.** The ACE doesn't report how far it moved; the unit
  estimates from time spent moving and the speed, and a stalled feed still reports
  moving, so the estimate runs long after a jam. That matters for pulling a failed
  load back without a splitter sensor (it may unwind far enough that the slot reads
  empty; re-seat the spool) and for calibration (5 mm resolution). With the
  splitter sensor the pull-back uses the sensor instead.
- **Two ACEs** (the Anycubic hub) are not supported yet.
- The hand-off values (`grab_length`, `grab_speed`) are starting points.

## Credits

Protocol and behaviour from printers-for-people/ACEResearch, utkabobr/DuckACE,
BlackFrogKok/BunnyACE, szkrisz/ACEPROSV08, swilsonnc/ACEPROK1Max, Kobra-S1/ACEPRO and
the AFC ACE_series branch. All GPL-3.0. Structure follows
[afc-creality-cfs](https://github.com/mikeinredding/afc-creality-cfs).

## Tests

```
./run-tests.sh                              # unit tests (clones AFC main into .deps/)
KLIPPER_PATH=~/klipper ./run-tests.sh       # plus real reactor + simulated ACE
AFC_BRANCH=DEV KLIPPER_PATH=~/klipper ./run-tests.sh
```

The reactor scenarios run the actual unit and lane code on Klipper's event loop
against `tests/acepro_sim.py`, a simulated ACE on a pseudo-terminal that drops off
USB after 3 s of silence, ignores early stops, answers FORBIDDEN while busy and
flickers slots empty, with a 5 ms watchdog that fails the test if anything blocks
the reactor.
