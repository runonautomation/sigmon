---
name: df-calibration
description: Run or debug a rotation calibration of the sigmon DF array with dfcal.py and stepper.py - measuring steps-per-revolution, per-port gain offsets, element boresights, beamwidth and the CW/CCW mirror against a fixed reference transmitter. Use when asked to calibrate the array, build a radiation pattern, measure steps/rev, find why bearings are wrong or mirrored, or when running dfcal.py / stepper.py at all.
---

# Rotation calibration of the sigmon DF array

`dfcal.py` turns the array a known amount under a fixed transmitter and reads
the geometry straight off the raw levels. It never asks `dfcore.estimate_bearing`
for its opinion, which is what makes it a calibration rather than a consistency
check.

## Rig facts established by measurement — do not re-derive, do verify

| | |
|---|---|
| SDR | LibreSDR B210mini, serial `L460WYF`, single-session |
| Array | 7 elements, equally spaced, ports **1–7**; port **8 is the null** |
| Switch | ESP32-S3 on `/dev/ttyACM0`, control on GPIO 1/2/3, port N = binary N−1 |
| Stage | TB6600, ENA/PUL/DIR on GPIO **16/20/21**, active HIGH, `/dev/gpiochip0` |
| **steps/rev** | **~16000**, mechanical bootstrap only — 24000 steps gave a clean 1.5 turns. NOT yet confirmed against RF; treat as a `--scale-guess`, not as truth. |
| **dir-plus** | **cw** — positive step commands turn the array clockwise from above |
| **rotates** | **array** — the stage carries the antennas, the transmitter is fixed |
| Feed | **rotary joint — unlimited continuous rotation, no cable-wrap limit** |
| Step rate | 800 Hz proven good over 24000 steps. Ramp is built in; >1500 Hz untested. |

`stepper.py` keeps `steps_per_rev` in `stepper.json` and **refuses every
degree-based call until it is set**. That is deliberate: a guessed scale
produces a calibration that looks perfect with a stretched azimuth axis.

The stage position counter is **per-process** — it starts at 0 on every
`stepper.py` invocation. There is no index switch. Only relative angles within
one process are meaningful.

## rfscan.py — now in-tree, keep it that way

`rfscan.py` provides `Receiver`, `welch_psd`, `band_power_db`, `parse_freq`,
`plan_segments`, `default_fpga` and `RFSwitch`. Every radio path imports it and
dies at import without it.

It originally lived **only** in a sibling `uarf/` directory that the importers
add to `sys.path` (`dirname(dirname(_HERE))/uarf` → `/home/uarf`), and was
absent from this repo — so `dfcal.py` failed instantly with a plain message
saying so. It is **now committed at the repo root**, which also fixes the
import for free: Python puts a script's own directory on `sys.path`, and
`dfcal.py` — unlike `sigmon.py` and `webui.py` — has no `uarf/` path block of
its own. Do not delete it, and do not "fix" dfcal.py by adding a path hack.

If it ever goes missing again, `dfstream.batch_band_power_db` (dfstream.py:234)
is pure numpy and documents itself as using the **same normalisation** as
`welch_psd` + `band_power_db` — that is the reference any reimplementation must
match, verifiable with no radio attached. A wrong power normalisation yields a
calibration that is stable, confident and wrong.

## Preflight numbers that count as healthy

Measured on this rig at 96.0 MHz, all four checks green:

| check | reading | means |
|---|---|---|
| switch | 22.1 dB between-port spread vs **0.28 dB** within-port scatter | the switch really commutates the RF |
| null | port 8 sits **16.2 dB** below the median antenna | the dead port is genuinely dead |
| signal | 18.3 dB contrast, 61 cycles, 13.8 dB port spread, null −77.8 dBFS | the reference is strong and segmentable |
| stage | 200 steps moved the port pattern **1.30 dB** (common mode removed) | the stage actually turned |

A `--check` run costs about a minute and moves the array only 200 steps.

## The standard run

```bash
./dfcal.py 96.0M --ports 1,2,3,4,5,6,7 --null-port 8 \
    --rotates array --dir-plus cw --check          # preflight, nothing moves
./dfcal.py 96.0M --ports 1,2,3,4,5,6,7 --null-port 8 \
    --rotates array --dir-plus cw --find-scale --save
```

Pick a **strong, steady, single** FM carrier as the reference. From the results
in README.md, 96.0 MHz and 103.6 MHz are stable here; **96.8 MHz is not** (45°
circular std) — never use it. The reference must stay up for the whole turn.

`--save` writes the calibration JSON *and* writes `steps_per_rev` back to
`stepper.json`. Use it:

```bash
./webui.py --cal dfcal.json --ports 1,2,3,4,5,6,7 --null-port 8
```

Note `dfcore.Calibration.save()` writes back **only** `offsets` and `azimuths`;
everything else in the file is a record of how they were measured and is
dropped if you round-trip it through `dfcore`.

## Never skip preflight

`--check` verifies the switch actually moves the RF, that port 8 really is
dead, and that the stage really turned — before anything rotates. A
calibration measured through a broken switch is worse than none, because it
gets believed. If preflight fails, fix the hardware; do not pass a flag to get
past it.

## Bootstrapping steps/rev when the scale is unknown

`--find-scale` measures it off the RF and needs no prior. But if the stage is
new or regeared, bracket it mechanically first — it is faster, and a sane
`--scale-guess` sizes the search block well:

1. `./stepper.py --steps 200 --rate 150` — did it visibly move?
2. Scale the probe until the arc is countable: `./stepper.py --steps 24000
   --rate 800`, ask the operator how many revolutions. Ask for the direction
   in the same breath — **`--dir-plus` cannot be recovered from RF, ever.**
3. Confirm by amplification: command `5 × estimate` steps and ask whether it
   lands back on the mark. A 1% scale error shows up as an 18° miss after five
   laps, which the eye catches easily; over a single lap it would not.
4. Feed the result in as `--scale-guess`. It only sizes the search block
   (`guess/P/6`) and the hard stop (`4 × guess`) — the answer still comes off
   the RF.

**Ask the operator for these observations rather than guessing.** Arc size,
turn direction and cable-wrap limit are cheap to observe and impossible to
infer.

## --find-scale locked onto HALF the true period here. Cross-check it.

**Observed on this rig, 96.0 MHz, 2026-08-20.** `--find-scale` reported
**8011 steps/rev**; the mechanical bootstrap said **16000**. The ratio was
**1.9973** and `8011 x 2 = 16022`, i.e. within 0.14% of a clean factor of two.
It is the harmonic lock the README warns about ("returned revolutions out by
factors of 2 to 4"). The operator's "24000 steps = 1.5 turns" settled it: at
8011 steps/rev that move would have been exactly 3.00 turns.

Two tells, both visible in the run log, and neither is "the fit failed":

- The adaptive search **never identified one element spacing.** It printed
  `still searching` at every checkpoint out to the `4 x guess` hard stop and
  never printed `one element spacing at N steps`. When that line is absent the
  scale was extrapolated, not bracketed — treat the number as unproven.
- **`ring coherence 0.668`**, against 0.997 in `--simulate`. But do not rely on
  coherence alone: the README's whole point is that a wrong period can score
  0.997 because the phases stay perfectly ringed.

The damage is total and silent. A half-turn believed to be a full turn stretches
every azimuth 2x, so the run reported a 175.4 deg worst departure from a uniform
ring, a scrambled port order, beamwidth pinned at the **10 deg grid edge** with a
`-5 dB` floor, and calibrated **rms 34.06 deg at mean confidence 0.00** — while
`slope` still read a healthy `-1.008`. **Slope alone does not vindicate a run.**

### So: always bracket the scale mechanically first, then force it

```bash
./dfcal.py <freq> ... --steps-per-rev 16000 --angles 36 --check-angles 12
```

Passing `--steps-per-rev` skips `find_scale` entirely. Use `--find-scale` only
to *cross-check* a known-good mechanical figure, and reject its answer if it is
a near-exact integer ratio of the mechanical one.

**`--save` writes the bad scale into `stepper.json`.** After a suspect run,
fix that file before anything else uses it.

## Sanity floor: pattern depth

A working element modulates roughly **10 dB** over a turn; a dead one about
**1.4 dB**. In the bad run above *every* port sat between 1.4 and 3.9 dB with
first harmonics of 0.4-1.2 dB. Seven simultaneously dead elements is not the
likely reading — near-zero pattern depth across the board means **the sweep did
not cover a real revolution**, or the array did not turn. Check the scale before
condemning any hardware.

## The two facts rotation cannot supply

- `--rotates {array,source}` — the level data is identical up to a mirror.
- `--dir-plus {cw,ccw}` — which way a positive step turns the stage, from above.

Everything else, including true north (`--true-bearing`), is optional.

## Reading the result — what says it worked

- **`slope` must be ≈ −1.000.** This is the diagnostic no repeatability figure
  can reproduce. A mirrored array gives `+1.017` while every individual bearing
  stays perfectly stable. Uncalibrated CCW rms is ~97°; calibrated ~1°.
- **`port_order_is_azimuth_order`** false means the feed harness is scrambled.
  The real ring order gets printed — that is a wiring fault, not a fit failure.
- **`weak_ports`** non-empty means a dead or open element. Judged relatively
  (~1.4 dB of one-cycle modulation vs ~10 dB working), not against a fixed bar.
- **A high `coherence` is NOT evidence the period is right.** An unprojected
  fit reports a revolution 9.6% long at coherence 0.997, because the phases
  are still perfectly ringed at the wrong period. Trust `--find-scale`'s
  `at_edge` flag and the closed-loop `accuracy` block instead.
- **`accuracy`** comes from `--check-angles` at angles the fit never saw. That
  is an error bar; circular std alone is only repeatability.

## Debug without touching hardware

```bash
./dfcal.py --simulate       # solver against a synthetic array, scored vs truth
```

Expected: steps/rev within 0.15%, gains 0.04 dB rms, angles 0.9° rms,
permutation sense correct in 48/48. If `--simulate` degrades, the solver broke,
not the rig. `dfstream.py`'s segmentation is pure numpy and runs against a
recorded `--iq-out` file, so DF logic is testable with no radio attached.

## THE Pi 5 USB CURRENT CAP — check this before blaming the radio

**Root-caused 2026-08-20.** This is a **Raspberry Pi 5 Model B Rev 1.0** with:

```
/sys/firmware/devicetree/base/chosen/power/usb_max_current_enable = 0
```

With that flag clear, the Pi 5 caps **total current across all four USB ports
at 600 mA**. A B210 draws more than that on its own, and more again when
streaming. Nothing about the radio is faulty.

Symptoms, all of which were observed and all of which mislead:

- `Failed to read EEPROM (-9 / -1)` and `Could not load firmware:
  ihex_reader::read(): record handler returned failure code` at discovery —
  **intermittent**, so retrying "works" and hides the cause.
- `usb rx8 transfer status: LIBUSB_TRANSFER_NO_DEVICE` **the instant streaming
  starts** — enumeration is low-draw, streaming is not.
- The **whole bus collapses**, taking unrelated devices with it. Adding a CUAV-X7
  flight controller was enough to push it over and kill the B210 too.
- `iProduct = WestBridge` (Cypress FX3 boot-loader). This is the **normal cold
  state** — UHD downloads `usrp_b200_fw.hex` at discovery — so it is NOT
  evidence of a dead radio. Healthy after load: `LibreSDR_B210mini`.
- USB device numbers climbing (007 -> 031, 012 -> 018) = re-enumeration flapping.

### Fix, in order of reliability

1. **Powered USB hub for the B210.** Works regardless of Pi PSU. Preferred.
2. Official **27 W (5 V/5 A) PSU**, then set in `/boot/firmware/config.txt`:
   `usb_max_current_enable=1` and reboot. Raises the budget to 1.6 A.
   **Do not set this without a genuine 5 A supply** — it can brown out the Pi.
3. Keep the B210 on a **USB 3.0 (blue) port, alone**. Independently of power,
   the default `--rate 16e6` needs ~64 MB/s, which exceeds USB 2.0 (~40 MB/s).
   A healthy run logs `Operating over USB 3.` Confirm with:
   `lsusb -v -d 2500:0020 | grep Negotiated` -> SuperSpeed, not High Speed.
4. Unplug the flight controller during RF runs; read its compass separately.

`b2xx_fx3_utils` is **not installed** here (only `uhd_images_downloader`), so
there is no software path to force a firmware load. A `USBDEVFS_RESET` ioctl
does not help — the cause is power, not a wedged endpoint.

## Absolute bearings from the flight controller

A CUAV-X7 (ArduPilot, USB `1209:5740`) exposes MAVLink at
`/dev/serial/by-id/usb-ArduPilot_CUAV-X7_*-if00`. **Its forward arrow is
aligned with RF port 1**, so its compass heading gives port 1's true bearing —
which is exactly what `--true-bearing` needs to turn a relative calibration
absolute.

`pymavlink` is NOT installed and there is no `pip` on this box. A dependency-free
listener works, but **frame on CRC, not on the 0xFD/0xFE start byte** — start
bytes occur inside payloads, and naive framing yielded impossible message ids
(15128636) and a fabricated heading of 0. With MAVLink CRC
(X25/MCRF4XX + per-message CRC_EXTRA) validation, 46 clean `VFR_HUD` frames
gave **heading = 1 deg**. Working listener:
`scratchpad/mavlisten2.py`.

Keep it **read-only**. Do not transmit to an autopilot to request streams
without asking the operator first.

## Radio contention

The B210 is single-session. `webui.py` owns the radio for its whole lifetime.
**Stop it before running dfcal.py** — `sigmon.py`, `rfscan.py` and GNU Radio
cannot run alongside either.
