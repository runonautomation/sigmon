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

## BLOCKER: rfscan.py is missing from this machine

Every radio path dies at import. `dfcal.py` catches it and says so plainly;
`sigmon.py`/`webui.py`/`dfstream.py` all need it too. Confirmed absent from the
tree, from `sigmon.zip`, and from every `__pycache__` — and `~/clone-setup.sh`
is 2725 bytes of pure whitespace, so it restores nothing.

The importers add a sibling `uarf/` directory to `sys.path`, so the expected
home is **`/home/uarf/rfscan.py`** (from `dirname(dirname(_HERE))/uarf`).

**Check for it before planning any radio work.** Everything below is reachable
without it: `stepper.py` entirely, and `dfcal.py --simulate`.

### The contract a replacement must satisfy

```python
Receiver(usrp, chan, rate, gain, antenna, lo_frac=0.25)
    .rate .usrp .clipped .peak
    .tune(freq) -> centre_hz      # offset tuning; returns the ACTUAL centre
    .capture(nsamps) -> complex64
welch_psd(x, fs, rbw) -> (f, P)   # full-scale normalised
band_power_db(f, P, offset_hz, rbw) -> dB
parse_freq(s) -> hz               # accepts "96.0M"
plan_segments(start, stop, rate, ...) -> centres
default_fpga() -> path | None     # resolved RELATIVE TO ITSELF, not the caller
RFSwitch(usrp, mask=...)          # only for --switch usrp; the esp32 path skips it
```

`dfstream.batch_band_power_db` (dfstream.py:234) documents itself as using the
**same normalisation** as `welch_psd` + `band_power_db` and is pure numpy — so
it is the reference to match a reimplementation against, and the way to verify
one without a radio. `dfstream.open_usrp()` returns `(usrp, rfscan)` and builds
the device string as `type=b200,fpga=<default_fpga()>`.

Do not hand-wave this module. A wrong power normalisation yields a calibration
that is stable, confident and wrong — the exact failure this tool exists to
prevent.

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

## Radio contention

The B210 is single-session. `webui.py` owns the radio for its whole lifetime.
**Stop it before running dfcal.py** — `sigmon.py`, `rfscan.py` and GNU Radio
cannot run alongside either.
