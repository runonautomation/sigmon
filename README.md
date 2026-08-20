# sigmon — monitor a band, bear the strong signals, store them

Headless CLI. Give it a frequency range; it sweeps, finds signals standing above
the noise floor, estimates a bearing for each by amplitude comparison across the
antennas on the RF switch, and writes everything to MongoDB.

Hardware: LibreSDR B210 clone; an 8-way RF switch whose common feeds `TX/RX A`,
driven by a **separate ESP32-S3 board** that commutates it on its own at
microsecond dwells. **Antennas on ports 1–4, port 5 left with no antenna** —
that dead port is the whole trick, being both the sync marker that cuts a
continuous recording into per-antenna slots and the in-band noise reference
subtracted from every level. The app detects which ports are live by measurement
and never assumes, and checks that port 5 really is dead.

Wiring as fitted, both established by measurement rather than assumed:
control lines on **GPIO 1/2/3** of the ESP32-S3, and the switch decodes
**port N as binary N−1** (port 1 = `000`). Both are settings now, not
recompiles — see the switch-controller section.

Working, on FM, 4 passes each at 200 µs slots:

| MHz | bearing | circular std | cycles |
|---|---|---|---|
| 92.0 | 52.0° | 0.6° | 99 |
| 91.8 | 49.8° | 1.3° | 99 |
| 96.0 | 316.9° | 0.5° | 99 |
| 105.0 | 315.0° | 0.4° | 99 |
| 103.6 | 225.6° | 0.2° | 99 |
| 96.8 | 302.6° | **45.1°** — unstable | 99 |

Different stations give distinctly different bearings, and the two pairs that
nearly agree (92.0/91.8 and 96.0/105.0) are consistent with shared transmitter
sites. Bearings are relative, not true north, until a calibration is supplied.

```bash
cp env.example .env          # change the password
./dfstream.py 96M --check-switch          # FIRST: does the switch actually move?
./dfstream.py 96M --ccw --repeat 0        # bear one frequency, fast
./dfstream.py 96M --hold 3                # or just hold antenna 3
docker compose up -d
./sigmon.py 88M 108M --commutate --auto-balance --ccw --stability-report
./sigquery.py signals
```

`--ccw` matters: bearings increase clockwise, so with an anticlockwise array
the default mapping **mirrors every bearing about 0°**. No stability check can
catch it — a mirror leaves circular std and boresight distance both unchanged —
so it has to come from how the hardware was built.

## Web UI

```bash
./webui.py --ccw --auto-balance          # http://127.0.0.1:8088
./webui.py --start 2400M --stop 2440M --gain 25 --peak-hold 8 --ccw
./webui.py --host 0.0.0.0                # reachable from other machines
```

Live spectrum, scrolling waterfall, and DF on any frequency. Click either
canvas to pick a frequency, then **DF once** for a single bearing or **pin** to
keep measuring it — pinned mode accumulates a circular mean and standard
deviation, which is the number that says whether a bearing means anything.

The polar plot shows the full likelihood curve, not just the answer: a sharp
lobe is a real fix, a fat one means the levels are consistent with many
bearings. Element boresights are ticked around the rim, and the panel warns
when the fit lands on one — the same snapping check `--stability-report` does,
because a degenerate estimator repeats perfectly and looks precise.

The B210 is single-session, so the server owns the radio for its lifetime and
nothing else (`sigmon.py`, `rfscan.py`, GNU Radio) can run alongside it.

Measured here: 209 ms per sweep of 88–108 MHz at 100 kHz steps (~5 waterfall
rows/s), and pinned DF on 96 MHz gave 63 measurements in 45 s at 0.3° circular
standard deviation.

The spectrum display uses ONE antenna (the first live port, or
`--spectrum-channel`); DF uses all of them. Sweeping four for the picture would
be four times slower for a display that looks the same.

Note that the throughput figures in this section are from the old
capture-per-antenna DF, which is still reachable with `--df-legacy`. The default
path is now the commutated stream described below.

## Direction finding: one recording, the switch running underneath it

The DF no longer takes a capture per antenna. It tunes once, starts **one
gapless capture**, and commutates the switch `1-2-3-4-null` while the samples
are streaming. The four antennas come out as slices of a single continuous
recording.

That is as fast as the hardware goes. Per-antenna captures paid the stream
start-up cost four times per cycle — and *that*, never the switch, was what set
the cycle time. With a sub-microsecond switch there is no reason to stop
receiving at all, so the only dead time left in a cycle is the null slot
itself, and the four levels sit **microseconds** apart instead of milliseconds.

### The null position is what makes it possible

Nothing tells the host *when* a GPIO write actually reached the switch. It
crosses USB with tens to hundreds of microseconds of latency and jitters by a
comparable amount — an error a whole slot wide. Commutating inside a continuous
stream is only useful if the stream can be cut back up correctly, and host
timestamps cannot do it.

The no-signal position solves it in the data. A dead port reads the receiver's
own noise floor, well below any live antenna, so it prints a periodic **dip** in
the power envelope. Find the dips and the recording is self-clocking: whatever
lies between two of them is exactly one `1-2-3-4` pass. Sync is re-established
every cycle rather than integrated from an assumed rate, so switch jitter, USB
latency, host scheduling — and even a dropped block of samples — move the dips
along with the data they corrupt.

Verified two ways. On a synthetic stream with 6% slot jitter, deleting 37 000
samples mid-record — what an overflow does — left 97 of 100 cycles recovered and
the bearing unchanged at 37.0°; a fixed-cadence scheme would have been
misaligned for the whole second half. And driving the real commutator thread
against a mock radio with **180 µs of unknown latency** between the GPIO write
and the RF change recovered 41.0° against a true 41.0°, with the four levels
correct to 0.1 dB. The latency simply does not enter the answer.

### The null slot is also a measurement

It reads the receive chain with no antenna on it — thermal noise, LO leakage,
whatever spur lives at this frequency and gain — at the same instant, through
the same hardware. Subtracting it **in power** from each antenna leaves the
antenna's own contribution.

This matters for the fit, not just for tidiness. An element pointing away from
the source does not read low; it reads the receiver floor, which is *identical
on every port*. The level spread therefore saturates exactly where the pattern
is doing its most useful work, and the fit sees four nearly equal levels, which
is consistent with a source anywhere. The eight-antenna array could not do this
at all — it had no dead port to ask.

What it does **not** remove is ambient noise the antenna itself picked up, which
is a real antenna output and is often the larger term at VHF. Measured on a
synthetic array with 15 dB of ambient over the receiver floor, a 15.9 dB true
spread read 13.3 dB either way. This recovers the receiver's contribution and
nothing else.

### What comes out

Each DF returns a bearing from the power-averaged cycles *and* the spread of the
per-cycle bearings inside the same recording. At 200 µs slots a 100 ms record
holds ~100 independent cycles, so that spread is a real distribution rather than
the handful of sweeps `sweep_std_deg` used to be built from — this measurement's
own error bar, available immediately instead of after pinning a frequency for a
minute.

Where to put the threshold between the null and the antennas is not knowable in
advance — it depends on the contrast, which depends on the band, the gain and
what is on the air — so a range of thresholds is tried and whichever recovers the
most cycles wins. A fixed fraction worked at high contrast and failed at
moderate: measured here, 96 MHz had 12 dB of contrast and synced all 99 cycles,
while 103.6 MHz had 5.9 dB and recovered 10%, because antenna fades reached below
a threshold set for the deeper case. With the search, all six frequencies tested
sync 99/99. Nothing is being tuned into existence — the cycle count is what is
maximised, and spurious dips do not land a cycle apart.

Sync is checked three ways and **fails loudly** rather than producing a bearing
from a wrong cut, which would look exactly like a right one:

| check | rejects |
|---|---|
| envelope contrast vs. the envelope's *own* statistical scatter | a lock on noise |
| dip spacing regular to < 30% (**median absolute deviation**) | dips that are fades, not the switch |
| ≥ 40% of expected cycles recovered | a wrong `--slot-us`, or missed nulls |

All three were confirmed against synthetic streams: flat noise, `--slot-us` set
to 350 and to 90 against a true 200, are each refused with the reason.

The regularity test uses a **median absolute deviation, not a standard
deviation**, and the difference is not cosmetic. A *missed* null merges two
cycles into one double-length gap, and a single such gap drags a std past any
sensible threshold even when every cycle that was found is regular to a percent
— so a perfectly good recording got rejected for the one thing the check was not
meant to police. Missed nulls are what `coverage` measures, and it measures them
properly. Measured on synthetic streams:

| case | old (std) | now (MAD) | verdict |
|---|---|---|---|
| clean | 2% | 2% | pass |
| 10% of nulls missed | 29% | 3% | pass |
| 25% of nulls missed | 45% | 3% | pass |
| genuinely irregular slots | 60% | 37% | **reject** |

At 10% and 25% missed the bearing still came out at exactly 37.0° against a true
37.0°, so those rejections were pure loss. The irregular case is still caught.

Processing costs ~20 ms per 100 ms record, so the DF rate is set by the
recording, not by the maths. Two things got it there: one batched FFT over all
~500 slots instead of a transform per slot, and a vectorised bearing fit (15×
faster than calling `estimate_bearing` in a loop). The band-limited sync
detector — needed only when the wideband one fails — costs 90 ms on its own and
is therefore computed lazily.

```bash
./dfstream.py 96M --ccw --repeat 0                    # bear continuously
./dfstream.py 96M --slot-us 120 --record-ms 300       # push the commutation
./dfstream.py 96M --iq-out iq/                        # keep the raw samples
./webui.py --ccw --auto-balance                       # same engine, in the UI
./webui.py --df-legacy                                # the old path, for A/B
```

## Microsecond detail — what is available and what is not

Yes: the recording is continuous and every sample is kept, so its time
resolution is **one sample — 62.5 ns at 16 Msps**, and 16 ns if the board is run
at its 61.44 MHz limit. Looking at how a signal changed over a couple of
microseconds needs nothing new; it is the same capture the bearing came out of.
`dfstream.fine_envelope()` returns it at any resolution, `--env-out` writes it
to CSV, and `--iq-out` keeps the raw IQ with a sidecar describing how to
segment it again.

Three *different* things are not available at that scale, and they are worth
keeping apart because they have different fixes:

**1. A level in a couple of microseconds — no, and no engineering fixes it.**
A power estimate over time `T` in bandwidth `B` has about `T·B` independent
looks and scatters by `8.7/√(T·B)` dB. At 200 kHz RBW a 2 µs window holds 0.4
looks, so its "level" is ~14 dB of noise. The fix is a wider RBW, not a faster
anything: 2 µs is usable at 2 MHz, not at 200 kHz. `dfstream.py` warns and tells
you which RBW would make the slot you asked for honest.

**2. Commutating every couple of microseconds — yes, with the ESP32 board.**
See the switch-controller section below. The ESP32 free-runs the commutation on
its own hardware at a **1 µs minimum dwell** — 5 µs per 1-2-3-4-null cycle,
200 000 cycles/s, measured on the actual board — so the switch stopped being the
limit at any dwell worth using. Driving the same switch from the host instead
tops out around 50 µs per slot.

A 1 µs dwell is real and useless, which is the point of limit 1: at 200 kHz RBW
it is 0.2 looks. What now sets the shortest useful slot is the receiver, not the
switch.

**3. The receiver following a switch step in a couple of microseconds —
measurable, so measure it.** The AD9361's decimation filters carry tens of
samples of group delay and its DC-offset tracking loop actively chases the step
at every commutation.

```bash
./dfstream.py 96M --probe-transition --res-us 0.25
```

averages the recording over **every** null→antenna edge in the capture, aligned
to sub-microsecond precision by interpolating each edge's half-power crossing,
and prints the step response as a profile. Read two numbers off it: when the
level goes flat (that is the floor under `--guard` and under `--slot-us`), and
whether it *overshoots* — which is the DC-offset loop reacting, not the RF. On a
synthetic chain with a 3 µs time constant it recovered 5.0 µs settling against
4.7 µs predicted.

That loop is now **off by default** during commutated DF (`--track-dc` restores
it). It is right for a stationary input and wrong for this one: the port changes
every slot, so the loop spends the recording chasing steps it will never catch
and injects its transient into the first microseconds of each slot — the same
ones the guard band is then forced to discard.

## Who moves the switch

Two controllers, selected with `--switch`, and they are three orders of
magnitude apart.

| | `--switch usrp` | `--switch esp32` |
|---|---|---|
| driver | three GPIO pins on the B210, host thread | separate ESP32-S3 board, serial console |
| shortest slot | ~50 µs | **1 µs** |
| per 1-2-3-4-null cycle | 250 µs | **5 µs** |
| cycles/s | 4 000 | **200 000** |
| host involvement | one USB write per edge, forever | configure once, then nothing |

Measured on the actual ESP32 board here, against its own step counter:

| asked | measured | error | per cycle |
|---|---|---|---|
| 1 000 ns | 1 000 ns | +0.0% | 5.0 µs |
| 20 000 ns | 20 006 ns | +0.0% | 100.0 µs |
| 50 000 ns | 50 016 ns | +0.0% | 250.1 µs |
| 200 000 ns | 200 058 ns | +0.0% | 1000.3 µs |

The board is told the sequence and the dwell once and then free-runs, so a DF
costs **exactly one capture** and no serial traffic at all. `swbackend.py` holds
both drivers behind one interface; ports keep each device's own numbering
(**1–4 antennas and 5 = no antenna** on the ESP32, 0–3 and 4 on the B210) rather
than being translated, because a translation in the middle is how an array ends
up reported one element out.

`--switch auto` prefers the ESP32 and says so loudly if it falls back — a silent
fallback would drop the commutation rate by 40 000× with nothing downstream to
notice.

**Flash wear is a real constraint.** Every ESP32 setting except `log` is written
to NVS, so anything that flips the switch between "held" and "iterating" on every
pass would write flash several times a second forever. Three things prevent that:
the backend sends nothing when the device already has the setting, a DF leaves
the switch commutating instead of stopping it, and a pinned DF suspends the
spectrum sweep rather than alternating with it. `nvs_writes` in `/api/state`
counts what was actually sent, so the claim is checkable. A full 5-port antenna
probe costs 6 writes; a DF costs 1–3.

`--df-legacy` is refused on the ESP32 for the same reason — it holds each antenna
in turn, which would be 64 flash writes per measurement. Use `--switch usrp` for
that comparison.

### Choosing an antenna by hand

`/api/antenna` with `{"port": N}` holds one antenna; `{"port": null}` returns to
auto. The web UI has a button row for it. Holding a port pins the spectrum to
that antenna and cancels any running DF — the two are contradictory requests, and
letting a DF keep moving the switch would make the displayed spectrum belong to
no particular antenna. From the command line:

```bash
./dfstream.py 96M --hold 3        # antenna 3, no radio needed
./swbackend.py --hold 5           # the no-signal position
./swbackend.py --exercise         # walk the ports, print expected line levels
```

### Check the switch actually moves before trusting anything

```bash
./dfstream.py 96M --check-switch
```

This is the check that has to come first on new wiring. A switch that is
unpowered, unwired, or listening to a different controller returns **the same
antenna on every port** — and that is not an error anywhere downstream. The
levels are real, the capture is clean, the four "antennas" simply agree
perfectly, and amplitude DF on that produces a bearing that is meaningless and
perfectly stable. Which is the worst combination this project keeps rediscovering.

It holds each port several times over, **interleaved**, because the obvious
version of the test cannot tell a switch from a drift. Measured here on a chain
with the switch not responding, a single pass over eight ports showed 1.9 dB of
"spread" that was really the receiver settling monotonically over the two seconds
the pass took. Repeating separates them — a real port difference repeats, drift
does not — and the same hardware then read 0.3 dB between ports against 0.1 dB of
scatter.

### Two wiring facts that were guesses, and are now settings

Both were found with `--check-switch` on this array, and both had produced a
convincing wrong answer first.

**The control-line pins.** The firmware defaulted to GPIO 4/5/6; the switch is
soldered to **1/2/3**. Nothing detects that from the firmware side — it drives
three pins that go nowhere, logs every port change, advances its step counter,
and the RF never moves. `--check-switch` read 0.3 dB between ports against
0.1 dB of scatter. The pins are now a stored setting:

```
gpio            -> lines L1=GPIO1 L2=GPIO2 L3=GPIO3
gpio 1 2 3      -> set and remember (validated, persisted to NVS)
```

**The truth table.** The firmware mapped port N to binary N; this switch decodes
**N−1**, so port 1 is `000`. Holding all eight patterns settled it:

| pattern | 000 | 001 | 010 | 011 | 100 | 101 | 110 | 111 |
|---|---|---|---|---|---|---|---|---|
| | ANT | ANT | ANT | ANT | — | — | — | — |

Off by one is a nasty failure precisely because it half-works: three of the four
antennas still answered, on the wrong ports; the fourth looked dead; and port 5,
the no-signal reference the whole method depends on, read as a live antenna. It
looks exactly like a broken element rather than a mapping error. Corrected in
`s_port_code[]`, with the host tests updated to match.

**A related red herring:** with the old table, line 1 was LOW on ports 1, 2 and 3
by design — it first went high on port 4. So "the ESP32 never drives line 1 high"
was the expected reading whenever the switch sat on an antenna port, and said
nothing about whether the board worked. `./swbackend.py --exercise` walks the
ports and prints the level each line should be at, for probing the header.

## Switching speed — what was wrong and what it actually bought

The first version took **500 ms** for one 8-antenna DF cycle. Measured
breakdown on this board:

| | |
|---|---|
| `sw.select()` | **0.026 ms** — switching was never the cost |
| `rx.tune()` at the same frequency | 0.044 ms |
| capture, 30 ms dwell | 31 ms |
| **cycle as originally written** | **500 ms** |

Almost all of it was self-inflicted: a `rx.tune()` **per antenna** at a
frequency that does not change during a DF, each followed by a 10 ms settle
sleep, plus a 2 ms switch settle where the switch needs ~0.03 ms and the
receiver settles in 1–3 samples. Tuning once and cutting the dwell brings the
same cycle to **15.9 ms — 31× faster**.

`sigmon.py` had a worse version of the same defect. It swept the *whole band*
on antenna 0, then the whole band on antenna 1, and so on — putting a full
sweep (~2 s) between two readings that amplitude comparison assumes were taken
at the same instant. Tuning is now on the outside and the antennas commutate on
the inside, so all eight readings of a frequency land inside one ~16 ms cycle.
Pass time went from 2.0 s to **0.28 s**.

**What this did not buy.** A controlled A/B at identical total capture time per
antenna — one 16 ms sweep against 16 × 1 ms sweeps, pinned on 96 MHz:

| | measurements | circular std | mean |
|---|---|---|---|
| 1 × 16 ms | 162 | 0.33° | 272.5° |
| 16 × 1 ms | 130 | 0.34° | 272.5° |

Indistinguishable. For a *static* FM transmitter the eight levels are equally
comparable whether they were taken across 16 ms or 128 ms, so simultaneity was
not what limited FM stability — SNR and the environment were. The speedup is
real and large, but it shows up as **throughput** (2.4× more measurements per
second), not as a lower standard deviation.

Where it should matter is anything that varies on the timescale of a sweep —
WiFi, mobile transmitters, fading paths. That is untested.

Each DF now also reports **`sweep_std_deg`**: the spread of the per-sweep
bearings inside a single measurement. It is an immediate quality figure, rather
than something you only learn after pinning a frequency for a minute. Typical
values here are 1–6°, which is the honest short-term uncertainty — the 0.3°
figure is what remains after averaging, and only ever described the average.

## Self-calibration by rotation

Everything above measures bearings that are *relative*. `dfcore.Calibration`
has always had slots for per-port gain offsets and per-port boresights and
nothing has ever filled them, so three things were assumed rather than known:
that the antennas are equally spaced and wired in azimuth order, that the ports
have equal gain, and that the beamwidth is `0.7x` (or `1.33x`) the spacing.
Each of them, wrong, produces a bearing that is stable, confident and wrong.

A rotation stage supplies the one thing amplitude DF never has: **ground
truth**. Turn the array a known amount under a fixed transmitter and every
antenna is forced through the same pattern cut, so the answers fall out of the
raw levels without ever asking the bearing estimator for its opinion.

```
./dfcal.py 96.0M --ports 1,2,3,4,5,6,7 --null-port 8 --find-scale --save
./dfcal.py 96.0M --check        # preflight only, nothing rotates
./dfcal.py --simulate           # the solver against a synthetic array
./stepper.py --deg 90           # the stage on its own
```

### What it measures, and why each one is measurable

| | |
|---|---|
| **gain offsets** | Over one full turn every element sweeps the *same* set of angles relative to the source, so its mean level over the turn cannot depend on where it is mounted. What is left between ports is gain. This is what `--auto-balance` approximates by averaging over a band and hoping the signals arrive from all directions; here they do, by construction. |
| **boresights** | Each port's level-versus-rotation curve *is* its element pattern, measured. The phase of its fundamental is that element's mounting angle. No assumption of equal spacing. |
| **beamwidth and back floor** | Fitted to the measured cut instead of picked from a rule. |
| **the CW/CCW mirror** | `dfcore` says no stability check can detect a mirrored array, and none can — the statistics are identical. Rotation detects it, because turning one element spacing makes each port read what its *neighbour* read, and which neighbour is the wiring sense. |
| **steps per revolution** | The stage's own scale (step angle x microstep DIPs x pulley ratio) is nowhere written down. It is measured off the RF: the rotation that permutes the ports by one is 1/N of a turn. |
| **accuracy** | Then, on angles the fit never saw, how far calibrated bearings actually land from where the stage says they should. That is an error bar, not a repeatability figure. |

### The two facts rotation cannot supply

`--rotates {array,source}` — whether the stage carries the array under a fixed
transmitter or a source around a fixed array. The level data is identical up to
a mirror. `--dir-plus {cw,ccw}` — which way a positive step command turns the
stage, seen from above. Watch it once and set it. Everything else, including
true north (`--true-bearing`), is optional.

### How the scale is found without a prior

The load-bearing estimator is `find_period`. Each port's level has a
fundamental at one cycle per revolution, and the P fundamentals are not
independent: their *phases* are the elements' mounting angles, so on a uniform
ring they sit 2π/P apart and in ring order. So the model fitted at each
candidate period is one complex amplitude shared by all P ports, each
pre-rotated by its place in the ring — `P + 2` free parameters instead of `3P`.
The permutation sense falls out as whichever derotation makes the sum cohere.

Two things about it are not cosmetic, and both were found by the simulator:

**Matching lags pairwise does not work.** Two spacings roll the vector by two,
three by three, and on a seven-element ring rolling by four is the same array
operation as rolling by minus three. A best-score search over lags picks
whichever multiple the noise favoured; the first version returned revolutions
out by factors of 2 to 4.

**The basis has to be projected off the constant.** At long periods a cosine
over a record barely one period long is nearly collinear with a constant, so an
unprojected fit lets the sinusoid absorb whatever DC survived the mean removal.
The estimate then climbs to whatever the longest period on the grid is — 9.6%
long, at a **coherence of 0.997**, because the phases are still perfectly
ringed at the wrong period. A high coherence is not evidence that the period is
right.

### Drift is removed before anything else

The reference is a broadcast station, not a lab source, and over the minute or
two a full turn takes its level wanders. That drift is common to every port —
all seven levels come out of one 100 ms capture — and not common to every
angle, so it lands squarely on the quantity being measured. Subtracting the
per-angle port mean removes it exactly, and the price is known rather than
hoped for: what gets subtracted is invariant under rotation by one spacing, so
it contains only harmonics that are multiples of P. The boresight estimator
uses the fundamental and the gain estimator uses the DC term. Neither is a
multiple of P.

### Results against a synthetic array

`--simulate` builds a ring with unequal gains, elements off their nominal
angles, a drifting reference and per-capture noise, then scores the recovered
numbers against the truth that generated them. Eight seeds each:

| injected | recovered |
|---|---|
| steps/rev 400 … 25 600 | within **0.15%**, with no prior beyond a block size |
| per-port gains, 2 dB sd | **0.04 dB** rms |
| element angles, 4° sd | **0.9°** rms |
| beamwidth 30 … 120° | exact to the 1° grid, for P = 4 … 8 |
| permutation sense | correct in 48/48, across all four `rotates` x `dir-plus` combinations |

Degradation is graceful: at 5 dB of per-capture noise and 15 dB of drift the
angles still come back to 4° rms and the sense is still right 8/8.

### The closed-loop test is the one that matters

Feed the recovered calibration back into `dfcore.estimate_bearing` and measure
bearings at angles the fit never saw:

| array wiring | uncalibrated rms | calibrated rms | slope |
|---|---|---|---|
| clockwise | 11.1° | **0.9°** | −1.000 |
| anticlockwise | **97.5°** | **1.1°** | −0.996 |

The second row is the mirror. Uncalibrated, an anticlockwise array reports
bearings that are 97° out on average and perfectly repeatable — and the
`slope` is `+1.017` where it must be `−1`. That slope is the diagnostic no
repeatability figure can reproduce: every angle can be individually stable
while the map between stage and bearing is reversed or stretched.

### Failure modes it detects rather than absorbs

- **A scrambled feed harness.** Ports fed in the wrong order come out with
  measured azimuths that are not in port order; `port_order_is_azimuth_order`
  goes false and the real ring order is printed. Tested with a 7-port
  permutation: detected, and the azimuths recovered to ~1°.
- **A dead element.** An open feed shows ~1.4 dB of one-cycle-per-turn
  modulation against ~10 dB for a working port, and is flagged relatively
  rather than against a fixed bar — removing the common mode leaves a few dB
  on a port that has no pattern of its own, so an absolute threshold either
  never fires or condemns working elements at a quiet site.
- **A live "null".** The dead position is both the sync marker and the noise
  reference. If it hears the source it is flagged, because it biases every
  antenna level most where the array points at the source — which is where the
  bearing is decided.
- **A switch that is not moving the RF**, a null that is not dead, and a stage
  that did not turn: all checked in preflight, before anything rotates.

### The stage

`stepper.py` wraps the GPIO lines `tb6600_go.py` confirmed (ENA/PUL/DIR on
GPIO 16/20/21, common cathode, everything active HIGH) and adds the three
things a calibration needs: a counted position, a scale that is `None` until
something measures it, and an approach direction. Every target is reached from
the same side, because gearbox backlash is typically a degree or more — the
same size as the errors being calibrated out — and ignoring it puts a
systematic offset between clockwise and anticlockwise measurements of the same
angle that the fit averages into a boresight that is neither.

## Files

```
swbackend.py         who moves the switch: esp32 board or the B210's GPIO
esp32switch/         the switch board's firmware, console protocol and client
dfstream.py          commutated-stream DF: one capture, switch running under it
dfcal.py             self-calibration: rotate the array, solve the geometry
stepper.py           the rotation stage, addressed in degrees
webui.py             web server: spectrum, waterfall, DF (owns the radio)
static/index.html    the page — vanilla JS, no CDN, no build step
sigmon.py            headless sweeper
dfcore.py            detection, bearing estimation, circular statistics
store.py             MongoDB writer with an explicit JSONL fallback
sigquery.py          read stored signals back out
docker-compose.yml   mongo (+ optional mongo-express via --profile ui)
```

`dfstream.py`'s segmentation is pure numpy and touches no hardware, so it can be
run against a recorded `--iq-out` file or a synthetic stream. That is how every
claim in the section above was checked.

## What it does per pass

1. Step the switch across the live ports, capture the span at each, Welch PSD.
2. Find peaks ≥ `--threshold` dB above the noise floor.
3. Fit a bearing per peak from the per-antenna levels, weighted by each
   element's own SNR.
4. Write one `observations` document per signal per pass; maintain one
   `signals` document per emitter with its circular mean bearing and stability.

Collections: `runs`, `observations`, `signals`. Bearings are stored as `null`
when the fit is rejected, so a missing bearing is never confused with 0°.

## Results on FM, all 8 antennas

8 passes, 88–108 MHz, `--auto-balance --ccw`, auto-selected 60° beamwidth —
5 stable, 2 usable, 1 unstable, and the fit genuinely interpolating (median
18.3° from the nearest boresight, against 11.2° for a uniform distribution):

| MHz | n | bearing | circular std |
|---|---|---|---|
| 96.000 | 8 | 335.6° | **0.3°** |
| 96.800 | 8 | 336.2° | **0.3°** |
| 93.800 | 5 | 290.7° | **0.2°** |
| 94.600 | 6 | 337.6° | **0.3°** |
| 90.600 | 8 | 29.1° | **0.5°** |

### Going from 4 to 8 antennas did not measurably improve stability

Interleaved back-to-back runs, so the RF environment cannot explain the
difference — fraction of signals coming out stable:

| config | runs |
|---|---|
| 4 ports (0–3) | 70%, 80%, 57% |
| 8 ports | 50%, 75%, 88% |

**The run-to-run spread is larger than the difference between configurations**,
so on this evidence the two are indistinguishable. What 8 elements does buy is
finer nominal resolution — 45° sectors instead of 90° — and the snapping check
still passes, so that resolution is real rather than quantised.

I also tested whether the longer 8-port sweep was decorrelating the first
antenna from the last through fading. It is not: halving the dwell so the
8-port sweep took the same wall time as a 4-port one made things *worse*
(7/8 stable → 5/8), which is what less averaging per antenna predicts.

### Beamwidth had to be retuned for 8 elements

Fraction stable, pooled over every run on this array:

| beamwidth | 8 elements |
|---|---|
| 0.7× spacing (31°) | 29% |
| 1.0× spacing (45°) | 64% |
| **1.33× spacing (60°)** | **72%** |

The 0.7× rule was tuned when there were 4 elements, where it prevents the fit
collapsing onto the boresights. With 8 it is too narrow — adjacent beams cross
about 6 dB down and the crossover is noisy. The default is now
`1.33 × spacing` for ≥6 elements and `0.7 ×` below that. Both are weakly
determined; the snapping check is what should judge a given site.

## Earlier results, 4 antennas (ports 0–3)

8 passes, 88–108 MHz, `--auto-balance`:

| MHz | n | bearing | circular std |
|---|---|---|---|
| 94.600 | 8 | 171.3° | **1.7°** |
| 89.400 | 8 | 276.2° | **0.8°** |
| 92.200 | 8 | 89.6° | **0.4°** |
| 91.600 | 8 | 88.3° | **0.5°** |
| 99.000 | 8 | 270.3° | **0.3°** |
| 96.000 | 6 | 177.4° | **2.6°** |

Static transmitters give repeatable bearings, and different stations give
distinctly different ones — so there is real directional information here, not
just noise.

## Three things that had to be fixed to get there

Each was found by looking at the data, and each silently produced
plausible-looking output first.

**The noise floor was the median.** Across a busy FM band most bins carry a
station, so the median sits *on* the signals and almost nothing clears the
threshold — one signal detected instead of eight. Now a 25th percentile.

**Live-port detection broke twice.** First version judged a port by its own
level spread; the tuning segments roll off at their edges, so even an open port
showed 15+ dB against a real antenna's 21 dB. Second version measured lift above
the per-frequency minimum across ports — clean while four ports were open
(1–4 dB vs 10–14 dB), but it silently assumed *some* port was empty to define
the floor. When all 8 were populated it confidently misclassified the three
weakest antennas as open.

Now the test is **agreement**: every antenna on the array sees the same
transmitters, so two live ports' detrended spectra correlate. An open port
correlates with nothing. This needs neither an empty reference port nor an
absolute threshold on "how much structure counts". Measured with all 8
populated: 0.60–0.94, against a 0.35 threshold and ~0 for a genuinely open
port.

**Bearings were snapping to the element boresights.** With beamwidth set equal
to element spacing (90° for four antennas) the residual is flattest exactly at
the crossovers, so the fit stops interpolating and just reports which antenna is
loudest — median 2.7° from a boresight. It still looked excellent, because a
quantised estimator repeats perfectly and the circular std was <1°. At 0.7×
spacing it interpolates instead (median 37° from a boresight) and is just as
repeatable — but see the 8-element section above, where 0.7× turned out to be
the *wrong* direction and the default is now 1.33×. The fix was never a
particular ratio; it was having `--stability-report` measure the snapping every
run and say so.

That last one is the important one: **stability is necessary but not
sufficient.** A degenerate estimator is perfectly stable.

## Results on 2.4 GHz WiFi — signals found, bearings NOT stable

Signals are detected and borne, but the bearings do not settle. Controlled
comparison, 2412–2442 MHz, 10 passes each, everything else identical:

| mode | circular std |
|---|---|
| `--peak-hold 1` (plain averaging) | **100.6°** — i.e. no information |
| `--peak-hold 8` (max-hold) | **50.2°** — better, still unusable |

An early 3-pass run showed 6.8° and looked promising. It was luck: with more
passes it degraded to 50°. Three passes is not enough to call anything stable,
and that first number should not have been believed.

**Why averaging fails.** The switch visits antennas one at a time and WiFi is
bursty, so each antenna sees a different set of packets. An average level then
measures how much traffic happened during that antenna's dwell — a duty-cycle
difference wearing a bearing's clothes. `--peak-hold N` splits the dwell into N
blocks and keeps the per-bin maximum, converging on the strongest burst
received, which is a property of the path rather than of the traffic. It halves
the instability, which confirms the diagnosis, but does not fix it.

**Why it still fails.** A WiFi channel is not one emitter. 2437 MHz carries an
access point *and its clients*, which are in different physical directions, plus
neighbouring networks on overlapping channels. Amplitude-comparing the channel
as a whole blends them, and the blend shifts with whoever is talking. There is
no single correct bearing for "the WiFi on channel 6" to converge to.

**What would actually be needed** to locate a specific access point:

1. Isolate one emitter by decoding 802.11 and filtering on BSSID/MAC, so only
   that device's frames are measured — beacons are ideal, being periodic,
   fixed-rate and from a fixed location. This is a demodulation job, not a
   spectrum-level one, and it is not implemented here.
2. Capture the full 20 MHz channel, which needs a master clock above the 16 MHz
   this board currently runs (it accepts up to 61.44 MHz).
3. Bearings from two or more separated positions, and a cut — a single fixed
   array gives direction, never position.

Steps 2 and 3 are straightforward; step 1 is the real work, and without it the
WiFi numbers above are the honest ceiling for this approach.

**Watch the gain.** 2.4 GHz overloaded the ADC at `--gain 50` (peak 1.41 of full
scale) and every level and bearing was wrong. The app warns; heed it. `--gain 25`
was clean.

## The honest limits

**Bearings are relative, not true north.** Without a calibration file the app
assumes the four antennas sit at 0/90/180/270° in port order. If they are
mounted in a different order or rotated, bearings are self-consistent but
rotated or scrambled. It says so at startup.

**A dead element used to rescale the whole array.** The boresights were spread
evenly over however many ports came out live, which is right only when every
element works. An array built as four at 0/90/180/270 with one dead does not
become three at 0/120/240 — the survivors are still bolted where they always
were — so every bearing came out rotated by an amount depending on which element
failed. Angles now come from each port's index in the *declared* array
(`azimuth_for_ports`), which is what the layout actually is. Found while port 4
was reading dead from the truth-table bug, which is exactly the case it matters
for.

**The absolute bearing depends on the assumed beamwidth.** Measured: 96 MHz
reads 137° at 60° beamwidth, 178° at 90°, 159° at 140°. Only the *pattern* of
bearings is meaningful until the antennas' real patterns are measured. Supply a
calibration JSON (`--cal`) with per-port `offsets` and `azimuths` to fix this.

**Four elements is few.** 90° spacing gives coarse angular discrimination; the
useful output is closer to a sector than a bearing. This was written when four
was all there was, then eight were fitted — and the interleaved runs below found
**no measurable stability difference between four and eight**. So it is fewer
independent constraints, but on this evidence not the thing holding the answer
back; the array is back to four plus the null port, which buys the noise
reference and the sync marker that eight could not provide.

**`--auto-balance` is a blind fix, not a calibration.** It equalises each
antenna's band-average level on the assumption that signals arrive from many
directions so the averages *should* match. Measured imbalance here was ~9.5 dB
between ch0 and ch3 — large enough that without this every bearing was dragged
toward the hottest port. It is the right default for a survey, and the wrong
thing if the emitters really are all in one direction.

**Locating a WiFi AP needs more than one bearing.** A single fixed array gives a
direction, not a position. Fixing a location needs bearings from two or more
separated points and a cut — either move the array and re-run, or run several
arrays. The stored `signals` documents carry what a triangulation step would
consume, but that step is not implemented.

## Storage

If MongoDB is unreachable the run continues and appends to
`sigmon_fallback.jsonl`, announcing the substitution at startup and in the
summary. A monitoring run that silently discards observations is worse than one
that refuses to start. `sigquery.py` reads either source with the same command.
