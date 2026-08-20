#!/usr/bin/env python3
"""Commutated-stream direction finding: four antennas, one null, one capture.

    ./dfstream.py 96M                     # bear 96 MHz, 100 ms record
    ./dfstream.py 96M --repeat 0          # keep going until Ctrl-C
    ./dfstream.py 96M --slot-us 120 --record-ms 300 --iq-out iq/

The hardware changed twice, and both changes point the same way.  There are now
FOUR antennas on the switch plus a fifth NO-SIGNAL position, and the switch
itself changes in well under a microsecond.

The old DF took one `capture()` per antenna: issue a stream command, wait for
the samples, tear the stream down, move the switch, repeat.  The per-capture
overhead -- not the dwell, and never the switch, which measured 0.026 ms --
set the cycle time.  With a sub-microsecond switch there is no reason to stop
receiving at all.

So: tune once, start ONE gapless capture, and commutate underneath it.  The
antennas become slices of a single continuous recording.  That is as fast as
the hardware can go: the only dead time left in a cycle is the null slot
itself, and the four antenna levels are separated by microseconds rather than
milliseconds.

The null position is what makes it work.
------------------------------------------------------------------------
Nothing tells the host WHEN a GPIO write actually reached the switch.  It
crosses USB with a latency of tens to hundreds of microseconds and jitters by a
comparable amount, so host time cannot be trusted to cut a 200 us slot out of
the sample stream -- the error is a whole slot wide.  Writing the switch inside
a continuous stream is only useful if the stream can be cut up correctly, and
timestamps cannot do it.

The no-signal position solves it in the data.  A port with nothing on it reads
the receiver's own noise floor, several dB below any live antenna, so it prints
a periodic DIP in the power envelope.  Find the dips and the recording is
self-clocking: whatever lies between two dips is exactly one 1-2-3-4 pass, and
sync is re-established every cycle instead of integrated from an assumed rate.
Switch jitter, USB latency, host scheduling and even a dropped sample block
move the dips with the data they corrupt.

The null slot is also a measurement, not just a marker.
------------------------------------------------------------------------
It reads the receiver noise, the LO leakage and any spur at THIS frequency,
THIS gain and THIS instant, on a chain otherwise identical to the antennas'.
Subtracting it in POWER from each antenna leaves the antenna's own
contribution.  Without that step a weak element does not read weak -- it reads
the receiver noise floor, which is the same on every port -- so the level
spread saturates and the fit is dragged toward equal levels, which is to say
toward no bearing at all.  This is the one thing the eight-antenna array could
not do, because it had no dead port to ask.
"""
import argparse
import os
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
# uarf sits beside this project, not two levels up. The older spelling of this
# resolved to <workspace>/../uarf and never imported anything; both are listed
# so an existing checkout that somehow relied on the other layout still works.
for _p in (os.path.join(os.path.dirname(HERE), "uarf"),
           os.path.join(os.path.dirname(os.path.dirname(HERE)), "uarf")):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

import dfcore                                           # noqa: E402


# ==========================================================================
# segmentation -- pure numpy, no hardware, so it can be tested from a file
# ==========================================================================
def detector_power(x, fs, offset_hz=None, rbw=None):
    """Instantaneous power at sample resolution, optionally band-limited.

    Two detectors, because the choice decides whether sync works at all.

    WIDEBAND (|x|^2) uses everything the antenna delivered across the whole
    sampled span.  At VHF that is dominated by ambient noise, which is well
    above the receiver's own, so a live port stands far above the null and the
    dips are obvious.

    NARROWBAND mixes the target to zero and box-car filters it to the RBW
    first, so the contrast that drives sync comes from the SIGNAL BEING BORNE
    rather than from the band.  This is the case wideband gets wrong: a station
    8 dB above the noise in 200 kHz is 8 - 10*log10(16 MHz / 200 kHz) = 11 dB
    BELOW it once smeared across the full span, so the wideband envelope is
    flat and sync fails on a signal that is perfectly measurable.

    Neither dominates -- wideband wins on a quiet band with a strong ambient,
    narrowband on a clean receiver with one loud carrier -- so analyse() builds
    both and keeps whichever actually separates the null from the antennas.

    The boxcar is a running mean over fs/rbw samples via a cumulative sum, so
    this is O(n) and stays out of the way of the point of the exercise.  Its
    cost is that it smears transitions by half its length, which is why the
    transition profile prefers the wideband detector.
    """
    if offset_hz is None or rbw is None or rbw >= fs / 4.0:
        return (x.real.astype(np.float64) ** 2 + x.imag.astype(np.float64) ** 2)
    L = int(np.clip(round(fs / rbw), 4, max(4, len(x) // 8)))
    n = np.arange(len(x), dtype=np.float64)
    bb = x * np.exp(-2j * np.pi * (offset_hz / fs) * n)
    cs = np.concatenate(([0.0 + 0j], np.cumsum(bb)))
    half = L // 2
    lo = np.clip(np.arange(len(x)) - half, 0, len(x))
    hi = np.clip(lo + L, 0, len(x))
    y = (cs[hi] - cs[lo]) / np.maximum(hi - lo, 1)
    return (y.real ** 2 + y.imag ** 2)


def block_mean_db(p, nblock):
    """Average a sample-rate power series into blocks, in dB.

    Short enough to resolve a slot boundary, long enough that the envelope is
    not itself noise.  ~16 blocks per slot is the useful range.
    """
    n = (len(p) // nblock) * nblock
    if n < nblock:
        return np.zeros(0)
    return 10.0 * np.log10(p[:n].reshape(-1, nblock).mean(axis=1) + 1e-30)


def block_power_db(x, nblock):
    """Wideband power envelope of a complex recording, one value per block."""
    return block_mean_db(detector_power(x, 1.0), nblock)


def find_null_runs(env_db, expect_blocks, min_contrast_db=3.0, low_frac=0.35):
    """Locate the no-signal slots as runs of low envelope.

    The threshold is set from the recording's own two levels rather than an
    absolute dBFS number: an antenna's level depends on the band, the gain and
    what is on the air, and none of that is known here.  The 8th percentile
    sits inside the null (which is a fifth of the cycle, less the transitions)
    and the 75th sits among the antennas.

    Returns (starts, stops, contrast_db).  `stops` is exclusive.  Runs whose
    length is nothing like a slot are dropped -- a fade on one antenna can dip
    below the threshold too, and accepting it would insert a phantom cycle
    boundary and scramble which slot belongs to which port.
    """
    if len(env_db) < 4:
        return np.zeros(0, int), np.zeros(0, int), 0.0
    lo = float(np.percentile(env_db, 8.0))
    hi = float(np.percentile(env_db, 75.0))
    contrast = hi - lo
    if contrast < min_contrast_db:
        return np.zeros(0, int), np.zeros(0, int), contrast

    below = env_db < (lo + low_frac * contrast)
    d = np.diff(np.concatenate(([0], below.view(np.int8), [0])).astype(np.int8))
    starts = np.flatnonzero(d == 1)
    stops = np.flatnonzero(d == -1)
    keep = ((stops - starts) >= max(2, 0.35 * expect_blocks)) & \
           ((stops - starts) <= 2.5 * expect_blocks)
    return starts[keep], stops[keep], contrast


def cycles_from_runs(starts, stops, expect_ant_blocks, tol=(0.6, 1.6)):
    """Pair consecutive null runs into cycles.

    Between the END of one null and the START of the next lie exactly the four
    antenna slots -- that is the whole content of a cycle.  A pair whose gap is
    not about the right size means a null was missed or invented, and the four
    slots cut out of it would be assigned to the wrong ports, so the pair is
    dropped rather than salvaged.

    Returns an (n_cycles, 4) array of block indices:
    (ant_lo, ant_hi, null_lo, null_hi), the null being the one that STARTS the
    cycle, so its noise reading is contemporaneous with the four antennas.
    """
    out = []
    for k in range(len(starts) - 1):
        a0, a1 = stops[k], starts[k + 1]
        w = a1 - a0
        if tol[0] * expect_ant_blocks <= w <= tol[1] * expect_ant_blocks:
            out.append((a0, a1, starts[k], stops[k]))
    return np.array(out, dtype=int).reshape(-1, 4)


def slot_bounds(cycles, nblock, n_ant=4, guard=0.25):
    """Sample ranges for every slot of every cycle, guard bands removed.

    The four antenna slots are the antenna region split in four.  They are
    assumed EQUAL in width, which the host's jittery writes do not quite
    deliver; the guard band is what covers that.  A quarter off each end throws
    away half the samples and is deliberately generous -- a slot that bleeds
    into its neighbour puts one antenna's power into another's level, which is
    a bearing error, while a shorter slot only costs SNR.

    Returns (n_cycles, n_ant + 1, 2); the last slot of each row is the null.
    """
    n = len(cycles)
    b = np.empty((n, n_ant + 1, 2), dtype=np.int64)
    for i, (a0, a1, n0, n1) in enumerate(cycles):
        w = (a1 - a0) / float(n_ant)
        for s in range(n_ant):
            lo = a0 + s * w
            b[i, s] = (round((lo + guard * w) * nblock),
                       round((lo + (1.0 - guard) * w) * nblock))
        nw = n1 - n0
        b[i, n_ant] = (round((n0 + guard * nw) * nblock),
                       round((n1 - guard * nw) * nblock))
    return b


def gather_slots(x, bounds, nfft_max=4096):
    """Stack every slot into one (n_slots, nfft) array for a single batched FFT.

    Doing this slot by slot is the obvious way and is what makes a Python DF
    slow: a hundred cycles is five hundred transforms.  One call over a
    rectangular array is roughly two orders of magnitude quicker, and the
    common length is set by the shortest slot so no slot is padded with
    fabricated samples.
    """
    flat = bounds.reshape(-1, 2)
    widths = flat[:, 1] - flat[:, 0]
    L = int(widths.min())
    if L < 64:
        return None, 0
    nfft = 1 << int(np.floor(np.log2(min(L, nfft_max))))
    S = np.empty((len(flat), nfft), dtype=np.complex64)
    for i, (lo, hi) in enumerate(flat):
        mid = (lo + hi) // 2                     # centre the window in the slot
        s = mid - nfft // 2
        S[i] = x[s:s + nfft]
    return S, nfft


def batch_band_power_db(S, fs, offset_hz, rbw):
    """Power in `rbw` around `offset_hz` for every row of S, in dB.

    Same normalisation as rfscan.welch_psd + band_power_db -- full-scale
    complex tone reads 0 dBFS -- so these levels are comparable with everything
    else in the app.
    """
    nfft = S.shape[1]
    w = np.hanning(nfft).astype(np.float32)
    F = np.fft.fft(S * w, axis=1)
    P = (F.real ** 2 + F.imag ** 2) / (fs * float(np.sum(w.astype(np.float64) ** 2)))
    f = np.fft.fftfreq(nfft, 1.0 / fs)
    sel = np.abs(f - offset_hz) <= rbw / 2.0
    if not sel.any():
        sel = np.zeros(nfft, bool)
        sel[int(np.argmin(np.abs(f - offset_hz)))] = True
    df = fs / nfft
    return 10.0 * np.log10(P[:, sel].sum(axis=1) * df + 1e-30), int(sel.sum())


def analyse(x, fs, offset_hz, rbw, slot_s, n_ant=4, guard=0.25,
            min_contrast_db=3.0, blocks_per_slot=16, nfft_max=4096):
    """Cut one commutated recording into per-cycle, per-antenna levels.

    Returns a dict.  `ok` is False with a `reason` when sync failed -- which is
    reported rather than papered over, because an unsynced recording still
    produces four numbers and they would look exactly like a bearing.
    """
    slot_n = max(8, int(round(slot_s * fs)))
    nblock = max(8, slot_n // blocks_per_slot)
    if len(x) < 6 * slot_n:
        return dict(ok=False, reason="recording shorter than a cycle",
                    contrast_db=0.0, n_cycles=0)

    exp_null = slot_n / float(nblock)
    # Sync is all-or-nothing -- a wrong cut gives four confident numbers that
    # are not the four antennas -- so both detectors are available rather than
    # guessing which one suits this band.  Wideband is tried FIRST and
    # narrowband only if it comes up empty: the band-limited one costs a
    # full-rate mix and running mean, 90 ms against 4 ms on a 100 ms record,
    # which would put the processing at the same order as the recording it is
    # meant to keep up with.  At VHF wideband nearly always wins anyway, so the
    # expensive path stays the exception.
    best = None
    for name, bw_hz in (("wideband", fs), ("narrowband", min(rbw, fs))):
        if best is not None and len(best[3]) >= 2:
            break
        det = detector_power(x, fs, None if name == "wideband" else offset_hz,
                             None if name == "wideband" else rbw)
        env = block_mean_db(det, nblock)
        # An envelope block averages nblock*B/fs independent looks, so it
        # scatters by 8.7/sqrt(looks) dB even on a perfectly flat input. A
        # threshold below that finds "nulls" in noise -- and they cut the
        # stream into four confident numbers that are not the four antennas.
        # The bar therefore rises with how noisy this detector's envelope is,
        # which is what makes the narrowband one safe to offer at all.
        looks = max(1.0, nblock * bw_hz / fs)
        need = max(min_contrast_db, 2.5 * 8.686 / np.sqrt(looks))

        # Where to put the threshold between the null and the antennas is not
        # knowable in advance: it depends on the contrast, which depends on the
        # band, the gain and what is on the air. A fixed fraction works at high
        # contrast and fails at moderate -- measured here, 96 MHz had 12 dB and
        # synced every cycle, while 103.6 MHz had 5.9 dB and recovered 10%,
        # because antenna fades reached below a threshold set for the deeper
        # case. So try a range and keep whatever recovers the most cycles.
        # There is no risk of tuning noise into a signal: the count is what is
        # being maximised, and spurious dips do not land a cycle apart.
        for frac in (0.20, 0.30, 0.35, 0.45, 0.55, 0.65):
            starts, stops, contrast = find_null_runs(env, exp_null, need,
                                                     low_frac=frac)
            cyc = (cycles_from_runs(starts, stops, n_ant * exp_null)
                   if len(starts) >= 2 else np.zeros((0, 4), int))
            if best is None or (len(cyc), contrast - need) > (len(best[3]),
                                                              best[4] - best[5]):
                best = (name, det, (starts, stops), cyc, contrast, need)
    detector, _det_p, (starts, stops), cycles, contrast, need = best

    if len(starts) < 2:
        return dict(
            ok=False, n_cycles=0, contrast_db=contrast, detector=detector,
            reason=(f"no null slots found: best envelope ({detector}) has "
                    f"{contrast:.1f} dB of contrast against the {need:.1f} dB "
                    f"this detector needs to be distinguishable from its own "
                    f"scatter. Either the switch is not commutating, the "
                    f"no-signal port is not actually dead, or nothing on any "
                    f"antenna stands above the receiver's own noise -- in which "
                    f"case there is no bearing to find either."))
    if len(cycles) < 1:
        return dict(ok=False, n_cycles=0, contrast_db=contrast,
                    detector=detector,
                    reason=f"{len(starts)} nulls found but none an intact cycle "
                           f"apart; --slot-us probably does not match what the "
                           f"host is achieving")

    # Two structural checks, because contrast alone can be cleared by chance.
    # A real commutation is REGULAR and it covers the whole recording; dips
    # that happen to sit four slots apart are neither.  Failing these means the
    # cut is not the switch, and four numbers from a wrong cut look exactly
    # like four antenna levels.
    ncent = (starts + stops) / 2.0
    gaps = np.diff(ncent)
    period_b = float(np.median(gaps))
    # Robust scale, not the standard deviation.  A MISSED null merges two
    # cycles into one gap of twice the length, and a single such gap drags a
    # plain std far past any sensible threshold even when every cycle that WAS
    # found is regular to a percent.  That is the wrong thing to fail on --
    # missed nulls are what `coverage` measures, and it measures them properly.
    # What this check is for is commutation that is genuinely irregular, i.e.
    # dips that are fades in the signal rather than the switch, and the median
    # absolute deviation sees that while ignoring the occasional double.
    # (1.4826 puts MAD on the same scale as a standard deviation.)
    mad = float(np.median(np.abs(gaps - period_b))) if len(gaps) else 0.0
    jitter = 1.4826 * mad / period_b if period_b > 0 else 1.0
    expect_cycles = len(x) / float((n_ant + 1) * slot_n)
    coverage = len(cycles) / max(expect_cycles, 1.0)
    if jitter > 0.30 or coverage < 0.40:
        # Say which test failed and what it implies, rather than listing every
        # cause every time. The two fail for different reasons and the fix is
        # different, so an undifferentiated message sends you looking in the
        # wrong place -- which is most of the cost of a failure like this.
        why = []
        if jitter > 0.30:
            why.append(f"the spacing between dips is irregular ({jitter*100:.0f}%"
                       f" MAD, need <30), so they are not all the null slot")
        if coverage < 0.40:
            why.append(f"only {coverage*100:.0f}% of the expected cycles were "
                       f"found (need >40), so most nulls were missed or "
                       f"--slot-us ({slot_s*1e6:.0f} us) does not match the "
                       f"real commutation")
        hint = (f"contrast is {contrast:.1f} dB on the {detector} envelope. "
                if contrast < 10.0 else "")
        if contrast < 10.0:
            hint += ("Below about 10 dB the antenna slots start dipping into "
                     "the null's range and get merged with it. Raising --gain "
                     "usually widens that margin: the null is receiver noise, "
                     "which grows more slowly with gain than the antennas do, "
                     "so LOW gain compresses the very contrast the sync needs. "
                     "Check --check-switch for the per-port levels.")
        return dict(
            ok=False, n_cycles=len(cycles), contrast_db=contrast,
            detector=detector, jitter=jitter, coverage=coverage,
            reason="; and ".join(why) + ". " + hint)

    bounds = slot_bounds(cycles, nblock, n_ant, guard)
    bounds[:, :, 0] = np.clip(bounds[:, :, 0], 0, len(x))
    bounds[:, :, 1] = np.clip(bounds[:, :, 1], 0, len(x))
    S, nfft = gather_slots(x, bounds, nfft_max)
    if S is None:
        return dict(ok=False, n_cycles=len(cycles), contrast_db=contrast,
                    reason="slots too short to transform; raise --slot-us")

    lv, nbins = batch_band_power_db(S, fs, offset_hz, rbw)
    lv = lv.reshape(len(cycles), n_ant + 1)
    raw = lv[:, :n_ant]
    null = lv[:, n_ant]

    # Measured, not assumed: the spacing between successive nulls IS the
    # achieved commutation period, jitter and all.
    period_s = period_b * nblock / fs

    return dict(
        ok=True, reason=None, n_cycles=len(cycles), contrast_db=contrast,
        jitter=jitter, coverage=coverage,
        raw_db=raw, null_db=null,
        levels_db=dfcore.subtract_null_db(raw, null[:, None]),
        cycle_s=period_s, nfft=nfft, nbins=nbins, detector=detector,
        slot_samples=int(np.median(bounds[:, 0, 1] - bounds[:, 0, 0])),
        n_avg=max(1, nbins),
        # kept so the caller can go back to the raw stream at sample
        # resolution -- transition_profile() needs exactly these
        cycles=cycles, nblock=nblock, bounds=bounds)


def slot_psd(x, fs, offset_hz, rbw, slot_s, n_ant=4, guard=0.25,
             min_contrast_db=3.0, nfft=None, nfft_max=4096):
    """Per-antenna mean PSD for a whole tuning segment, from one recording.

    analyse() asks one frequency of the recording; this asks all of them.  Same
    segmentation, same null slots, but each slot is transformed in full and the
    cycles are averaged per antenna, so one capture yields a spectrum for every
    antenna at once.

    That is what a band sweep wants.  Holding each antenna in turn and sweeping
    it costs one capture per antenna per segment, puts a whole sweep between
    antenna 0's reading of a frequency and antenna 3's -- and, on the esp32
    board, writes flash on every port change.  Commutating instead costs one
    capture per segment total, and the four levels at every frequency in the
    span come from the same milliseconds.

    Returns (freqs_baseband, P) with P of shape (n_ant + 1, nfft); the last row
    is the null, in the same power units as rfscan.welch_psd.
    """
    seg = analyse(x, fs, offset_hz, rbw, slot_s, n_ant=n_ant, guard=guard,
                  min_contrast_db=min_contrast_db, nfft_max=nfft_max)
    if not seg["ok"]:
        return None, None, seg

    S, n = gather_slots(x, seg["bounds"], nfft if nfft else nfft_max)
    if S is None:
        seg = dict(seg, ok=False, reason="slots too short to transform")
        return None, None, seg

    w = np.hanning(n).astype(np.float32)
    F = np.fft.fft(S * w, axis=1)
    P = (F.real ** 2 + F.imag ** 2) / (fs * float(np.sum(w.astype(np.float64) ** 2)))
    P = P.reshape(seg["n_cycles"], n_ant + 1, n).mean(axis=0)
    return np.fft.fftshift(np.fft.fftfreq(n, 1.0 / fs)), np.fft.fftshift(P, axes=1), seg


# ==========================================================================
# microsecond-scale detail -- what the recording can and cannot tell you
# ==========================================================================
# The stream is continuous and every sample is kept, so the time resolution of
# the RECORDING is one sample: 62.5 ns at 16 Msps, and 16 ns if the board is
# run at its 61.44 MHz limit.  Looking at how the signal changed over a couple
# of microseconds is therefore available now, from the same capture the
# bearing came out of -- that is what fine_envelope() below is for.
#
# Three separate things are NOT available at that scale, and they are worth
# keeping apart because they have different fixes:
#
#   1. LEVELS in a couple of microseconds.  A power estimate over time T in
#      bandwidth B has ~T*B independent looks and a scatter of 8.7/sqrt(T*B)
#      dB.  At 200 kHz RBW a 2 us window holds 0.4 looks, so its "level" is
#      about 14 dB of noise.  This is physics; the fix is a wider RBW, not a
#      faster anything.  dfcore.min_slot_seconds() is this rule.
#
#   2. COMMUTATING every couple of microseconds.  The switch is now
#      sub-microsecond; the host is the limit.  Measured on this machine, the
#      busy-waiting loop below holds its period to the microsecond down to
#      50 us slots -- median exactly on target, 95th percentile within 0.1 us
#      -- even with a 26 us GPIO write in the loop, which is what one
#      set_gpio_attr() costs.  Below ~50 us the write is most of the slot and
#      there is nothing left to schedule with.  A few writes per thousand are
#      late by 100-350 us when the OS preempts the thread; those cycles fail
#      the spacing check and are dropped rather than mismeasured.
#      So: tens of microseconds, yes.  A couple of microseconds needs the
#      writes timed in the FPGA -- UHD timed commands (set_command_time around
#      the GPIO write) or the ATR state machine -- which is a real path and is
#      not implemented here.
#
#   3. The receiver following a switch step in a couple of microseconds.  The
#      AD9361's decimation filters have tens of samples of group delay and its
#      DC-offset tracking loop actively chases the step at every commutation.
#      This one is measurable rather than assumed -- transition_profile()
#      averages the recording over every null->antenna edge and shows how long
#      the chain really takes to settle, which is what should set --guard and
#      the floor under --slot-us.


def fine_envelope(x, fs, res_us=0.25):
    """Power envelope at an arbitrary time resolution, in dB and microseconds.

    Returns (t_us, env_db).  res_us can go down to one sample; below about
    0.5 us the per-block value is dominated by the noise of having averaged
    only a handful of samples, so it is only meaningful averaged over many
    repetitions (see transition_profile) or for a strong signal.
    """
    n = max(1, int(round(res_us * 1e-6 * fs)))
    env = block_power_db(x, n)
    return np.arange(len(env)) * (n / fs) * 1e6, env


def _refine_edge(p, fs, approx, res_us=1.0, span_us=20.0, rising=True):
    """Sub-microsecond position of one switch transition, from the data.

    The coarse segmentation only knows the edge to within one envelope block.
    Interpolating the half-amplitude crossing of a 1 us envelope pins it far
    closer, which matters here: alignment error smears the averaged profile and
    would make the receiver look slower to settle than it is.
    """
    n = max(1, int(round(res_us * 1e-6 * fs)))
    half = int(span_us * 1e-6 * fs)
    lo = max(0, approx - half)
    hi = min(len(p), approx + half)
    if hi - lo < 4 * n:
        return None
    m = ((hi - lo) // n) * n
    lin = p[lo:lo + m].reshape(-1, n).mean(axis=1)
    if len(lin) < 4:
        return None
    a, b = lin[:max(1, len(lin) // 4)].mean(), lin[-max(1, len(lin) // 4):].mean()
    if (b > a) != bool(rising) or abs(b - a) < 1e-30:
        return None
    mid = 0.5 * (a + b)
    cross = np.flatnonzero((lin[:-1] < mid) != (lin[1:] < mid))
    if not len(cross):
        return None
    i = int(cross[len(cross) // 2])
    d = lin[i + 1] - lin[i]
    frac = 0.0 if abs(d) < 1e-30 else float(np.clip((mid - lin[i]) / d, 0.0, 1.0))
    return lo + (i + frac) * n


def transition_profile(x, fs, cycles, nblock, res_us=0.25, span_us=40.0,
                       n_max=400, det_p=None):
    """Average the recording through every no-signal -> antenna transition.

    Every cycle contains one, at a known-to-within-a-block position, and they
    are all the same event: the switch steps from a dead port to a live one.
    Aligning them at sub-microsecond precision and averaging turns a 0.25 us
    envelope -- far too noisy to read on its own -- into a clean picture of how
    the receive chain actually responds to a step, on this board, at this rate
    and this gain.

    Read it for two numbers: how long until the level is flat (that is the
    floor under --guard and under --slot-us), and whether it OVERSHOOTS, which
    is the AD9361's DC-offset tracking loop reacting to the step rather than
    anything in the RF.

    Returns (t_us, profile_db, n_used) with t = 0 at the transition.
    """
    if not len(cycles):
        return np.zeros(0), np.zeros(0), 0
    # Wideband by default: the band-limited detector's boxcar is fs/rbw samples
    # long and would smear the very edge being measured, reporting the filter's
    # settling time as the receiver's.
    p = detector_power(x, fs) if det_p is None else det_p
    res_n = max(1, int(round(res_us * 1e-6 * fs)))
    half_n = int(span_us * 1e-6 * fs)
    nb = 2 * (half_n // res_n)
    if nb < 4:
        return np.zeros(0), np.zeros(0), 0
    acc = np.zeros(nb)
    used = 0
    for a0 in cycles[:n_max, 0]:
        e = _refine_edge(p, fs, int(a0) * nblock, rising=True)
        if e is None:
            continue
        s = int(round(e)) - (nb // 2) * res_n
        if s < 0 or s + nb * res_n > len(p):
            continue
        acc += p[s:s + nb * res_n].reshape(nb, res_n).mean(axis=1)
        used += 1
    if not used:
        return np.zeros(0), np.zeros(0), 0
    t = (np.arange(nb) - nb // 2) * (res_n / fs) * 1e6
    return t, 10.0 * np.log10(acc / used + 1e-30), used


def settling_us(t_us, prof_db, tol_db=1.0):
    """Time from the transition until the level stays within tol of final.

    Deliberately "stays within", not "first enters": a ringing or overshooting
    response crosses the band on its way through and would otherwise be
    credited with settling before it had.

    The tolerance is widened to whatever scatter the profile ITSELF has, and
    that is not a refinement -- without it this function reports the window
    edge and nothing else.  The profile is an average of a few dozen 0.25 us
    blocks, so it wobbles by several tenths of a dB all the way out; "the last
    time it left a fixed 1 dB band" then just finds the last piece of noise.
    Measured on this array it returned 23.5 us at a 200 us slot and 39.25 us at
    800 us -- growing with the window rather than describing the hardware --
    while the real response was within 1 dB of settled by 5 us and the true
    transient was a 2-8 dB spike lasting under 2 us.

    So: measure the late-window scatter, require the excursion to be bigger
    than that before it counts.
    """
    if len(t_us) < 8:
        return float("nan")
    # Smooth to about 1 us first. Each profile point is a 0.25 us block
    # averaged over a few dozen edges and still scatters by ~0.5 dB (2.3 dB
    # peak to peak, measured); at that level individual points cross any useful
    # tolerance all the way out, and the answer becomes "wherever the window
    # happened to end".
    res = float(t_us[1] - t_us[0]) if len(t_us) > 1 else 1.0
    w = max(1, int(round(1.0 / max(res, 1e-6))))
    if w > 1:
        k = np.ones(w) / w
        prof_db = np.convolve(prof_db, k, mode="same")
        edge = w  # the convolution is unreliable within a window of each end
        t_us, prof_db = t_us[edge:-edge], prof_db[edge:-edge]

    post = prof_db[t_us > 0]
    tp = t_us[t_us > 0]
    if len(post) < 8:
        return float("nan")
    tail = post[-max(4, len(post) // 3):]
    final = float(np.median(tail))
    tol = max(tol_db, 4.0 * float(np.std(tail)))
    bad = np.flatnonzero(np.abs(post - final) > tol)
    if not len(bad):
        return float(tp[0])
    return float(tp[min(bad[-1] + 1, len(tp) - 1)])


def check_switch(rx, backend, freq, ports, rbw=200e3, passes=4, nsamps=200000,
                 settle=0.05, min_db=2.0):
    """Does this controller actually move the RF switch?

    Everything downstream assumes it does, and nothing downstream can tell.  A
    switch that is unpowered, unwired, or listening to a different controller
    returns the same antenna on every port -- which is not an error anywhere:
    the levels are real, the capture is clean, the four "antennas" simply agree
    perfectly.  Amplitude DF on that produces a bearing.  It is meaningless and
    it is stable, which is the worst combination.

    The ports are visited INTERLEAVED and several times over, because the
    obvious version of this test -- walk the ports once and look at the spread
    -- cannot tell a switch from a drift.  Measured here on a chain with the
    switch not connected at all, a single pass over eight ports gave 1.9 dB of
    "spread" that was really the receiver settling, monotonically, over the two
    seconds the pass took.  Repeating the pass separates them: a real port
    difference repeats, drift does not.

    Returns a dict with the per-port means, the between-port spread, the
    within-port scatter, and a verdict.
    """
    centre = rx.tune(freq)
    time.sleep(0.2)
    centre = rx.usrp.get_rx_freq(rx.chan)
    ports = list(ports)
    obs = {p: [] for p in ports}
    for _ in range(max(2, passes)):
        for p in ports:
            backend.hold(p)
            time.sleep(settle)
            x = rx.capture(nsamps)
            if len(x) < nsamps // 2:
                continue
            f, P = rfscan_welch(x, rx.rate, rbw)
            obs[p].append(band_power(f, P, freq - centre, rbw))

    means = {p: float(np.mean(v)) if v else float("nan") for p, v in obs.items()}
    # Within-port scatter measured on DIFFERENCES from each pass's own mean, so
    # a slow drift common to every port does not inflate it and hide a real
    # response.
    resid = []
    for k in range(max(len(v) for v in obs.values())):
        row = [obs[p][k] for p in ports if len(obs[p]) > k]
        if len(row) == len(ports):
            row = np.array(row) - np.mean(row)
            resid.append(row)
    resid = np.array(resid) if resid else np.zeros((1, len(ports)))
    within = float(np.mean(np.std(resid, axis=0))) if len(resid) > 1 else float("nan")
    between = float(np.ptp([means[p] for p in ports]))

    responds = (between > min_db and np.isfinite(within) and between > 3.0 * within)
    return dict(ports=ports, means=means, between_db=between, within_db=within,
                passes=len(resid), responds=bool(responds),
                backend=backend.name, freq_hz=float(freq))


def rfscan_welch(x, fs, rbw):
    import rfscan
    return rfscan.welch_psd(x, fs, rbw)


def band_power(f, P, centre, rbw):
    import rfscan
    return rfscan.band_power_db(f, P, centre, rbw)


def freeze_dc_offset(rx):
    """Stop the AD9361 tracking its DC offset while the switch is commutating.

    The tracking loop is right for a stationary input and wrong for this one:
    the port changes every slot, so the loop spends the recording chasing steps
    it will never catch and injects a settling transient at every transition --
    into the same first microseconds of each slot that the guard band is then
    forced to discard.  Turning it off costs a fixed DC term, which lands at
    the LO and is already tuned out of the analysed band by lo_frac.
    """
    try:
        rx.usrp.set_rx_dc_offset(False, rx.chan)
        return True
    except Exception:                                           # noqa: BLE001
        return False


# ==========================================================================
# hardware
# ==========================================================================
class Commutator(threading.Thread):
    """Walks the switch through the sequence in a tight busy-waiting loop.

    Busy-waits rather than sleeps because `time.sleep` cannot resolve a 200 us
    slot -- its granularity on Linux is 50 us at best and routinely worse, so
    sleeping would make the slots wildly unequal.  This pins one core for the
    length of the recording, which for a 100 ms record is a fair trade and for
    a multi-second one is not; keep records short and repeat them.

    Write times are recorded for DIAGNOSTICS only.  They say how fast the host
    actually managed to commutate, which is worth knowing, but they are not
    used to cut up the samples -- the null dips do that, because host time has
    no fixed relationship to when the RF actually changed.
    """

    def __init__(self, sw, sequence, slot_s, max_writes):
        super().__init__(daemon=True)
        self.sw, self.sequence, self.slot_s = sw, list(sequence), float(slot_s)
        self.times = np.zeros(int(max_writes), dtype=np.float64)
        self.n_writes = 0
        self._stop = threading.Event()

    def run(self):
        seq, n = self.sequence, len(self.sequence)
        slot, perf, times = self.slot_s, time.perf_counter, self.times
        select, cap = self.sw.select, len(times)
        stop = self._stop
        i = 0
        t_next = perf()
        while not stop.is_set() and i < cap:
            select(seq[i % n])
            times[i] = perf()
            i += 1
            t_next += slot
            while perf() < t_next:
                if stop.is_set():
                    break
        self.n_writes = i

    def stop(self, park=None):
        self._stop.set()
        self.join(timeout=2.0)
        if park is not None:
            try:
                self.sw.select(park)
            except Exception:                                   # noqa: BLE001
                pass

    def achieved_slot_s(self):
        t = self.times[:self.n_writes]
        return float(np.median(np.diff(t))) if len(t) > 2 else float("nan")


def record(rx, backend, freq, sequence, slot_s, record_s, tune_settle=0.01,
           park=None, verify=True):
    """Tune once, commutate, and take ONE gapless capture across the lot.

    `backend` is a swbackend.SwitchBackend -- either the B210's own GPIO driven
    from a host thread, or the esp32 board free-running on its own.  With the
    esp32 already iterating, begin_cycle() sends nothing and this reduces to
    exactly one capture: no host in the commutation loop at all.

    `park=None` leaves the switch running afterwards, which is what you want
    when the next thing is another DF -- stopping and restarting costs a serial
    round trip and, on the esp32, a flash write.
    """
    rx.tune(freq)
    time.sleep(tune_settle)
    centre = rx.usrp.get_rx_freq(rx.chan)

    nsamps = int(record_s * rx.rate)
    ov0 = rx.overflows
    backend.begin_cycle(sequence, slot_s)
    # A few cycles of run-up so the capture opens mid-commutation rather than
    # on a stationary port.
    time.sleep(max(3.0 * slot_s * len(sequence), 1e-4))

    # The device's own step counter, read either side of the capture, is the
    # one check that the lines actually MOVED. Without it a switch that has
    # stopped -- crashed firmware, unplugged board, a `port` command from
    # somewhere else -- still yields a full recording of one antenna, and the
    # segmentation would report low contrast rather than the real cause.
    s0 = backend.steps() if verify else None
    t0 = time.time()
    x = rx.capture(nsamps)
    wall = time.time() - t0
    s1 = backend.steps() if verify else None
    backend.end_cycle(park=park)

    info = dict(centre_hz=float(centre), samples=len(x), wall_s=wall,
                # An overflow DROPS samples, so the stream is no longer a
                # continuous timeline. The null dips resync each cycle, so the
                # measurement usually survives -- but it is reported, because a
                # silently shortened record is a silently wrong commutation
                # rate.
                overflows=rx.overflows - ov0,
                switch=backend.name,
                achieved_slot_s=backend.achieved_slot_s())
    if s0 is not None and s1 is not None:
        info["switch_steps"] = s1 - s0
        info["switch_moving"] = s1 > s0
    return x, info


def achieved_slot(slot_us, info):
    """The slot length to segment against: what the host managed, not what it
    was asked for.

    A busy-waiting loop cannot run FASTER than requested, only slower, and it
    goes slower whenever the machine is loaded or the requested slot approaches
    the ~26 us a GPIO write costs.  Segmenting against the requested figure
    then looks for cycles at the wrong scale and the tolerance windows reject
    them -- a sync failure caused by an argument rather than by the radio.  The
    commutator recorded when it actually wrote, so use that.

    Only used to SIZE the search.  Where the slots fall is still decided by the
    null dips, because host time never says when the RF changed.
    """
    got = info.get("achieved_slot_s")
    if got and np.isfinite(got) and got > 0:
        return float(got)
    return slot_us * 1e-6


def measure(rx, backend, freq, ports, null_port, azimuths, beamwidth, a,
            offsets=None, balance=None, iq_out=None):
    """One full DF at one frequency: record, segment, level, bear.

    Two bearings come out and they answer different questions.  The AGGREGATE
    one averages the cycles in power first and fits once, which is the best
    estimate the recording supports.  The per-cycle spread is the honest
    uncertainty of that estimate, and it is a real distribution over hundreds
    of independent looks rather than a handful of sweeps -- so for the first
    time it can be believed after a single measurement.
    """
    seq = list(ports) + [null_port]
    x, info = record(rx, backend, freq, seq, a.slot_us * 1e-6,
                     a.record_ms * 1e-3, tune_settle=a.tune_settle,
                     park=getattr(a, "park", None),
                     verify=getattr(a, "verify_switch", True))
    if len(x) < 1024:
        return dict(ok=False, reason=f"short capture ({len(x)} samples)", **info)
    if info.get("switch_moving") is False:
        return dict(ok=False, **info, reason=(
            f"the switch did not move during the capture (step counter "
            f"unchanged at {info.get('switch_steps', 0)}). Every slot in this "
            f"recording is the same antenna, so there is nothing to compare."))

    seg = analyse(x, rx.rate, freq - info["centre_hz"], a.rbw,
                  achieved_slot(a.slot_us, info), n_ant=len(ports),
                  guard=a.guard, min_contrast_db=a.min_contrast)
    out = dict(info)
    out.update({k: v for k, v in seg.items()
                if k not in ("raw_db", "null_db", "levels_db",
                             "cycles", "nblock", "bounds")})
    if not seg["ok"]:
        out["ok"] = False
        return out

    lv = seg["levels_db"]                            # (n_cycles, n_ant)
    if offsets is not None:
        lv = lv - np.asarray(offsets, float)[None, :]
    if balance is not None:
        lv = lv - np.asarray(balance, float)[None, :]

    # Average POWER across cycles, not dB. A dB average is the geometric mean
    # and is pulled down by the deepest fade rather than tracking the mean
    # power the antenna actually received.
    agg = 10.0 * np.log10(np.mean(10.0 ** (lv / 10.0), axis=0))
    snr = agg - agg.min()
    n_avg = seg["n_avg"] * seg["n_cycles"]
    bearing, resid, grid = dfcore.estimate_bearing(agg, azimuths, beamwidth,
                                                  snr, n_avg=max(n_avg, 1))
    conf = dfcore.bearing_confidence(resid, grid)

    # Weights fixed from the aggregate SNR: per-cycle weights would jitter with
    # the very noise the spread is trying to measure, and inflate it.
    w = 1.0 / np.maximum(dfcore.sigma_from_snr(snr, seg["n_avg"]), 1e-6) ** 2
    per_cycle = dfcore.estimate_bearings_batch(lv, azimuths, beamwidth, w=w)

    out.update(dict(
        ok=True, freq_hz=float(freq), bearing_deg=bearing, confidence=conf,
        levels_db=agg.tolist(), snr_db=snr.tolist(),
        null_db=float(np.mean(seg["null_db"])),
        raw_db=(10.0 * np.log10(np.mean(10.0 ** (seg["raw_db"] / 10.0), axis=0))).tolist(),
        cycle_std_deg=dfcore.circ_std_deg(per_cycle) if len(per_cycle) > 2 else None,
        cycle_mean_deg=dfcore.circ_mean_deg(per_cycle) if len(per_cycle) > 2 else None,
        per_cycle_deg=per_cycle.tolist(),
        resid=resid, grid=grid, azimuths_deg=list(np.asarray(azimuths, float)),
        beamwidth_deg=float(beamwidth), channels=list(ports)))

    if iq_out:
        out["iq_path"] = _save_iq(iq_out, x, freq, rx.rate, seq, a, out)
    return out


def _save_iq(directory, x, freq, fs, seq, a, out):
    """Raw recording plus what is needed to segment it again offline."""
    import json
    os.makedirs(directory, exist_ok=True)
    stem = os.path.join(directory, f"df_{int(freq)}_{int(time.time()*1e3)}")
    x.astype(np.complex64).tofile(stem + ".c64")
    with open(stem + ".json", "w") as f:
        json.dump(dict(freq_hz=float(freq), centre_hz=out["centre_hz"],
                       rate_hz=float(fs), sequence=list(seq),
                       slot_us=a.slot_us, guard=a.guard, rbw_hz=a.rbw,
                       n_samples=int(len(x)), dtype="complex64",
                       cycle_s=out.get("cycle_s"),
                       overflows=out.get("overflows")), f, indent=2)
    return stem + ".c64"


# ==========================================================================
# CLI
# ==========================================================================
def open_usrp():
    import rfscan
    import uhd
    # rfscan.default_fpga() reads the marker relative to ITSELF, which is where
    # detect_fpga.sh writes it. Spelling the path relative to this file instead
    # is what the other entry points did, and they were pointing one directory
    # too high.
    fpga = rfscan.default_fpga()
    dev = "type=b200" + (f",fpga={fpga}" if fpga else "")
    try:
        return uhd.usrp.MultiUSRP(dev), rfscan
    except RuntimeError as e:
        sys.exit(f"error: could not open the radio.\n  {e}\n"
                 "  The B210 is single-session -- stop webui.py / sigmon.py "
                 "/ rfscan.py / GNU Radio first.")


def _check_switch_cli(rx, backend, ports, a):
    probe = list(ports) + [a.null_port]
    print(f"\n[dfstream] holding each of {probe} {a.check_passes} times over at "
          f"{a.freq/1e6:.3f} MHz ...")
    r = check_switch(rx, backend, a.freq, probe, rbw=a.rbw,
                     passes=a.check_passes)
    lo = min(r["means"].values())
    print(f"\n  {'port':>6} {'mean dBFS':>11} {'rel':>7}")
    for p in probe:
        tag = "  (declared no-signal)" if p == a.null_port else ""
        print(f"  {p:>6} {r['means'][p]:>11.1f} {r['means'][p]-lo:>+7.1f}{tag}")
    print(f"\n  between-port spread : {r['between_db']:6.1f} dB")
    print(f"  within-port scatter : {r['within_db']:6.1f} dB  "
          f"(over {r['passes']} passes)")

    if r["responds"]:
        print(f"\n  -> the switch RESPONDS to the {r['backend']} controller.")
        nl = r["means"][a.null_port]
        below = min(r["means"][p] for p in ports) - nl
        print(f"     The no-signal port sits {below:+.1f} dB below the quietest "
              f"antenna;\n     commutated sync needs it at least 3 dB below.")
        if below < 3.0:
            print("     That is not enough margin -- check that port really is "
                  "unconnected.")
    else:
        print(f"\n  -> the switch DOES NOT RESPOND to the {r['backend']} "
              f"controller.")
        print("     Every port returns the same signal, so there is nothing to")
        print("     amplitude-compare. A bearing fitted to this would be")
        print("     meaningless and perfectly repeatable, which is why this")
        print("     check exists. Look at, in order: whether the switch has")
        print("     power; whether its control lines go to this controller;")
        print("     and whether its RF common is really what feeds the SDR.")
    return 0 if r["responds"] else 2


def _probe_transition(rx, backend, ports, a):
    """Measure the receive chain's step response through the real switch.

    This is the measurement that decides how fast the commutation can honestly
    go.  Everything else about the timing is either known (the switch, now
    sub-microsecond) or a host-side number that can be watched directly (the
    GPIO write rate); what is neither is how long the AD9361 takes to present a
    settled level after the port changes underneath it.
    """
    seq = list(ports) + [a.null_port]
    x, info = record(rx, backend, a.freq, seq, a.slot_us * 1e-6,
                     a.record_ms * 1e-3, tune_settle=a.tune_settle)
    seg = analyse(x, rx.rate, a.freq - info["centre_hz"], a.rbw,
                  achieved_slot(a.slot_us, info), n_ant=len(ports),
                  guard=a.guard, min_contrast_db=a.min_contrast)

    print(f"\n[dfstream] {len(x)} samples "
          f"({len(x)/rx.rate*1e3:.1f} ms) via {info['switch']}, slot "
          f"{info['achieved_slot_s']*1e6:.1f} us (asked {a.slot_us:.0f})"
          + (f", {info['switch_steps']} switch steps"
             if "switch_steps" in info else "")
          + (f", OVERFLOW x{info['overflows']}" if info["overflows"] else ""))
    if not seg["ok"]:
        print(f"[dfstream] cannot profile: {seg['reason']}")
        return 1
    print(f"[dfstream] synced on {seg['n_cycles']} cycles, "
          f"envelope contrast {seg['contrast_db']:.1f} dB, measured cycle "
          f"{seg['cycle_s']*1e6:.1f} us ({1e-3/seg['cycle_s']:.2f} kcycle/s)")

    t, prof, n = transition_profile(x, rx.rate, seg["cycles"], seg["nblock"],
                                    res_us=a.res_us, span_us=a.span_us)
    if not n:
        print("[dfstream] no clean transitions to average")
        return 1
    st = settling_us(t, prof, tol_db=1.0)
    base = float(np.median(prof[t < -a.span_us / 4]))
    final = float(np.median(prof[t > a.span_us / 2]))
    peak = float(np.max(prof[(t > 0) & (t < a.span_us)]))

    print(f"\n  no-signal -> antenna step, averaged over {n} transitions, "
          f"{a.res_us:.2f} us resolution")
    print(f"  {'t (us)':>9}  {'dBFS':>8}   profile")
    lo, hi = min(base, final) - 1.0, max(peak, final) + 1.0
    for i in range(0, len(t), max(1, len(t) // 48)):
        bar = int(np.clip((prof[i] - lo) / max(hi - lo, 1e-9), 0, 1) * 46)
        print(f"  {t[i]:>9.2f}  {prof[i]:>8.1f}   "
              f"{'|' if abs(t[i]) < a.res_us else ' '}{'#'*bar}")
    print(f"\n  null level      {base:8.1f} dBFS")
    print(f"  settled level   {final:8.1f} dBFS  (step {final-base:+.1f} dB)")
    print(f"  peak after step {peak:8.1f} dBFS  (overshoot {peak-final:+.1f} dB)")
    print(f"  settles to within 1 dB by {st:.2f} us after the transition")
    print()
    print(f"  -> a guard of {st:.1f} us at the start of each slot is enough; "
          f"--guard {a.guard:.2f} currently discards "
          f"{a.slot_us*a.guard:.1f} us.")
    print(f"  -> the shortest honest slot is about "
          f"{max(st*2, dfcore.min_slot_seconds(a.rbw)*1e6):.0f} us at this RBW: "
          f"{st:.1f} us of settling plus enough time to measure a level.")
    if peak - final > 1.5:
        print("  -> the overshoot is the DC-offset tracking loop reacting to "
              "the step.\n     Run without --track-dc (the default) if you "
              "have not already.")

    if a.env_out:
        te, env = fine_envelope(x, rx.rate, a.res_us)
        np.savetxt(a.env_out, np.column_stack([te, env]), fmt="%.4f",
                   delimiter=",", header="t_us,dBFS", comments="")
        print(f"\n  full envelope ({len(env)} points at {a.res_us} us) "
              f"-> {a.env_out}")
    try:
        sw.select(a.null_port)
    except Exception:                                           # noqa: BLE001
        pass
    return 0


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("freq", help="frequency to bear, e.g. 96M")
    p.add_argument("--switch", default="auto", choices=("auto", "esp32", "usrp"),
                   help="who moves the switch. 'esp32' free-runs on its own "
                        "board at microsecond dwells; 'usrp' drives the B210's "
                        "GPIO from a host thread and cannot go below ~50 us. "
                        "'auto' prefers the esp32 and says so if it falls back.")
    p.add_argument("--switch-device", default="/dev/ttyACM0",
                   help="serial device of the esp32 switch board")
    p.add_argument("--hold", type=int, default=None, metavar="PORT",
                   help="select one antenna and exit -- manual control, no DF")
    p.add_argument("--ports", default=None,
                   help="switch codes for the four antennas, in array order "
                        "(default: 1,2,3,4 on the esp32, 0,1,2,3 on the usrp)")
    p.add_argument("--null-port", type=int, default=None,
                   help="switch code of the NO-SIGNAL position (default: 5 on "
                        "the esp32, 4 on the usrp). It is the sync marker and "
                        "the noise reference; the whole method needs it to be "
                        "genuinely dead.")
    p.add_argument("--no-verify-switch", action="store_false",
                   dest="verify_switch",
                   help="skip reading the device's step counter around each "
                        "capture. That check is what distinguishes a stopped "
                        "switch from a weak signal.")
    p.add_argument("--slot-us", type=float, default=200.0,
                   help="microseconds per switch position. The switch itself is "
                        "sub-microsecond and the host loop holds its period to "
                        "the microsecond down to 50 us, so the real floor is "
                        "the receiver's settling (--probe-transition measures "
                        "it) and the RBW (a short slot cannot carry a level).")
    p.add_argument("--record-ms", type=float, default=100.0,
                   help="length of the single continuous capture")
    p.add_argument("--guard", type=float, default=0.25,
                   help="fraction discarded at each end of every slot")
    p.add_argument("--min-contrast", type=float, default=3.0,
                   help="dB the antennas must stand above the null slot for "
                        "sync to be trusted")
    p.add_argument("--rate", type=float, default=16e6)
    p.add_argument("--lo-frac", type=float, default=0.25, dest="lo_frac",
                   help="LO offset as a fraction of the rate. At 0.25 the LO "
                        "leakage and its Nyquist alias coincide at -Fs/4, "
                        "leaving |f| < Fs/4 clean.")
    p.add_argument("--rbw", type=float, default=200e3)
    p.add_argument("--gain", type=float, default=30.0)
    p.add_argument("--antenna", default="TX/RX")
    p.add_argument("--rx-chan", type=int, default=0)
    p.add_argument("--gpio-mask", type=lambda s: int(s, 0), default=0xE0)
    p.add_argument("--tune-settle", type=float, default=0.01)
    p.add_argument("--beamwidth", type=float, default=None)
    p.add_argument("--counter-clockwise", "--ccw", action="store_true",
                   dest="counter_clockwise")
    p.add_argument("--array-offset", type=float, default=0.0)
    p.add_argument("--cal", default=None)
    p.add_argument("--repeat", type=int, default=1, help="0 = until Ctrl-C")
    p.add_argument("--iq-out", default=None,
                   help="directory to write each raw recording into")
    p.add_argument("--track-dc", action="store_true",
                   help="leave the AD9361 DC-offset tracking loop ON. Off by "
                        "default: it chases every switch step and puts a "
                        "settling transient at the start of every slot.")
    p.add_argument("--probe-transition", action="store_true",
                   help="measure how the receive chain responds to a switch "
                        "step, averaged over every cycle, at --res-us "
                        "resolution. This is what says how short a slot and "
                        "how small a guard the hardware will actually stand.")
    p.add_argument("--res-us", type=float, default=0.25,
                   help="time resolution of the transition profile (us)")
    p.add_argument("--span-us", type=float, default=40.0,
                   help="how far either side of the transition to profile (us)")
    p.add_argument("--env-out", default=None,
                   help="write the full fine-resolution power envelope of one "
                        "recording to this CSV (t_us, dBFS)")
    p.add_argument("--check-switch", action="store_true",
                   help="verify that the selected controller actually moves the "
                        "RF switch, by holding each port several times over and "
                        "seeing whether the level responds. Run this FIRST on "
                        "any new wiring: a switch that is not connected gives "
                        "four identical antennas, and a bearing fitted to those "
                        "is meaningless AND perfectly stable.")
    p.add_argument("--check-passes", type=int, default=4)
    a = p.parse_args()

    import swbackend

    # The switch board is independent of the radio, so a manual antenna
    # selection needs neither -- and asking for the B210 would fail whenever
    # webui.py already holds it, which is exactly when you want to poke the
    # switch by hand.
    if a.hold is not None:
        b = swbackend.open_backend(a.switch, device=a.switch_device,
                                   auto=(a.switch == "auto"))
        b.hold(a.hold)
        print(f"[dfstream] {b.describe()}: holding port {a.hold}")
        b.close()
        return 0

    usrp, rfscan = open_usrp()
    a.freq = rfscan.parse_freq(a.freq)
    backend = swbackend.open_backend(a.switch, usrp=usrp,
                                     device=a.switch_device,
                                     gpio_mask=a.gpio_mask,
                                     auto=(a.switch == "auto"))
    ports = ([int(c) for c in a.ports.split(",")] if a.ports
             else list(backend.default_ports))
    if a.null_port is None:
        a.null_port = backend.default_null

    rx = rfscan.Receiver(usrp, a.rx_chan, a.rate, a.gain, a.antenna,
                         lo_frac=a.lo_frac)
    if not a.track_dc:
        freeze_dc_offset(rx)
    cal = dfcore.Calibration.load(a.cal)
    # From each port's place in the DECLARED array, not from how many happen
    # to be live -- a dead element must not rescale the geometry.
    az = cal.azimuth_for_ports(ports, ports,
                               counter_clockwise=a.counter_clockwise,
                               offset_deg=a.array_offset)
    bw = a.beamwidth or 0.7 * (360.0 / len(ports))
    offsets = np.array([cal.offsets.get(c, 0.0) for c in ports])

    print(f"[dfstream] {a.freq/1e6:.4f} MHz, rate {rx.rate/1e6:.3f} Msps, "
          f"gain {a.gain:.0f} dB")
    print(f"[dfstream] switch: {backend.describe()}")
    print(f"[dfstream] sequence {ports} then null port {a.null_port}, "
          f"{a.slot_us:.0f} us/slot -> {len(ports)+1} slots = "
          f"{(len(ports)+1)*a.slot_us:.0f} us/cycle nominal")
    rec_us, why = swbackend.recommend_slot_us(a.rbw, a.guard)
    print(f"[dfstream] shortest useful slot here is {rec_us:.0f} us "
          f"({why}-limited)"
          + ("" if backend.name != "esp32" else
             " -- the esp32 would go to 1 us, the receiver will not"))
    print(f"[dfstream] azimuths {np.round(az,1).tolist()} deg, "
          f"beamwidth {bw:.0f} deg"
          + ("" if cal.calibrated else "   (NO CALIBRATION: bearings are relative)"))
    print(f"[dfstream] sample period {1e9/rx.rate:.1f} ns, so the recording "
          f"resolves microsecond detail; a LEVEL does not")

    # Time-bandwidth, not an implementation limit: a slot shorter than a few
    # cycles of the RBW cannot carry a level, however fast the switch is.
    need_us = dfcore.min_slot_seconds(a.rbw) * 1e6 / (1.0 - 2.0 * a.guard)
    if a.slot_us < need_us:
        print(f"[dfstream] WARNING: {a.slot_us:.0f} us slots at {a.rbw/1e3:.0f} kHz "
              f"RBW leave {(a.slot_us*(1-2*a.guard))*a.rbw*1e-6:.1f} independent "
              f"looks per level.")
        print(f"           Scatter will be about "
              f"{8.686/np.sqrt(max(a.slot_us*(1-2*a.guard)*a.rbw*1e-6, 1e-3)):.0f} dB "
              f"per cycle. Widen --rbw to about "
              f"{4e6/(a.slot_us*(1-2*a.guard))/1e3:.0f} kHz, or use "
              f"--slot-us {need_us:.0f}.")

    if a.check_switch:
        try:
            return _check_switch_cli(rx, backend, ports, a)
        finally:
            backend.close()

    if a.probe_transition:
        try:
            return _probe_transition(rx, backend, ports, a)
        finally:
            backend.end_cycle(park=a.null_port)
            backend.close()

    hist = []
    npass = 0
    try:
        while a.repeat == 0 or npass < a.repeat:
            npass += 1
            r = measure(rx, backend, a.freq, ports, a.null_port, az, bw, a,
                        offsets=offsets, iq_out=a.iq_out)
            if not r.get("ok"):
                print(f"  pass {npass}: {r.get('reason')}")
                continue
            hist.append(r["bearing_deg"])
            rate = 1.0 / r["cycle_s"] if r["cycle_s"] and np.isfinite(r["cycle_s"]) else float("nan")
            print(f"  pass {npass}: {r['bearing_deg']:6.1f} deg   "
                  f"conf {r['confidence']:.2f}   "
                  f"cycle-std {r['cycle_std_deg']:5.1f} deg over "
                  f"{r['n_cycles']:4d} cycles   "
                  f"{rate/1e3:5.2f} kcycle/s   "
                  f"contrast {r['contrast_db']:4.1f} dB"
                  + (f"   OVERFLOW x{r['overflows']}" if r["overflows"] else ""))
            print("            levels " + "  ".join(
                f"ch{c}:{v:+6.1f}" for c, v in zip(ports, r["levels_db"])) +
                f"   null {r['null_db']:.1f} dBFS")
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        # Park on the dead position: it is the one setting that cannot be
        # feeding anything into the receiver when nobody is looking.
        try:
            backend.end_cycle(park=a.null_port)
        except Exception:                                       # noqa: BLE001
            pass
        backend.close()

    if len(hist) >= 3:
        print(f"\n[dfstream] {len(hist)} passes: mean "
              f"{dfcore.circ_mean_deg(hist):.1f} deg, circular std "
              f"{dfcore.circ_std_deg(hist):.1f} deg")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
