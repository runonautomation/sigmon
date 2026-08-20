"""Signal detection and amplitude-comparison bearing estimation.

The radio side is an 8-way RF switch on GPIO 5/6/7 of a LibreSDR B210, common
feeding TX/RX A.  Four ports carry antennas; four are open.  One receiver behind
a switch means there is only ever one receive chain, so there are no
inter-channel gain mismatches to track -- the usual weak point of amplitude DF
is absent here.  What is left is the antenna patterns and the calibration.

Bearing comes from the RATIOS between the live antennas' levels.  No phase, so
nothing here needs coherence, and switch timing is irrelevant at these dwells.
"""
import json
import os

import numpy as np


# --------------------------------------------------------------------------
# which switch ports actually have an antenna on them
# --------------------------------------------------------------------------
def spectral_structure(level_table, step_hz=None, rbw_hz=None):
    """Per-channel measure of "does this port see narrowband signals?".

    An open port sees only the receiver's own noise floor.  That is SMOOTH
    against frequency -- it has the analog roll-off and the tuning-segment
    edges, but no narrow features.  A port with an antenna on it shows
    stations: sharp, isolated peaks.

    So detrend each channel with a rolling median wide enough to follow the
    roll-off but too wide to follow a station, and measure what is left.  The
    residual is a few tenths of a dB for an open port and several dB for a live
    one, and -- unlike anything measured against the other channels -- it needs
    no open port anywhere to compare against.
    """
    lt = np.asarray(level_table, float)
    n_freq = lt.shape[1]
    win = 9
    if step_hz and rbw_hz:
        # Wide enough to span several resolution cells so a station cannot
        # drag the median with it, odd so the window is centred.
        win = max(5, int(round(5.0 * max(rbw_hz, step_hz) / step_hz)))
    win = min(win if win % 2 else win + 1, max(3, (n_freq // 2) * 2 - 1))

    out = np.zeros(lt.shape[0])
    half = win // 2
    for c in range(lt.shape[0]):
        row = lt[c]
        smooth = np.array([np.nanmedian(row[max(0, i - half):i + half + 1])
                           for i in range(n_freq)])
        resid = row - smooth
        out[c] = float(np.nanstd(resid))
    return out


def detrended(level_table, step_hz=None, rbw_hz=None):
    """Level table with the smooth part removed, leaving only narrow features."""
    lt = np.asarray(level_table, float)
    n_freq = lt.shape[1]
    win = 15
    if step_hz and rbw_hz:
        win = max(9, int(round(8.0 * max(rbw_hz, step_hz) / step_hz)))
    win = min(win if win % 2 else win + 1, max(3, (n_freq // 2) * 2 - 1))
    half = win // 2
    out = np.zeros_like(lt)
    for c in range(lt.shape[0]):
        row = lt[c]
        smooth = np.array([np.nanmedian(row[max(0, i - half):i + half + 1])
                           for i in range(n_freq)])
        out[c] = row - smooth
    return out


def channel_agreement(level_table, step_hz=None, rbw_hz=None):
    """For each port, its best correlation with any other port.

    Every antenna on the array sees the SAME transmitters -- at different
    levels, because of the patterns, but the same peaks at the same
    frequencies.  So two live ports' detrended spectra correlate strongly.  An
    open port sees only its own thermal noise, which correlates with nothing.

    This is the discriminator that survives both cases: it does not need an
    open port to exist (unlike a floor comparison) and it does not need an
    absolute threshold on how much structure counts (unlike a bare residual),
    because it asks a relative question about agreement instead.

    The correlation is taken on DETRENDED spectra.  On raw levels the shared
    analog roll-off makes even two open ports correlate strongly, which would
    call the whole array live.
    """
    d = detrended(level_table, step_hz, rbw_hz)
    n = d.shape[0]
    best = np.zeros(n)
    for i in range(n):
        top = 0.0
        for j in range(n):
            if i == j:
                continue
            a, b = d[i], d[j]
            ok = np.isfinite(a) & np.isfinite(b)
            if ok.sum() < 8:
                continue
            sa, sb = a[ok].std(), b[ok].std()
            if sa < 1e-9 or sb < 1e-9:
                continue
            r = float(np.corrcoef(a[ok], b[ok])[0, 1])
            top = max(top, r)
        best[i] = top
    return best


def find_live_channels(level_table, margin_db=3.0, step_hz=None, rbw_hz=None,
                       min_structure_db=1.0, min_agreement=0.35):
    """Which channels carry an antenna, decided from the data.

    Two metrics, because each fails where the other works:

      structure -- detrended residual (see spectral_structure).  Reference
          free: it asks whether THIS port sees narrowband signals, so it works
          when every port is populated.
      lift -- how far the port sits above the per-frequency minimum across
          ports.  Sharper when some ports really are open, because then that
          minimum IS the noise floor -- but meaningless when none are, since
          the minimum is then just the weakest antenna.

    An earlier version used `lift` alone.  With four ports open it separated
    them cleanly (1-4 dB open vs 10-14 dB live).  With all eight populated it
    silently misclassified the three weakest antennas as open, because there
    was no longer an empty port to define the floor -- and it said so with
    exactly the same confidence.  Structure is now the primary test and lift
    only breaks ties.

    Returns (live_indices, info) where info has both metrics per channel.
    """
    lt = np.asarray(level_table, float)
    n_chan = lt.shape[0]

    structure = spectral_structure(lt, step_hz, rbw_hz)
    agreement = channel_agreement(lt, step_hz, rbw_hz)
    floor_per_freq = np.nanmin(lt, axis=0)
    lift = np.nanmean(lt - floor_per_freq[None, :], axis=1)

    # Agreement is the primary test.  A port is live if its narrowband detail
    # matches some other port's -- i.e. they are looking at the same
    # transmitters.
    live = [i for i in range(n_chan) if agreement[i] >= min_agreement]

    # Fall back only if agreement finds nothing at all, which would mean either
    # a single live port (nothing to agree with) or a band with no signals in
    # it.  Neither is a case to guess at, so use the weaker tests and let the
    # caller see all three numbers.
    if not live:
        live = [i for i in range(n_chan) if structure[i] >= min_structure_db]
    if not live:
        quiet = np.min(lift)
        live = [i for i in range(n_chan) if lift[i] > quiet + margin_db]

    return live, dict(structure=structure, lift=lift, agreement=agreement)


# --------------------------------------------------------------------------
# detection
# --------------------------------------------------------------------------
def psd_peak_hold(x, fs, rbw, nblocks, welch):
    """Max-hold PSD: split the capture into blocks and keep the per-bin maximum.

    For a continuous emitter (FM) averaging is right -- it buys SNR.  For a
    bursty one (WiFi) it is actively wrong.  The switch visits the antennas one
    at a time, so each sees a DIFFERENT set of packets; the average level then
    measures how much traffic happened to occur during that antenna's dwell,
    not how well that antenna hears the transmitter.  That is a duty-cycle
    difference masquerading as a bearing.

    The peak over many blocks converges on the strongest burst received, which
    is a property of the path rather than of the traffic, and is therefore
    comparable between antennas.

    Returns (freqs, psd) like welch(), so callers are unchanged.
    """
    n = max(1, int(nblocks))
    blk = len(x) // n
    if blk < 64 or n == 1:
        return welch(x, fs, rbw)
    f = None
    acc = None
    for i in range(n):
        seg = x[i * blk:(i + 1) * blk]
        try:
            f, P = welch(seg, fs, rbw)
        except RuntimeError:
            continue
        acc = P if acc is None else np.maximum(acc, P)
    if acc is None:
        return welch(x, fs, rbw)
    return f, acc


def noise_floor(levels_db, percentile=25.0):
    """Robust noise floor.

    The median is wrong for a band that is mostly occupied: across the FM
    broadcast band a large fraction of the bins carry a station, so the median
    sits ON the signals and everything real then fails to clear the threshold.
    A low percentile tracks the genuinely empty bins instead.
    """
    return float(np.percentile(np.asarray(levels_db, float), percentile))


def detect_signals(freqs, best_db, threshold_db, min_separation_bins=2):
    """Peaks in `best_db` that stand `threshold_db` above the noise floor.

    Local maxima only, and never two peaks within `min_separation_bins` -- one
    strong FM station spills into the adjacent RBW bins and would otherwise be
    reported several times as several signals.
    """
    floor = noise_floor(best_db)
    cand = []
    n = len(best_db)
    for i in range(n):
        if best_db[i] - floor < threshold_db:
            continue
        lo = max(0, i - min_separation_bins)
        hi = min(n, i + min_separation_bins + 1)
        if best_db[i] >= np.max(best_db[lo:hi]):
            cand.append(i)

    out = []
    for i in cand:
        if out and (i - out[-1][0]) <= min_separation_bins:
            if best_db[i] > out[-1][1]:
                out[-1] = (i, best_db[i])
            continue
        out.append((i, best_db[i]))
    return [dict(index=i, freq=float(freqs[i]), level_db=float(v),
                 snr_db=float(v - floor)) for i, v in out], floor


# --------------------------------------------------------------------------
# bearing
# --------------------------------------------------------------------------
def element_pattern_db(theta_deg, boresight_deg, beamwidth_deg, floor_db=-20.0):
    """Gaussian main lobe with a back floor.  -12(t/B)^2 is -3 dB at t=B/2."""
    d = (np.asarray(theta_deg, float) - boresight_deg + 180.0) % 360.0 - 180.0
    return np.maximum(-12.0 * (d / beamwidth_deg) ** 2, floor_db)


def sigma_from_snr(snr_db, n_avg):
    """Level scatter in dB for a power estimate at a given SNR.

    For P = S + N averaged over M samples the fractional error is about
    (1 + N/S)/sqrt(M).  Weak elements are far less trustworthy than strong
    ones, and the fit has to know that.
    """
    snr = 10.0 ** (np.asarray(snr_db, float) / 10.0)
    return 8.686 * (1.0 + 1.0 / np.maximum(snr, 1e-9)) / np.sqrt(max(n_avg, 1))


def estimate_bearing(levels_db, azimuths_deg, beamwidth_deg, snr_db,
                     n_avg=1000, floor_db=-20.0, grid_step=0.5):
    """Weighted max-likelihood bearing from per-element levels.

    The source power is unknown, so at each candidate bearing the best-fit power
    is the weighted mean offset between measured and modelled levels; the
    bearing minimising the weighted residual wins.

    Weighting matters more than it looks.  An element pointing away from the
    source sits near the pattern floor where its dB value is mostly noise;
    weighting all elements equally lets the least trustworthy one drive the
    answer.  Weights are 1/sigma^2 from each element's own SNR.

    Returns (bearing_deg, residual_curve, grid).
    """
    levels_db = np.asarray(levels_db, float)
    azimuths_deg = np.asarray(azimuths_deg, float)
    grid = np.arange(0.0, 360.0, grid_step)

    G = np.stack([element_pattern_db(grid, b, beamwidth_deg, floor_db)
                  for b in azimuths_deg])            # (n_elem, n_grid)

    sigma = sigma_from_snr(snr_db, n_avg)
    w = 1.0 / np.maximum(sigma, 1e-6) ** 2
    w = w / w.sum()

    d = levels_db[:, None] - G
    c = (w[:, None] * d).sum(axis=0, keepdims=True)
    resid = (w[:, None] * (d - c) ** 2).sum(axis=0)
    return float(grid[int(np.argmin(resid))]), resid, grid


def estimate_bearings_batch(levels_db, azimuths_deg, beamwidth_deg, w=None,
                            floor_db=-20.0, grid_step=1.0):
    """estimate_bearing for MANY level vectors at once.

    The commutated recorder produces one level vector per switch cycle --
    hundreds or thousands of them per measurement -- and their spread is the
    whole point: it is the measured uncertainty of the bearing, from real
    independent looks rather than from an assumed noise model.  Calling
    estimate_bearing in a loop rebuilds the pattern matrix every time and turns
    that into the slowest part of the app, so the grid is built once and the
    residual is evaluated as one array operation.

    `w` is FIXED across cycles on purpose.  Per-cycle weights would be derived
    from per-cycle SNR, i.e. from the same noise whose effect the spread is
    trying to measure, and would let a cycle vote on how much it counts.

    Returns one bearing per row of `levels_db`.
    """
    L = np.atleast_2d(np.asarray(levels_db, float))
    az = np.asarray(azimuths_deg, float)
    grid = np.arange(0.0, 360.0, grid_step)
    G = np.stack([element_pattern_db(grid, b, beamwidth_deg, floor_db)
                  for b in az])                        # (n_elem, n_grid)
    if w is None:
        w = np.ones(len(az))
    w = np.asarray(w, float)
    w = w / w.sum()

    d = L[:, :, None] - G[None, :, :]                  # (n, n_elem, n_grid)
    c = np.einsum("e,neg->ng", w, d)[:, None, :]
    resid = np.einsum("e,neg->ng", w, (d - c) ** 2)
    return grid[np.argmin(resid, axis=1)]


def subtract_null_db(ant_db, null_db, floor_excess_db=0.5):
    """Remove the receiver's own noise from an antenna level, in POWER.

    The no-signal switch position measures the receive chain with no antenna on
    it: thermal noise, LO leakage, whatever spur lives at this frequency and
    gain.  Every antenna level contains that same term added to what the
    antenna actually delivered, so the difference of the POWERS is the antenna's
    own contribution and the difference of the dB values is nothing at all.

    Why it matters for a bearing rather than just for tidiness: an element
    pointing away from the source does not read low, it reads the receiver
    floor -- which is identical on every port.  The level spread therefore
    saturates at the point where the pattern is doing its most useful work, and
    the fit sees four nearly equal levels, which is consistent with a source
    anywhere.  Subtracting the floor restores the dynamic range the pattern
    needs.

    What it does NOT remove is the ambient noise the antenna itself picked up,
    which is a real antenna output and is often the larger term at VHF.  So the
    level spread still compresses at the bottom of the pattern -- measured on a
    synthetic array with 15 dB of ambient over the receiver floor, a 15.9 dB
    true spread read 13.3 dB either way.  This step recovers the receiver's
    contribution and nothing else; it is not a substitute for elements that can
    actually hear the difference.

    Where an antenna is not meaningfully above the null there is no
    information, only subtraction noise, so the result is clamped to what a
    signal `floor_excess_db` above the null would read rather than allowed to
    go to zero power and -inf dB.
    """
    a = 10.0 ** (np.asarray(ant_db, float) / 10.0)
    n = 10.0 ** (np.asarray(null_db, float) / 10.0)
    floor = n * (10.0 ** (floor_excess_db / 10.0) - 1.0)
    return 10.0 * np.log10(np.maximum(a - n, floor) + 1e-30)


def min_slot_seconds(rbw_hz, looks=4.0):
    """Shortest switch dwell that can still yield a level in this RBW.

    A power estimate in a bandwidth B over a time T has about T*B independent
    looks, and its scatter is ~8.7/sqrt(T*B) dB.  This is physics, not an
    implementation limit: at 200 kHz RBW a 2 us slot contains 0.4 looks and the
    "level" it returns is ~14 dB of noise.  Wanting microsecond slots therefore
    means wanting a wide RBW -- 2 us is usable at 2 MHz, not at 200 kHz.
    """
    return float(looks) / float(rbw_hz)


def bearing_confidence(resid, grid):
    """How sharply the fit picks one bearing, 0..1.

    A signal arriving where the array has real discrimination gives a deep,
    narrow minimum.  A flat residual means the levels are consistent with many
    bearings, and the reported number is then close to meaningless -- which the
    caller needs to know rather than discover later.
    """
    r = np.asarray(resid, float)
    if not np.isfinite(r).all() or np.ptp(r) <= 0:
        return 0.0
    rn = (r - r.min()) / np.ptp(r)
    # Fraction of the grid within 10% of the minimum: small = sharp.
    frac = float(np.mean(rn < 0.1))
    return float(np.clip(1.0 - frac / 0.5, 0.0, 1.0))


# --------------------------------------------------------------------------
# circular statistics -- bearings live on a circle
# --------------------------------------------------------------------------
def circ_mean_deg(deg):
    z = np.exp(1j * np.radians(np.asarray(deg, float)))
    return float(np.degrees(np.angle(z.mean())) % 360.0)


def circ_std_deg(deg):
    """Circular standard deviation.  Never use np.std on bearings: a set
    straddling 0/360 would report a huge spread for a tight cluster."""
    z = np.exp(1j * np.radians(np.asarray(deg, float)))
    R = abs(z.mean())
    if R <= 1e-12:
        return 180.0
    return float(np.degrees(np.sqrt(-2.0 * np.log(min(R, 1.0)))))


# --------------------------------------------------------------------------
# calibration
# --------------------------------------------------------------------------
class Calibration:
    """Per-channel gain offsets in dB, optionally per frequency.

    Without this, bearings are RELATIVE -- repeatable, comparable to each
    other, but not tied to true north or to any real angle.  Uncalibrated
    stability is still the right first thing to measure, so this is optional
    and its absence is reported rather than hidden.
    """

    def __init__(self, offsets=None, azimuths=None, source=None):
        self.offsets = dict(offsets or {})
        self.azimuths = dict(azimuths or {})
        self.source = source

    @classmethod
    def load(cls, path):
        if not path or not os.path.exists(path):
            return cls()
        with open(path) as f:
            d = json.load(f)
        return cls(offsets={int(k): float(v) for k, v in d.get("offsets", {}).items()},
                   azimuths={int(k): float(v) for k, v in d.get("azimuths", {}).items()},
                   source=path)

    def save(self, path):
        with open(path, "w") as f:
            json.dump({"offsets": {str(k): v for k, v in self.offsets.items()},
                       "azimuths": {str(k): v for k, v in self.azimuths.items()}},
                      f, indent=2)

    @property
    def calibrated(self):
        return bool(self.offsets)

    def apply(self, levels_db, channels):
        return np.array([levels_db[i] - self.offsets.get(ch, 0.0)
                         for i, ch in enumerate(channels)])

    def azimuth_for(self, channels, n_live, counter_clockwise=False,
                    offset_deg=0.0):
        """Element boresights.  Default: spread evenly over 360 in port order.

        That is an ASSUMPTION about how the antennas are physically arranged.
        It is right only if they were mounted in port order around the circle;
        if not, bearings will be self-consistent but rotated or scrambled.

        `counter_clockwise` matters more than it looks.  Bearings increase
        clockwise, so if the array is wired anticlockwise the port order runs
        backwards and every reported bearing is MIRRORED about the 0 deg axis:
        a source at 30 deg reads as 330 deg.  Nothing in the stability report
        can catch that -- a mirror leaves the circular standard deviation, and
        the distance to the nearest boresight, exactly unchanged -- so it has
        to be set from how the hardware was actually built.
        """
        if self.azimuths:
            return np.array([self.azimuths.get(ch, 0.0) for ch in channels])
        step = 360.0 / n_live
        idx = np.arange(n_live)
        return ((-idx * step if counter_clockwise else idx * step)
                + offset_deg) % 360.0

    def azimuth_for_ports(self, live, layout, counter_clockwise=False,
                          offset_deg=0.0):
        """Boresights from each port's place in the PHYSICAL array.

        azimuth_for() spreads however many ports are live evenly over the
        circle, which is right only when every element works.  It is wrong the
        moment one does not: an array built as four elements at 0/90/180/270
        with the fourth dead does not become three elements at 0/120/240 -- the
        three survivors are still bolted where they always were.  Using the
        live count silently rescales the whole geometry, and every bearing
        comes out rotated by an amount that depends on which element failed.

        Measured here: port 4 read the same as the deliberately dead port 5 at
        96, 103.6 and 105 MHz, to within 0.1 dB.  Three live elements out of
        four is exactly the case this exists for.

        `layout` is the declared array in wiring order; a live port's angle
        comes from its index there.
        """
        if self.azimuths:
            return np.array([self.azimuths.get(ch, 0.0) for ch in live])
        layout = list(layout)
        n = max(len(layout), 1)
        step = 360.0 / n
        idx = np.array([layout.index(p) if p in layout else i
                        for i, p in enumerate(live)], float)
        return ((-idx * step if counter_clockwise else idx * step)
                + offset_deg) % 360.0
