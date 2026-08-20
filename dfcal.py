#!/usr/bin/env python3
"""Self-calibration of the DF array by rotating it under a fixed signal.

Everything else in this repo measures bearings that are RELATIVE.  dfcore's
`Calibration` has slots for per-port gain offsets and per-port boresights, and
nothing has ever filled them: `azimuth_for_ports` spreads the ports evenly
around the circle because that is the best guess available, and the docstring
says so.  Three things are therefore assumed rather than known --

    that the antennas are equally spaced and wired in azimuth order,
    that the ports have equal gain,
    that the element beamwidth is 0.7x (or 1.33x) the spacing,

-- and each of them, wrong, produces a stable and confident bearing that is
also wrong.  The README's own numbers make the point: the 0.7x beamwidth rule
gave 29% stable signals on the 8-element array and 1.33x gave 72%, from
identical data.  Nothing in the measurement could tell which was right.

A rotation stage fixes this, because it supplies the one thing amplitude DF
never has: GROUND TRUTH.  Rotate the array by a known angle under a fixed
transmitter and every antenna is forced through the same pattern cut, so the
answers fall out of the raw levels without ever asking the bearing estimator
for its opinion:

  gain offsets -- over one full turn every element sweeps the SAME set of
      angles relative to the source, so its mean level over the turn cannot
      depend on where it is mounted.  Whatever difference is left between
      ports is gain, and it is exact rather than inferred.  (This is what
      webui's `--auto-balance` approximates by averaging over a band and
      hoping the band's signals arrive from all directions.  Here they really
      do, by construction.)

  boresights -- each port's level-versus-rotation curve IS its element
      pattern, measured.  The angle of its peak is that element's mounting
      angle.  No assumption of equal spacing, and a scrambled feed harness
      shows up as ports whose measured angles are not in port order -- the
      exact failure the README documents having hit once already, as an
      off-by-one in the switch truth table that "half-worked".

  beamwidth -- fit the measured cut instead of picking a rule of thumb.

  the CW/CCW mirror -- dfcore says "nothing in the stability report can catch
      that", and nothing can, because a mirrored array has identical
      statistics.  Rotation catches it: turning the stage one element spacing
      makes each port read what its NEIGHBOUR read, and which neighbour --
      +1 or -1 -- is the wiring sense.  See `find_period`.

  steps per revolution -- the stage's own scale is unknown (motor step angle x
      microstep DIPs x pulley ratio), and `stepper.py` refuses to talk in
      degrees until something measures it.  That something is here, and it
      measures it off the RF: the rotation that cyclically permutes the ports
      by one is exactly 1/N of a turn for an N-element ring.

  an accuracy figure -- and then, on angles that were NOT used in the fit,
      how far the calibrated bearings actually land from where the stage says
      they should.  That is a real error bar, not a repeatability figure.
      A stable wrong answer has a small circular standard deviation and a
      large error here.

WHAT ROTATION CANNOT TELL YOU.  Two facts, and both are settings:

  --rotates {array,source}   Whether the stage carries the ARRAY under a fixed
      transmitter or carries a source around a fixed array.  The level data is
      identical up to a mirror, so this cannot be inferred -- and getting it
      wrong mirrors every bearing about the home axis.
  --dir-plus {cw,ccw}        Which way, seen from above, a positive step
      command turns the stage.  Watch it once and set it.

  Everything else, including true north, is optional: pass --true-bearing with
  the known azimuth of the reference transmitter and the output is absolute;
  leave it off and it is referenced to the stage's home position, which is
  still a fixed physical direction you can measure later with a compass.

Typical run, on a 7-antenna ring with port 8 as the dead position:

    ./dfcal.py 96.0M --ports 1,2,3,4,5,6,7 --null-port 8 --find-scale --save
    ./dfcal.py 96.0M --check          # preflight only, no rotation
    ./dfcal.py --simulate             # the solver against a synthetic array

and then the rest of the repo picks it up with `--cal dfcal.json`.
"""
import argparse
import json
import os
import sys
import time

import numpy as np

import dfcore

_HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CAL = os.path.join(_HERE, "dfcal.json")


# ==========================================================================
# solver -- pure numpy, no hardware, so every claim below is testable offline
# ==========================================================================
def wrap180(d):
    return (np.asarray(d, float) + 180.0) % 360.0 - 180.0


def common_mode(L):
    """Levels with the per-angle mean across ports removed.

    The reference transmitter is not a lab source.  Over the minute or two a
    full turn takes, its level at this site wanders -- multipath, the
    receiver's own gain, someone walking past.  That drift is COMMON to every
    port, because all seven levels come out of one 100 ms capture, and it is
    not common to every angle, because the angles are minutes apart.  Left in,
    it lands squarely on the quantity being measured: the per-port mean over
    the turn.

    Subtracting the port-mean at each angle removes it exactly, and the price
    is known rather than hoped for.  What gets subtracted is
    (1/P) sum_j G(theta - a_j - phi), and for a ring of P equally spaced
    elements that sum is invariant under rotation by one spacing -- so it
    contains ONLY harmonics that are multiples of P.  The boresight estimator
    uses the fundamental (one cycle per turn) and the gain estimator uses the
    DC term relative to the array mean.  Neither is a multiple of P for P > 1,
    so neither is touched.  With unequal spacing the cancellation is partial
    and the residual shows up in `ring_residual_db`.
    """
    L = np.asarray(L, float)
    return L - np.nanmean(L, axis=1, keepdims=True)


def find_shift_lag(Ln, min_lag=2, min_overlap=6, min_score=0.5):
    """The rotation that cyclically permutes the ports by one, in SAMPLES.

    For a ring of P identical elements, turning by one element spacing puts
    each element where its neighbour was, so the level VECTOR at sample i+lag
    is the vector at sample i rolled by one place.  The SMALLEST lag that does
    that is one spacing; the sign of the roll is the sense of the permutation.

    Smallest, not best-scoring.  Two spacings roll the vector by two, three by
    three, and on a seven-element ring rolling by four is the same array
    operation as rolling by minus three -- so a global best-score search picks
    whichever multiple the noise happened to favour and reports it as one
    spacing.  That is exactly what made the first version of this return
    revolutions that were out by factors of two to four.

    This is used only to decide when enough of a turn has been collected;
    `find_period` does the actual scale measurement, using every element
    jointly.  Returns dict(lag, shift, score) or None.
    """
    Ln = np.asarray(Ln, float)
    n, P = Ln.shape
    var = float(np.nanmean(np.nanvar(Ln, axis=1)))
    if n < min_lag + min_overlap or var <= 1e-12:
        return None
    for lag in range(min_lag, n - min_overlap):
        a, b = Ln[:n - lag], Ln[lag:]
        for s in (1, -1):
            sc = 1.0 - float(np.nanmean((b - np.roll(a, -s, axis=1)) ** 2)) / (2.0 * var)
            if sc >= min_score:
                return dict(lag=float(lag), shift=int(s), score=float(sc))
    return None


def find_period(Ln, n_elem, n_grid=3000, min_samples_per_rev=None,
                min_periods=1.0):
    """Samples per revolution, and the permutation sense, from every element
    at once.

    Each port's level as the stage turns has a fundamental at one cycle per
    revolution, and -- this is the part worth having -- the P fundamentals are
    not independent.  Their PHASES are the elements' mounting angles, so on a
    uniform ring they are spaced 2*pi/P apart and in ring order.

    So the model fitted at each candidate period is not P independent
    sinusoids but ONE complex amplitude shared by all P ports, each port's
    copy pre-rotated by its place in the ring:

        Ln[i,k] ~ m_k + Re{ B * exp(j*2*pi*i/N) * exp(-j*sigma*2*pi*k/P) }

    P + 2 free parameters instead of 3P, which is what makes it survive a
    reference that fades by several dB between angles -- and `sigma`, the
    permutation sense, falls out as whichever sign lets the sum cohere.

    Two details that are not cosmetic:

      The basis is projected off the constant before it is used.  At long
      periods a cosine over a record barely one period long is nearly
      collinear with a constant, so an unprojected fit lets the sinusoid
      absorb whatever DC and trend survived the mean removal.  The magnitude
      then climbs monotonically toward the longest period on the grid and the
      estimator reports a revolution about 10% too long -- with a coherence of
      0.997, because the PHASES are still perfectly ringed at the wrong
      period.  A high coherence is not evidence that the period is right.

      Periods longer than the record are not on the grid at all.  A period
      that has not completed cannot be measured, only extrapolated, and the
      extrapolation is exactly where the bias above lives.

    Returns dict(period_samples, sense, coherence, score, at_edge) or None.
    """
    Ln = np.asarray(Ln, float)
    n, P = Ln.shape
    Y = np.nan_to_num(Ln - np.nanmean(Ln, axis=0, keepdims=True))
    if n < 8 or float(np.var(Y)) <= 1e-12:
        return None

    lo = float(min_samples_per_rev or 2.5 * n_elem)
    hi = float(n) / max(min_periods, 1.0)
    if hi <= lo * 1.05:
        return None
    grid = np.linspace(lo, hi, n_grid)
    i = np.arange(n, dtype=float)
    ring = np.exp(1j * 2.0 * np.pi * np.arange(P) / P)

    def amps(N):
        u = np.exp(-2j * np.pi * i / N)
        u = u - u.mean()                      # off the constant, see above
        d = float((np.abs(u) ** 2).sum())
        if d <= 1e-12:
            return None, 0.0
        return (Y * u[:, None]).sum(axis=0), d

    plus = np.zeros(n_grid)
    minus = np.zeros(n_grid)
    for j, N in enumerate(grid):
        c, d = amps(N)
        if c is None:
            continue
        s = 1.0 / np.sqrt(d)
        # u carries exp(-j...), so c_k is the CONJUGATE of the port's
        # fundamental and the sigma = +1 ring is therefore the conjugate one.
        plus[j] = abs((c * ring.conj()).sum()) * s
        minus[j] = abs((c * ring).sum()) * s

    jp, jm = int(plus.argmax()), int(minus.argmax())
    sense, curve, j = ((+1, plus, jp) if plus[jp] >= minus[jm]
                       else (-1, minus, jm))
    N = float(grid[j])
    if 0 < j < n_grid - 1:                    # parabolic refine on the grid
        y0, y1, y2 = curve[j - 1], curve[j], curve[j + 1]
        den = y0 - 2 * y1 + y2
        if abs(den) > 1e-12:
            N += 0.5 * (y0 - y2) / den * (grid[1] - grid[0])
    N = float(np.clip(N, lo, hi))
    c, _ = amps(N)
    coh = float(abs((c * (ring.conj() if sense > 0 else ring)).sum())
                / max(np.abs(c).sum(), 1e-12))
    return dict(period_samples=N, sense=int(sense), coherence=coh,
                score=float(curve[j]), at_edge=bool(j <= 1 or j >= n_grid - 2),
                grid=grid, curve=curve)


def scale_from_period(Ln, block_steps, n_elem, **kw):
    """find_period, expressed in steps."""
    r = find_period(Ln, n_elem, **kw)
    if r is None:
        return None
    spr = r["period_samples"] * block_steps
    return dict(steps_per_rev=float(spr), sense=r["sense"],
                spacing_steps=float(spr / n_elem), at_edge=r["at_edge"],
                coherence=r["coherence"], period_samples=r["period_samples"])


def solve_gains(Ln):
    """Per-port gain offset in dB, as a mean over one full uniform turn.

    Sign matches dfcore.Calibration.apply, which SUBTRACTS the offset: a port
    that reads hot gets a positive number.  They sum to zero by construction,
    because only differences between ports mean anything -- an overall gain is
    absorbed by the fit's free source power.
    """
    g = np.nanmean(np.asarray(Ln, float), axis=0)
    return g - np.nanmean(g)


def solve_boresights(P):
    """Each element's mounting angle, from the fundamental of its pattern cut.

    `P` is (n_angle, n_port), sampled uniformly over exactly one turn, gains
    already removed.  Column k is element k's pattern as the array turns, so
    it is a peaked even function of (phi - phi_k) and its first Fourier
    coefficient has phase phi_k.

    Using the fundamental rather than the peak SAMPLE is what makes a coarse
    sweep usable: 36 samples over a turn puts the peak sample up to 5 deg from
    the true peak, while the fundamental is a weighted average of every sample
    and lands inside a degree.  It is also far less sensitive to a single
    fading sample, which a peak-finder would follow straight off the rail.

    Returns (angles_deg, strength) -- strength is |c1| in dB, i.e. how much
    one-cycle-per-turn modulation that port actually showed.  A port with an
    open feed shows none, and its angle is then meaningless rather than wrong.
    """
    P = np.asarray(P, float)
    n = P.shape[0]
    phi = np.arange(n) * 2.0 * np.pi / n
    z = (np.nan_to_num(P) * np.exp(1j * phi)[:, None]).sum(axis=0) / n
    return np.degrees(np.angle(z)) % 360.0, 2.0 * np.abs(z)


def fit_beamwidth(P, peaks_deg, floor_grid=None, bw_grid=None):
    """Beamwidth and back-floor of the element pattern, fitted to the cut.

    Every port's cut is shifted to put its own peak at zero and then averaged,
    which is legitimate here precisely because the elements are nominally
    identical -- and if they are not, `per_port` says so.  The model is
    dfcore.element_pattern_db, so what comes out can be handed straight back
    to the estimator that will use it.

    The fit is a grid search rather than a gradient method because the model
    has a hard max() in it: the residual is piecewise-smooth with a kink where
    the Gaussian meets the floor, and gradient methods stall on that kink at
    whatever beamwidth they happened to start near.
    """
    P = np.asarray(P, float)
    n, n_port = P.shape
    phi = np.arange(n) * 360.0 / n
    bw_grid = np.arange(10.0, 181.0, 1.0) if bw_grid is None else bw_grid
    floor_grid = np.arange(-40.0, -4.9, 1.0) if floor_grid is None else floor_grid

    bw_grid = np.asarray(bw_grid, float)
    floor_grid = np.asarray(floor_grid, float)

    def _fit(d, y):
        """Least squares over the (beamwidth, floor) grid, all at once.

        The source power is not a third grid axis: at every candidate shape it
        is the mean offset between the measurement and the model, which is
        solved in closed form and subtracted.
        """
        ok = np.isfinite(y)
        d, y = d[ok], y[ok]
        if len(d) < 6:
            return float("nan"), float("nan"), float("nan")
        g = -12.0 * (wrap180(d)[None, :] / bw_grid[:, None]) ** 2   # (B, n)
        m = np.maximum(g[:, None, :], floor_grid[None, :, None])    # (B, F, n)
        diff = y[None, None, :] - m
        diff = diff - diff.mean(axis=-1, keepdims=True)             # free power
        r = (diff ** 2).mean(axis=-1)
        b, f = np.unravel_index(int(np.argmin(r)), r.shape)
        return float(bw_grid[b]), float(floor_grid[f]), float(np.sqrt(r[b, f]))

    per_port = []
    for k in range(n_port):
        bw, fl, rms = _fit(wrap180(phi - peaks_deg[k]), P[:, k])
        per_port.append(dict(beamwidth_deg=bw, floor_db=fl, rms_db=rms))

    d_all = np.concatenate([wrap180(phi - peaks_deg[k]) for k in range(n_port)])
    y_all = np.concatenate([P[:, k] for k in range(n_port)])
    bw, fl, rms = _fit(d_all, y_all)
    return dict(beamwidth_deg=float(bw), floor_db=float(fl), rms_db=float(rms),
                per_port=per_port)


def roll_sense_from_peaks(peaks_deg, n_elem):
    """Which way the ports permute, read off the measured peak angles.

    Port k peaks at stage angle psi_k = const - sigma * k * 360/P, so the
    average step between consecutive ports is -sigma * 360/P and its sign is
    sigma.  Available from the calibration turn alone, so it cross-checks the
    value the scale search got from a completely different statistic -- and
    the two disagreeing is worth knowing about, because it means the ring is
    not being read in ring order.
    """
    d = wrap180(np.diff(np.asarray(peaks_deg, float)))
    m = np.degrees(np.angle(np.mean(np.exp(1j * np.radians(d)))))
    return -1 if m > 0 else 1


def geometry_report(ports, peaks_deg, eps, roll_sense, rotates,
                    true_bearing=None, home_angle_deg=0.0):
    """Turn measured peak-rotation-angles into element azimuths, and check them.

    The peak of port k's cut occurs at the stage angle that points element k
    at the source.  Which way that maps to a mounting azimuth depends on what
    is bolted to the stage:

        array rotates  ->  a_k = theta - phi_k
        source rotates ->  a_k = theta + phi_k

    and theta, the source's true bearing, is a single unknown common to every
    element.  Without --true-bearing it is set so that port 1 sits at 0 deg,
    which leaves every azimuth referenced to the array's own first element --
    exactly what the uncalibrated code already assumed, but now MEASURED, so
    the spacings and the order are real even when the reference is not.
    """
    ports = list(ports)
    phi = np.asarray(peaks_deg, float)
    e = 1.0 if eps >= 0 else -1.0
    # phi_k is the STAGE angle at which element k points straight at the
    # source, and a positive stage command turns the world by e*phi.  So the
    # element's own azimuth is the source bearing offset by that rotation,
    # with the sign set by which of the two is bolted to the stage:
    #     array on the stage   ->  a_k = theta - e*phi_k
    #     source on the stage  ->  a_k = theta + e*phi_k
    # It is `e` here and never the permutation sense: they are different
    # facts, and using one for the other mirrors the array whenever the two
    # happen to differ -- which is precisely the source-on-the-stage case.
    a = (-e * phi) if rotates == "array" else (e * phi)
    if true_bearing is None:
        a = (a - a[0]) % 360.0
        reference = f"port {ports[0]} = 0 deg (home-relative)"
    else:
        a = (a + float(true_bearing) + float(home_angle_deg)) % 360.0
        reference = f"true north, from --true-bearing {true_bearing:g} deg"

    n = len(ports)
    nominal = np.arange(n) * 360.0 / n
    # How far the measured ring is from a uniform one, after allowing for the
    # array's own rotation -- a rigid offset is not an error, a scrambled or
    # unevenly spaced ring is.
    off = np.degrees(np.angle(np.mean(np.exp(1j * np.radians(a - nominal)))))
    spacing_err = wrap180(a - nominal - off)

    order = np.argsort(a % 360.0)
    in_order = bool(np.all(np.diff(np.concatenate([order, order[:1]])) % n == 1))
    gaps = wrap180(np.diff(np.concatenate([a[order], a[order][:1] + 360.0])))
    return dict(azimuths_deg=a.tolist(), reference=reference,
                nominal_deg=nominal.tolist(),
                spacing_error_deg=spacing_err.tolist(),
                max_spacing_error_deg=float(np.max(np.abs(spacing_err))),
                array_offset_deg=float(off % 360.0),
                port_order_is_azimuth_order=in_order,
                azimuth_order=[ports[i] for i in order],
                measured_spacings_deg=np.round(gaps % 360.0, 2).tolist(),
                roll_sense=int(roll_sense), dir_sense=int(e),
                # Wiring sense w: rotating the array by one spacing permutes
                # the ports by sigma = e*w, so w = sigma*e -- and the source
                # case flips it, for the same reason it flips the azimuths.
                counter_clockwise=bool(
                    (roll_sense * e if rotates == "array"
                     else -roll_sense * e) < 0))


def solve(angles_deg, L, ports, eps=1, rotates="array", true_bearing=None,
          null_col=None, roll_sense=None):
    """The whole solve, from one full-turn level table.

    `L` is (n_angle, n_port) in dB, uniformly sampled over exactly 360 deg,
    with the null slot already subtracted (dfstream does that).  `null_col`,
    if given, is an extra column holding the dead port, which is not part of
    the array but IS the best available check that the reference was steady.
    """
    L = np.asarray(L, float)
    Ln = common_mode(L)
    gains = solve_gains(Ln)
    P = Ln - gains[None, :]
    peaks, strength = solve_boresights(P)
    bw = fit_beamwidth(P, peaks)
    measured_roll = roll_sense_from_peaks(peaks, len(ports))
    geo = geometry_report(ports, peaks, eps,
                          measured_roll if roll_sense is None else roll_sense,
                          rotates, true_bearing)
    geo["roll_sense_from_peaks"] = int(measured_roll)
    geo["roll_sense_agrees"] = bool(roll_sense is None
                                    or int(roll_sense) == int(measured_roll))

    # Modulation depth per port: how much of a pattern this element actually
    # has. An element that does not vary over a full turn is not contributing
    # a bearing, whatever its level -- an open feed, a shorted port, or an
    # omnidirectional antenna where a directional one was assumed.
    mod = np.nanmax(P, axis=0) - np.nanmin(P, axis=0)

    # Residual after the ring model: what one-cycle-per-turn fitting could not
    # explain. Large means the elements are not identical or the site is not
    # behaving like a single fixed source.
    model = np.stack([dfcore.element_pattern_db(
        np.arange(L.shape[0]) * 360.0 / L.shape[0], peaks[k],
        bw["beamwidth_deg"], bw["floor_db"]) for k in range(L.shape[1])], axis=1)
    model = model - np.nanmean(model, axis=0, keepdims=True)
    Pc = P - np.nanmean(P, axis=0, keepdims=True)
    ring_res = float(np.sqrt(np.nanmean((Pc - model) ** 2)))

    out = dict(
        ports=list(ports),
        offsets_db=gains.tolist(),
        peak_rotation_deg=peaks.tolist(),
        modulation_db=mod.tolist(),
        fundamental_db=strength.tolist(),
        beamwidth=bw,
        ring_residual_db=ring_res,
        n_angles=int(L.shape[0]),
        **geo)
    if null_col is not None:
        nc = np.asarray(null_col, float)
        out["null_modulation_db"] = float(np.nanmax(nc) - np.nanmin(nc))
        out["null_mean_dbfs"] = float(np.nanmean(nc))
    return out


# ==========================================================================
# validation -- on angles that were not used in the fit
# ==========================================================================
def bearing_accuracy(stage_deg, bearing_deg, eps=1, rotates="array"):
    """How well calibrated bearings track the stage, with the offset free.

    The source's true bearing is unknown, so a rigid offset cannot be an
    error; everything else can.  Two numbers come out and they fail
    differently:

      rms_deg   scatter about the best-fit line -- how good the bearings are.
      slope     how much reported bearing moves per degree of stage, which
          must be -1 (array) or +1 (source).  A slope of, say, 0.8 means the
          array is being modelled with the wrong geometry or the stage's scale
          is wrong, and it is INVISIBLE to any repeatability measure: each
          angle is perfectly repeatable and the map between them is stretched.
    """
    x = np.asarray(stage_deg, float)
    y = np.asarray(bearing_deg, float)
    e = 1.0 if eps >= 0 else -1.0
    # Turning the array by +e degrees moves the source e degrees the other way
    # in the array's own frame; turning the source moves it with the stage.
    want = -e if rotates == "array" else e

    # Offset from the circular mean of the residual, so 0/360 cannot break it.
    resid = wrap180(y - want * x)
    off = np.degrees(np.angle(np.mean(np.exp(1j * np.radians(resid)))))
    err = wrap180(resid - off)

    # Slope on the unwrapped pair, which is only meaningful with >= 3 angles.
    slope = float("nan")
    if len(x) >= 3:
        yu = np.unwrap(np.radians(y))
        xu = np.radians(x)
        A = np.stack([xu, np.ones_like(xu)], axis=1)
        slope = float(np.linalg.lstsq(A, yu, rcond=None)[0][0])
    return dict(offset_deg=float(off % 360.0),
                errors_deg=err.tolist(),
                rms_deg=float(np.sqrt(np.mean(err ** 2))),
                max_deg=float(np.max(np.abs(err))),
                slope=slope, expected_slope=float(want),
                source_bearing_deg=float(off % 360.0))


# ==========================================================================
# hardware
# ==========================================================================
class Rig:
    """Radio + RF switch + rotation stage, with one job: levels at an angle."""

    def __init__(self, a):
        self.a = a
        # rfscan.py is imported by dfstream, sigmon and webui alike and is not
        # in this repository. Say so once, plainly, rather than letting a bare
        # ModuleNotFoundError out of three call levels down.
        try:
            import rfscan                                       # noqa: F401
        except ImportError as e:
            raise SystemExit(
                "error: rfscan.py is missing.\n"
                "  dfstream.py, sigmon.py and webui.py all import it for "
                "Receiver, welch_psd,\n"
                "  band_power_db, parse_freq, plan_segments and "
                "default_fpga(). Nothing in this\n"
                "  repository provides it and this git tree has no commits, "
                "so it was never\n"
                "  added. Put it next to these files (or on PYTHONPATH) "
                "before running any\n"
                f"  radio path.  [{e}]") from e
        import swbackend
        self.dfstream = __import__("dfstream")
        self.usrp, self.rfscan = self.dfstream.open_usrp()
        self.backend = swbackend.open_backend(
            a.switch, usrp=self.usrp, device=a.switch_device,
            auto=(a.switch == "auto"))
        self.ports = ([int(c) for c in a.ports.split(",")] if a.ports
                      else list(self.backend.default_ports))
        self.null_port = (a.null_port if a.null_port is not None
                          else self.backend.default_null)
        self.rx = self.rfscan.Receiver(self.usrp, a.rx_chan, a.rate, a.gain,
                                       a.antenna, lo_frac=a.lo_frac)
        if not a.track_dc:
            self.dfstream.freeze_dc_offset(self.rx)

        import stepper
        stepper.install_signal_guard()
        self.stage = stepper.Stepper.from_config(
            a.stage_config, steps_per_rev=a.steps_per_rev, rate=a.step_rate,
            backlash_steps=a.backlash_steps, dir_sign=a.dir_sign)
        self.stepper_mod = stepper

    # -- one measurement -------------------------------------------------
    def levels(self, freq, repeats=None, azimuths=None, beamwidth=None,
               offsets=None):
        """Mean per-port level in dB at the current stage angle.

        Averaged in POWER over `repeats` captures for the same reason
        dfstream averages cycles in power: a dB mean is the geometric mean and
        a single deep fade drags it somewhere the antenna never was.

        Returns (levels, null_db, info).  `levels` is None when the recording
        could not be segmented, which is reported at the call site rather than
        substituted for -- a missing angle is a gap in the pattern, and a
        guessed one is a wrong pattern.
        """
        n = repeats or self.a.repeats
        az = (np.arange(len(self.ports)) * 360.0 / len(self.ports)
              if azimuths is None else np.asarray(azimuths, float))
        bw = beamwidth or (360.0 / len(self.ports))
        acc, nulls, last = [], [], {}
        for _ in range(max(1, n)):
            r = self.dfstream.measure(self.rx, self.backend, freq, self.ports,
                                      self.null_port, az, bw, self.a,
                                      offsets=offsets)
            last = r
            if not r.get("ok"):
                continue
            acc.append(np.asarray(r["levels_db"], float))
            nulls.append(float(r["null_db"]))
        if not acc:
            return None, None, last
        A = np.stack(acc)
        lv = 10.0 * np.log10(np.mean(10.0 ** (A / 10.0), axis=0))
        info = dict(last)
        info["n_ok"] = len(acc)
        info["level_scatter_db"] = float(np.mean(np.std(A, axis=0))) if len(A) > 1 else 0.0
        return lv, float(np.mean(nulls)), info

    def close(self):
        try:
            self.backend.end_cycle(park=self.null_port)
            self.backend.close()
        except Exception:                                       # noqa: BLE001
            pass
        try:
            self.stage.close()
        except Exception:                                       # noqa: BLE001
            pass


# --------------------------------------------------------------------------
def preflight(rig, freq, show=print):
    """Everything that must be true before a rotation is worth starting.

    Each of these has a documented history of producing a confident wrong
    answer in this repo, so they are checked in the order that a failure would
    invalidate everything after it.
    """
    ok = True
    show("=" * 72)
    show("PREFLIGHT")
    show("=" * 72)

    # 1. Does the switch move the RF at all?
    r = rig.dfstream.check_switch(rig.rx, rig.backend, freq,
                                  list(rig.ports) + [rig.null_port],
                                  rbw=rig.a.rbw, passes=rig.a.check_passes)
    show(f"  switch:   between-port spread {r['between_db']:.1f} dB against "
         f"{r['within_db']:.2f} dB of within-port scatter over {r['passes']} passes")
    if not r["responds"]:
        show("            FAIL -- the ports do not differ by more than the "
             "noise. Every antenna is the same antenna; a bearing from this "
             "would be meaningless AND stable. Check the control-line pins "
             "and the truth table (see README) before anything else.")
        ok = False

    # 2. Is the declared null actually dead?
    means = r["means"]
    ant = [means[p] for p in rig.ports if np.isfinite(means[p])]
    nullv = means.get(rig.null_port, float("nan"))
    if ant and np.isfinite(nullv):
        margin = float(np.median(ant) - nullv)
        show(f"  null:     port {rig.null_port} sits {margin:+.1f} dB below the "
             f"median antenna")
        if margin < rig.a.min_contrast:
            show("            FAIL -- the no-signal position is not dead. It "
                 "is both the sync marker and the noise reference, so a live "
                 "null breaks the segmentation and the floor subtraction at "
                 "the same time.")
            ok = False

    # 3. Can a DF be segmented here at all, and is there a signal?
    lv, nl, info = rig.levels(freq, repeats=1)
    if lv is None:
        show(f"  signal:   FAIL -- {info.get('reason', 'no measurement')}")
        return False
    spread = float(np.max(lv) - np.min(lv))
    show(f"  signal:   contrast {info.get('contrast_db', float('nan')):.1f} dB, "
         f"{info.get('n_cycles', 0)} cycles, port spread {spread:.1f} dB, "
         f"null {nl:.1f} dBFS")
    if spread < 1.0:
        show("            WARNING -- under 1 dB between the best and worst "
             "port. Either the source is nearly equidistant from every "
             "element or the antennas are not directional at this frequency. "
             "Rotation will show which, but the pattern will be shallow.")

    # 4. Does the stage move, and does the RF notice?
    if rig.a.no_stage:
        show("  stage:    skipped (--no-stage)")
        return ok
    probe = rig.a.probe_steps
    before = lv
    rig.stage.move_steps(probe)
    after, _, _ = rig.levels(freq, repeats=1)
    rig.stage.move_steps(-probe)
    if after is None:
        show("  stage:    could not measure after the test move")
        return False
    delta = float(np.max(np.abs(common_mode(np.stack([before, after]))[1]
                                - common_mode(np.stack([before, after]))[0])))
    show(f"  stage:    {probe} steps changed the port pattern by "
         f"{delta:.2f} dB (largest per-port change, common mode removed)")
    if delta < 0.3:
        show("            WARNING -- the RF barely noticed. Either the stage "
             "did not turn (stalled, or ENA asserted), or "
             f"{probe} steps is a very small angle. Raise --probe-steps, or "
             "watch the stage while `./stepper.py --steps 2000` runs.")
    show("=" * 72)
    return ok


def find_scale(rig, freq, show=print):
    """Steps per revolution, measured off the RF.

    Walks the stage forward in blocks, watching for the rotation that makes
    each port read what its neighbour read.  That is one element spacing by
    definition, so P of them is a revolution -- and it is found without any
    prior knowledge of the microstep setting or the pulley ratio, which is the
    whole point: those are exactly the numbers nobody wrote down.

    The search is adaptive.  It stops widening as soon as one spacing is
    identified, then deliberately keeps going for a further P-1 spacings so
    the scale can be fitted over a whole revolution instead of extrapolated
    from a seventh of one.
    """
    P = len(rig.ports)
    block = rig.a.scale_block or max(8, int(rig.a.scale_guess / P / 6))
    hard_max = int(rig.a.scale_guess * rig.a.scale_max_mult)
    show(f"[scale] walking in {block}-step blocks, up to {hard_max} steps "
         f"({rig.a.scale_guess} steps/rev guessed only to size the block)")

    rows, steps = [], []
    lv, _, info = rig.levels(freq, repeats=1)
    if lv is None:
        show(f"[scale] FAIL: {info.get('reason')}")
        return None
    rows.append(lv)
    steps.append(0)
    target_blocks = None
    pos = 0
    while pos < hard_max:
        rig.stage.move_steps(block)
        pos += block
        lv, _, info = rig.levels(freq, repeats=1)
        if lv is None:
            show(f"  {pos:6d} steps: dropped ({info.get('reason')})")
            rows.append(np.full(P, np.nan))
            steps.append(pos)
            continue
        rows.append(lv)
        steps.append(pos)
        Ln = common_mode(np.stack(rows))
        if target_blocks is None and len(rows) >= 8:
            hit = find_shift_lag(Ln)
            if hit and hit["score"] > rig.a.scale_score:
                # A whole revolution plus real margin. find_period will
                # not consider a period longer than the record, so stopping
                # at exactly one turn puts the answer on the edge of its own
                # search grid, where the parabolic refinement is clipped.
                target_blocks = int(np.ceil(hit["lag"] * P * 1.15)) + max(4, P)
                show(f"  {pos:6d} steps: one element spacing at "
                     f"{hit['lag']*block:.0f} steps (roll {hit['shift']:+d}, "
                     f"score {hit['score']:.2f}) -- continuing to a full turn")
        if target_blocks is not None and len(rows) >= target_blocks:
            break
        if len(rows) % 10 == 0:
            show(f"  {pos:6d} steps: {len(rows)} samples, still searching")

    Ln = common_mode(np.stack(rows))
    fit = scale_from_period(Ln, block, P)
    if fit is None:
        show("[scale] FAIL: no rotation permuted the ports. Either the stage "
             "never turned far enough (raise --scale-max-mult), the elements "
             "are not a uniform ring, or the reference signal is not steady "
             "enough to compare angles minutes apart.")
        return None
    spr = fit["steps_per_rev"]
    show(f"[scale] {spr:,.0f} steps/rev  ({360.0/spr*1000:.3f} millideg/step), "
         f"spacing {fit['spacing_steps']:,.0f} steps = {360.0/P:.2f} deg")
    show(f"        ring coherence {fit['coherence']:.3f} of 1.0 -- how well the "
         f"{P} elements' pattern peaks lay on an evenly spaced ring at this "
         f"period")
    if fit["coherence"] < 0.6:
        show("        WARNING: low coherence. This is the best period on "
             "offer but the elements did not agree on it, so treat the scale "
             "as provisional -- collect more blocks, or find a steadier "
             "reference.")
    show(f"        permutation sense {fit['sense']:+d}: one spacing of + steps "
         f"makes each port read what port k{fit['sense']:+d} read")
    if fit.get("at_edge"):
        show("        WARNING: the best period is at the edge of the search "
             "range, which means a whole revolution did not fit in the "
             "record. The number below is an extrapolation. Re-run with a "
             "larger --scale-max-mult.")
    for common in (200, 400, 1600, 3200, 6400, 12800, 25600):
        if abs(spr - common) / common < 0.03:
            show(f"        that is within 3% of {common}, a standard "
                 f"200-step motor at {common//200}x microstepping with no "
                 f"gearing")
            break
    rig.stage.steps_per_rev = int(round(spr))
    return fit


def sweep(rig, freq, n_angles, show=print, tag="sweep"):
    """One full turn, `n_angles` uniform samples, returning to home.

    Angles are visited in one direction only.  A there-and-back sweep would
    halve the drift by interleaving, and would put the gearbox backlash
    straight into the middle of the measurement as a fixed offset between the
    two halves -- which is indistinguishable from an array that is mounted
    a degree off.
    """
    spr = rig.stage.steps_per_rev
    if not spr:
        raise RuntimeError("steps per revolution unknown -- run --find-scale")
    L, N, angles, drops = [], [], [], 0
    show(f"[{tag}] {n_angles} angles over 360 deg "
         f"({360.0/n_angles:.2f} deg, {spr//n_angles} steps apart), "
         f"{rig.a.repeats} captures each")
    t0 = time.time()
    for i in range(n_angles):
        target = int(round(i * spr / float(n_angles)))
        rig.stage.goto_steps(target)
        lv, nl, info = rig.levels(freq)
        ang = i * 360.0 / n_angles
        if lv is None:
            drops += 1
            show(f"  {ang:6.1f} deg: dropped -- {info.get('reason')}")
            lv, nl = np.full(len(rig.ports), np.nan), np.nan
        elif rig.a.verbose:
            show(f"  {ang:6.1f} deg: " + " ".join(f"{v:+6.1f}" for v in lv)
                 + f"   null {nl:6.1f}   scatter {info['level_scatter_db']:.2f} dB")
        L.append(lv)
        N.append(nl)
        angles.append(ang)
    show(f"[{tag}] {n_angles - drops}/{n_angles} angles measured in "
         f"{time.time()-t0:.0f} s")
    rig.stage.goto_steps(0)
    return np.array(angles), np.stack(L), np.array(N), drops


def validate(rig, freq, cal, n_angles, eps, show=print):
    """Bearings at angles the fit never saw, with the calibration applied.

    Deliberately offset by half a grid step from the calibration sweep: angles
    the fit was given back are a test of arithmetic, not of the array.
    """
    spr = rig.stage.steps_per_rev
    az = np.array([cal["azimuths"][str(p)] for p in rig.ports])
    bw = cal["beamwidth_deg"]
    off = np.array([cal["offsets"][str(p)] for p in rig.ports])
    half = 0.5 * spr / cal["n_angles"]
    show(f"[check] {n_angles} hold-out angles, offset half a grid step "
         f"({half*360.0/spr:.2f} deg) from the calibration sweep")
    rows = []
    for i in range(n_angles):
        target = int(round(half + i * spr / float(n_angles)))
        rig.stage.goto_steps(target)
        stage_deg = target * 360.0 / spr
        for label, o in (("cal", off), ("raw", None)):
            r = rig.dfstream.measure(rig.rx, rig.backend, freq, rig.ports,
                                     rig.null_port,
                                     az if label == "cal" else
                                     np.arange(len(rig.ports)) * 360.0 / len(rig.ports),
                                     bw if label == "cal" else
                                     # webui's own rule, so the comparison is
                                     # against what this repo does today
                                     ((1.33 if len(rig.ports) >= 6 else 0.7)
                                      * 360.0 / len(rig.ports)),
                                     rig.a, offsets=o)
            if not r.get("ok"):
                continue
            rows.append(dict(stage_deg=stage_deg, which=label,
                             bearing=float(r["bearing_deg"]),
                             conf=float(r["confidence"]),
                             cycle_std=r.get("cycle_std_deg")))
    rig.stage.goto_steps(0)

    out = {}
    for label in ("cal", "raw"):
        sel = [r for r in rows if r["which"] == label]
        if len(sel) < 3:
            continue
        acc = bearing_accuracy([r["stage_deg"] for r in sel],
                               [r["bearing"] for r in sel],
                               eps=eps, rotates=rig.a.rotates)
        acc["n"] = len(sel)
        acc["mean_confidence"] = float(np.mean([r["conf"] for r in sel]))
        acc["per_angle"] = [dict(stage_deg=round(r["stage_deg"], 2),
                                 bearing_deg=round(r["bearing"], 2),
                                 error_deg=round(e, 2))
                            for r, e in zip(sel, acc["errors_deg"])]
        out[label] = acc
    return out


# ==========================================================================
# simulation -- the solver against a known truth, no hardware
# ==========================================================================
def simulate(n_port=7, n_angles=48, beamwidth=60.0, floor=-20.0, source=137.0,
             gain_sd=2.0, azimuth_sd=4.0, noise_sd=0.4, drift_sd=1.5,
             sense=1, rotates="array", seed=7, scale_blocks=None,
             steps_per_rev=3200):
    """A synthetic array with known errors, so the solver can be scored.

    This is not a toy: every failure mode the routine is meant to catch is
    switched on here at once -- unequal gains, elements off their nominal
    angles, a drifting reference and per-capture noise -- and the recovered
    numbers are compared against the truth that generated them.  If the solver
    cannot recover a synthetic array it will not recover a real one, and that
    can be established without going outside.
    """
    rng = np.random.default_rng(seed)
    ports = list(range(1, n_port + 1))
    nominal = np.arange(n_port) * 360.0 / n_port
    true_az = nominal + rng.normal(0, azimuth_sd, n_port)
    true_gain = rng.normal(0, gain_sd, n_port)
    true_gain -= true_gain.mean()

    def cut(phi_deg):
        s = 1.0 if sense >= 0 else -1.0
        if rotates == "array":
            d = source - true_az - s * phi_deg
        else:
            d = source + s * phi_deg - true_az
        return dfcore.element_pattern_db(d, 0.0, beamwidth, floor) + true_gain

    def sample(phi_deg, drift):
        return cut(phi_deg) + drift + rng.normal(0, noise_sd, n_port)

    # -- the scale search, on blocks of steps rather than degrees
    block = scale_blocks or max(4, steps_per_rev // n_port // 6)
    rows, drift = [], 0.0
    pos, hard = 0, int(steps_per_rev * 1.4)
    while pos <= hard:
        drift += rng.normal(0, drift_sd * 0.3)
        rows.append(sample(pos * 360.0 / steps_per_rev, drift))
        pos += block
    fit = scale_from_period(common_mode(np.stack(rows)), block, n_port)

    # -- the full turn
    angles = np.arange(n_angles) * 360.0 / n_angles
    L, drift = [], 0.0
    for a in angles:
        drift += rng.normal(0, drift_sd)
        L.append(sample(a, drift))
    L = np.stack(L)

    got = solve(angles, L, ports, eps=sense, rotates=rotates,
                roll_sense=fit["sense"] if fit else None)

    # -- score against the truth
    rec_az = np.array(got["azimuths_deg"])
    # Both are only defined up to a rigid rotation, so remove the common part
    # before comparing -- otherwise the reference convention would show up as
    # an error the solver did not make.
    d = wrap180(rec_az - true_az)
    d = wrap180(d - np.degrees(np.angle(np.mean(np.exp(1j * np.radians(d))))))
    score = dict(
        steps_per_rev_true=steps_per_rev,
        steps_per_rev_est=None if fit is None else round(fit["steps_per_rev"], 1),
        steps_per_rev_error_pct=None if fit is None else round(
            100.0 * (fit["steps_per_rev"] - steps_per_rev) / steps_per_rev, 2),
        roll_sense_true=sense * (1 if rotates == "array" else -1),
        roll_sense_est=None if fit is None else fit["sense"],
        roll_sense_from_peaks=got["roll_sense_from_peaks"],
        counter_clockwise_est=got["counter_clockwise"],
        scale_coherence=None if fit is None else round(fit["coherence"], 3),
        gain_rms_db=round(float(np.sqrt(np.mean(
            (np.array(got["offsets_db"]) - true_gain) ** 2))), 3),
        azimuth_rms_deg=round(float(np.sqrt(np.mean(d ** 2))), 3),
        azimuth_max_deg=round(float(np.max(np.abs(d))), 3),
        beamwidth_true=beamwidth,
        beamwidth_est=round(got["beamwidth"]["beamwidth_deg"], 1),
        floor_true=floor, floor_est=round(got["beamwidth"]["floor_db"], 1),
        order_ok=got["port_order_is_azimuth_order"],
        ring_residual_db=round(got["ring_residual_db"], 2))
    return got, score, dict(azimuths=true_az.tolist(), gains=true_gain.tolist())


# ==========================================================================
# CLI
# ==========================================================================
def _report(cal, show=print):
    ports = cal["ports"]
    show("")
    show("=" * 72)
    show("CALIBRATION")
    show("=" * 72)
    show(f"  reference:      {cal['reference']}")
    show(f"  beamwidth:      {cal['beamwidth_deg']:.0f} deg "
         f"(floor {cal['floor_db']:.0f} dB, fit residual "
         f"{cal['beamwidth']['rms_db']:.2f} dB)")
    rule = 0.7 if len(ports) < 6 else 1.33
    show(f"                  the built-in rule would have used "
         f"{rule * 360.0/len(ports):.0f} deg")
    show(f"  ring residual:  {cal['ring_residual_db']:.2f} dB unexplained by "
         f"one identical pattern per element")
    show("")
    show("  port   azimuth   vs nominal    gain     pattern depth   1st harm")
    for i, p in enumerate(ports):
        show(f"  {p:>4}   {cal['azimuths'][str(p)]:7.2f}   "
             f"{cal['spacing_error_deg'][i]:+8.2f}   "
             f"{cal['offsets'][str(p)]:+6.2f} dB   "
             f"{cal['modulation_db'][i]:8.1f} dB   "
             f"{cal['fundamental_db'][i]:6.1f} dB")
    show("")
    show(f"  measured spacings (deg): {cal['measured_spacings_deg']}")
    show(f"  worst departure from a uniform ring: "
         f"{cal['max_spacing_error_deg']:.2f} deg")
    if not cal["port_order_is_azimuth_order"]:
        show(f"  WARNING: the ports are not in azimuth order. Going round the "
             f"ring they run {cal['azimuth_order']}, not {ports}. Every "
             f"uncalibrated bearing this array has ever produced was "
             f"scrambled; with this file loaded they are not.")
    if cal.get("counter_clockwise"):
        show("  the array is wired ANTICLOCKWISE -- uncalibrated bearings "
             "from it were MIRRORED about the home axis (dfcore's "
             "`counter_clockwise` case, which no stability check can detect).")
    # Relative, not a fixed dB bar. Removing the common mode leaves a few dB
    # of the OTHER elements' pattern on a port that has none of its own, so an
    # absolute threshold set low enough to be safe never fires and one set
    # high enough to fire condemns working elements at a quiet site.
    fund = np.asarray(cal["fundamental_db"], float)
    med = float(np.median(fund))
    weak = [p for i, p in enumerate(ports) if fund[i] < 0.35 * med]
    if weak:
        show(f"  WARNING: ports {weak} showed under 35% of the median "
             f"one-cycle-per-turn modulation ({med:.1f} dB). An element that "
             f"cannot tell pointing at the source from pointing away "
             f"contributes no bearing information, only weight -- and the "
             f"weighting in dfcore.estimate_bearing is by SNR, which a dead "
             f"port with a hot preamp can still win. Check those feeds, and "
             f"drop them from --ports until they are fixed.")
    if cal.get("null_modulation_db") is not None:
        nm = cal["null_modulation_db"]
        amod = float(np.median(cal["modulation_db"]))
        show(f"  null port varied {nm:.2f} dB over the turn, against "
             f"{amod:.1f} dB for a working element -- it has no antenna on "
             f"it, so anything much above the drift floor is leakage into the "
             f"switch or a mis-declared port")
        if nm > 0.3 * amod:
            show("           WARNING: that is too much. The null is both the "
                 "sync marker and the noise reference the level subtraction "
                 "uses, so a null that hears the source biases every antenna "
                 "level toward it -- and it does so most where the array is "
                 "pointing at the source, which is where the bearing is "
                 "decided.")


def _report_check(v, show=print):
    show("")
    show("=" * 72)
    show("ACCURACY, on angles the fit never saw")
    show("=" * 72)
    for label, title in (("raw", "uncalibrated (even spacing, no gains)"),
                         ("cal", "calibrated")):
        if label not in v:
            continue
        a = v[label]
        show(f"  {title}:")
        show(f"      rms {a['rms_deg']:5.2f} deg, worst {a['max_deg']:5.2f} deg, "
             f"over {a['n']} angles")
        show(f"      slope {a['slope']:+.3f} deg/deg (must be "
             f"{a['expected_slope']:+.0f}); mean confidence "
             f"{a['mean_confidence']:.2f}")
    if "cal" in v and "raw" in v:
        d = v["raw"]["rms_deg"] - v["cal"]["rms_deg"]
        show(f"  calibration changed the rms error by {-d:+.2f} deg "
             + ("(better)" if d > 0 else "(WORSE -- do not save this)"))
    if "cal" in v and abs(abs(v["cal"]["slope"]) - 1.0) > 0.1:
        show("  WARNING: the slope is not +-1. The reported bearing is not "
             "tracking the stage one-for-one, which no repeatability figure "
             "can see: every angle can be perfectly repeatable while the map "
             "between stage and bearing is stretched. Suspect the stage "
             "scale (--find-scale) or a non-uniform ring.")
    if "cal" in v:
        show(f"  the reference transmitter sits at "
             f"{v['cal']['source_bearing_deg']:.1f} deg in the calibrated "
             f"frame -- pass it back as --true-bearing next time, with a "
             f"compass reading, to make the file absolute")


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("freq", nargs="?", default=None,
                   help="the reference transmitter, e.g. 96.0M. A strong, "
                        "steady, distant carrier -- a local FM station is "
                        "ideal and a bursty one (WiFi) is not.")
    # array
    p.add_argument("--ports", default="1,2,3,4,5,6,7",
                   help="antenna ports in wiring order (default: the "
                        "7-element ring)")
    p.add_argument("--null-port", type=int, default=8,
                   help="the dead position (default 8)")
    p.add_argument("--switch", default="auto", choices=("auto", "esp32", "usrp"))
    p.add_argument("--switch-device", default="/dev/ttyACM0")
    # geometry -- the two facts rotation cannot supply
    p.add_argument("--rotates", default="array", choices=("array", "source"),
                   help="what is bolted to the stage. Getting this wrong "
                        "MIRRORS every bearing and nothing in the data can "
                        "tell.")
    p.add_argument("--dir-plus", default="cw", choices=("cw", "ccw"),
                   help="which way a positive step command turns the stage, "
                        "seen from above")
    p.add_argument("--true-bearing", type=float, default=None,
                   help="known azimuth of the reference transmitter, if you "
                        "have one. Makes the output absolute instead of "
                        "home-relative.")
    # stage
    p.add_argument("--stage-config", default=None)
    p.add_argument("--steps-per-rev", type=int, default=None)
    p.add_argument("--step-rate", type=float, default=None)
    p.add_argument("--backlash-steps", type=int, default=None)
    p.add_argument("--dir-sign", type=int, default=None, choices=(1, -1))
    p.add_argument("--no-stage", action="store_true",
                   help="do not touch the motor (preflight and --simulate only)")
    # what to run
    p.add_argument("--check", action="store_true", help="preflight only")
    p.add_argument("--find-scale", action="store_true",
                   help="measure steps per revolution off the RF first")
    p.add_argument("--angles", type=int, default=36,
                   help="samples in the calibration turn")
    p.add_argument("--check-angles", type=int, default=12,
                   help="hold-out angles for the accuracy test (0 to skip)")
    p.add_argument("--repeats", type=int, default=3,
                   help="captures averaged per angle")
    p.add_argument("--simulate", action="store_true",
                   help="run the solver on a synthetic array and score it")
    p.add_argument("--out", default=DEFAULT_CAL)
    p.add_argument("--save", action="store_true",
                   help="write the calibration file (otherwise dry run)")
    p.add_argument("--verbose", action="store_true")
    # scale search
    p.add_argument("--scale-guess", type=int, default=3200,
                   help="rough steps/rev, used ONLY to size the search blocks")
    p.add_argument("--scale-block", type=int, default=None)
    p.add_argument("--scale-max-mult", type=float, default=4.0)
    p.add_argument("--scale-score", type=float, default=0.55)
    p.add_argument("--probe-steps", type=int, default=200)
    # radio -- same defaults and meanings as dfstream
    p.add_argument("--slot-us", type=float, default=200.0)
    p.add_argument("--record-ms", type=float, default=100.0)
    p.add_argument("--guard", type=float, default=0.25)
    p.add_argument("--min-contrast", type=float, default=3.0)
    p.add_argument("--rate", type=float, default=16e6)
    p.add_argument("--lo-frac", type=float, default=0.25)
    p.add_argument("--rbw", type=float, default=200e3)
    p.add_argument("--gain", type=float, default=30.0)
    p.add_argument("--antenna", default="TX/RX")
    p.add_argument("--rx-chan", type=int, default=0)
    p.add_argument("--tune-settle", type=float, default=0.01)
    p.add_argument("--track-dc", action="store_true")
    p.add_argument("--check-passes", type=int, default=3)
    a = p.parse_args()
    a.park = None
    a.verify_switch = True

    if a.simulate:
        eps = 1 if a.dir_plus == "cw" else -1
        got, score, truth = simulate(
            n_port=len(a.ports.split(",")), n_angles=a.angles,
            sense=eps, rotates=a.rotates)
        cal = _to_cal_file(got, a, eps=eps, freq=None)
        _report(cal)
        print("")
        print("=" * 72)
        print("SIMULATION SCORE -- recovered against the truth that made it")
        print("=" * 72)
        for k, v in score.items():
            print(f"  {k:26} {v}")
        return 0

    if not a.freq:
        p.error("a reference frequency is required (or use --simulate)")

    eps = 1 if a.dir_plus == "cw" else -1
    rig = Rig(a)
    freq = rig.rfscan.parse_freq(a.freq)
    print(f"[dfcal] {freq/1e6:.4f} MHz, ports {rig.ports}, null "
          f"{rig.null_port}, {rig.backend.describe()}")
    print(f"[dfcal] {rig.stage.describe()}")
    print(f"[dfcal] the stage carries the {a.rotates}; +steps turns it "
          f"{a.dir_plus.upper()}")

    try:
        if not preflight(rig, freq):
            print("\n[dfcal] preflight failed -- not rotating. Fix the above "
                  "first; a calibration measured through a broken switch is "
                  "worse than none, because it is believed.")
            return 1
        if a.check:
            return 0

        roll_sense = None
        if a.find_scale or not rig.stage.steps_per_rev:
            fit = find_scale(rig, freq)
            if fit is None:
                return 1
            roll_sense = fit["sense"]
            rig.stage.goto_steps(0)

        angles, L, N, drops = sweep(rig, freq, a.angles)
        if drops > a.angles // 4:
            print(f"[dfcal] {drops} of {a.angles} angles failed to segment -- "
                  f"too many gaps to fit a pattern through. Raise "
                  f"--record-ms or --slot-us, or pick a stronger reference.")
            return 1

        got = solve(angles, L, rig.ports, eps=eps, rotates=a.rotates,
                    true_bearing=a.true_bearing, null_col=N,
                    roll_sense=roll_sense)
        cal = _to_cal_file(got, a, eps=eps, freq=freq)
        _report(cal)

        if a.check_angles:
            v = validate(rig, freq, cal, a.check_angles, eps)
            cal["accuracy"] = v
            _report_check(v)

        if a.save:
            with open(a.out, "w") as f:
                json.dump(cal, f, indent=2)
            print(f"\n[dfcal] wrote {a.out}")
            print(f"[dfcal] use it:  ./webui.py --cal {a.out} "
                  f"--ports {a.ports} --null-port {a.null_port}")
            print(f"[dfcal] NOTE: dfcore.Calibration.save() writes back only "
                  f"`offsets` and `azimuths`; everything else in this file is "
                  f"a record of how they were measured and would be dropped.")
            if rig.stage.steps_per_rev:
                print(f"[dfcal] stage config -> "
                      f"{rig.stage.save_config(a.stage_config or __import__('stepper').CONFIG)}")
        else:
            print("\n[dfcal] dry run -- pass --save to write the file")
    except KeyboardInterrupt:
        print("\ninterrupted")
        return 130
    finally:
        rig.close()
    return 0


def _to_cal_file(got, a, eps, freq):
    """dfcore.Calibration's two dicts, plus everything used to get them."""
    ports = got["ports"]
    cal = dict(got)
    cal.pop("azimuths_deg", None)
    cal["offsets"] = {str(p): round(float(v), 3)
                      for p, v in zip(ports, got["offsets_db"])}
    cal["azimuths"] = {str(p): round(float(v), 3)
                       for p, v in zip(ports, got["azimuths_deg"])}
    cal["beamwidth_deg"] = round(got["beamwidth"]["beamwidth_deg"], 1)
    cal["floor_db"] = round(got["beamwidth"]["floor_db"], 1)
    fund = np.asarray(got["fundamental_db"], float)
    cal["weak_ports"] = [p for p, f in zip(ports, fund)
                         if f < 0.35 * float(np.median(fund))]
    cal["meta"] = dict(
        tool="dfcal.py", freq_hz=freq, rotates=a.rotates, dir_plus=a.dir_plus,
        dir_sense=int(eps), roll_sense=got.get("roll_sense"),
        counter_clockwise=got.get("counter_clockwise"),
        true_bearing=a.true_bearing,
        null_port=a.null_port, angles=a.angles, repeats=a.repeats,
        rbw_hz=a.rbw, slot_us=a.slot_us, record_ms=a.record_ms,
        created=time.strftime("%Y-%m-%dT%H:%M:%S"))
    return cal


if __name__ == "__main__":
    raise SystemExit(main())
