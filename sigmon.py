#!/usr/bin/env python3
"""sigmon -- monitor a frequency range, find strong signals, bear them, log them.

    ./sigmon.py 88M 108M                       # FM band, run until Ctrl-C
    ./sigmon.py 88M 108M --passes 20           # bounded run
    ./sigmon.py 2412M 2462M --step 5M --rbw 5M --gain 25 \
                --peak-hold 8 --auto-balance        # 2.4 GHz, bursty
    ./sigmon.py 88M 108M --stability-report    # how repeatable are the bearings?

How it works, per pass:

  1. Step the RF switch across the live antenna ports, capturing the whole span
     at each and computing a Welch PSD.  Which ports are live is DETECTED, not
     assumed -- an open port sits at the receiver's own noise floor, and
     including one would corrupt every bearing.
  2. Find peaks standing --threshold dB above the noise floor.
  3. For each peak, take the per-antenna levels and fit a bearing by weighted
     amplitude comparison.
  4. Write an observation per signal per pass, and keep a running per-emitter
     summary with its circular mean bearing and stability.

Bearings are RELATIVE unless a calibration file is supplied: repeatable and
comparable to each other, but not tied to true north.  Stability is the useful
thing to measure first, and it does not need calibration.
"""
import argparse
import os
import signal
import sys
import time
import uuid

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.dirname(_HERE))
# uarf sits beside this project. The older spelling resolved to
# <workspace>/../uarf, which does not exist, so `import rfscan` only ever
# worked from an externally set PYTHONPATH.
for _p in (os.path.join(os.path.dirname(_HERE), "uarf"),
           os.path.join(os.path.dirname(os.path.dirname(_HERE)), "uarf")):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

import rfscan                      # noqa: E402  also sets UHD_IMAGES_DIR
import uhd                         # noqa: E402

import dfcore                      # noqa: E402
import dfstream                    # noqa: E402
import swbackend                   # noqa: E402
from store import Store, utcnow    # noqa: E402

try:
    sys.stdout.reconfigure(line_buffering=True)
except AttributeError:
    pass

STOP = False


def _sigint(_sig, _frm):
    global STOP
    if STOP:
        sys.exit(130)
    STOP = True
    print("\n[sigmon] stopping after this pass (Ctrl-C again to force)")


def parse_freq(s):
    return rfscan.parse_freq(s)


class Sweeper:
    """Capture the span on every live antenna port and return a level table."""

    def __init__(self, usrp, args, backend):
        self.usrp = usrp
        self.a = args
        # The switch may not be on the B210 at all any more. When the esp32
        # board drives it, writing the B210's GPIO changes nothing -- and the
        # sweep would quietly return the SAME antenna four times, which looks
        # like four antennas that agree perfectly.
        self.backend = backend
        self.rx = rfscan.Receiver(usrp, args.rx_chan, args.rate, args.gain,
                                  args.antenna, lo_frac=args.lo_frac)
        # plan_segments returns (centre, lo, hi) per tuning segment.
        self.segments = rfscan.plan_segments(args.start, args.stop,
                                             self.rx.rate * args.seg_frac)
        self.nsamps = int(args.dwell * self.rx.rate)

        # One global frequency grid, built once, with each point assigned to
        # exactly one segment.  Deriving the grid per segment instead lets the
        # point count drift between channels, and then the level table is
        # ragged and every bearing is computed from mismatched frequencies.
        self.freqs = np.arange(args.start, args.stop + args.step / 2, args.step)
        self.seg_of = []
        for fq in self.freqs:
            best_i, best_d = 0, None
            for i, (centre, lo, hi) in enumerate(self.segments):
                d = abs(fq - centre)
                if lo - args.step <= fq <= hi + args.step and (best_d is None or d < best_d):
                    best_i, best_d = i, d
            if best_d is None:                      # outside every segment
                best_i = int(np.argmin([abs(fq - c) for c, _, _ in self.segments]))
            self.seg_of.append(best_i)
        self.seg_of = np.array(self.seg_of)

    def sweep(self, channels):
        """Return (freqs, table) with table[i] the levels for channels[i].

        Loop order matters more than anything else here.  The obvious nesting --
        for each antenna, sweep the whole band -- puts a FULL SWEEP between
        antenna 0's reading and antenna 7's at any given frequency.  Amplitude
        comparison assumes the eight levels describe the same signal at the same
        moment, and over a second or two of separation they simply do not: the
        bearing is then computed from levels taken at different times, which is
        what made survey bearings unreliable.

        Tuning on the outside and commutating on the inside puts all eight
        readings of a frequency within one antenna cycle -- about 16 ms -- and
        as a bonus retunes once per segment instead of once per antenna per
        segment.  Measured on this board: sw.select() costs 0.026 ms, so the
        commutation itself is free; it was the retune and its settle that made
        the old order expensive.
        """
        if self.a.commutate:
            return self.sweep_commutated(channels)
        rows = [np.full(len(self.freqs), np.nan) for _ in channels]
        for i, (centre, _lo, _hi) in enumerate(self.segments):
            sel = np.nonzero(self.seg_of == i)[0]
            if not len(sel):
                continue
            self.rx.tune(centre)
            time.sleep(self.a.tune_settle)
            for c_i, ch in enumerate(channels):
                self.backend.hold(ch)
                if self.a.switch_settle > 0:
                    time.sleep(self.a.switch_settle)
                x = self.rx.capture(self.nsamps)
                if len(x) < self.nsamps // 2:
                    raise RuntimeError("short capture from the radio")
                if self.a.peak_hold > 1:
                    f, P = dfcore.psd_peak_hold(x, self.rx.rate, self.a.rbw,
                                                self.a.peak_hold,
                                                rfscan.welch_psd)
                else:
                    f, P = rfscan.welch_psd(x, self.rx.rate, self.a.rbw)
                for k in sel:
                    rows[c_i][k] = rfscan.band_power_db(
                        f, P, self.freqs[k] - centre, self.a.rbw)
        return self.freqs, np.vstack(rows)

    def sweep_commutated(self, channels):
        """One continuous capture per tuning segment, switch running under it.

        The loop-order fix documented above put all four readings of a
        frequency inside one antenna cycle.  This goes further and removes the
        per-antenna capture entirely: with the esp32 free-running there is one
        capture per SEGMENT, and the four spectra come out of it together.

        Requires the null port, like every other commutated measurement --
        without a dip to sync on there is no way to say which samples were
        which antenna.
        """
        seq = list(channels) + [self.a.null_port]
        rows = [np.full(len(self.freqs), np.nan) for _ in channels]
        slot_s = self.a.slot_us * 1e-6
        for i, (centre, _lo, _hi) in enumerate(self.segments):
            sel = np.nonzero(self.seg_of == i)[0]
            if not len(sel):
                continue
            x, info = dfstream.record(
                self.rx, self.backend, centre, seq, slot_s,
                self.a.record_ms * 1e-3, tune_settle=self.a.tune_settle,
                park=None, verify=self.a.verify_switch)
            if info.get("switch_moving") is False:
                raise RuntimeError(
                    "the switch did not move during the capture -- every slot "
                    "is the same antenna")
            f, P, seg = dfstream.slot_psd(
                x, self.rx.rate, 0.0, self.a.rbw,
                dfstream.achieved_slot(self.a.slot_us, info),
                n_ant=len(channels), guard=self.a.guard,
                min_contrast_db=self.a.min_contrast)
            if f is None:
                raise RuntimeError(f"commutation sync failed: {seg['reason']}")
            self.last_sync = seg
            for c_i in range(len(channels)):
                for k in sel:
                    rows[c_i][k] = rfscan.band_power_db(
                        f, P[c_i], self.freqs[k] - centre, self.a.rbw)
        return self.freqs, np.vstack(rows)


def probe_live_channels(sweeper, args, probe_ports):
    """Work out which switch ports have antennas, by measurement."""
    print("[sigmon] probing which switch ports carry an antenna ...")
    # Probed with the ORDINARY sweep even under --commutate: the commutated
    # sweep needs the null port to already be known-dead to sync on, so it
    # cannot be the thing that establishes it.
    was, sweeper.a.commutate = sweeper.a.commutate, False
    try:
        freqs, table = sweeper.sweep(list(probe_ports))
    finally:
        sweeper.a.commutate = was
    live_i, info = dfcore.find_live_channels(
        table, args.live_margin, step_hz=args.step, rbw_hz=args.rbw,
        min_structure_db=args.min_structure, min_agreement=args.min_agreement)
    # Rows are probe order; the values are switch port numbers. Not the same
    # thing once the ports start at 1.
    live = [probe_ports[i] for i in live_i]
    for i, ch in enumerate(probe_ports):
        mark = "antenna" if ch in live else "open"
        note = "  <- no-signal reference" if ch == args.null_port else ""
        print(f"    port {ch}: agreement {info['agreement'][i]:+5.2f}   "
              f"structure {info['structure'][i]:5.2f} dB   "
              f"lift {info['lift'][i]:5.1f} dB   -> {mark}{note}")
    print(f"    (agreement is the test: >= {args.min_agreement:.2f} correlation with")
    print("     another port's narrowband detail means both see the same")
    print("     transmitters, so both have antennas. structure and lift are")
    print("     shown for context; lift only means anything when some ports")
    print("     really are open.)")
    if args.null_port in live:
        print(f"\n[sigmon] WARNING: port {args.null_port} is declared the "
              f"NO-SIGNAL position but reads as live. Something is connected "
              f"to it;\n         dfstream.py cannot sync on it and its noise "
              f"reference would remove real signal.")
    live = [c for c in live if c != args.null_port]

    if len(live) < 3:
        print(f"\n[sigmon] only {len(live)} live port(s) found. Amplitude DF needs")
        print("         at least 3 to constrain a bearing in 360 degrees; with 2")
        print("         the answer is ambiguous and with 1 there is none.")
        if len(live) < 2:
            sys.exit(1)
    return live


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("start", type=parse_freq, help="range start, e.g. 88M")
    p.add_argument("stop", nargs="?", type=parse_freq, default=None)
    p.add_argument("--step", type=parse_freq, default=None,
                   help="frequency step (default: the RBW)")
    p.add_argument("--rbw", type=parse_freq, default=200e3,
                   help="resolution bandwidth (default 200k, an FM channel)")
    p.add_argument("--rate", type=float, default=16e6)
    p.add_argument("--lo-frac", type=float, default=0.25, dest="lo_frac",
                   help="LO offset as a fraction of the rate (see webui.py)")
    p.add_argument("--seg-frac", type=float, default=0.44, dest="seg_frac",
                   help="fraction of the rate kept from each tuning")
    p.add_argument("--gain", type=float, default=30)
    p.add_argument("--antenna", default="TX/RX", help="switch common port")
    p.add_argument("--rx-chan", type=int, default=0)
    p.add_argument("--gpio-mask", type=lambda s: int(s, 0), default=0xE0)
    p.add_argument("--switch", default="auto", choices=("auto", "esp32", "usrp"),
                   help="who moves the switch. The esp32 board commutates by "
                        "itself; the B210's GPIO needs a host thread. Getting "
                        "this wrong is not subtle-but-harmless: if the switch "
                        "is on the esp32 and this says usrp, every antenna in "
                        "the table is the same antenna.")
    p.add_argument("--switch-device", default="/dev/ttyACM0")
    p.add_argument("--ports", default=None,
                   help="the four antenna ports in array order (default: "
                        "1,2,3,4 on the esp32, 0,1,2,3 on the usrp)")
    p.add_argument("--null-port", type=int, default=None,
                   help="switch code of the NO-SIGNAL position (default: 5 on "
                        "the esp32, 4 on the usrp). Probed like any other and "
                        "expected to come out dead -- the cheapest possible "
                        "check that it really is the reference the commutated "
                        "measurements rely on.")
    p.add_argument("--commutate", action="store_true",
                   help="sweep with ONE continuous capture per tuning segment "
                        "and the switch running underneath it, instead of one "
                        "capture per antenna per segment. Needs the null port. "
                        "On the esp32 this is also what keeps the sweep from "
                        "writing flash on every port change.")
    p.add_argument("--slot-us", type=float, default=200.0,
                   help="[--commutate] microseconds per switch position")
    p.add_argument("--record-ms", type=float, default=100.0,
                   help="[--commutate] capture length per tuning segment")
    p.add_argument("--guard", type=float, default=0.25,
                   help="[--commutate] fraction discarded at each slot end")
    p.add_argument("--min-contrast", type=float, default=3.0,
                   help="[--commutate] dB the antennas must stand above the null")
    p.add_argument("--no-verify-switch", action="store_false",
                   dest="verify_switch",
                   help="skip the switch step-counter check around each capture")
    p.add_argument("--dwell", type=float, default=0.005,
                   help="capture per antenna per segment (s). Short on purpose: "
                        "a fast antenna cycle keeps the levels comparable, which "
                        "matters more than per-capture SNR. Raise --passes to "
                        "average instead of raising this.")
    p.add_argument("--switch-settle", type=float, default=0.0002)
    p.add_argument("--tune-settle", type=float, default=0.01)
    p.add_argument("--threshold", type=float, default=10.0,
                   help="dB above the noise floor to call something a signal")
    p.add_argument("--passes", type=int, default=0, help="0 = until Ctrl-C")
    p.add_argument("--interval", type=float, default=0.0,
                   help="seconds to wait between passes")
    p.add_argument("--beamwidth", type=float, default=None,
                   help="element 3 dB beamwidth (default: 0.7 x element "
                        "spacing; equal to the spacing makes the fit snap to "
                        "the boresights -- see --stability-report)")
    p.add_argument("--channels", default=None,
                   help="force the live port list, e.g. 0,1,2,3")
    p.add_argument("--cal", default=None, help="calibration JSON")
    p.add_argument("--counter-clockwise", "--ccw", action="store_true",
                   dest="counter_clockwise",
                   help="ports run anticlockwise around the array. Bearings "
                        "increase clockwise, so getting this wrong MIRRORS "
                        "every bearing about 0 deg -- and no stability check "
                        "can detect it, because a mirror leaves circular std "
                        "unchanged.")
    p.add_argument("--array-offset", type=float, default=0.0,
                   help="degrees to rotate the assumed element azimuths, i.e. "
                        "where port 0 actually points")
    p.add_argument("--live-margin", type=float, default=3.0)
    p.add_argument("--min-structure", type=float, default=1.0,
                   help="dB of detrended narrowband detail for a port to count "
                        "as having an antenna (see dfcore.spectral_structure)")
    p.add_argument("--min-agreement", type=float, default=0.35,
                   help="correlation with another port's detrended spectrum "
                        "for a port to count as having an antenna")
    p.add_argument("--peak-hold", type=int, default=1, metavar="N",
                   help="max-hold over N sub-blocks per dwell. Use for BURSTY "
                        "signals (WiFi): averaging measures duty cycle, not "
                        "path gain, and each antenna sees different traffic. "
                        "1 = plain averaging, right for continuous signals (FM).")
    p.add_argument("--show-levels", action="store_true",
                   help="print the per-antenna levels behind each bearing")
    p.add_argument("--auto-balance", action="store_true",
                   help="equalise the antennas' band-average levels first")
    p.add_argument("--min-confidence", type=float, default=0.0,
                   help="drop bearings whose fit is flatter than this (0..1)")
    p.add_argument("--mongo-uri", default=os.environ.get(
        "SIGMON_MONGO_URI", "mongodb://sigmon:sigmon@127.0.0.1:27017/?authSource=admin"))
    p.add_argument("--mongo-db", default=os.environ.get("SIGMON_MONGO_DB", "signals"))
    p.add_argument("--fallback", default="sigmon_fallback.jsonl")
    p.add_argument("--stability-report", action="store_true",
                   help="print per-signal bearing stability at the end")
    p.add_argument("--top", type=int, default=12,
                   help="how many signals to show per pass")
    a = p.parse_args()

    if a.stop is None:
        a.stop = a.start + a.rbw
    if a.step is None:
        a.step = a.rbw
    if a.stop < a.start:
        p.error("stop must be above start")

    signal.signal(signal.SIGINT, _sigint)

    # rfscan.default_fpga() resolves the marker relative to itself, which is
    # where detect_fpga.sh writes it; the path spelled out here was one
    # directory too high and always found nothing.
    fpga = rfscan.default_fpga()
    dev = "type=b200" + (f",fpga={fpga}" if fpga else "")
    try:
        usrp = uhd.usrp.MultiUSRP(dev)
    except RuntimeError as e:
        sys.exit(f"error: could not open the radio.\n  {e}\n"
                 "  The B210 is single-session -- is something else holding it?")

    backend = swbackend.open_backend(a.switch, usrp=usrp,
                                     device=a.switch_device,
                                     gpio_mask=a.gpio_mask,
                                     auto=(a.switch == "auto"))
    ports = ([int(c) for c in a.ports.split(",")] if a.ports
             else list(backend.default_ports))
    if a.null_port is None:
        a.null_port = backend.default_null
    probe_ports = list(ports) + [a.null_port]

    args_ns = a
    sweeper = Sweeper(usrp, args_ns, backend)
    print(f"[sigmon] {a.start/1e6:.3f} - {a.stop/1e6:.3f} MHz, step "
          f"{a.step/1e6:.3f} MHz, RBW {a.rbw/1e3:.0f} kHz, "
          f"{len(sweeper.segments)} tuning segment(s)")
    print(f"[sigmon] switch: {backend.describe()}")
    if backend.name == "esp32" and not a.commutate:
        # Every port change on that board is a flash write, and this path makes
        # one per antenna per segment on every pass, forever.
        print(f"[sigmon] NOTE: without --commutate this holds each antenna in "
              f"turn, which is {len(probe_ports)*len(sweeper.segments)} NVS "
              f"writes per pass on the esp32. Use --commutate for a long run.")

    if a.channels:
        live = [int(x) for x in a.channels.split(",")]
        print(f"[sigmon] live ports forced to {live}")
    else:
        live = probe_live_channels(sweeper, a, probe_ports)
    n_live = len(live)

    cal = dfcore.Calibration.load(a.cal)
    azimuths = cal.azimuth_for_ports(live, ports,
                                     counter_clockwise=a.counter_clockwise,
                                     offset_deg=a.array_offset)
    # The right beamwidth depends on how many elements there are, and the two
    # cases fail in opposite directions.  Both measured on this array, 8 passes
    # of the FM band each:
    #
    #   4 elements, 90 deg spacing: at 1.0x spacing the fit collapses onto the
    #       boresights (median 2.7 deg from one) -- four constraints are too
    #       few, so it degenerates into "which antenna is loudest". 0.7x fixes
    #       it (median 37 deg) with no loss of repeatability.
    #
    #   8 elements, 45 deg spacing: 0.7x (31 deg) is now too NARROW -- adjacent
    #       beams cross about 6 dB down and the crossover is noisy. Pooling
    #       every run made on this array, fraction of signals coming out
    #       stable: 0.7x spacing 29%, 1.0x 64%, 1.33x 72%.
    #
    # So: narrow the beams when elements are scarce, widen past the spacing
    # when they are not. Both numbers are weakly determined -- the run-to-run
    # spread at a fixed beamwidth is comparable to the difference between
    # beamwidths -- so treat them as a starting point and let the snapping
    # check in --stability-report judge a given site.
    spacing = 360.0 / n_live
    beamwidth = a.beamwidth or ((1.33 if n_live >= 6 else 0.7) * spacing)
    print(f"[sigmon] using ports {live} at assumed azimuths "
          f"{np.round(azimuths,1).tolist()} deg, beamwidth {beamwidth:.0f} deg")
    print(f"[sigmon] array wired "
          f"{'ANTICLOCKWISE' if a.counter_clockwise else 'clockwise'}"
          f"{f', port 0 at {a.array_offset:.0f} deg' if a.array_offset else ''}")
    if cal.calibrated:
        print(f"[sigmon] calibration from {cal.source}")
    else:
        print("[sigmon] NO CALIBRATION: bearings are relative, not true north.")
        print("         They are still comparable pass-to-pass, which is what")
        print("         the stability report measures.")

    store = Store(a.mongo_uri, a.mongo_db, a.fallback)
    run_id = str(uuid.uuid4())
    store.insert_run(dict(
        run_id=run_id, ts=utcnow(), start_hz=a.start, stop_hz=a.stop,
        step_hz=a.step, rbw_hz=a.rbw, rate_hz=sweeper.rx.rate, gain_db=a.gain,
        channels=live, azimuths_deg=azimuths.tolist(), beamwidth_deg=beamwidth,
        calibrated=cal.calibrated, threshold_db=a.threshold))
    print(f"[sigmon] run_id {run_id}\n")

    n_avg = int(a.dwell * sweeper.rx.rate * a.rbw / sweeper.rx.rate)
    history = {}
    npass = 0
    t_start = time.time()

    try:
        while not STOP and (a.passes == 0 or npass < a.passes):
            npass += 1
            t0 = time.time()
            try:
                freqs, table = sweeper.sweep(live)
            except RuntimeError as e:
                print(f"[sigmon] pass {npass}: {e}; retrying")
                continue

            if sweeper.rx.clipped:
                print(f"[sigmon] WARNING: ADC overload (peak {sweeper.rx.peak:.2f} "
                      f"of full scale) -- levels and bearings are both wrong. "
                      f"Lower --gain.")

            table = np.vstack([table[i] - cal.offsets.get(ch, 0.0)
                               for i, ch in enumerate(live)])

            if a.auto_balance:
                # Blind gain equalisation.  Signals across a whole broadcast
                # band arrive from many directions, so each antenna's average
                # over the band should be about equal IF the antennas are
                # equivalent.  Whatever difference remains is gain imbalance --
                # feedline loss, a mismatched element, one antenna simply
                # placed better -- and it biases every bearing toward the
                # hottest port.  Subtracting it is the cheapest way to tell a
                # real bearing spread from that artefact.
                bal = np.nanmean(table, axis=1)
                table = table - (bal - bal.mean())[:, None]
                if npass == 1:
                    print("[sigmon] auto-balance offsets (dB): " +
                          ", ".join(f"ch{c}:{v:+.1f}"
                                    for c, v in zip(live, bal - bal.mean())))
            best = table.max(axis=0)
            sigs, floor = dfcore.detect_signals(freqs, best, a.threshold)

            print(f"--- pass {npass}  {time.time()-t0:.2f} s  "
                  f"floor {floor:.1f} dBFS  {len(sigs)} signal(s) ---")
            if sigs:
                print(f"  {'MHz':>10} {'dBFS':>8} {'SNR':>7} {'bearing':>9} "
                      f"{'conf':>6} {'n':>4} {'stab':>8}")

            for s in sorted(sigs, key=lambda d: -d["level_db"])[:a.top]:
                lv = table[:, s["index"]]
                snr = lv - floor
                bearing, resid, grid = dfcore.estimate_bearing(
                    lv, azimuths, beamwidth, snr, n_avg=max(n_avg, 1))
                conf = dfcore.bearing_confidence(resid, grid)

                keep = conf >= a.min_confidence
                h = history.setdefault(s["freq"], [])
                if keep:
                    h.append(bearing)

                stab = dfcore.circ_std_deg(h) if len(h) >= 3 else float("nan")
                stab_s = f"{stab:7.1f}d" if np.isfinite(stab) else f"{'-':>8}"
                print(f"  {s['freq']/1e6:>10.3f} {s['level_db']:>8.1f} "
                      f"{s['snr_db']:>6.1f}d {bearing:>8.1f}d {conf:>6.2f} "
                      f"{len(h):>4} {stab_s}")
                if a.show_levels:
                    # The levels are the whole input to the bearing.  If one
                    # port is always the strongest, every bearing points at it
                    # and the spread is an artefact, not direction finding.
                    rel = lv - lv.max()
                    print("             " + "  ".join(
                        f"ch{c}:{d:+6.1f}" for c, d in zip(live, rel)) +
                        f"   (spread {np.ptp(lv):.1f} dB, "
                        f"strongest ch{live[int(np.argmax(lv))]})")

                store.insert_observation(dict(
                    run_id=run_id, ts=utcnow(), freq_hz=s["freq"],
                    level_dbfs=s["level_db"], snr_db=s["snr_db"],
                    noise_floor_dbfs=floor,
                    channels=live, levels_dbfs=lv.tolist(),
                    bearing_deg=bearing if keep else None,
                    confidence=conf, calibrated=cal.calibrated))

                if len(h) >= 3:
                    store.upsert_signal(s["freq"], dict(
                        last_seen=utcnow(), freq_hz=s["freq"],
                        bearing_mean_deg=dfcore.circ_mean_deg(h),
                        bearing_std_deg=dfcore.circ_std_deg(h),
                        n_observations=len(h),
                        last_level_dbfs=s["level_db"],
                        last_snr_db=s["snr_db"],
                        calibrated=cal.calibrated, run_id=run_id))

            if a.interval and not STOP:
                time.sleep(a.interval)
    finally:
        elapsed = time.time() - t_start
        print(f"\n[sigmon] {npass} pass(es) in {elapsed:.1f} s")
        print(f"[sigmon] {store.summary()}")

        if a.stability_report and history:
            print("\nbearing stability (circular statistics over the run)")
            print("-" * 64)
            print(f"  {'MHz':>10} {'n':>5} {'mean':>9} {'circ std':>10} {'verdict':>12}")
            for f in sorted(history, key=lambda k: -len(history[k])):
                h = history[f]
                if len(h) < 3:
                    continue
                sd = dfcore.circ_std_deg(h)
                verdict = ("stable" if sd < 5 else
                           "usable" if sd < 15 else
                           "unstable")
                print(f"  {f/1e6:>10.3f} {len(h):>5} "
                      f"{dfcore.circ_mean_deg(h):>8.1f}d {sd:>9.1f}d "
                      f"{verdict:>12}")
            print()
            print("  A static FM transmitter should read 'stable'.  Anything")
            print("  worse means the array, not the emitter: check for ADC")
            print("  overload, too few live ports, or antennas whose patterns")
            print("  are too similar to tell directions apart.")

            # Stability is necessary but NOT sufficient.  A fit with too little
            # angular discrimination collapses onto the element boresights: it
            # then reports the same value every pass -- perfectly "stable" --
            # while carrying no more information than "which antenna is
            # loudest".  Measure the distance to the nearest boresight and say
            # so, rather than letting a small circular std imply precision.
            allb = [dfcore.circ_mean_deg(h) for h in history.values() if len(h) >= 3]
            if allb:
                off = []
                for b in allb:
                    d = np.abs((np.asarray(azimuths) - b + 180.0) % 360.0 - 180.0)
                    off.append(float(np.min(d)))
                off = np.array(off)
                spacing = 360.0 / n_live
                print()
                print("  boresight-snapping check")
                print("  " + "-" * 60)
                print(f"  median distance from the nearest element boresight: "
                      f"{np.median(off):.1f} deg")
                print(f"  (uniformly distributed bearings would average "
                      f"{spacing/4:.1f} deg)")
                if np.median(off) < spacing / 8:
                    print()
                    print("  -> BEARINGS ARE SNAPPING TO THE ELEMENT BORESIGHTS.")
                    print("     The fit is not interpolating between antennas, it is")
                    print("     effectively reporting which one is loudest, so the real")
                    print(f"     resolution is about {spacing:.0f} deg -- not the sub-degree")
                    print("     figure in the std column.  That column measures")
                    print("     repeatability, and a quantised estimator repeats")
                    print("     perfectly.  Treat these as sectors, not bearings.")
                else:
                    print("  -> bearings fall between boresights, so the fit is")
                    print("     genuinely interpolating rather than picking a winner.")
        store.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
