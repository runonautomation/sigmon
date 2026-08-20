#!/usr/bin/env python3
"""sigmon web UI -- live spectrum, waterfall, and direction finding on demand.

    ./webui.py                      # 88-108 MHz, listen on 127.0.0.1:8088
    ./webui.py --start 2400M --stop 2440M --gain 25 --peak-hold 8
    ./webui.py --host 0.0.0.0       # reachable from other machines

The B210 is single-session: exactly one process can hold the radio.  So this
server owns it for its whole lifetime and browsers only ever talk to the server.
Nothing else (sigmon.py, rfscan.py, GNU Radio) can run at the same time.

One worker thread drives the radio and alternates between two jobs:

  spectrum -- sweep the configured span on ONE antenna and publish a frame.
      One antenna, not all four, because the display wants frames per second
      and a 4-port sweep is four times slower for a picture that looks the
      same.
  DF -- ONE continuous capture at one frequency with the switch commutating
      1-2-3-4-null underneath it, cut back into per-antenna levels using the
      null slot as the sync marker.  See dfstream.py for why that is the fast
      way and why the null position is what makes it possible.
      Requested from the UI, or repeated continuously when a frequency is
      pinned.

Bearings are relative unless a calibration file is supplied; see README.md.
"""
import argparse
import os
import sys
import threading
import time
from collections import deque

import numpy as np
from flask import Flask, jsonify, request, send_from_directory

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
# uarf sits beside this project. The older spelling resolved to
# <workspace>/../uarf, which does not exist, so `import rfscan` only ever
# worked from an externally set PYTHONPATH.
for _p in (os.path.join(os.path.dirname(_HERE), "uarf"),
           os.path.join(os.path.dirname(os.path.dirname(_HERE)), "uarf")):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

import rfscan                       # noqa: E402  also sets UHD_IMAGES_DIR
import uhd                          # noqa: E402
import dfcore                       # noqa: E402
import dfstream                     # noqa: E402
import swbackend                    # noqa: E402
from store import Store, utcnow     # noqa: E402

# --------------------------------------------------------------------------
# Why lo_frac is 0.25 and only 0.44 of the rate is kept per tuning.
#
# Offsetting the LO keeps its leakage out of the analysed band, but on this
# board the leakage also comes back ALIASED: measured across 600-2450 MHz, a
# ~1 MHz wide notch sits at exactly (lo_offset - Fs/2), 2.4-4 dB deep, and does
# not move with the tuned frequency -- it is the DC image folded about Nyquist
# in the digital down-conversion, not anything in the RF.
#
#     lo_frac 0.25 -> LO offset +4.00 MHz -> notch measured at -3.98 MHz
#     lo_frac 0.35 -> LO offset +5.60 MHz -> notch measured at -2.41 MHz
#     lo_frac 0.45 -> LO offset +7.20 MHz -> notch measured at -0.80 MHz
#
# So there are two artefacts, at -lo_off and at lo_off - Fs/2, and they move in
# opposite directions: whatever you gain on one you lose on the other. The best
# available choice makes them COINCIDE, which is lo_off = Fs/4 -- lo_frac 0.25
# -- putting both at -Fs/4 and leaving everything inside |f| < Fs/4 clean.
#
# The sweep then has to stop before it reaches them, which is what seg_frac
# does. Measured ripple across one tuning, quiet band, 16 Msps:
#
#     lo_frac 0.35, keep 0.60 of the rate   4.44 dB   <- the old default
#     lo_frac 0.25, keep 0.44               1.26 dB
#     lo_frac 0.25, keep 0.40               1.10 dB
#
# 4.4 dB of ripple, repeating once per tuning segment, is what shows up in a
# wide sweep as regular dips -- at 2400-2500 MHz they landed every 9.09 MHz,
# which is the segment width, and that is how this was found.
#
# One more, unrelated but in the same place: at 16 Msps the decimation from the
# 48 MHz master clock is 3, which is ODD, and UHD disables the halfband filter
# and warns about CIC rolloff. 12 Msps decimates by 4 and measures flatter
# still (0.84 dB across +-3.30 MHz at lo_frac 0.30). Worth having if the extra
# instantaneous bandwidth is not needed.
# --------------------------------------------------------------------------
DEFAULT_LO_FRAC = 0.25
DEFAULT_SEG_FRAC = 0.44

HERE = os.path.dirname(os.path.abspath(__file__))
app = Flask(__name__, static_folder=None)


class Radio:
    """Owns the USRP.  All hardware access happens on the worker thread."""

    def __init__(self, a):
        self.a = a
        self.lock = threading.Lock()
        # NOT self.stop: _configure() assigns self.stop as the stop
        # FREQUENCY, which silently replaced the Event and killed the
        # worker thread on its first loop check.
        self.shutdown = threading.Event()

        # Published state, guarded by self.lock
        self.frame = None            # latest spectrum frame
        self.seq = 0
        self.live = []
        self.azimuths = np.zeros(0)
        self.beamwidth = 0.0
        self.status = "starting"
        self.error = None
        self.sweep_ms = 0.0
        self.overload = False
        self.pinned = None           # frequency being continuously borne
        self.history = {}            # freq -> deque of bearings
        self.last_df = None

        self._df_request = None      # (freq, threading.Event, result-holder)
        # Manual antenna override from the UI. While it is set the spectrum
        # stays on that port and no DF runs on its own -- someone is looking at
        # one antenna on purpose.
        self.manual_port = None
        self._held_port = None

        # rfscan.default_fpga() resolves the marker relative to itself, which
        # is where detect_fpga.sh writes it; the path spelled out here was one
        # directory too high and always found nothing.
        fpga = rfscan.default_fpga()
        dev = "type=b200" + (f",fpga={fpga}" if fpga else "")
        self.usrp = uhd.usrp.MultiUSRP(dev)

        self.backend = swbackend.open_backend(
            a.switch, usrp=self.usrp, device=a.switch_device,
            gpio_mask=a.gpio_mask, auto=(a.switch == "auto"))
        print(f"[webui] switch: {self.backend.describe()}", flush=True)
        if a.df_legacy and self.backend.name == "esp32":
            # Legacy DF holds each antenna in turn, and on this board every
            # port change is an NVS write. Sixteen sweeps x four antennas is
            # 64 flash writes per measurement, repeated for as long as a
            # frequency is pinned -- that wears the part out to run a
            # comparison. The comparison is still available on --switch usrp.
            sys.exit("error: --df-legacy holds each antenna in turn, and every "
                     "port change on the esp32 writes NVS.\n"
                     "  Run the comparison with --switch usrp instead.")
        if a.ports:
            self.ports = [int(c) for c in a.ports.split(",")]
        else:
            self.ports = list(self.backend.default_ports)
        self.null_port = (a.null_port if a.null_port is not None
                          else self.backend.default_null)
        self.probe_ports = list(self.ports) + [self.null_port]

        self.rx = rfscan.Receiver(self.usrp, a.rx_chan, a.rate, a.gain,
                                  a.antenna, lo_frac=a.lo_frac)
        self.cal = dfcore.Calibration.load(a.cal)
        if not a.track_dc:
            # The commutated DF steps the input every slot; the AD9361's
            # DC-offset loop would spend the recording chasing those steps and
            # injecting a transient into the start of each one.
            dfstream.freeze_dc_offset(self.rx)
        self.store = Store(a.mongo_uri, a.mongo_db, a.fallback, quiet=False)
        self.run_id = None
        self._configure(a.start, a.stop, a.step, a.rbw, a.gain)

    # -- configuration --------------------------------------------------
    def _configure(self, start, stop, step, rbw, gain):
        self.start, self.stop, self.step, self.rbw = start, stop, step, rbw
        self.rx.usrp.set_rx_gain(gain, self.a.rx_chan)
        self.gain = gain
        self.segments = rfscan.plan_segments(start, stop,
                                            self.rx.rate * self.a.seg_frac)
        self.freqs = np.arange(start, stop + step / 2, step)
        seg_of = []
        for fq in self.freqs:
            best_i, best_d = 0, None
            for i, (centre, lo, hi) in enumerate(self.segments):
                d = abs(fq - centre)
                if lo - step <= fq <= hi + step and (best_d is None or d < best_d):
                    best_i, best_d = i, d
            if best_d is None:
                best_i = int(np.argmin([abs(fq - c) for c, _, _ in self.segments]))
            seg_of.append(best_i)
        self.seg_of = np.array(seg_of)
        self.nsamps = int(self.a.dwell * self.rx.rate)

    def reconfigure(self, **kw):
        with self.lock:
            self._pending_config = kw

    # -- primitives -----------------------------------------------------
    def _psd(self, x):
        if self.a.peak_hold > 1:
            return dfcore.psd_peak_hold(x, self.rx.rate, self.rbw,
                                        self.a.peak_hold, rfscan.welch_psd)
        return rfscan.welch_psd(x, self.rx.rate, self.rbw)

    def _hold(self, ch):
        """Select a port, and skip the call when it is already selected.

        This is called once per spectrum sweep, several times a second, for
        hours. On the esp32 every port change is a flash write, so re-asserting
        the port we are already on would be ~5 NVS writes a second forever. The
        backend tracks this too; the check is here as well because it is the
        caller that knows nothing changed.
        """
        if self._held_port != ch:
            self.backend.hold(ch)
            self._held_port = ch
            time.sleep(self.a.switch_settle)

    def _levels_for_channel(self, ch):
        """Full-span level vector for one switch port."""
        self._hold(ch)
        out = np.full(len(self.freqs), np.nan)
        for i, (centre, _lo, _hi) in enumerate(self.segments):
            sel = np.nonzero(self.seg_of == i)[0]
            if not len(sel):
                continue
            self.rx.tune(centre)
            time.sleep(self.a.tune_settle)
            x = self.rx.capture(self.nsamps)
            if len(x) < self.nsamps // 2:
                raise RuntimeError("short capture")
            f, P = self._psd(x)
            for k in sel:
                out[k] = rfscan.band_power_db(f, P, self.freqs[k] - centre, self.rbw)
        return out

    def _level_at(self, ch, freq):
        """Level on one port at one frequency, tuned directly (no sweep)."""
        self._hold(ch)
        self.rx.tune(freq)
        time.sleep(self.a.tune_settle)
        x = self.rx.capture(self.nsamps)
        if len(x) < self.nsamps // 2:
            raise RuntimeError("short capture")
        f, P = self._psd(x)
        centre = self.usrp.get_rx_freq(self.a.rx_chan)
        return rfscan.band_power_db(f, P, freq - centre, self.rbw)

    # -- jobs -----------------------------------------------------------
    def probe_live(self):
        table = np.vstack([self._levels_for_channel(c)
                           for c in self.probe_ports])
        live_i, info = dfcore.find_live_channels(
            table, step_hz=self.step, rbw_hz=self.rbw,
            min_agreement=self.a.min_agreement)
        # find_live_channels indexes rows; the ports are the backend's own
        # numbering (1..5 on the esp32, 0..4 on the usrp) and must not be
        # confused with row numbers -- that is exactly how an array ends up
        # reported one element out.
        live = [self.probe_ports[i] for i in live_i]
        self.agreement_by_port = {p: float(info["agreement"][i])
                                  for i, p in enumerate(self.probe_ports)}

        # The no-signal port is probed along with the rest, and it must come
        # out DEAD. That is not a formality: the whole commutated method reads
        # it as the sync marker and as the noise reference, and if something is
        # actually connected to it the recording has no dips to cut on and the
        # subtraction removes real signal. Cheapest possible check, and it
        # fails loudly instead of producing a plausible bearing.
        self.null_ok = self.null_port not in live
        if not self.null_ok:
            print(f"[webui] WARNING: port {self.null_port} was declared the "
                  f"NO-SIGNAL position but reads as live (agreement "
                  f"{self.agreement_by_port.get(self.null_port, 0.0):+.2f}). "
                  f"Commutated DF will not sync.", flush=True)
        live = [c for c in live if c != self.null_port]

        if self.a.channels:
            live = [int(x) for x in self.a.channels.split(",")]
        # Keep the wiring order the array was declared in. find_live_channels
        # returns rows in probe order, which is the same thing today, but the
        # azimuths are assigned by POSITION in this list -- so if it ever
        # diverged, every bearing would rotate.
        live = [p for p in self.ports if p in live] or live
        n = len(live)
        # Angles come from each port's place in the DECLARED array, not from
        # how many are live: three survivors of a four-element ring are still
        # bolted at 0/90/180, not spread to 0/120/240.
        az = self.cal.azimuth_for_ports(
            live, self.ports, counter_clockwise=self.a.counter_clockwise,
            offset_deg=self.a.array_offset)
        bw = self.a.beamwidth or ((1.33 if n >= 6 else 0.7) * (360.0 / max(n, 1)))

        # Blind gain equalisation, from the same probe sweep. Signals across a
        # broadcast band arrive from many directions, so each antenna's average
        # over the band should be about equal if the antennas are equivalent;
        # whatever is left is gain imbalance and it drags every bearing toward
        # the hottest port. Computed here rather than per-measurement because a
        # single-frequency DF has no band to average over.
        bal = None
        if self.a.auto_balance:
            rows = np.vstack([table[c] for c in live])
            m = np.nanmean(rows, axis=1)
            bal = m - m.mean()
            print("[webui] auto-balance offsets (dB): " +
                  ", ".join(f"ch{c}:{v:+.1f}" for c, v in zip(live, bal)),
                  flush=True)
        self.balance = bal

        with self.lock:
            self.live, self.azimuths, self.beamwidth = live, az, bw
            self.agreement = info["agreement"].tolist()
        return live

    def do_df(self, freq):
        """Bearing at one frequency, from ONE continuous commutated recording.

        The radio never stops receiving: the switch walks 1-2-3-4-null
        underneath a single capture and the antennas come out as slices of it
        (see dfstream.py).  That removes the per-antenna capture overhead
        entirely -- which, not the switch, was what set the old cycle time --
        and puts the four levels microseconds apart instead of milliseconds.

        --df-legacy keeps the old capture-per-antenna path so the two can be
        run back to back against the same signal.  Everything else in this
        project that looked like an improvement was measured that way, and two
        of those measurements said the improvement was not real.
        """
        if self.a.df_legacy:
            return self._do_df_legacy(freq)

        with self.lock:
            live, az, bw = list(self.live), np.array(self.azimuths), self.beamwidth
        if len(live) < 3:
            return {"error": f"only {len(live)} live antenna(s); need 3+"}

        offsets = np.array([self.cal.offsets.get(c, 0.0) for c in live])
        bal = self.balance if self.a.auto_balance else None
        r = dfstream.measure(self.rx, self.backend, float(freq), live,
                             self.null_port, az, bw, self._dfargs(),
                             offsets=offsets, balance=bal,
                             iq_out=self.a.iq_out)
        # The DF left the switch commutating (park=None), so whatever port the
        # spectrum sweep last held is no longer selected.
        self._held_port = None
        if not r.get("ok"):
            with self.lock:
                self.last_df = {"freq_hz": float(freq), "error": r.get("reason"),
                                "ts": utcnow().isoformat()}
            return {"error": r.get("reason", "sync failed"),
                    "contrast_db": r.get("contrast_db"),
                    "n_cycles": r.get("n_cycles", 0)}

        bearing, conf = r["bearing_deg"], r["confidence"]
        levels = np.asarray(r["levels_db"], float)

        h = self.history.setdefault(round(freq), deque(maxlen=400))
        h.append(bearing)
        stab = dfcore.circ_std_deg(h) if len(h) >= 3 else None
        mean = dfcore.circ_mean_deg(h) if len(h) >= 3 else None

        resid = np.asarray(r["resid"], float)
        grid = np.asarray(r["grid"], float)
        pol = (resid.max() - resid) / (np.ptp(resid) or 1.0)
        step = max(1, len(grid) // 180)

        res = dict(
            freq_hz=float(freq), bearing_deg=bearing, confidence=conf,
            # The spread over the cycles INSIDE this one recording. Hundreds of
            # independent looks taken milliseconds apart, so unlike the old
            # sweep_std_deg it is a real distribution rather than a handful of
            # samples -- this measurement's own error bar, available now.
            sweep_std_deg=r["cycle_std_deg"], sweeps=r["n_cycles"],
            cycle_std_deg=r["cycle_std_deg"], n_cycles=r["n_cycles"],
            cycle_us=(r["cycle_s"] * 1e6 if r["cycle_s"] else None),
            contrast_db=round(float(r["contrast_db"]), 1),
            detector=r["detector"], null_dbfs=round(float(r["null_db"]), 1),
            overflows=r["overflows"], record_ms=self.a.record_ms,
            channels=live, levels_db=levels.tolist(),
            azimuths_deg=list(np.asarray(az, float)), beamwidth_deg=float(bw),
            n=len(h), mean_deg=mean, std_deg=stab,
            polar=[[float(grid[i]), float(pol[i])]
                   for i in range(0, len(grid), step)],
            calibrated=self.cal.calibrated, ts=utcnow().isoformat())

        floor = float(levels.min())
        self.store.insert_observation(dict(
            run_id=self.run_id, ts=utcnow(), freq_hz=float(freq),
            level_dbfs=float(levels.max()), snr_db=float(np.ptp(levels)),
            noise_floor_dbfs=floor, null_dbfs=float(r["null_db"]),
            channels=live, levels_dbfs=levels.tolist(), bearing_deg=bearing,
            confidence=conf, n_cycles=r["n_cycles"],
            cycle_std_deg=r["cycle_std_deg"],
            calibrated=self.cal.calibrated))
        if stab is not None:
            self.store.upsert_signal(float(freq), dict(
                last_seen=utcnow(), freq_hz=float(freq),
                bearing_mean_deg=mean, bearing_std_deg=stab,
                n_observations=len(h), last_level_dbfs=float(levels.max()),
                calibrated=self.cal.calibrated, run_id=self.run_id))
        with self.lock:
            self.last_df = res
        return res

    def _dfargs(self):
        """The knobs dfstream.measure() reads, as one object."""
        return argparse.Namespace(
            slot_us=self.a.slot_us, record_ms=self.a.record_ms,
            guard=self.a.guard, min_contrast=self.a.min_contrast,
            rbw=self.rbw, tune_settle=self.a.tune_settle,
            # park=None: leave the switch commutating between measurements.
            # Stopping and restarting it costs a serial round trip and, on the
            # esp32, a flash write -- per DF, forever, while a frequency is
            # pinned.
            park=None, verify_switch=self.a.verify_switch)

    def _do_df_legacy(self, freq):
        """The previous method: one capture per antenna, kept for comparison."""
        with self.lock:
            live, az, bw = list(self.live), np.array(self.azimuths), self.beamwidth
        if len(live) < 3:
            return {"error": f"only {len(live)} live antenna(s); need 3+"}

        # Tune ONCE, then commutate. The frequency does not change during a DF,
        # so retuning per antenna bought nothing and cost a 10 ms settle each
        # time. Measured: the 8-antenna cycle was 500 ms, of which the captures
        # were 250 ms and essentially all the rest was that retune and its
        # sleep. sw.select() itself is 0.026 ms -- switching was never the cost.
        self.rx.tune(freq)
        time.sleep(self.a.tune_settle)
        centre = self.usrp.get_rx_freq(self.a.rx_chan)

        # Many FAST sweeps rather than one slow one. Amplitude comparison
        # assumes every antenna sees the same signal; the longer one cycle
        # takes, the less true that is. A 1 ms dwell makes a cycle ~16 ms, so
        # the eight levels are near-simultaneous, and averaging N cycles buys
        # back the SNR that the short dwell gave up -- without reintroducing
        # the slow drift that made the levels incomparable in the first place.
        nsamps = max(256, int(self.a.df_dwell * self.rx.rate))
        acc = np.zeros(len(live))
        per_sweep = []
        for _ in range(max(1, self.a.df_sweeps)):
            lv = np.empty(len(live))
            for i, ch in enumerate(live):
                # Deliberately NOT self._hold(): this path exists to reproduce
                # the old timing, and the sticky check would change it. It is
                # refused on the esp32 at start-up for the same reason.
                self.backend.hold(ch)
                self._held_port = None
                if self.a.df_settle > 0:
                    time.sleep(self.a.df_settle)
                x = self.rx.capture(nsamps)
                if len(x) < nsamps // 2:
                    raise RuntimeError("short capture")
                f, P = self._psd(x)
                lv[i] = rfscan.band_power_db(f, P, freq - centre, self.rbw)
            acc += 10.0 ** (lv / 10.0)          # average POWER, not dB
            per_sweep.append(lv)

        levels = 10.0 * np.log10(acc / len(per_sweep))
        levels = levels - np.array([self.cal.offsets.get(c, 0.0) for c in live])
        if self.a.auto_balance and self.balance is not None:
            levels = levels - self.balance

        # Bearing from each individual sweep too. Their spread is an immediate
        # quality figure for THIS measurement -- available now, rather than
        # after pinning the frequency for a minute.
        sweep_bearings = []
        if len(per_sweep) > 2:
            off = np.array([self.cal.offsets.get(c, 0.0) for c in live])
            for lv in per_sweep:
                l2 = lv - off
                if self.a.auto_balance and self.balance is not None:
                    l2 = l2 - self.balance
                b, _r, _g = dfcore.estimate_bearing(
                    l2, az, bw, l2 - l2.min(), n_avg=max(nsamps, 1))
                sweep_bearings.append(b)

        floor = float(np.min(levels))
        snr = levels - floor
        bearing, resid, grid = dfcore.estimate_bearing(
            levels, az, bw, snr, n_avg=max(int(self.a.dwell * self.rx.rate), 1))
        conf = dfcore.bearing_confidence(resid, grid)

        h = self.history.setdefault(round(freq), deque(maxlen=400))
        h.append(bearing)
        stab = dfcore.circ_std_deg(h) if len(h) >= 3 else None
        mean = dfcore.circ_mean_deg(h) if len(h) >= 3 else None

        # Residual curve for the polar plot, normalised so 1.0 is the best fit.
        r = np.asarray(resid, float)
        pol = (r.max() - r) / (np.ptp(r) or 1.0)
        step = max(1, len(grid) // 180)

        sweep_std = (dfcore.circ_std_deg(sweep_bearings)
                     if len(sweep_bearings) > 2 else None)

        res = dict(freq_hz=float(freq), bearing_deg=bearing, confidence=conf,
                   sweep_std_deg=sweep_std, sweeps=len(per_sweep),
                   channels=live, levels_db=levels.tolist(),
                   azimuths_deg=az.tolist(), beamwidth_deg=bw,
                   n=len(h), mean_deg=mean, std_deg=stab,
                   polar=[[float(grid[i]), float(pol[i])]
                          for i in range(0, len(grid), step)],
                   calibrated=self.cal.calibrated,
                   ts=utcnow().isoformat())

        self.store.insert_observation(dict(
            run_id=self.run_id, ts=utcnow(), freq_hz=float(freq),
            level_dbfs=float(levels.max()), snr_db=float(np.ptp(levels)),
            noise_floor_dbfs=floor, channels=live,
            levels_dbfs=levels.tolist(), bearing_deg=bearing,
            confidence=conf, calibrated=self.cal.calibrated))
        if stab is not None:
            self.store.upsert_signal(float(freq), dict(
                last_seen=utcnow(), freq_hz=float(freq),
                bearing_mean_deg=mean, bearing_std_deg=stab,
                n_observations=len(h), last_level_dbfs=float(levels.max()),
                calibrated=self.cal.calibrated, run_id=self.run_id))
        with self.lock:
            self.last_df = res
        return res

    # -- worker ---------------------------------------------------------
    def run(self):
        self.balance = None          # replaced by probe_live() if --auto-balance
        try:
            with self.lock:
                self.status = "probing antennas"
            live = self.probe_live()
            self.run_id = str(int(time.time()))
            self.store.insert_run(dict(
                run_id=self.run_id, ts=utcnow(), start_hz=self.start,
                stop_hz=self.stop, step_hz=self.step, rbw_hz=self.rbw,
                rate_hz=self.rx.rate, gain_db=self.gain, channels=live,
                azimuths_deg=list(self.azimuths), beamwidth_deg=self.beamwidth,
                calibrated=self.cal.calibrated, source="webui"))
            with self.lock:
                self.status = "running"
        except Exception as e:                              # noqa: BLE001
            with self.lock:
                self.status, self.error = "failed", f"{type(e).__name__}: {e}"
            return

        spectrum_ch = (self.a.spectrum_channel
                       if self.a.spectrum_channel is not None
                       else (self.live[0] if self.live else 0))

        while not self.shutdown.is_set():
            # Apply a pending reconfiguration between jobs, never mid-capture.
            pend = getattr(self, "_pending_config", None)
            if pend:
                self._pending_config = None
                try:
                    self._configure(pend.get("start", self.start),
                                    pend.get("stop", self.stop),
                                    pend.get("step", self.step),
                                    pend.get("rbw", self.rbw),
                                    pend.get("gain", self.gain))
                except Exception as e:                      # noqa: BLE001
                    with self.lock:
                        self.error = f"reconfigure failed: {e}"

            req = self._df_request
            if req is not None:
                self._df_request = None
                freq, ev, holder = req
                try:
                    holder["result"] = self.do_df(freq)
                except Exception as e:                      # noqa: BLE001
                    holder["result"] = {"error": f"{type(e).__name__}: {e}"}
                ev.set()
                continue

            if self.pinned is not None:
                # Pinned DF suspends the spectrum rather than alternating with
                # it. The two want the switch in opposite states -- commutating
                # vs. held on one port -- and flipping between them every pass
                # would cost two serial round trips and, on the esp32, two
                # flash writes, several times a second for as long as the pin
                # is up. Pinning is also when DF rate matters most, and this
                # way a pinned measurement costs exactly one capture.
                try:
                    self.do_df(self.pinned)
                except Exception as e:                      # noqa: BLE001
                    with self.lock:
                        self.error = f"df: {e}"
                with self.lock:
                    self.spectrum_paused = True
                continue

            # Manual override wins over the auto-chosen display port: if
            # someone has selected an antenna, that is what they want to see.
            ch = self.manual_port if self.manual_port is not None else spectrum_ch

            t0 = time.time()
            try:
                levels = self._levels_for_channel(ch)
            except Exception as e:                          # noqa: BLE001
                with self.lock:
                    self.error = f"sweep: {e}"
                time.sleep(0.5)
                continue
            dt = (time.time() - t0) * 1e3

            with self.lock:
                self.seq += 1
                self.frame = dict(
                    seq=self.seq, ts=time.time(),
                    f0=float(self.freqs[0]), f1=float(self.freqs[-1]),
                    n=len(self.freqs),
                    levels=[round(float(v), 2) if np.isfinite(v) else None
                            for v in levels])
                self.sweep_ms = dt
                self.overload = bool(self.rx.clipped)
                self.spectrum_channel = ch
                self.spectrum_paused = False

    def request_df(self, freq, timeout=30.0):
        ev = threading.Event()
        holder = {}
        self._df_request = (float(freq), ev, holder)
        if not ev.wait(timeout):
            return {"error": "timed out waiting for the radio"}
        return holder.get("result", {"error": "no result"})


RADIO = None


@app.route("/")
def index():
    return send_from_directory(os.path.join(HERE, "static"), "index.html")


@app.route("/api/state")
def api_state():
    r = RADIO
    with r.lock:
        return jsonify(dict(
            status=r.status, error=r.error, live=r.live,
            azimuths=list(np.round(r.azimuths, 1)) if len(r.azimuths) else [],
            beamwidth=r.beamwidth, agreement=getattr(r, "agreement", []),
            start=r.start, stop=r.stop, step=r.step, rbw=r.rbw, gain=r.gain,
            rate=r.rx.rate, sweep_ms=round(r.sweep_ms, 1),
            overload=r.overload, pinned=r.pinned,
            spectrum_channel=getattr(r, "spectrum_channel", None),
            balance=(list(np.round(r.balance, 2))
                     if getattr(r, 'balance', None) is not None else None),
            calibrated=r.cal.calibrated, using_mongo=r.store.using_mongo,
            counter_clockwise=r.a.counter_clockwise,
            peak_hold=r.a.peak_hold, seq=r.seq,
            df_mode="legacy" if r.a.df_legacy else "commutated",
            null_port=r.null_port, null_ok=getattr(r, "null_ok", None),
            slot_us=r.a.slot_us, record_ms=r.a.record_ms, guard=r.a.guard,
            switch=r.backend.name, switch_desc=r.backend.describe(),
            ports=r.probe_ports, antennas=r.ports,
            manual_port=r.manual_port,
            spectrum_paused=getattr(r, "spectrum_paused", False),
            nvs_writes=getattr(r.backend, "nvs_writes", None),
            agreement_by_port=getattr(r, "agreement_by_port", {})))


@app.route("/api/spectrum")
def api_spectrum():
    r = RADIO
    since = request.args.get("since", type=int, default=-1)
    with r.lock:
        f = r.frame
        if f is None:
            return jsonify({"seq": -1})
        if since >= 0 and f["seq"] <= since:
            return jsonify({"seq": f["seq"], "unchanged": True})
        return jsonify(f)


@app.route("/api/df", methods=["POST"])
def api_df():
    freq = (request.json or {}).get("freq_hz")
    if freq is None:
        return jsonify({"error": "freq_hz required"}), 400
    return jsonify(RADIO.request_df(float(freq)))


@app.route("/api/pin", methods=["POST"])
def api_pin():
    freq = (request.json or {}).get("freq_hz")
    RADIO.pinned = None if freq is None else float(freq)
    if RADIO.pinned is None:
        return jsonify({"pinned": None})
    return jsonify({"pinned": RADIO.pinned})


@app.route("/api/antenna", methods=["POST"])
def api_antenna():
    """Manual antenna selection: {"port": N} to hold one, {"port": null} for auto.

    Holding a port also un-pins any running DF -- the two are contradictory
    requests (one port held still vs. all of them commutating) and silently
    letting the DF keep moving the switch would make the displayed spectrum
    belong to no particular antenna.
    """
    r = RADIO
    port = (request.json or {}).get("port")
    if port is not None:
        port = int(port)
        if port not in r.probe_ports:
            return jsonify({"error": f"port {port} is not one of "
                                     f"{r.probe_ports}"}), 400
        r.pinned = None
    r.manual_port = port
    return jsonify({"manual_port": port, "ports": r.probe_ports,
                    "null_port": r.null_port})


@app.route("/api/switch")
def api_switch():
    """What the switch board is doing, read from the device itself."""
    r = RADIO
    b = r.backend
    out = {"backend": b.name, "describe": b.describe(),
           "ports": r.probe_ports, "antennas": r.ports,
           "null_port": r.null_port, "manual_port": r.manual_port,
           "nvs_writes": getattr(b, "nvs_writes", None)}
    try:
        st = b.sw.state() if b.name == "esp32" else None
    except Exception as e:                                      # noqa: BLE001
        st = None
        out["error"] = f"{type(e).__name__}: {e}"
    if st:
        out.update({k: st.get(k) for k in
                    ("port", "pattern", "dwell_ns", "iterate", "seq", "steps")})
    return jsonify(out)


@app.route("/api/last_df")
def api_last_df():
    with RADIO.lock:
        return jsonify(RADIO.last_df or {})


@app.route("/api/config", methods=["POST"])
def api_config():
    d = request.json or {}
    kw = {k: float(d[k]) for k in ("start", "stop", "step", "rbw", "gain")
          if k in d and d[k] is not None}
    if "start" in kw and "stop" in kw and kw["stop"] <= kw["start"]:
        return jsonify({"error": "stop must be above start"}), 400
    RADIO.reconfigure(**kw)
    return jsonify({"ok": True, "applied": kw})


def parse_freq(s):
    return rfscan.parse_freq(s)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--start", type=parse_freq, default=88e6)
    p.add_argument("--stop", type=parse_freq, default=108e6)
    p.add_argument("--step", type=parse_freq, default=100e3)
    p.add_argument("--rbw", type=parse_freq, default=200e3)
    p.add_argument("--rate", type=float, default=16e6)
    p.add_argument("--lo-frac", type=float, default=DEFAULT_LO_FRAC,
                   dest="lo_frac",
                   help="LO offset as a fraction of the rate. 0.25 puts the LO\n"
                        "leakage and its Nyquist alias on the same frequency, "
                        "which is the only choice that leaves one artefact "
                        "instead of two.")
    p.add_argument("--seg-frac", type=float, default=DEFAULT_SEG_FRAC,
                   dest="seg_frac",
                   help="fraction of the rate kept from each tuning. Must stay "
                        "clear of the artefacts at +-lo_frac*rate; 0.44 measured "
                        "1.26 dB of ripple against 4.44 dB at the old 0.60.")
    p.add_argument("--gain", type=float, default=30)
    p.add_argument("--antenna", default="TX/RX")
    p.add_argument("--rx-chan", type=int, default=0)
    p.add_argument("--gpio-mask", type=lambda s: int(s, 0), default=0xE0)
    p.add_argument("--dwell", type=float, default=0.01,
                   help="capture per antenna per segment for the SPECTRUM (s)")

    # -- which device moves the switch --
    p.add_argument("--switch", default="auto", choices=("auto", "esp32", "usrp"),
                   help="'esp32' is the separate switch board: it free-runs the "
                        "commutation on its own at microsecond dwells and the "
                        "host does nothing but capture. 'usrp' drives the "
                        "B210's GPIO from a host thread and cannot go below "
                        "~50 us per slot. 'auto' prefers the esp32.")
    p.add_argument("--switch-device", default="/dev/ttyACM0",
                   help="serial device of the esp32 switch board")
    p.add_argument("--ports", default=None,
                   help="the four antenna ports in array order (default: "
                        "1,2,3,4 on the esp32, 0,1,2,3 on the usrp)")
    p.add_argument("--no-verify-switch", action="store_false",
                   dest="verify_switch",
                   help="skip reading the switch's step counter around each "
                        "capture; that check is what tells a stopped switch "
                        "from a weak signal")

    # -- commutated DF (dfstream) --
    p.add_argument("--null-port", type=int, default=None,
                   help="switch code of the NO-SIGNAL position (default: 5 on "
                        "the esp32, 4 on the usrp). It is both the sync marker "
                        "that cuts the continuous recording into antenna slots "
                        "and the in-band noise reference subtracted from every "
                        "level.")
    p.add_argument("--slot-us", type=float, default=200.0,
                   help="microseconds per switch position. The switch is now "
                        "sub-microsecond; the floor is the host GPIO write "
                        "(~26 us measured) and the receiver's settling, which "
                        "dfstream.py --probe-transition measures directly.")
    p.add_argument("--record-ms", type=float, default=100.0,
                   help="length of the single continuous capture per DF")
    p.add_argument("--guard", type=float, default=0.25,
                   help="fraction discarded at each end of every slot, to cover "
                        "host-side jitter in when the switch actually moved")
    p.add_argument("--min-contrast", type=float, default=3.0,
                   help="dB the antennas must stand above the null slot before "
                        "the sync is trusted")
    p.add_argument("--iq-out", default=None,
                   help="directory to write each DF's raw recording into")
    p.add_argument("--track-dc", action="store_true",
                   help="leave the AD9361 DC-offset tracking loop on")
    p.add_argument("--df-legacy", action="store_true",
                   help="use the old one-capture-per-antenna DF instead, for a "
                        "back-to-back comparison against the commutated one")

    # -- legacy DF only --
    p.add_argument("--df-dwell", type=float, default=0.001,
                   help="[--df-legacy] capture per antenna per DF sweep (s)")
    p.add_argument("--df-settle", type=float, default=0.0002,
                   help="[--df-legacy] wait after a switch change during DF (s)")
    p.add_argument("--df-sweeps", type=int, default=16,
                   help="[--df-legacy] fast cycles averaged per DF measurement")
    p.add_argument("--switch-settle", type=float, default=0.0005)
    p.add_argument("--tune-settle", type=float, default=0.01)
    p.add_argument("--peak-hold", type=int, default=1)
    p.add_argument("--channels", default=None)
    p.add_argument("--spectrum-channel", type=int, default=None,
                   help="port used for the spectrum display (default: first live)")
    p.add_argument("--beamwidth", type=float, default=None)
    p.add_argument("--counter-clockwise", "--ccw", action="store_true",
                   dest="counter_clockwise")
    p.add_argument("--array-offset", type=float, default=0.0)
    p.add_argument("--auto-balance", action="store_true")
    p.add_argument("--min-agreement", type=float, default=0.35)
    p.add_argument("--cal", default=None)
    p.add_argument("--mongo-uri", default=os.environ.get(
        "SIGMON_MONGO_URI",
        "mongodb://sigmon:sigmon@127.0.0.1:27017/?authSource=admin"))
    p.add_argument("--mongo-db", default=os.environ.get("SIGMON_MONGO_DB", "signals"))
    p.add_argument("--fallback", default="sigmon_fallback.jsonl")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=8088)
    a = p.parse_args()

    global RADIO
    try:
        RADIO = Radio(a)
    except RuntimeError as e:
        sys.exit(f"error: could not open the radio.\n  {e}\n"
                 "  The B210 is single-session -- stop sigmon.py / rfscan.py "
                 "/ GNU Radio first.")

    th = threading.Thread(target=RADIO.run, daemon=True)
    th.start()

    print(f"\n  sigmon web UI on http://{a.host}:{a.port}\n")
    # threaded=True so a DF request does not block spectrum polling.
    app.run(host=a.host, port=a.port, threaded=True, debug=False)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
