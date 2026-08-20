#!/usr/bin/env python3
"""
rfscan -- 8-channel RF switch scanner for a LibreSDR / USRP B210.

Cycles an 8-way RF switch driven from the B210's front-panel GPIO header
(3 bits = binary channel select) and reports the received level across a
frequency band for every channel.

    $ ./rfscan.py                 # sweep the default 95 - 105 MHz band
    $ ./rfscan.py 100M            # single frequency
    $ ./rfscan.py 95M 105M        # explicit band

For each channel the app asserts the 3 GPIO select bits, lets the switch and
the AD9361 settle, captures IQ, and computes a Welch power spectrum. Levels
are integrated over a resolution bandwidth (default 200 kHz, the FM broadcast
channel width) at each requested frequency.

Why it captures a band rather than retuning per point: the B210 can see the
whole 95-105 MHz span in one or two captures, so a 0.5 s dwell buys far more
averaging than 50 individual retunes would. If the span does not fit in the
achievable sample rate (e.g. on a USB 2.0 port) it is split into segments
automatically.

Gain is fixed and AGC is disabled -- otherwise levels between channels would
not be comparable, which is the entire point of the exercise.
"""
import argparse
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
NCHAN = 8

# UHD needs the FX3 firmware (usrp_b200_fw.hex) from its images directory, and
# only ever looks at $UHD_IMAGES_DIR, $UHD_PKG_PATH or the compiled-in
# /usr/share/uhd/images -- the latter exists only if uhd_images_downloader was
# run as root. Point it at the copy bundled here so the app just works.
# Set before importing uhd, and never override a choice the user already made.
_BUNDLED_IMAGES = os.path.join(HERE, "uhd_images")
if not os.environ.get("UHD_IMAGES_DIR") and os.path.isdir(_BUNDLED_IMAGES):
    os.environ["UHD_IMAGES_DIR"] = _BUNDLED_IMAGES

try:
    import uhd
except ImportError:
    sys.exit("error: python3-uhd is not installed.\n"
             "  sudo apt install -y uhd-host libuhd-dev python3-uhd")


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------
def parse_freq(s):
    """Accept 100e6, 100M, 100MHz, 95.5m, or a bare number meaning MHz."""
    t = str(s).strip().lower().replace("hz", "")
    mult = 1.0
    if t.endswith("g"):
        mult, t = 1e9, t[:-1]
    elif t.endswith("m"):
        mult, t = 1e6, t[:-1]
    elif t.endswith("k"):
        mult, t = 1e3, t[:-1]
    try:
        v = float(t) * mult
    except ValueError:
        raise argparse.ArgumentTypeError(f"cannot parse frequency {s!r}")
    # A bare "100" almost certainly means 100 MHz, not 100 Hz.
    if mult == 1.0 and v < 1e5:
        v *= 1e6
    return v


def default_fpga():
    """detect_fpga.sh records the working bitstream here."""
    marker = os.path.join(HERE, ".fpga_image")
    if os.path.exists(marker):
        p = open(marker).read().strip()
        if p and os.path.exists(p):
            return p
    return None


def welch_psd(x, fs, rbw):
    """Welch PSD, normalised so a full-scale sine reads 0 dBFS. Returns (f, P)."""
    # Aim for ~8 bins per resolution bandwidth so the RBW integration is smooth.
    target = max(64, int(fs / max(rbw / 8.0, 1.0)))
    nfft = 1 << int(np.ceil(np.log2(target)))
    nfft = min(nfft, 1 << int(np.floor(np.log2(max(len(x), 64)))))
    if nfft < 64 or len(x) < nfft:
        raise RuntimeError(f"not enough samples ({len(x)}) for an {nfft}-point FFT")

    w = np.hanning(nfft)
    step = max(1, nfft // 2)
    acc = np.zeros(nfft)
    n = 0
    for i in range(0, len(x) - nfft + 1, step):
        acc += np.abs(np.fft.fft(x[i:i + nfft] * w)) ** 2
        n += 1
    P = np.fft.fftshift(acc / n)
    # PSD per Hz. These are complex baseband samples, so full scale is |x| = 1
    # and a unit-amplitude complex tone integrates to exactly 0 dBFS. (No /2 --
    # that factor is the real-sine convention and would read 3 dB high here.)
    P /= (fs * np.sum(w ** 2))
    f = np.fft.fftshift(np.fft.fftfreq(nfft, 1.0 / fs))
    return f, P


def band_power_db(f, P, centre, rbw):
    """Integrate the PSD over centre +/- rbw/2 and return dB."""
    sel = np.abs(f - centre) <= rbw / 2.0
    if not sel.any():
        sel = [np.argmin(np.abs(f - centre))]
    df = f[1] - f[0]
    return 10.0 * np.log10(np.sum(P[sel]) * df + 1e-30)


def plan_segments(start, stop, usable_bw):
    """Split [start, stop] into tuning segments no wider than usable_bw."""
    span = stop - start
    if span <= 0:
        return [(start, start, start)]
    n = max(1, int(np.ceil(span / usable_bw)))
    seg_w = span / n
    segs = []
    for i in range(n):
        lo = start + i * seg_w
        hi = lo + seg_w
        segs.append(((lo + hi) / 2.0, lo, hi))
    return segs


# --------------------------------------------------------------------------
# switch control
# --------------------------------------------------------------------------
class RFSwitch:
    """8-way RF switch on 3 GPIO bits of the B210 front-panel header.

    The select bits do not have to start at GPIO0. The mask says which pins
    carry them and the channel number is shifted into place, so on this board
    -- where the switch is wired to GPIO 5, 6 and 7 -- mask 0xE0 means channel
    3 drives 0b011 << 5 = 0b01100000.

    The lowest pin in the mask is the least significant bit of the channel
    number (GPIO5 here). If the channels come out in a scrambled order, the
    switch numbers its bits the other way round: use reverse=True.
    """

    def __init__(self, usrp, bank=None, mask=0xE0, invert=False, reverse=False):
        banks = usrp.get_gpio_banks(0)
        if bank is None:
            bank = "FP0" if "FP0" in banks else banks[0]
        if bank not in banks:
            raise RuntimeError(f"GPIO bank {bank!r} not found; device has {banks}")
        if mask == 0:
            raise ValueError("--gpio-mask must select at least one pin")
        self.usrp, self.bank, self.mask = usrp, bank, mask
        self.invert, self.reverse = invert, reverse

        self.shift = (mask & -mask).bit_length() - 1   # index of lowest set pin
        self.width = mask >> self.shift                # 0b111 for three pins
        self.nbits = self.width.bit_length()

        # CTRL=0 -> the bits are driven by software, not by the ATR state machine.
        usrp.set_gpio_attr(bank, "CTRL", 0x00, mask)
        usrp.set_gpio_attr(bank, "DDR", mask, mask)   # 1 = output
        self.select(0)

    def code(self, ch):
        """The full GPIO byte written for channel ch."""
        c = ch
        if self.reverse:
            c = int(format(ch, f"0{self.nbits}b")[::-1], 2)
        if self.invert:
            c = ~c
        return (c & self.width) << self.shift

    def select(self, ch):
        if not 0 <= ch < NCHAN:
            raise ValueError(f"channel {ch} out of range 0-7")
        self.usrp.set_gpio_attr(self.bank, "OUT", self.code(ch), self.mask)

    def readback(self):
        try:
            return self.usrp.get_gpio_attr(self.bank, "READBACK") & self.mask
        except Exception:
            return None

    def pins(self):
        """Names of the pins in the mask, most significant first."""
        return [f"GPIO{i}" for i in range(self.shift + self.nbits - 1,
                                          self.shift - 1, -1)]


# --------------------------------------------------------------------------
# receiver
# --------------------------------------------------------------------------
class Receiver:
    def __init__(self, usrp, chan, rate, gain, antenna, lo_frac):
        self.usrp, self.chan, self.lo_frac = usrp, chan, lo_frac
        usrp.set_rx_antenna(antenna, chan)
        usrp.set_rx_rate(rate, chan)
        self.rate = usrp.get_rx_rate(chan)

        # Fixed gain, no AGC -- levels must be comparable between channels.
        try:
            usrp.set_rx_agc(False, chan)
        except Exception:
            pass
        usrp.set_rx_gain(gain, chan)
        try:
            usrp.set_rx_bandwidth(min(self.rate * 1.2, 56e6), chan)
        except Exception:
            pass
        try:
            usrp.set_rx_dc_offset(True, chan)
        except Exception:
            pass

        sa = uhd.usrp.StreamArgs("fc32", "sc16")
        sa.channels = [chan]
        self.streamer = usrp.get_rx_stream(sa)
        self.md = uhd.types.RXMetadata()
        self.maxn = self.streamer.get_max_num_samps()
        self.overflows = 0
        # Largest sample magnitude seen, to catch ADC overload. Once the input
        # clips, every level in the table is wrong -- compressed on the strong
        # channel and lifted elsewhere by the resulting intermodulation -- so
        # this has to be reported rather than quietly averaged away.
        self.peak = 0.0

    @property
    def clipped(self):
        return self.peak >= 0.98

    def tune(self, centre):
        # Offset the LO so its leakage lands outside the analysed band.
        tr = uhd.types.TuneRequest(centre, self.lo_frac * self.rate)
        self.usrp.set_rx_freq(tr, self.chan)
        return self.usrp.get_rx_freq(self.chan)

    def lo_locked(self):
        try:
            return self.usrp.get_rx_sensor("lo_locked", self.chan).to_bool()
        except Exception:
            return None

    def capture(self, nsamps):
        cmd = uhd.types.StreamCMD(uhd.types.StreamMode.num_done)
        cmd.num_samps = int(nsamps)
        cmd.stream_now = True
        self.streamer.issue_stream_cmd(cmd)

        out = np.empty(int(nsamps), dtype=np.complex64)
        buf = np.empty((1, self.maxn), dtype=np.complex64)
        got = 0
        deadline = time.time() + 5.0 + nsamps / self.rate
        while got < nsamps and time.time() < deadline:
            n = self.streamer.recv(buf, self.md, 1.0)
            ec = self.md.error_code
            if ec == uhd.types.RXMetadataErrorCode.overflow:
                self.overflows += 1
                continue
            if ec == uhd.types.RXMetadataErrorCode.timeout:
                break
            if ec != uhd.types.RXMetadataErrorCode.none:
                raise RuntimeError(f"RX error: {self.md.strerror()}")
            take = min(n, nsamps - got)
            out[got:got + take] = buf[0, :take]
            got += take
        if got:
            self.peak = max(self.peak, float(np.max(np.abs(out[:got]))))
        return out[:got]


# --------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------
def main():
    p = argparse.ArgumentParser(
        description="Scan 8 RF switch channels over a frequency band on a B210 / LibreSDR.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="examples:\n"
               "  ./rfscan.py                 sweep 95-105 MHz on all 8 channels\n"
               "  ./rfscan.py 100M            single frequency\n"
               "  ./rfscan.py 88M 108M --step 0.2M --repeat 0\n"
               "  ./rfscan.py --test-switch   cycle the GPIO slowly to verify wiring\n")
    p.add_argument("freqs", nargs="*", type=parse_freq, metavar="FREQ",
                   help="one frequency, or START STOP (default: 95M 105M)")
    p.add_argument("--step", type=parse_freq, default=200e3,
                   help="frequency step (default 200k, the FM channel raster)")
    p.add_argument("--rbw", type=parse_freq, default=200e3,
                   help="resolution bandwidth each level is integrated over (default 200k)")
    p.add_argument("--dwell", type=float, default=0.5,
                   help="seconds spent measuring each channel (default 0.5)")
    p.add_argument("--settle", type=float, default=2.0,
                   help="warm-up seconds before the first measurement (default 2.0)")
    p.add_argument("--switch-settle", type=float, default=0.01,
                   help="seconds after changing the GPIO before capturing (default 0.01)")
    p.add_argument("--channels", default="0-7",
                   help="channels to scan, e.g. 0-7 or 0,2,5 (default 0-7)")
    p.add_argument("--repeat", type=int, default=1,
                   help="number of passes; 0 = loop until Ctrl-C (default 1)")
    p.add_argument("--gain", type=float, default=40.0, help="RX gain in dB (default 40)")
    p.add_argument("--rate", type=parse_freq, default=None,
                   help="force sample rate (default: chosen to cover the span)")
    p.add_argument("--max-rate", type=parse_freq, default=16e6,
                   help="upper bound on the auto-chosen rate (default 16M; use ~8M on USB 2.0)")
    p.add_argument("--antenna", default="TX/RX",
                   help="RX antenna port. Default TX/RX: that is where the "
                        "switch common was measured to be (see switch_map.py)")
    p.add_argument("--rx-chan", type=int, default=0, help="B210 RX channel (default 0)")
    p.add_argument("--fpga", default=None, help="LibreSDR FPGA bitstream (.bin)")
    p.add_argument("--args", default="", help="extra UHD device args")
    p.add_argument("--gpio-bank", default=None, help="GPIO bank (default FP0)")
    p.add_argument("--gpio-mask", type=lambda s: int(s, 0), default=0xE0,
                   help="GPIO pins carrying the 3 select lines "
                        "(default 0xE0 = GPIO 5,6,7)")
    p.add_argument("--gpio-invert", action="store_true",
                   help="switch uses active-low select lines")
    p.add_argument("--gpio-reverse", action="store_true",
                   help="switch numbers its select bits the other way round")
    p.add_argument("--cal-offset", type=float, default=0.0,
                   help="dB added to every level, to report dBm against a known reference")
    p.add_argument("--csv", default=None, help="append results to this CSV file")
    p.add_argument("--test-switch", action="store_true",
                   help="cycle channels 0-7 slowly and exit, for verifying wiring")
    a = p.parse_args()

    # Line-buffer stdout: --test-switch and --repeat 0 run indefinitely, and
    # Python block-buffers when piped, which would hide all output until exit.
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except Exception:
        pass

    # ---- frequency plan ----
    if len(a.freqs) == 0:
        start, stop = 95e6, 105e6
    elif len(a.freqs) == 1:
        start = stop = a.freqs[0]
    elif len(a.freqs) == 2:
        start, stop = sorted(a.freqs)
    else:
        p.error("give at most two frequencies (START STOP)")

    if stop > start:
        npts = int(round((stop - start) / a.step)) + 1
        points = start + np.arange(npts) * a.step
        points = points[points <= stop + a.step / 2]
    else:
        points = np.array([start])

    try:
        if "-" in a.channels:
            lo, hi = a.channels.split("-")
            chans = list(range(int(lo), int(hi) + 1))
        else:
            chans = [int(c) for c in a.channels.split(",") if c != ""]
    except ValueError:
        p.error(f"cannot parse --channels {a.channels!r}")
    if not chans or any(not 0 <= c < NCHAN for c in chans):
        p.error("--channels must select values in 0-7")

    # ---- open the device ----
    fpga = a.fpga or default_fpga()
    dev_args = "type=b200"
    if fpga:
        if not os.path.exists(fpga):
            sys.exit(f"error: FPGA image not found: {fpga}")
        dev_args += f",fpga={os.path.abspath(fpga)}"
    if a.args:
        dev_args += "," + a.args

    print(f"UHD {uhd.get_version_string()}")
    print(f"opening: {dev_args}")
    if not fpga:
        print("  note: no LibreSDR bitstream given -- run ./detect_fpga.sh first,")
        print("        or pass --fpga fpga_images/usrp_b210_fpga_XC7A100T.bin")
    try:
        usrp = uhd.usrp.MultiUSRP(dev_args)
    except Exception as e:
        sys.exit(f"error: could not open the device: {e}")

    sw = RFSwitch(usrp, a.gpio_bank, a.gpio_mask, a.gpio_invert, a.gpio_reverse)
    flags = ("".join([", active-low" if a.gpio_invert else "",
                      ", bit-reversed" if a.gpio_reverse else ""]))
    print(f"RF switch on GPIO bank {sw.bank}, pins "
          f"{'/'.join(sw.pins())} (mask 0x{sw.mask:02X}){flags}")

    if a.test_switch:
        print(f"\ncycling channels 0-7 on {'/'.join(sw.pins())}, "
              f"1 s each -- Ctrl-C to stop")
        try:
            while True:
                for c in range(NCHAN):
                    sw.select(c)
                    rb = sw.readback()
                    want = sw.code(c)
                    sel = format(want >> sw.shift, f"0{sw.nbits}b")
                    rbs = "" if rb is None else \
                        f"  readback 0b{rb:08b}{'' if rb == want else '  MISMATCH'}"
                    print(f"  ch{c}  {'/'.join(sw.pins())} = {sel}"
                          f"  byte 0b{want:08b}{rbs}")
                    time.sleep(1.0)
        except KeyboardInterrupt:
            print("\nstopped")
        return 0

    # ---- sample rate and segmentation ----
    span = max(stop - start, a.rbw * 4)
    usable_frac = 0.60          # analysed fraction of the rate; keeps the LO spike out
    lo_frac = 0.35              # LO offset as a fraction of the rate
    want_rate = span / usable_frac if a.rate is None else a.rate
    want_rate = float(np.clip(want_rate, 2e6, a.max_rate))

    rx = Receiver(usrp, a.rx_chan, want_rate, a.gain, a.antenna, lo_frac)
    usable_bw = rx.rate * usable_frac
    segs = plan_segments(start, stop, usable_bw)

    print(f"rate: {rx.rate/1e6:.3f} Msps  (usable {usable_bw/1e6:.2f} MHz per tune)")
    print(f"band: {start/1e6:.3f} - {stop/1e6:.3f} MHz in {len(points)} steps "
          f"of {a.step/1e3:.0f} kHz, RBW {a.rbw/1e3:.0f} kHz")
    print(f"tuning segments: {len(segs)}   gain: {a.gain:.0f} dB (AGC off)   "
          f"antenna: {a.antenna}")
    print(f"channels: {chans}   dwell {a.dwell:.2f} s/ch")

    unit = "dBm" if a.cal_offset else "dBFS"

    # Warm-up: let the LO and the AD9361 settle before anything is believed.
    rx.tune(segs[0][0])
    print(f"\nwarming up for {a.settle:.1f} s ...")
    time.sleep(a.settle)
    lk = rx.lo_locked()
    print(f"LO locked: {lk}" if lk is not None else "LO lock sensor unavailable")
    if lk is False:
        print("  warning: LO is not locked, levels will be unreliable")

    csv_f = None
    if a.csv:
        new = not os.path.exists(a.csv)
        csv_f = open(a.csv, "a")
        if new:
            csv_f.write("timestamp,pass,channel,freq_hz,level_db,unit\n")

    # ---- scan ----
    per_seg = a.dwell / len(segs)
    npass = 0
    try:
        while a.repeat == 0 or npass < a.repeat:
            npass += 1
            levels = {}
            t0 = time.time()
            print(f"\n--- pass {npass} ---")

            for ch in chans:
                sw.select(ch)
                time.sleep(a.switch_settle)

                f_all, P_all = [], []
                for centre, lo, hi in segs:
                    actual = rx.tune(centre)
                    time.sleep(min(0.02, per_seg / 4))
                    n = max(4096, int(rx.rate * per_seg * 0.75))
                    x = rx.capture(n)
                    if len(x) < 4096:
                        print(f"  ch{ch}: only {len(x)} samples, skipping segment")
                        continue
                    f, P = welch_psd(x, rx.rate, a.rbw)
                    keep = np.abs(f) <= usable_bw / 2.0
                    f_all.append(f[keep] + actual)
                    P_all.append(P[keep])

                if not f_all:
                    levels[ch] = np.full(len(points), np.nan)
                    continue
                f_rf = np.concatenate(f_all)
                P_rf = np.concatenate(P_all)
                order = np.argsort(f_rf)
                f_rf, P_rf = f_rf[order], P_rf[order]

                lv = np.array([band_power_db(f_rf, P_rf, pt, a.rbw) + a.cal_offset
                               for pt in points])
                levels[ch] = lv

                best = int(np.nanargmax(lv))
                print(f"  ch{ch}: peak {lv[best]:7.1f} {unit} @ {points[best]/1e6:8.3f} MHz"
                      f"   mean {np.nanmean(lv):7.1f} {unit}")

                if csv_f:
                    ts = time.time()
                    for pt, v in zip(points, lv):
                        csv_f.write(f"{ts:.3f},{npass},{ch},{pt:.0f},{v:.2f},{unit}\n")
                    csv_f.flush()

            # ---- table ----
            print()
            hdr = "  Freq (MHz) " + "".join(f"{'ch%d' % c:>9}" for c in chans)
            print(hdr)
            print("  " + "-" * (len(hdr) - 2))
            for i, pt in enumerate(points):
                row = "".join(
                    f"{levels[c][i]:>9.1f}" if np.isfinite(levels[c][i]) else f"{'--':>9}"
                    for c in chans)
                print(f"  {pt/1e6:10.3f} {row}")
            print(f"  levels in {unit}, RBW {a.rbw/1e3:.0f} kHz, "
                  f"gain {a.gain:.0f} dB")
            if rx.overflows:
                print(f"  warning: {rx.overflows} overflow(s) -- lower --max-rate "
                      f"or use a USB 3.0 port")
                rx.overflows = 0
            if rx.clipped:
                print(f"  WARNING: ADC overload (peak {rx.peak:.3f} of full "
                      f"scale) -- every level above is wrong. Lower --gain.")
            elif rx.peak > 0.5:
                print(f"  note: peak {rx.peak:.2f} of full scale, close to "
                      f"overload; consider a lower --gain")
            rx.peak = 0.0
            print(f"  pass took {time.time() - t0:.2f} s")
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        if csv_f:
            csv_f.close()
        try:
            sw.select(0)
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
