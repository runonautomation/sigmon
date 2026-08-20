#!/usr/bin/env python3
"""Which device actually moves the RF switch, and how fast it can move it.

There are two, and they are not close:

  usrp -- three GPIO pins on the B210, written from a Python loop.  The switch
      is sub-microsecond but the host is not, and every edge costs a USB round
      trip.  Measured here, a busy-waiting loop holds its period to the
      microsecond down to 50 us slots -- 250 us per 1-2-3-4-null cycle, 4 000
      cycles/s -- and a few writes per thousand are late by 100-350 us when the
      OS preempts the thread.

  esp32 -- a separate board that commutates BY ITSELF.  It is told the
      sequence and the dwell once, over a serial console, and then free-runs:
      a task pinned to the second core busy-waits on the CPU cycle counter and
      writes all three control lines with two register stores.  Measured on
      this board, over the device's own step counter:

          asked      measured    error    per 5-port cycle
          1,000 ns   1,000 ns    +0.0%      5.0 us   200,000 cycles/s
         20,000 ns  20,006 ns    +0.0%    100.0 us    10,000 cycles/s
         50,000 ns  50,016 ns    +0.0%    250.1 us     4,000 cycles/s
        200,000 ns 200,058 ns    +0.0%   1000.3 us     1,000 cycles/s

      The host is not in the loop at all, so a DF becomes nothing but a
      capture.  The fastest usrp slot is the SLOWEST interesting esp32 one.

That changes which limit binds.  With the esp32 the switch is no longer the
constraint at any dwell it will accept, so what sets the shortest useful slot
is the receiver: how long the AD9361 takes to present a settled level after the
port changes (dfstream.py --probe-transition measures it), and how much time a
level needs to exist at all in the chosen RBW (dfcore.min_slot_seconds -- 4
looks at 200 kHz is 20 us).  A 1 us dwell is real and useless: it is 0.2 looks.

Both backends present the same three operations, so dfstream and webui do not
care which is fitted.

A note on flash.  Every esp32 setting except `log` is written to NVS, so
flipping between hold and iterate on every pass would wear it out -- at five
sweeps a second that is ten writes a second, forever.  EspBackend therefore
tracks what the device already has and sends nothing when it matches, and the
callers keep the mode STICKY rather than toggling it per operation.
`nvs_writes` counts what was actually sent, so the claim is checkable rather
than hoped for.
"""
import os
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
for _p in (os.path.join(_HERE, "esp32switch", "tools"),):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)


class SwitchBackend:
    """Interface. `ports` are in the BACKEND's own numbering, not the app's.

    The two devices number their ports differently -- the B210 writes a 3-bit
    GPIO code 0..7, the esp32 console takes 1..8 -- and translating between
    them in the middle of the app is how a bearing ends up rotated by one
    element.  So there is no translation: each backend states which numbers
    mean the four antennas and which means the dead position, and those numbers
    travel unchanged all the way to the level table.
    """

    name = "?"
    default_ports = ()
    default_null = None

    def hold(self, port):
        """Select one port and stay there.  Stops any commutation."""
        raise NotImplementedError

    def begin_cycle(self, ports, slot_s):
        """Commutate `ports` in order, `slot_s` each.  Idempotent."""
        raise NotImplementedError

    def end_cycle(self, park=None):
        """Stop commutating.  `park=None` means LEAVE IT RUNNING."""
        raise NotImplementedError

    def steps(self):
        """Total port changes since boot, or None if the device cannot say."""
        return None

    def achieved_slot_s(self):
        return float("nan")

    def describe(self):
        return self.name

    def close(self):
        pass


# --------------------------------------------------------------------------
class UsrpBackend(SwitchBackend):
    """Three GPIO pins on the B210, commutated by a host thread."""

    name = "usrp"
    default_ports = (0, 1, 2, 3)
    default_null = 4

    def __init__(self, usrp, mask=0xE0):
        import rfscan
        self.sw = rfscan.RFSwitch(usrp, mask=mask)
        self.com = None

    def hold(self, port):
        self.end_cycle(park=None)
        self.sw.select(int(port))

    def begin_cycle(self, ports, slot_s):
        import dfstream
        self.end_cycle(park=None)
        # Sized for the whole recording plus slack; the loop stops on its event
        # long before this unless something has gone badly wrong.
        self.com = dfstream.Commutator(self.sw, list(ports), slot_s,
                                       max_writes=200000)
        self.com.start()

    def end_cycle(self, park=None):
        if self.com is not None:
            self.com.stop(park=park)
            self.com = None
        elif park is not None:
            self.sw.select(int(park))

    def achieved_slot_s(self):
        return self.com.achieved_slot_s() if self.com else float("nan")

    def describe(self):
        return "usrp GPIO (host-timed, ~50 us slots at best)"


# --------------------------------------------------------------------------
class EspBackend(SwitchBackend):
    """The ESP32 switch board, commutating on its own.

    Configure and forget: once `iterate on` has been sent the board free-runs
    at the requested dwell until told otherwise, so taking a DF costs exactly
    one capture and no serial traffic.  That is the whole reason this backend
    exists.
    """

    name = "esp32"
    default_ports = (1, 2, 3, 4)
    default_null = 5

    def __init__(self, device="/dev/ttyACM0", baud=115200, verify=True):
        from switchctl import SwitchSerial, SwitchError
        self._err = SwitchError
        # Opening the port resets these devkits, which loses the step counter
        # but not the settings (they live in NVS). SwitchSerial already clears
        # DTR/RTS and HUPCL, which prevents it on UART-bridge boards and does
        # not on native USB. Either way this is opened ONCE for the life of the
        # process -- state is continuous within a session and not across them.
        self.sw = SwitchSerial(device, baud)
        self.device = device
        self.nvs_writes = 0
        self._seq = None
        self._dwell_ns = None
        self._iterating = None
        self._sync_from_device()
        if verify:
            self.info = self.sw.state()

    def _sync_from_device(self):
        """Learn what the board already has, so nothing needs re-sending."""
        st = self.sw.state()
        self._seq = list(st.get("seq") or [])
        self._dwell_ns = int(st.get("dwell_ns") or 0)
        self._iterating = bool(st.get("iterate"))
        self._port = int(st.get("port") or 0)
        return st

    # -- the interface --------------------------------------------------
    def hold(self, port):
        port = int(port)
        if self._iterating or self._port != port:
            self.sw.hold(port)                 # `port` also stops iteration
            self.nvs_writes += 1
            self._iterating, self._port = False, port

    def begin_cycle(self, ports, slot_s):
        want_seq = [int(p) for p in ports]
        want_ns = int(round(slot_s * 1e9))
        # Only what actually differs. Re-sending an identical `seq` or `dwell`
        # is a flash write for no change, and this is called on every DF.
        if self._seq != want_seq:
            self.sw.sequence(want_seq)
            self.nvs_writes += 1
            self._seq = want_seq
        if self._dwell_ns != want_ns:
            self.sw.dwell(want_ns)
            self.nvs_writes += 1
            self._dwell_ns = want_ns
        if not self._iterating:
            self.sw.iterate(True)
            self.nvs_writes += 1
            self._iterating = True

    def end_cycle(self, park=None):
        """park=None leaves the board free-running, which is the point."""
        if park is None:
            return
        self.hold(park)

    def steps(self):
        try:
            return int(self.sw.state().get("steps", 0))
        except (self._err, ValueError, KeyError):
            return None

    def achieved_slot_s(self):
        return self._dwell_ns * 1e-9 if self._dwell_ns else float("nan")

    def measure_rate(self, seconds=2.0):
        """Independent check of the real period, from the device's counter.

        Worth running once at start-up: it is the only thing that confirms the
        lines are actually moving, and it is measured by the device rather than
        asserted by us.
        """
        rate, per_ns = self.sw.measure_rate(seconds)
        return rate, per_ns

    def describe(self):
        d = self._dwell_ns or 0
        return (f"esp32 on {self.device} (self-timed, {d/1000:.0f} us/slot"
                f", {'iterating' if self._iterating else 'held'})")

    def close(self):
        try:
            self.sw.close()
        except Exception:                                       # noqa: BLE001
            pass


# --------------------------------------------------------------------------
def open_backend(kind, usrp=None, device="/dev/ttyACM0", baud=115200,
                 gpio_mask=0xE0, auto=True):
    """Pick a backend.  `kind` is 'esp32', 'usrp' or 'auto'.

    'auto' prefers the esp32 and falls back to the B210's GPIO with a printed
    reason, because a silent fallback would drop the commutation rate by three
    orders of magnitude and nothing downstream would say so -- the bearings
    would just quietly come from 250 us cycles instead of 5 us ones.
    """
    if kind == "usrp":
        return UsrpBackend(usrp, mask=gpio_mask)
    if kind == "esp32":
        return EspBackend(device, baud)
    try:
        return EspBackend(device, baud)
    except Exception as e:                                      # noqa: BLE001
        if not auto:
            raise
        print(f"[switch] esp32 on {device} unavailable ({type(e).__name__}: {e})"
              f"\n[switch] falling back to the B210's GPIO -- commutation drops "
              f"from microseconds to ~50 us per slot", flush=True)
        return UsrpBackend(usrp, mask=gpio_mask)


def exercise(backend, hold_s=2.0, ports=None, show=print):
    """Walk the ports slowly and say what each control line should read.

    For probing the switch header with a meter, and for one specific confusion
    that costs an afternoon: on this truth table LINE 1 IS LOW FOR PORTS 1, 2
    AND 3.  It first goes high at port 4.  So "the ESP32 never drives line 1
    high" is the expected reading if the switch is sitting on an antenna port,
    and says nothing about whether the board works.

    The decisive pair is port 7 (all three lines high) against port 8 (all
    three low).  If those two do not differ at the header, the problem is
    before the switch -- pin mapping, wiring or supply.  If they do differ and
    the RF still does not change, the problem is after it.
    """
    st = None
    if getattr(backend, "sw", None) is not None and backend.name == "esp32":
        try:
            st = backend.sw.state()
        except Exception:                                       # noqa: BLE001
            st = None
    codes = (st or {}).get("codes")
    gpio = (st or {}).get("gpio")
    if gpio:
        show(f"control lines: L1=GPIO{gpio[0]}  L2=GPIO{gpio[1]}  "
             f"L3=GPIO{gpio[2]}   (pattern is L1 L2 L3, leftmost first)")
    show("NOTE: line 1 is LOW on ports 1-3 by design; it first goes high on "
         "port 4.\n      Compare port 7 (111) against port 8 (000) to prove "
         "the board drives all three.")
    for p in (ports or range(1, 9)):
        backend.hold(p)
        pat = codes[p - 1] if codes and 1 <= p <= len(codes) else "???"
        show(f"  port {p}: expect L1={pat[0]} L2={pat[1]} L3={pat[2]}  "
             f"(pattern {pat})")
        time.sleep(hold_s)


def recommend_slot_us(rbw_hz, guard=0.25, settle_us=5.0, looks=4.0):
    """Shortest slot worth asking for, and why.

    Two independent floors, and the switch is no longer either of them once the
    esp32 is fitted:

      measurement -- a level needs `looks` independent looks to mean anything,
          and a slot only contributes its unguarded middle, so
          slot >= looks / (rbw * (1 - 2*guard)).
      settling -- the receive chain has to have finished responding to the step
          before any of the slot is usable, so slot >= settle / guard.

    Returns (slot_us, which_binds).
    """
    need_meas = looks / (rbw_hz * max(1.0 - 2.0 * guard, 1e-3)) * 1e6
    need_settle = settle_us / max(guard, 1e-3)
    if need_meas >= need_settle:
        return need_meas, "rbw"
    return need_settle, "settling"


# --------------------------------------------------------------------------
def main():
    """Quick check of whichever backend is present."""
    import argparse
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--switch", default="auto", choices=("auto", "esp32", "usrp"))
    p.add_argument("--device", default="/dev/ttyACM0")
    p.add_argument("--slot-us", type=float, default=50.0)
    p.add_argument("--seconds", type=float, default=2.0)
    p.add_argument("--rbw", type=float, default=200e3)
    p.add_argument("--exercise", action="store_true",
                   help="walk the ports slowly and print the level each "
                        "control line should be at, for probing the header")
    p.add_argument("--hold", type=int, default=None, metavar="PORT",
                   help="select one port and exit")
    a = p.parse_args()

    b = open_backend(a.switch, device=a.device, auto=False)
    print(f"[switch] {b.describe()}")

    if a.hold is not None:
        b.hold(a.hold)
        print(f"[switch] holding port {a.hold}")
        b.close()
        return 0

    if a.exercise:
        try:
            exercise(b, hold_s=1.5)
        except KeyboardInterrupt:
            print("\nstopped")
        b.close()
        return 0
    print(f"[switch] antennas {list(b.default_ports)}, "
          f"no-signal port {b.default_null}")
    slot, why = recommend_slot_us(a.rbw)
    print(f"[switch] at {a.rbw/1e3:.0f} kHz RBW the shortest useful slot is "
          f"{slot:.0f} us ({why}-limited), not the switch")

    seq = list(b.default_ports) + [b.default_null]
    b.begin_cycle(seq, a.slot_us * 1e-6)
    if isinstance(b, EspBackend):
        rate, per = b.measure_rate(a.seconds)
        print(f"[switch] measured {per:,.0f} ns/slot "
              f"({rate/len(seq)/1e3:.2f} kcycle/s over {len(seq)} ports), "
              f"asked {a.slot_us*1000:,.0f} ns")
        print(f"[switch] {b.nvs_writes} NVS write(s) this session")
    else:
        time.sleep(a.seconds)
        print(f"[switch] host loop achieved "
              f"{b.achieved_slot_s()*1e6:.1f} us/slot")
    b.end_cycle(park=b.default_null)
    b.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
