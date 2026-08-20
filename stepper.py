#!/usr/bin/env python3
"""The rotation stage: a TB6600 + stepper, addressed in DEGREES.

`tb6600_go.py` proves the wiring and spins the motor for a number of seconds.
That is the right tool for finding out which pin is which; it is the wrong one
for calibration, which needs to ask for 12.86 degrees and know afterwards how
far it actually went.  This wraps the same GPIO lines in the three things a
calibration run needs and nothing else:

  a POSITION.  Every move is counted, so the stage always knows where it is
      relative to wherever it was when the process started ("home").  There is
      no index switch on this rig, so home is a convention rather than a
      measurement -- but a convention that survives a whole run is enough,
      because the calibration only ever needs RELATIVE angles.

  a SCALE.  steps_per_rev is not knowable from the driver: it is the motor's
      own step angle times whatever the DIP switches are set to times whatever
      pulley ratio is bolted on.  Guessing it wrong scales every calibrated
      angle by the same wrong factor, which looks like a working calibration
      with a stretched azimuth axis.  So it is left as None until something
      MEASURES it -- dfcal.py measures it off the RF -- and every degree-based
      call refuses to run until it is set.

  a DIRECTION that means something.  Backlash in a belt or gearbox is
      typically a degree or more, which is the same size as the errors this is
      trying to calibrate out.  Ignoring it puts a systematic offset between
      clockwise and anticlockwise measurements of the same angle, and the fit
      averages the two into a boresight that is neither.  So every target is
      approached from the same side (`approach_steps`), and the backlash figure
      itself is measurable rather than assumed.

Wiring is the one confirmed in tb6600_go.py -- ENA-/PUL-/DIR- to Pi GND, the
+ sides to GPIO 16/20/21, so every signal is ACTIVE HIGH and idle is LOW:

    ENA (GPIO16)  LOW  = driver enabled, holding torque
                  HIGH = driver offline, motor free
    DIR (GPIO21)  LOW / HIGH select the two directions
    PUL (GPIO20)  a step is a HIGH pulse

Usage:
    ./stepper.py --deg 90                 # needs a calibrated steps-per-rev
    ./stepper.py --steps 800              # always available
    ./stepper.py --steps-per-rev 3200 --deg -45 --save
    ./stepper.py --measure-backlash 400   # how much slack, in steps
"""
import argparse
import json
import os
import signal
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(_HERE, "stepper.json")

# tb6600_id.py lives next to the other rig scripts, not in this repo; it is the
# only thing here that talks to the GPIO character device.
for _p in ("/home/uarf", _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)


def _load_lines():
    from tb6600_id import HIGH, LOW, Lines
    return HIGH, LOW, Lines


class StepperError(RuntimeError):
    pass


class Stepper:
    """A rotation stage that counts.

    `steps_per_rev` may be None: step-based moves still work, degree-based ones
    raise.  That is deliberate -- see the module docstring.
    """

    def __init__(self, pul=20, dir_pin=21, ena=16, steps_per_rev=None,
                 rate=600.0, pulse_s=0.002, min_rate=120.0, ramp_steps=60,
                 backlash_steps=0, approach_steps=200, dir_sign=+1,
                 settle_s=0.35, hold=True, consumer=b"sigmon-stage"):
        HIGH, LOW, Lines = _load_lines()
        self.ON, self.IDLE = HIGH, LOW          # common cathode: active HIGH
        self.pul, self.dir_pin, self.ena = int(pul), int(dir_pin), int(ena)
        self.steps_per_rev = steps_per_rev
        self.rate = float(rate)
        self.pulse_s = float(pulse_s)
        self.min_rate = float(min_rate)
        self.ramp_steps = int(ramp_steps)
        self.backlash_steps = int(backlash_steps)
        self.approach_steps = int(approach_steps)
        self.dir_sign = 1 if int(dir_sign) >= 0 else -1
        self.settle_s = float(settle_s)
        self.hold = bool(hold)

        self.position = 0            # signed steps from home
        self.total_steps = 0         # every pulse ever issued, for wear/sanity
        self._last_dir = None        # which way the last motion went

        self.lines = Lines([self.ena, self.dir_pin, self.pul],
                           consumer=consumer, idle=self.IDLE)
        # ENA idle (LOW) = driver ENABLED and holding. Holding torque matters
        # here: an unpowered stage settles wherever the cable drag leaves it,
        # and every angle after that is wrong by an amount nothing records.
        self.lines.set({self.ena: self.IDLE if self.hold else self.ON,
                        self.dir_pin: self.IDLE, self.pul: self.IDLE})
        time.sleep(0.2)

    # -- configuration ---------------------------------------------------
    @classmethod
    def from_config(cls, path=CONFIG, **kw):
        cfg = {}
        if path and os.path.exists(path):
            with open(path) as f:
                cfg = json.load(f)
        cfg = {k: v for k, v in cfg.items()
               if k in ("steps_per_rev", "backlash_steps", "dir_sign", "rate",
                        "pulse_s", "settle_s", "approach_steps", "ramp_steps",
                        "min_rate")}
        cfg.update({k: v for k, v in kw.items() if v is not None})
        return cls(**cfg)

    def save_config(self, path=CONFIG, extra=None):
        d = dict(steps_per_rev=self.steps_per_rev,
                 backlash_steps=self.backlash_steps,
                 dir_sign=self.dir_sign, rate=self.rate,
                 pulse_s=self.pulse_s, settle_s=self.settle_s,
                 approach_steps=self.approach_steps,
                 ramp_steps=self.ramp_steps, min_rate=self.min_rate)
        if extra:
            d.update(extra)
        with open(path, "w") as f:
            json.dump(d, f, indent=2)
        return path

    # -- degrees <-> steps -----------------------------------------------
    @property
    def calibrated(self):
        return bool(self.steps_per_rev)

    def steps_for(self, deg):
        if not self.steps_per_rev:
            raise StepperError(
                "steps_per_rev is unknown, so degrees mean nothing yet. "
                "Run `dfcal.py --find-scale` to measure it off the RF, or "
                "pass --steps-per-rev if the DIP switches and pulley ratio "
                "are known.")
        return int(round(float(deg) * self.steps_per_rev / 360.0))

    def deg_for(self, steps):
        if not self.steps_per_rev:
            raise StepperError("steps_per_rev is unknown")
        return float(steps) * 360.0 / self.steps_per_rev

    @property
    def angle_deg(self):
        """Position in degrees from home, unwrapped (can exceed 360)."""
        return self.deg_for(self.position)

    # -- motion ----------------------------------------------------------
    def _set_dir(self, forward):
        """`forward` is the +position direction, after dir_sign."""
        level = self.ON if (forward == (self.dir_sign > 0)) else self.IDLE
        self.lines.set({self.dir_pin: level})
        # The TB6600 wants the direction line settled before the first edge;
        # skipping this loses the first step of every reversal, which shows up
        # as backlash that is not mechanical and does not repeat.
        time.sleep(0.005)

    def _pulse_train(self, n, rate, on_step=None):
        """n HIGH pulses on PUL, period-locked, with a short accel ramp.

        The ramp is not decoration: a stepper asked to start at 600 Hz from
        rest under a loaded rotor stalls, and a stalled step is a silent
        position error -- the counter says the stage moved and it did not.
        """
        if n <= 0:
            return 0
        ramp = min(self.ramp_steps, max(0, n // 2))
        t = time.perf_counter()
        edge = t
        for i in range(n):
            if ramp and (i < ramp or i >= n - ramp):
                k = i if i < ramp else (n - 1 - i)
                f = (k + 1) / float(ramp + 1)
                r = self.min_rate + (rate - self.min_rate) * f
            else:
                r = rate
            period = 1.0 / max(r, 1.0)
            high_t = min(self.pulse_s, period * 0.5)

            self.lines.set({self.pul: self.ON})
            edge += high_t
            d = edge - time.perf_counter()
            if d > 0:
                time.sleep(d)
            self.lines.set({self.pul: self.IDLE})
            edge += period - high_t
            d = edge - time.perf_counter()
            if d > 0:
                time.sleep(d)
            if on_step and (i % 200 == 199):
                on_step(i + 1, n)
        self.lines.set({self.pul: self.IDLE})
        return n

    def move_steps(self, steps, settle=True, rate=None):
        """Signed move, no backlash handling.  Returns the new position."""
        steps = int(steps)
        if steps == 0:
            return self.position
        if not self.hold:
            raise StepperError("driver is released (ENA asserted); "
                               "call enable() before moving")
        fwd = steps > 0
        self._set_dir(fwd)
        n = self._pulse_train(abs(steps), rate or self.rate)
        self.position += n if fwd else -n
        self.total_steps += n
        self._last_dir = fwd
        if settle and self.settle_s > 0:
            # Mechanical ringing after a move is real and is not in the step
            # counter. Measuring RF into a stage that is still swinging puts a
            # random angle error on that sample only, which reads as pattern
            # noise and inflates every uncertainty downstream.
            time.sleep(self.settle_s)
        return self.position

    def goto_steps(self, target, settle=True):
        """Move to an absolute step position, always arriving from the SAME
        side so gearbox slack contributes the same offset every time.

        With `approach_steps = A` and a target below the current position, the
        stage overshoots by A and comes back -- so every arrival is a positive
        move of at least A, and the backlash is taken up in the same direction
        it was taken up last time.  A must exceed the real slack, which
        `measure_backlash()` reports.
        """
        target = int(target)
        delta = target - self.position
        if delta == 0 and self._last_dir is not False:
            return self.position
        A = max(self.approach_steps, self.backlash_steps + 10)
        if delta < 0 or self._last_dir is False:
            self.move_steps(delta - A, settle=False)
            self.move_steps(A, settle=settle)
        else:
            self.move_steps(delta, settle=settle)
        return self.position

    def move_deg(self, deg, settle=True):
        return self.move_steps(self.steps_for(deg), settle=settle)

    def goto_deg(self, deg, settle=True):
        return self.goto_steps(self.steps_for(deg), settle=settle)

    def home(self, settle=True):
        """Back to the position the process started at."""
        return self.goto_steps(0, settle=settle)

    # -- driver power ----------------------------------------------------
    def enable(self):
        self.lines.set({self.ena: self.IDLE})       # idle LOW = enabled
        self.hold = True
        time.sleep(0.1)

    def release(self):
        """Drop holding torque.  The position counter is now a guess."""
        self.lines.set({self.ena: self.ON})
        self.hold = False

    # -- measurement -----------------------------------------------------
    def measure_backlash(self, span=400, probe=None, tol=None):
        """How many steps of slack there are, using an external `probe`.

        `probe()` must return a number that varies monotonically with angle
        over a small range -- in this rig that is an RF level from one antenna
        near the steep flank of its pattern, which dfcal supplies.  Without a
        probe this cannot be done at all, and returns None rather than a
        plausible zero.

        Method: read at the target arriving from +, then from -, and count how
        many steps of + motion it takes for the second reading to come back to
        the first.  That count IS the slack.
        """
        if probe is None:
            return None
        self.move_steps(+span)
        self.move_steps(-span)
        self.move_steps(+span)          # arrive from +, slack taken up
        ref = probe()
        self.move_steps(-span)
        self.move_steps(+span - 1)      # arrive from + but one short
        # Creep forward and watch for the reading to return.
        tol = tol if tol is not None else 0.15
        for k in range(0, 4 * span):
            if abs(probe() - ref) <= tol:
                return k
            self.move_steps(1, settle=False)
            time.sleep(0.05)
        return None

    # -- lifecycle -------------------------------------------------------
    def describe(self):
        spr = self.steps_per_rev
        return ("stage: PUL=GPIO%d DIR=GPIO%d ENA=GPIO%d, %s, %g Hz, "
                "backlash %d steps, at %s"
                % (self.pul, self.dir_pin, self.ena,
                   f"{spr} steps/rev ({360.0/spr:.4f} deg/step)" if spr
                   else "steps/rev UNKNOWN",
                   self.rate, self.backlash_steps,
                   f"{self.angle_deg:+.2f} deg" if spr
                   else f"{self.position:+d} steps"))

    def close(self, park_home=False):
        try:
            if park_home and self.steps_per_rev is not None:
                self.goto_steps(0, settle=False)
        except Exception:                                       # noqa: BLE001
            pass
        try:
            self.lines.close()          # everything back to idle, then freed
        except Exception:                                       # noqa: BLE001
            pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def install_signal_guard():
    """SIGTERM/SIGHUP must unwind through the same cleanup as Ctrl-C.

    Straight from tb6600_go.py, and for the same reason: a killed process
    leaves the pads driven, and ENA stuck HIGH leaves the driver offline while
    everything upstream still believes the stage is holding position.
    """
    def _raise(*_):
        raise KeyboardInterrupt
    for s in (signal.SIGTERM, signal.SIGHUP):
        try:
            signal.signal(s, _raise)
        except (ValueError, OSError):
            pass


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pul", type=int, default=20)
    ap.add_argument("--dir", type=int, default=21, dest="dir_pin")
    ap.add_argument("--ena", type=int, default=16)
    ap.add_argument("--steps-per-rev", type=int, default=None)
    ap.add_argument("--dir-sign", type=int, default=None, choices=(1, -1),
                    help="flip if +steps turns the stage the wrong way")
    ap.add_argument("--rate", type=float, default=None)
    ap.add_argument("--backlash-steps", type=int, default=None)
    ap.add_argument("--steps", type=int, default=None, help="signed step move")
    ap.add_argument("--deg", type=float, default=None, help="signed degree move")
    ap.add_argument("--turn", type=float, default=None,
                    help="signed move in whole revolutions")
    ap.add_argument("--release", action="store_true",
                    help="drop holding torque and exit (stage free to move)")
    ap.add_argument("--config", default=CONFIG)
    ap.add_argument("--save", action="store_true", help="write the config back")
    a = ap.parse_args()

    install_signal_guard()
    st = Stepper.from_config(a.config, pul=a.pul, dir_pin=a.dir_pin, ena=a.ena,
                             steps_per_rev=a.steps_per_rev, rate=a.rate,
                             dir_sign=a.dir_sign,
                             backlash_steps=a.backlash_steps)
    print("[stage] " + st.describe())
    try:
        if a.release:
            st.release()
            print("[stage] released -- holding torque off, position counter "
                  "is now meaningless")
        elif a.steps is not None:
            st.move_steps(a.steps)
            print(f"[stage] now at {st.position:+d} steps")
        elif a.deg is not None or a.turn is not None:
            deg = (a.deg or 0.0) + 360.0 * (a.turn or 0.0)
            st.move_deg(deg)
            print(f"[stage] moved {deg:+.3f} deg -> {st.angle_deg:+.2f} deg "
                  f"({st.position:+d} steps)")
    except StepperError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        if a.save:
            print(f"[stage] config -> {st.save_config(a.config)}")
        st.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
