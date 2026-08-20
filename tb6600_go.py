#!/usr/bin/env python3
"""
Spin the motor with the driver rewired to the MINUS side (common cathode).

    ENA- , PUL- , DIR-   ---> Pi GND
    ENA+ ---> GPIO16      PUL+ ---> GPIO20      DIR+ ---> GPIO21

    CONFIRMED 2026-08-20, all three:
      GPIO16 = ENA  holding torque drops when it is driven HIGH
      GPIO20 = PUL  motor runs on pulses here
      GPIO21 = DIR  motor reverses when it is driven HIGH

That inverts everything the old scripts assumed:

    GPIO HIGH -> current through the opto -> signal ASSERTED
    GPIO LOW  -> opto off                 -> signal idle   <-- resting state

So:
    PUL (GPIO20)  a step is a HIGH pulse, resting LOW.
    DIR (GPIO21)  LOW = one direction, HIGH = the other.
    ENA (GPIO16)  LOW = idle = driver ENABLED and holding.
                  HIGH = ENA asserted = TB6600 goes offline, motor free-spins.
                  We hold it LOW unless you pass --ena-high.

Usage:
    python3 tb6600_go.py                 # forward 10 s, pause, reverse 10 s
    python3 tb6600_go.py --once          # just one run, no direction flip
    python3 tb6600_go.py --rate 1200     # faster (default 600 Hz)
    python3 tb6600_go.py --dur 20        # longer per run
    python3 tb6600_go.py --ena-high      # assert ENA -> should NOT move (sanity check)
"""
import argparse
import signal
import sys
import time

sys.path.insert(0, "/home/uarf")
from tb6600_id import HIGH, LOW, Lines, revs

IDLE = LOW          # common-cathode: LOW is the de-asserted / resting level
ON = HIGH           # HIGH asserts the signal


def spin(lines, pul, rate, dur, pw, label):
    period = 1.0 / rate
    high_t = min(pw, period * 0.5)      # HIGH = the step pulse now
    low_t = period - high_t
    steps_planned = int(rate * dur)

    print()
    print("=" * 68)
    print(f">>> {label}")
    print(f"    pulsing GPIO{pul} HIGH at {rate:g} Hz for {dur:g}s "
          f"= ~{steps_planned} steps  [{high_t*1000:.1f} ms pulse]")
    print(f"    that is {revs(steps_planned)}")
    print("=" * 68, flush=True)

    lines.set({pul: IDLE})
    time.sleep(0.1)                     # let the driver settle before stepping

    t0 = time.perf_counter()
    end = t0 + dur
    edge = t0
    n = 0
    tick = t0 + 1.0
    while time.perf_counter() < end:
        lines.set({pul: ON})
        edge += high_t
        d = edge - time.perf_counter()
        if d > 0:
            time.sleep(d)
        lines.set({pul: IDLE})
        edge += low_t
        n += 1
        d = edge - time.perf_counter()
        if d > 0:
            time.sleep(d)
        if time.perf_counter() >= tick:
            print(f"    ...{time.perf_counter()-t0:4.1f}s   {n} steps", flush=True)
            tick += 1.0

    lines.set({pul: IDLE})
    actual = n / (time.perf_counter() - t0)
    print(f"    done: {n} steps, actual rate {actual:.1f} Hz", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pul", type=int, default=20)
    ap.add_argument("--dir", type=int, default=21)
    ap.add_argument("--ena", type=int, default=16)
    ap.add_argument("--rate", type=float, default=600.0, help="step rate Hz (default 600)")
    ap.add_argument("--dur", type=float, default=10.0, help="seconds per run (default 10)")
    ap.add_argument("--pw", type=float, default=0.002, help="HIGH pulse width s (default 2 ms)")
    ap.add_argument("--gap", type=float, default=3.0, help="pause between the two runs")
    ap.add_argument("--once", action="store_true", help="one run only, skip the reverse")
    ap.add_argument("--ena-high", action="store_true",
                    help="assert ENA (drive it HIGH) -- driver should go offline, no movement")
    a = ap.parse_args()

    pul, dr, ena = a.pul, a.dir, a.ena
    ena_level = ON if a.ena_high else IDLE

    print()
    print("Wiring assumed: ENA-/PUL-/DIR- -> Pi GND  (common cathode)")
    print("                ENA+/PUL+/DIR+ -> GPIO    => ALL SIGNALS ACTIVE HIGH")
    print(f"    GPIO{ena} = ENA  held {'HIGH (ASSERTED -> driver OFFLINE)' if a.ena_high else 'LOW (idle -> driver ENABLED)'}")
    print(f"    GPIO{dr} = DIR")
    print(f"    GPIO{pul} = PUL  (step = HIGH pulse)")

    # SIGTERM/SIGHUP must unwind through the same cleanup as Ctrl-C, or the
    # pads are left driven -- ENA stuck HIGH would leave the driver offline.
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt))
    signal.signal(signal.SIGHUP, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt))

    lines = Lines([ena, dr, pul], consumer=b"tb6600-go", idle=IDLE)
    try:
        lines.set({ena: ena_level, dr: IDLE, pul: IDLE})
        time.sleep(0.2)
        spin(lines, pul, a.rate, a.dur, a.pw,
             f"RUN A -- GPIO{dr} (DIR) LOW / idle")

        if not a.once:
            print(f"\n    pausing {a.gap:g}s -- note which way it turned ...", flush=True)
            time.sleep(a.gap)
            lines.set({dr: ON})
            time.sleep(0.2)
            spin(lines, pul, a.rate, a.dur, a.pw,
                 f"RUN B -- GPIO{dr} (DIR) HIGH / ASSERTED  -> should REVERSE")
            print("\nDid it reverse between RUN A and RUN B?")
            print("  reversed        -> confirmed: GPIO%d = PUL, GPIO%d = DIR" % (pul, dr))
            print("  same direction  -> GPIO%d is not DIR" % dr)
            print("  never moved     -> see the notes below")
    except KeyboardInterrupt:
        print("\ninterrupted -- releasing pins")
    finally:
        lines.close()          # all three back to LOW (idle), then freed
        print("pins released.")

    if not a.ena_high:
        print()
        print("If nothing moved, in order of likelihood:")
        print("  1. 3.3 V may be too weak for the opto -- the TB6600 input resistors are")
        print("     sized for 5 V. Try --pw 0.005 --rate 50, or drive the + side from 5 V")
        print("     and switch the - side (the old active-LOW wiring) instead.")
        print("  2. Motor coil pairs wrong: A+/A- must be one coil, B+/B- the other.")
        print("  3. VCC/GND to the driver, and the DIP current/microstep switches.")
        print("  4. Try swapping which pin is PUL:  --pul 20 --dir 21")
    return 0


if __name__ == "__main__":
    sys.exit(main())
