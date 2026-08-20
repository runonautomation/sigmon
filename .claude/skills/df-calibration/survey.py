#!/usr/bin/env python3
"""Hold one antenna, sweep candidate bands, report the strongest steady signals.

Calibration needs a CONTINUOUS single emitter from a fixed site. Broadcast
(DAB/DVB-T) and cellular downlink qualify; WiFi does not -- the README already
shows why bursty multi-emitter channels cannot converge.
"""
import sys, argparse, time
import numpy as np
sys.path.insert(0, "/home/uarf/sigmon")
import dfcal

BANDS = [("FM",        88e6,  108e6, 25),
         ("DAB III",  174e6,  240e6, 25),
         ("UHF TV",   470e6,  700e6, 30),
         ("UHF TV hi",700e6,  790e6, 30),
         ("LTE800/900",791e6, 960e6, 30),
         ("LTE1800",  1805e6, 1880e6, 35),
         ("LTE2100",  2110e6, 2170e6, 35),
         ("2.4 ISM",  2400e6, 2483e6, 25)]
STEP = 4e6

class Got(Exception):
    def __init__(self, ns): self.ns = ns
_o = argparse.ArgumentParser.parse_args
argparse.ArgumentParser.parse_args = lambda s,a=None,n=None: (_ for _ in ()).throw(Got(_o(s,a,n)))
sys.argv = ["dfcal.py","96.0M","--ports","1,2,3,4,5,6,7","--null-port","8",
            "--steps-per-rev","16000","--gain","30"]
try: dfcal.main(); raise SystemExit("no parse")
except Got as g: a = g.ns
argparse.ArgumentParser.parse_args = _o
a.park=None; a.verify_switch=True

rig = dfcal.Rig(a)
rf = rig.rfscan
print("[survey] holding port 1, 2 sweeps per point (steady check)", flush=True)
rig.backend.hold(rig.ports[0])
nsamp = int(rig.rx.rate * 0.02)
rows = []
try:
    for name, lo, hi, gain in BANDS:
        try:
            rig.rx.usrp.set_rx_gain(float(gain), a.rx_chan)
        except Exception:
            pass
        best = []
        f = lo
        while f <= hi:
            try:
                c = rig.rx.tune(f)
                p1 = rf.band_power_db(*rf.welch_psd(rig.rx.capture(nsamp), rig.rx.rate, 200e3), f - c, 200e3)
                p2 = rf.band_power_db(*rf.welch_psd(rig.rx.capture(nsamp), rig.rx.rate, 200e3), f - c, 200e3)
                best.append((f, (p1 + p2) / 2.0, abs(p1 - p2)))
            except Exception as e:
                pass
            f += STEP
        if not best:
            print(f"  {name:12} no data"); continue
        best.sort(key=lambda r: -r[1])
        floor = np.median([b[1] for b in best])
        print(f"\n  === {name:12} gain {gain} dB, floor {floor:6.1f} dBFS ===", flush=True)
        for f, p, jit in best[:4]:
            print(f"      {f/1e6:8.1f} MHz  {p:7.1f} dBFS  ({p-floor:+5.1f} over floor)"
                  f"  steadiness {jit:4.2f} dB")
            rows.append((name, f, p, p - floor, jit))
finally:
    rig.close()

print("\n=== best calibration candidates (strong AND steady) ===")
rows.sort(key=lambda r: -(r[3] - 6 * r[4]))
for name, f, p, over, jit in rows[:10]:
    print(f"  {f/1e6:8.1f} MHz  {name:12} {over:+5.1f} dB over floor, "
          f"jitter {jit:4.2f} dB")
