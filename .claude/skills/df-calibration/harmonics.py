#!/usr/bin/env python3
"""Dump raw level-vs-rotation and decompose it into harmonics.

dfcal reports 'pattern depth' and '1st harm' but not where the rest of the
variation lives.  If the ONE-cycle-per-turn term is not dominant, no amount of
fixing the stage scale will help: the curve being fitted is not the element
pattern.
"""
import sys, argparse, json
import numpy as np
sys.path.insert(0, "/home/uarf/sigmon")
import dfcal

class Got(Exception):
    def __init__(self, ns): self.ns = ns

_orig = argparse.ArgumentParser.parse_args
def _fake(self, args=None, namespace=None):
    raise Got(_orig(self, args=args, namespace=namespace))

argparse.ArgumentParser.parse_args = _fake
sys.argv = ["dfcal.py", "96.0M", "--ports", "1,2,3,4,5,6,7", "--null-port", "8",
            "--rotates", "array", "--dir-plus", "cw", "--steps-per-rev", "16000",
            "--angles", "36", "--repeats", "3"]
try:
    dfcal.main()
    raise SystemExit("parser never fired")
except Got as g:
    a = g.ns
argparse.ArgumentParser.parse_args = _orig
a.park = None
a.verify_switch = True

rig = dfcal.Rig(a)
freq = rig.rfscan.parse_freq(a.freq)
print(f"[diag] {freq/1e6:.4f} MHz, ports {rig.ports}, {rig.stage.describe()}", flush=True)
try:
    angles, L, N, drops = dfcal.sweep(rig, freq, a.angles, tag="diag")
finally:
    rig.close()

angles = np.asarray(angles, float); L = np.asarray(L, float); N = np.asarray(N, float)
out = "/tmp/claude-1000/-home-uarf-sigmon/d119d42b-e671-464b-8377-533682d7aa7d/scratchpad/sweep36.npz"
np.savez(out, angles=angles, L=L, N=N, ports=np.array(rig.ports))
print(f"[diag] {drops} drops; raw levels -> {out}")

def harm(y, nmax=10):
    y = np.asarray(y, float); n = len(y)
    y = y - np.nanmean(y)
    F = np.fft.rfft(np.nan_to_num(y)) / n
    return 2.0 * np.abs(F[1:nmax + 1])

print("\n=== harmonic amplitudes (dB), common mode REMOVED ===")
Ln = dfcal.common_mode(L)
hdr = "  port  ptp " + " ".join(f"  h{k}" for k in range(1, 9))
print(hdr)
for i, p in enumerate(rig.ports):
    h = harm(Ln[:, i])
    ptp = np.nanmax(Ln[:, i]) - np.nanmin(Ln[:, i])
    print(f"  {p:4d} {ptp:5.2f} " + " ".join(f"{v:5.2f}" for v in h[:8]))

print("\n=== harmonic amplitudes (dB), RAW (drift left in) ===")
print(hdr)
for i, p in enumerate(rig.ports):
    h = harm(L[:, i])
    ptp = np.nanmax(L[:, i]) - np.nanmin(L[:, i])
    print(f"  {p:4d} {ptp:5.2f} " + " ".join(f"{v:5.2f}" for v in h[:8]))

hn = harm(N); print("\n  null ptp %.2f dB, h1..h4 %s"
                    % (np.nanmax(N) - np.nanmin(N), " ".join(f"{v:.2f}" for v in hn[:4])))

Hs = np.array([harm(Ln[:, i]) for i in range(L.shape[1])])
tot = Hs.sum(axis=0)
print("\n=== summed across ports ===")
for k in range(8):
    bar = "#" * int(round(40 * tot[k] / max(tot.max(), 1e-9)))
    print(f"  h{k+1}: {tot[k]:5.2f} dB  {bar}")
print(f"\n  dominant harmonic = h{int(np.argmax(tot))+1}")
print("  h1 (element pattern) is %.0f%% of the largest term"
      % (100.0 * tot[0] / max(tot.max(), 1e-9)))
