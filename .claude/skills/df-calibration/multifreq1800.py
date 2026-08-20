#!/usr/bin/env python3
"""One rotation, several reference stations, complex-h1 averaged across them.

The element's mounting angle is the SAME at every frequency; the multipath that
is corrupting it is not.  So averaging the complex fundamental across widely
separated FM carriers should let the geometry add coherently while the
interference partially cancels.  If ring coherence rises with the number of
frequencies averaged, the array geometry is recoverable and the single-frequency
result was multipath-limited.  If it does not rise, the elements simply lack the
directivity amplitude DF needs.
"""
import sys, argparse, json, time
import numpy as np
sys.path.insert(0, "/home/uarf/sigmon")
import dfcal

FREQS = [1805e6 + 5e6*i for i in range(16)]   # LTE1800 DL, 1805-1880
NANG = 36

class Got(Exception):
    def __init__(self, ns): self.ns = ns
_orig = argparse.ArgumentParser.parse_args
argparse.ArgumentParser.parse_args = lambda s, args=None, namespace=None: (_ for _ in ()).throw(Got(_orig(s, args, namespace)))
sys.argv = ["dfcal.py", "96.0M", "--ports", "1,2,3,4,5,6,7", "--null-port", "8",
            "--rotates", "array", "--dir-plus", "cw", "--steps-per-rev", "16000",
            "--angles", str(NANG), "--repeats", "2", "--gain", "40"]
try:
    dfcal.main(); raise SystemExit("parser never fired")
except Got as g:
    a = g.ns
argparse.ArgumentParser.parse_args = _orig
a.park = None; a.verify_switch = True

rig = dfcal.Rig(a)
P = len(rig.ports); spr = rig.stage.steps_per_rev
print(f"[multi] {P} ports, {NANG} angles, {len(FREQS)} freqs "
      f"({', '.join(f'{f/1e6:.0f}' for f in FREQS)} MHz)", flush=True)

L = np.full((len(FREQS), NANG, P), np.nan)
t0 = time.time()
try:
    for i in range(NANG):
        rig.stage.goto_steps(int(round(i * spr / NANG)))
        for fi, f in enumerate(FREQS):
            lv, nl, info = rig.levels(f)
            if lv is not None:
                L[fi, i, :] = lv
        if i % 6 == 0:
            print(f"  angle {i:2d}/{NANG}  t={time.time()-t0:5.0f}s", flush=True)
    rig.stage.goto_steps(0)
finally:
    rig.close()

out = "/tmp/claude-1000/-home-uarf-sigmon/d119d42b-e671-464b-8377-533682d7aa7d/scratchpad/multifreq1800.npz"
np.savez(out, L=L, freqs=np.array(FREQS), ports=np.array(rig.ports))
print(f"[multi] done in {time.time()-t0:.0f}s -> {out}")

def h1(y):
    y = np.nan_to_num(np.asarray(y, float)); y = y - y.mean()
    return 2.0 * np.fft.rfft(y)[1] / len(y)

def coh(vs, P):
    v = np.array([vs[k] * np.exp(-1j * 2 * np.pi * k / P) for k in range(P)])
    return abs(v.sum()) / max(np.abs(v).sum(), 1e-12)

print("\n=== per-frequency ring coherence of h1 ===")
H = np.zeros((len(FREQS), P), complex)
for fi, f in enumerate(FREQS):
    Ln = dfcal.common_mode(L[fi])
    H[fi] = [h1(Ln[:, k]) for k in range(P)]
    print(f"  {f/1e6:6.1f} MHz   coherence {coh(H[fi], P):.3f}   "
          f"mean |h1| {np.abs(H[fi]).mean():.2f} dB")

print("\n=== coherence vs number of frequencies averaged (complex mean) ===")
for n in range(1, len(FREQS) + 1):
    avg = H[:n].mean(axis=0)
    print(f"  {n} freq(s): coherence {coh(avg, P):.3f}   mean |h1| {np.abs(avg).mean():.2f} dB")

# quality-weighted coherent average: a frequency whose ring is incoherent is
# contributing multipath, not geometry, so let it carry less weight.
w = np.array([max(coh(H[fi], P) - 1.0/P, 0.0) for fi in range(len(FREQS))])
w = w / max(w.sum(), 1e-9)
wavg = (H * w[:, None]).sum(axis=0)
print("\n=== quality-weighted average ===")
print(f"  weights {np.round(w,3).tolist()}")
print(f"  coherence CW {coh(wavg,P):.3f}  CCW {coh(wavg[::1]*0+wavg,P):.3f}")
for s_ in (+1,-1):
    ww=np.array([wavg[k]*np.exp(-1j*s_*2*np.pi*k/P) for k in range(P)])
    print(f"  sense {s_:+d}: coherence {abs(ww.sum())/np.abs(ww).sum():.3f}")
avg = wavg
ph = np.degrees(np.angle(avg)) % 360
rel = (ph - ph[0]) % 360
nom = np.arange(P) * 360.0 / P
print("\n=== averaged h1: implied mounting angles ===")
print("  port   |h1|    az rel p1   nominal    error")
for k, p in enumerate(rig.ports):
    e = ((rel[k] - nom[k] + 180) % 360) - 180
    print(f"  {p:4d}  {abs(avg[k]):5.2f}   {rel[k]:8.1f}  {nom[k]:8.1f}   {e:+6.1f}")
sp = np.diff(np.concatenate([np.sort(rel), [np.sort(rel)[0] + 360]]))
print(f"  ring order : {[int(rig.ports[i]) for i in np.argsort(rel)]}")
print(f"  spacings   : {np.round(sp,1).tolist()}  (ideal {360/P:.1f}, sd {sp.std():.1f})")
