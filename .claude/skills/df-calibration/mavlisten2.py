#!/usr/bin/env python3
"""Passive MAVLink listener with CRC validation. READ ONLY -- sends nothing.

Framing on 0xFD/0xFE alone false-syncs on payload bytes. Every frame here is
checked with the MAVLink CRC (X25/MCRF4XX + per-message CRC_EXTRA), so only
genuine frames are decoded.
"""
import os, struct, sys, time, glob
from collections import Counter

PORT = sys.argv[1] if len(sys.argv) > 1 else sorted(
    glob.glob("/dev/serial/by-id/*ArduPilot*if00"))[0]
DUR = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0

CRC_EXTRA = {0:50, 1:124, 2:137, 22:220, 24:24, 26:170, 27:144, 29:115, 30:39,
             32:185, 33:104, 34:237, 35:244, 36:222, 42:28, 62:183, 65:118,
             74:20, 77:143, 111:103, 116:76, 125:203, 129:19, 136:1, 137:143,
             147:154, 148:178, 152:208, 163:127, 164:154, 165:21, 168:1,
             173:83, 178:47, 182:229, 193:71, 241:90, 242:104, 253:83}
AP_TYPE = {0:"Generic", 3:"ArduPilotMega (ArduPilot)", 12:"PX4"}
MAV_TYPE = {0:"Generic", 1:"Fixed wing", 2:"Quadrotor", 10:"Ground rover",
            13:"Hexarotor", 18:"Onboard controller"}
NAMES = {0:"HEARTBEAT",1:"SYS_STATUS",24:"GPS_RAW_INT",27:"RAW_IMU",30:"ATTITUDE",
         33:"GLOBAL_POSITION_INT",42:"MISSION_CURRENT",74:"VFR_HUD",
         125:"POWER_STATUS",147:"BATTERY_STATUS",148:"AUTOPILOT_VERSION",
         152:"MEMINFO",163:"AHRS",165:"HWSTATUS",168:"WIND",178:"AHRS2",
         193:"EKF_STATUS_REPORT",241:"VIBRATION",253:"STATUSTEXT"}

def crc16(data, extra):
    crc = 0xFFFF
    for b in tuple(data) + (extra,):
        t = (b ^ (crc & 0xFF)) & 0xFF
        t = (t ^ (t << 4)) & 0xFF
        crc = ((crc >> 8) ^ (t << 8) ^ (t << 3) ^ (t >> 4)) & 0xFFFF
    return crc

fd = os.open(PORT, os.O_RDONLY | os.O_NONBLOCK)
print(f"[mav] {os.path.basename(PORT)}, {DUR:.0f}s, passive (nothing transmitted)")
buf = b""; good = Counter(); bad = 0; hb = None; banner = []; head = {}
t0 = time.time()
while time.time() - t0 < DUR:
    try: c = os.read(fd, 4096)
    except BlockingIOError: c = b""
    if not c: time.sleep(0.01); continue
    buf += c
    i = 0
    while i < len(buf):
        b0 = buf[i]
        if b0 not in (0xFD, 0xFE): i += 1; continue
        if b0 == 0xFD:
            if len(buf) - i < 12: break
            plen = buf[i+1]; signed = 13 if (buf[i+2] & 0x01) else 0
            total = 12 + plen + signed
            if len(buf) - i < total: break
            msgid = buf[i+7] | (buf[i+8] << 8) | (buf[i+9] << 16)
            pay = buf[i+10:i+10+plen]
            ck = struct.unpack_from("<H", buf, i+10+plen)[0]
            body = buf[i+1:i+10+plen]
        else:
            if len(buf) - i < 8: break
            plen = buf[i+1]; total = 8 + plen
            if len(buf) - i < total: break
            msgid = buf[i+5]; pay = buf[i+6:i+6+plen]
            ck = struct.unpack_from("<H", buf, i+6+plen)[0]
            body = buf[i+1:i+6+plen]
        ex = CRC_EXTRA.get(msgid)
        if ex is None or crc16(body, ex) != ck:
            bad += 1; i += 1; continue
        good[msgid] += 1
        p = bytes(pay) + b"\x00" * 80
        if msgid == 0:
            cm, t, ap, bm, ss, mv = struct.unpack_from("<IBBBBB", p, 0)
            hb = dict(type=t, autopilot=ap, base_mode=bm, custom_mode=cm,
                      system_status=ss, mavlink_version=mv)
        elif msgid == 253:
            txt = p[1:51].split(b"\x00")[0].decode("ascii", "replace")
            if txt and txt not in banner: banner.append(txt)
        elif msgid == 74:
            head["VFR_HUD.heading"] = struct.unpack_from("<h", p, 16)[0]
        elif msgid == 30:
            yaw = struct.unpack_from("<f", p, 12)[0]
            head["ATTITUDE.yaw"] = round((yaw*57.29577951+360) % 360, 1)
        elif msgid == 33:
            head["GLOBAL_POSITION_INT.hdg"] = struct.unpack_from("<H", p, 26)[0]/100.0
        elif msgid == 163:
            head["AHRS.error_yaw"] = round(struct.unpack_from("<f", p, 16)[0], 4)
        i += total
    buf = buf[i:]
os.close(fd)

print(f"[mav] {sum(good.values())} CRC-VALID frames, {bad} rejected")
if hb:
    print("\n=== HEARTBEAT (CRC-valid) ===")
    print(f"  autopilot      {hb['autopilot']} = {AP_TYPE.get(hb['autopilot'],'?')}")
    print(f"  vehicle type   {hb['type']} = {MAV_TYPE.get(hb['type'],'?')}")
    print(f"  mavlink ver    {hb['mavlink_version']}   custom_mode {hb['custom_mode']}"
          f"   sys_status {hb['system_status']}")
    print(f"  base_mode      0x{hb['base_mode']:02x}"
          f" ({'ARMED' if hb['base_mode'] & 0x80 else 'disarmed'})")
else:
    print("\n  no CRC-valid HEARTBEAT")
if banner:
    print("\n=== STATUSTEXT ==="); [print("  "+b) for b in banner[:12]]
print("\n=== heading ===")
print("  " + (", ".join(f"{k}={v}" for k,v in sorted(head.items())) or "none"))
print("\n=== valid message mix ===")
for m,c in good.most_common(14):
    print(f"  {m:5d} {NAMES.get(m,'?'):22} x{c}")
