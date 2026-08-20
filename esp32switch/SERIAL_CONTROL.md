# Controlling the switch from another program (serial)

The firmware exposes a line-oriented console on the board's serial port. Any
language that can open a tty can drive the RF switch through it — set a port,
set a dwell time, start and stop iterating, and read back the full state as
JSON.

Everything below was captured from a real board (ESP32-S3 devkit on
`/dev/ttyACM0`); the response strings are exact.

A ready-to-use Python client lives in [`tools/switchctl.py`](tools/switchctl.py).

---

## 1. Port setup

| Setting | Value |
|---|---|
| Device | `/dev/ttyACM0` (native USB) or `/dev/ttyUSB0` (USB-UART bridge) |
| Baud | `115200` — ignored on native USB, required on a UART bridge |
| Framing | 8N1, no flow control |
| Line ending to send | **`\n` only** |
| Prompt | `switch> ` |

### Opening the port reboots the board

On native-USB (USB-Serial-JTAG) boards, opening the port resets the chip — a
fresh connection reliably produces `ESP-ROM:esp32s3-20210327` before anything
else. Consequences for a controlling program:

- **Open the port once and keep it open** for the life of your program. State is
  continuous within a session; it is not continuous across connections.
- After opening, **wait for the console to be ready** (~1.3 s) before sending
  commands. Commands sent earlier are silently lost. Either wait for the
  `ready -- type 'help' for serial commands` line, or sleep ~2 s.
- Anything you persist survives the reboot anyway (port, dwell, sequence and the
  iterate flag are stored in NVS), but the `steps` counter restarts at 0.

Clearing DTR/RTS before `open()` and disabling `HUPCL` do not prevent this on
USB-Serial-JTAG; they are still worth doing on UART-bridge boards, where they
do prevent a reset. `tools/switchctl.py` does both.

## 2. Wire protocol

Send one command terminated by a single `\n`. The device replies with:

```
<echo of your command>\r\n
<zero or more response lines>\r\n
switch> 
```

Read until you see the `switch> ` prompt, drop the first line (the echo), and
what remains is the response. Send **`\n`, not `\r\n`** — `\r\n` is two line
terminators and makes the REPL print two prompts, which desynchronises your
reads.

### Silence the log lines first

By default the firmware's `ESP_LOG` output is interleaved with command
responses:

```
port 5
I (2706) switch: port 5 -> 101 (L1=1 L2=0 L3=1)
holding port 5 (101)
switch> 
```

Send `log off` as your first command and responses arrive clean:

```
port 5
holding port 5 (101)
switch> 
```

`log on` restores them. If you prefer to leave logging on, discard lines
matching `^[IWE] \(\d+\)`.

## 3. Error handling

Two failure shapes, both detectable from the response text:

| Condition | Device output |
|---|---|
| Bad arguments | the command's usage line, then `Command returned non-zero error code: 0x1 (ERROR)` |
| Unknown command | `Unrecognized command` |

```
port 9
usage: port <1..8 | 3-bit pattern>   e.g. "port 3" or "port 011"
Command returned non-zero error code: 0x1 (ERROR)
switch> 
```

```
dwell 10
dwell must be 1000..3600000000000 ns
Command returned non-zero error code: 0x1 (ERROR)
switch> 
```

Treat the presence of `Command returned non-zero error code` or
`Unrecognized command` as a failure, and use the line before it as the reason.

## 4. Command reference

Exact request/response pairs, with logging off.

| Send | Response | Notes |
|---|---|---|
| `port 3` | `holding port 3 (011)` | 1–8; stops iterating |
| `port 011` | `holding port 3 (011)` | pattern form of the same port |
| `dwell` | `dwell 1000000 ns (1.000 ms)` | query |
| `dwell 250000` | `dwell 250000 ns (250.000 us)` | nanoseconds, 1000 … 3600000000000 |
| `count 5` | `switching between ports 1..5` | switch between the first n ports |
| `seq 1-5` | `sequence: 1 2 3 4 5` | inclusive range; `5-1` descends |
| `seq 001,010,111` | `sequence: 1 2 7` | patterns work anywhere ports do |
| `iterate on` | `iterate on` | starts from the first sequence entry |
| `iterate off` | `iterate off` | stops, holds wherever it is |
| `step` | `port 2 (010)` | advance one place, then hold |
| `gpio` | `lines L1=GPIO1 L2=GPIO2 L3=GPIO3` | query the control-line pins |
| `gpio 1 2 3` | `lines L1=GPIO1 L2=GPIO2 L3=GPIO3` | move them; distinct, 0..31, persisted |
| `json` | one line of JSON | see below |
| `log off` / `log on` | `log off` / `log on` | suppress/restore log lines |
| `status` | 8 human-readable lines | for people; use `json` from code |
| `wifi <ssid> [pw]` | `stored; joining "<ssid>"` | credentials go to NVS |
| `help` | command list | |

Port ↔ pattern: on the switch fitted here port *N* carries the binary of
*N−1*, so port 1 is `000` and port 8 is `111`. **Don't hardcode that** — read
`codes` from `json` (see below). It is a table in the firmware precisely because
switches differ, and the previous entry in this document described the opposite
mapping, which cost an afternoon: three of four antennas answered on the wrong
ports, the fourth looked dead, and the deliberately-unconnected reference port
read as live. Prove the mapping instead of trusting it —
`dfstream.py --check-switch` holds every port and reports which respond.

The control-line pins are likewise a setting (`gpio`), not a rebuild: a wrong
pin map is invisible from the firmware side, which happily drives three pins
that go nowhere and reports success on every command.

## 5. Reading state: `json`

`json` prints the entire state on one line — the same object as the HTTP
`GET /api/state`:

```json
{"port":1,"pattern":"000","dwell_ns":250000,"iterate":true,"seq":[1,2,3,4,5],
 "seq_idx":0,"steps":1402,"codes":["000","001","010","011","100","101","110","111"],
 "dwell_min":1000,"dwell_max":3600000000000,"ports":8,"gpio":[1,2,3],
 "sta":{"connected":false,"ssid":"","ip":"0.0.0.0"},
 "ap":{"ssid":"ESP32-Switch","ip":"192.168.4.1"}}
```

| Field | Meaning |
|---|---|
| `port` | port currently selected, 1–8 |
| `pattern` | control lines for that port, e.g. `"011"`, line 1 leftmost |
| `dwell_ns` | dwell time in nanoseconds |
| `iterate` | `true` while walking the sequence |
| `seq` | iteration order, as port numbers |
| `seq_idx` | index into `seq` of the current step |
| `steps` | port changes since boot (see the caveat below) |
| `codes` | port → pattern map, `codes[0]` is port 1 — the authoritative truth table; read it, do not assume it |
| `dwell_min` / `dwell_max` | accepted dwell range, in ns |
| `ports` | number of ports (8) |
| `gpio` | the three control-line GPIO numbers (settable with the `gpio` command) |
| `sta` / `ap` | station and SoftAP status |

**`steps` update cadence.** With a dwell below 2 ms the iterate loop is a
busy-wait that publishes progress roughly every 50 ms, so `steps` advances in
jumps. At or above 2 ms every step is published immediately. Either way `steps`
is exact over intervals longer than ~100 ms — which makes it a good way to
verify the real switching rate (below) — but don't expect it to be current to
the last step.

## 6. Examples

### Python — using the supplied client

```python
import sys
sys.path.insert(0, 'tools')
from switchctl import SwitchSerial

with SwitchSerial('/dev/ttyACM0') as sw:      # opens, waits, sends "log off"
    sw.hold(3)                                 # hold port 3
    sw.cycle(5, dwell_ns=250_000)              # switch between ports 1-5 @ 250 us
    sw.cycle([1, 2, 3, 6, 7], dwell_ns=1_000)  # or an explicit subset @ 1 us
    print(sw.state()['pattern'])               # -> e.g. '011'
    rate, ns = sw.measure_rate(3.0)            # verify the real period
    print(f'{rate:,.0f} steps/s = {ns:,.0f} ns/step')
    sw.iterate(False)
    sw.hold(1)
```

Bad input raises `SwitchError`:

```python
from switchctl import SwitchSerial, SwitchError
with SwitchSerial() as sw:
    try:
        sw.hold(9)
    except SwitchError as err:
        print(err)   # 'port 9' rejected: usage: port <1..8 | 3-bit pattern> ...
```

### Python — minimal, no dependencies beyond pyserial

Everything you need is 20 lines:

```python
import json, time, serial

ser = serial.Serial()
ser.port, ser.baudrate, ser.timeout = '/dev/ttyACM0', 115200, 0.1
ser.dtr = ser.rts = False
ser.open()
time.sleep(2.0)                      # opening reboots the board; let it come up
ser.reset_input_buffer()

def cmd(line, wait=0.7):
    ser.reset_input_buffer()
    ser.write(line.encode() + b'\n')  # single \n
    out, end = b'', time.time() + wait
    while time.time() < end and not out.endswith(b'switch> '):
        out += ser.read(4096)
    lines = [l.strip() for l in out.decode(errors='replace').replace('\r', '\n').split('\n')]
    return [l for l in lines if l and not l.startswith('switch>') and l != line]

cmd('log off')
cmd('count 5')
cmd('dwell 250000')
cmd('iterate on')
state = json.loads(next(l for l in cmd('json') if l.startswith('{')))
print(state['port'], state['pattern'], state['dwell_ns'])
```

### Shell

```bash
stty -F /dev/ttyACM0 115200 raw -echo -hupcl
cat /dev/ttyACM0 &                 # keep a reader attached
sleep 2                            # opening reboots the board

printf 'log off\n'      > /dev/ttyACM0
printf 'port 3\n'       > /dev/ttyACM0
printf 'count 5\n'      > /dev/ttyACM0
printf 'dwell 250000\n' > /dev/ttyACM0
printf 'iterate on\n'   > /dev/ttyACM0
printf 'json\n'         > /dev/ttyACM0
```

Produces:

```
log off
switch> port 3
holding port 3 (011)
switch> count 5
switching between ports 1..5
switch> dwell 250000
dwell 250000 ns (250.000 us)
switch> iterate on
iterate on
switch> json
{"port":1,"pattern":"000","dwell_ns":250000,"iterate":true,...}
switch> 
```

### C

```c
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

static int sw_open(const char *dev)
{
    int fd = open(dev, O_RDWR | O_NOCTTY);
    if (fd < 0) return -1;

    struct termios tio;
    tcgetattr(fd, &tio);
    cfmakeraw(&tio);
    cfsetspeed(&tio, B115200);
    tio.c_cflag &= ~HUPCL;            /* don't drop the lines on close */
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 5;              /* 0.5 s read timeout */
    tcsetattr(fd, TCSANOW, &tio);
    sleep(2);                         /* opening reboots the board */
    tcflush(fd, TCIFLUSH);
    return fd;
}

/* Send one command, collect the reply up to the prompt. */
static int sw_cmd(int fd, const char *cmd, char *out, size_t len)
{
    char line[128];
    int n = snprintf(line, sizeof(line), "%s\n", cmd);   /* single \n */
    if (write(fd, line, n) != n) return -1;

    size_t used = 0;
    while (used + 1 < len) {
        ssize_t got = read(fd, out + used, len - used - 1);
        if (got <= 0) break;
        used += (size_t)got;
        out[used] = '\0';
        if (strstr(out, "switch> ")) break;
    }
    out[used] = '\0';
    return strstr(out, "error code") || strstr(out, "Unrecognized") ? 1 : 0;
}

int main(void)
{
    char buf[1024];
    int fd = sw_open("/dev/ttyACM0");
    if (fd < 0) { perror("open"); return 1; }

    sw_cmd(fd, "log off", buf, sizeof(buf));
    sw_cmd(fd, "count 5", buf, sizeof(buf));
    sw_cmd(fd, "dwell 250000", buf, sizeof(buf));
    sw_cmd(fd, "iterate on", buf, sizeof(buf));
    if (sw_cmd(fd, "json", buf, sizeof(buf)) == 0) printf("%s", buf);

    close(fd);
    return 0;
}
```

## 7. Recipes

**Hold one port and nothing else**

```
log off
port 5
```

**Switch between 5 ports at 250 µs each**

```
count 5
dwell 250000
iterate on
```

**Switch between a specific subset**

```
seq 1,2,3,6,7
dwell 1000
iterate on
```

**Stop and pin a port** — `port` implies stop, so one command is enough:

```
port 3
```

**Verify the real switching rate** — read `steps` twice around a known delay:

```
json          -> steps = 6062,  t0
(sleep 3 s)
json          -> steps = 18059, t1
rate = (18059 - 6062) / (t1 - t0) = 3999 steps/s = 250,079 ns/step
```

## 8. Timing you can rely on

Measured on an ESP32-S3 devkit at 240 MHz, iterating over 5 ports, dwell
verified against the device's own step counter:

| Asked | Measured per step | Error |
|---|---|---|
| 1,000 ns | 1,000 ns | +0.0% |
| 2,000 ns | 2,000 ns | +0.0% |
| 5,000 ns | 5,001 ns | +0.0% |
| 20,000 ns | 20,005 ns | +0.0% |
| 100,000 ns | 100,017 ns | +0.0% |
| 250,000 ns | 250,049 ns | +0.0% |

The dwell is a *period*, not a delay — the time spent writing the control lines
counts towards it — so the mean rate is accurate down to the 1 µs minimum.
Individual steps still jitter when WiFi is busy; see the timing section in
[README.md](README.md#dwell-timing--what-to-expect) for what bounds that.

Two things to keep in mind when driving this from a program:

- A command takes effect within one dwell period. Sending `iterate off` does not
  guarantee the lines stop changing before the response comes back, though it is
  guaranteed before the *next* command's response.
- Every setting except `log` is written to NVS, so hammering `dwell` or `port` in
  a tight loop writes flash. For fast switching, configure once and let the
  device iterate rather than driving each step over serial.
