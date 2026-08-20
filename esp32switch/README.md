# esp32switch — ESP32-C3 SP8T RF switch controller

Drives a 3-bit-controlled RF switch (SP8T) from an ESP32-C3. Three GPIOs carry
the switch's control lines; the app either **holds** one port or **iterates**
through a set of ports, holding each for a dwell time you give in nanoseconds.
Everything — port, dwell, and which ports to switch between — is settable from
the web UI or the serial console.

```
port 5              hold port 5, nothing else
count 5             switch between ports 1-5
seq 1 2 3 6 7       switch between exactly those ports
dwell 250000        250000 ns (250 us) on each one
iterate on          go
```

Builds for the **ESP32-C3** and the **ESP32-S3**. To drive it from another
program over the serial port, see **[SERIAL_CONTROL.md](SERIAL_CONTROL.md)** —
protocol, exact responses, and tested Python/C/shell clients.

## Wiring

| Switch control | ESP32-C3 | ESP32-S3 | Pattern bit |
|---|---|---|---|
| Line 1 | GPIO3 | GPIO4 | leftmost |
| Line 2 | GPIO4 | GPIO5 | middle |
| Line 3 | GPIO5 | GPIO6 | rightmost |

The defaults avoid each chip's strapping pins (C3: GPIO2/8/9; S3: GPIO0/3/45/46).
Override them with `SW_GPIO_L1/L2/L3` in `main/switch_ctl.h`; they must stay
within GPIO0–31, which the header asserts at compile time. If your switch's
control inputs are active low, set `SW_ACTIVE_LOW` to 1 in the same header.

Common ground between the C3 and the switch, and the switch's Vctrl must be
3.3 V-tolerant. If it needs 5 V control, put a level shifter in between.

## Port ↔ pattern map

Port *N* carries the binary of *N*, and port 8 carries `000`:

| Port | L1 L2 L3 | | Port | L1 L2 L3 |
|---|---|---|---|---|
| 1 | `001` | | 5 | `101` |
| 2 | `010` | | 6 | `110` |
| 3 | `011` | | 7 | `111` |
| 4 | `100` | | 8 | `000` |

Everything that takes a port accepts either form: `3` and `011` mean the same
port. If your switch's truth table differs, edit the `s_port_code[]` table at
the top of `main/switch_ctl.c` — it's the single source of truth for the mapping,
and the web UI reads the patterns back from the device.

## Build and flash

```bash
. ~/esp/esp-idf/export.sh
idf.py set-target esp32c3      # or esp32s3; only needed when switching chips
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

Shared settings live in `sdkconfig.defaults`; per-chip settings are in
`sdkconfig.defaults.esp32c3` and `sdkconfig.defaults.esp32s3`, which ESP-IDF
loads automatically for the selected target. Changing target regenerates
`sdkconfig`, so `set-target` is the only step needed to move between boards.

Both default to a devkit with the console on native USB (`/dev/ttyACM0`). For a
board with a USB-UART bridge instead, run `idf.py menuconfig` → *Component
config* → *ESP System Settings* → *Channel for console output* → *UART0*, and
flash on `/dev/ttyUSB0`.

## First run / WiFi

The SoftAP is always up, so the UI is reachable even with no network configured:

- SSID `ESP32-Switch`, password `switch1234`, UI at <http://192.168.4.1>

To join your own network, use the serial console:

```
switch> wifi MySSID mypassword
switch> status
```

Credentials are stored in NVS and the station reconnects on boot. The AP stays
up alongside it. Change the AP name/password with `WIFI_AP_SSID` and
`WIFI_AP_PASS` in `main/wifi_mgr.h`.

## Serial console

```
switch> help                    list all commands
switch> status                  port, pattern, dwell, sequence, network
switch> port 3                  hold port 3
switch> port 011                same thing, by pattern
switch> dwell 500000            dwell = 500000 ns (500 µs)
switch> dwell                   print the current dwell
switch> count 5                 switch between the first 5 ports (1-5)
switch> seq 1 2 3 6 7           exact iteration order
switch> seq 1-5                 same as "count 5"; ranges may descend ("5-1")
switch> seq 001,010,011         patterns work anywhere a port does
switch> iterate on              start walking the sequence
switch> iterate off             stop, hold wherever it is
switch> step                    advance one place by hand
switch> json                    entire state on one line, for programs
switch> log off                 silence log lines while scripting
switch> wifi MySSID mypassword  store credentials and join
```

For driving this port from a program, see
[SERIAL_CONTROL.md](SERIAL_CONTROL.md) and `tools/switchctl.py`:

```bash
./tools/switchctl.py cycle 5 --dwell 250000
./tools/switchctl.py rate
./tools/switchctl.py hold 3
```

## HTTP API

`GET /api/state` returns the full state as JSON. `GET` or `POST /api/set` applies
any combination of parameters and returns the new state:

| Parameter | Meaning |
|---|---|
| `port=<1..8 or bits>` | hold one port; stops iteration |
| `dwell_ns=<ns>` | dwell time in nanoseconds, 1000 … 3600000000000 |
| `count=<1..8>` | switch between the first n ports |
| `seq=<list>` | iteration order: ports, ranges (`1-5`) or patterns, up to 16 entries |
| `iterate=<0\|1>` | stop / start iterating |
| `step=1` | advance one place in the sequence |

```bash
curl "http://192.168.4.1/api/set?port=3"                              # hold port 3
curl "http://192.168.4.1/api/set?count=5&dwell_ns=250000&iterate=1"   # cycle ports 1-5
curl "http://192.168.4.1/api/set?seq=1,2,3,6,7&iterate=1"             # cycle a subset
curl "http://192.168.4.1/api/state"
```

Parameters are applied in the order listed above, so a single request can
reconfigure and start in one go. `port=` always stops iteration — that is how
you pin a single port.

Port, dwell, iterate flag and sequence are persisted in NVS and restored on
boot.

## Dwell timing — what to expect

The dwell is a **period**, not a delay: the loop computes each step's deadline
from the previous one, so the time spent writing the control lines counts
towards the dwell instead of being added on top of it. Two timing sources:

- **≥ 2 ms** — `esp_timer` drives the steps; no CPU spinning.
- **< 2 ms** — a dedicated task waits on the CPU cycle counter, and writes all
  three lines with two register stores rather than three `gpio_set_level()`
  calls. On a dual-core part (S3) that task is pinned to the second core so it
  never competes with WiFi.

Measured on an ESP32-S3 devkit at 240 MHz, iterating over 5 ports, checked
against the firmware's own step counter:

| Asked | Measured per step | Error |
|---|---|---|
| 1,000 ns (the minimum) | 1,000 ns | +0.0% |
| 5,000 ns | 5,001 ns | +0.0% |
| 20,000 ns | 20,005 ns | +0.0% |
| 100,000 ns | 100,017 ns | +0.0% |
| 250,000 ns | 250,049 ns | +0.0% |

So the *mean* rate is accurate across the whole range. Individual steps are not
equally tight:

- On a single-core part (C3) the iterate task runs at priority 10, below WiFi
  (23) and LWIP (18), so radio activity preempts it and adds jitter to
  individual dwells. That is deliberate — a busy-wait above those priorities
  starves the network stack and takes the web UI down with it.
- If you need every edge to land on time rather than the average to be right,
  drive the lines from the RMT or MCPWM peripheral instead of the CPU.

Two more things worth knowing:

- A sub-millisecond dwell keeps a task spinning, so the idle task is starved.
  `CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0=n` in `sdkconfig.defaults` is what
  keeps the task watchdog from firing. Verified stable over 14.7M switch
  operations with no watchdog reset.
- Transitions clear before they set, so the switch never momentarily decodes two
  paths; the pattern is still briefly `000` mid-change. Your switch also needs
  its own settling time (hundreds of ns to a few µs — check the datasheet), so
  don't transmit through it while it is stepping.

## Tests

`switch_ctl.c` compiles against stub ESP-IDF headers, so the truth table, the
port/pattern/sequence parsing and the hold/iterate state machine can be
exercised on the host with no board attached:

```bash
cd test/host && make run
```

47 assertions covering the truth table, port/pattern parsing, ranges, malformed
input, the `count` shorthand, dwell bounds and the hold/iterate/step
transitions.

## Layout

```
CMakeLists.txt          project file
sdkconfig.defaults      target, console, watchdog and clock settings
sdkconfig.defaults.esp32c3   per-chip settings, loaded automatically
sdkconfig.defaults.esp32s3
main/
  main.c                app_main: NVS, switch, wifi, web, console
  switch_ctl.c/.h       port truth table, dwell engine, NVS persistence
  wifi_mgr.c/.h         AP+STA, credentials in NVS, auto-reconnect
  web.c/.h              HTTP server, /api/state and /api/set
  state_json.c/.h       one JSON encoder, shared by HTTP and console
  console_cmds.c/.h     serial REPL commands
  index.html            web UI, embedded into the binary
tools/switchctl.py      serial client: library + CLI
test/host/              host-side logic tests (stub headers + Makefile)
SERIAL_CONTROL.md       controlling the switch from another program
```
