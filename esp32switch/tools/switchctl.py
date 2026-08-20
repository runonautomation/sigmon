#!/usr/bin/env python3
"""Drive the ESP32 RF switch over its serial console.

Usable as a library or a CLI:

    from switchctl import SwitchSerial
    with SwitchSerial('/dev/ttyACM0') as sw:
        sw.hold(3)
        sw.cycle(5, dwell_ns=250_000)
        print(sw.state()['pattern'])

    ./switchctl.py hold 3
    ./switchctl.py cycle 5 --dwell 250000
    ./switchctl.py state
    ./switchctl.py rate --seconds 3

Requires pyserial (`pip install pyserial`).
"""

import argparse
import json
import sys
import termios
import time

import serial

PROMPT = b'switch> '
READY_BANNER = 'ready -- type'


class SwitchError(RuntimeError):
    """A command was rejected by the device."""


class SwitchSerial:
    """Line-oriented client for the switch console.

    The device echoes each command, prints its response, then a `switch> `
    prompt. Commands are terminated with a single '\\n' -- sending '\\r\\n'
    makes the REPL emit two prompts, which desynchronises reads.
    """

    def __init__(self, port='/dev/ttyACM0', baud=115200, timeout=3.0,
                 reset=False, quiet=True):
        self.timeout = timeout
        # Opening a port normally asserts DTR/RTS, and those lines drive EN/BOOT
        # on the devkits -- that reboots the chip and loses the step counter.
        # Setting them False *before* open() is what avoids the reset.
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = baud
        self.ser.timeout = 0.1
        self.ser.dtr = False
        self.ser.rts = False
        self.ser.open()
        self._disable_hupcl()
        if reset:
            self.reset()
        else:
            time.sleep(0.2)
        self.ser.reset_input_buffer()
        self.sync()
        if quiet:
            # Silence ESP_LOG output so responses arrive unmixed.
            self.command('log off')

    # ---------------------------------------------------------------- plumbing

    def _disable_hupcl(self):
        """Stop the tty layer dropping the modem lines when the port closes.

        With HUPCL set (the default), closing the port pulls DTR/RTS low, which
        on these devkits resets the chip -- so the *next* program to connect
        finds a freshly rebooted device with its step counter back at zero.
        """
        try:
            attrs = termios.tcgetattr(self.ser.fileno())
            attrs[2] &= ~termios.HUPCL          # cflag
            termios.tcsetattr(self.ser.fileno(), termios.TCSANOW, attrs)
        except (termios.error, OSError):
            pass                                 # not a tty we can tune; harmless

    def close(self):
        self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def reset(self):
        """Reboot the device (RTS -> EN) and wait for the console to come up."""
        self.ser.setRTS(True)
        time.sleep(0.15)
        self.ser.setRTS(False)
        deadline = time.time() + 10.0
        buf = ''
        while time.time() < deadline:
            buf += self.ser.read(4096).decode('utf-8', 'replace')
            if READY_BANNER in buf:
                time.sleep(0.2)
                return
        raise SwitchError('device did not report ready after reset')

    def sync(self):
        """Get to a known state: bare newline, swallow whatever comes back."""
        self.ser.write(b'\n')
        self.ser.flush()
        self._read_to_prompt(timeout=1.5, required=False)

    def _read_to_prompt(self, timeout=None, required=True):
        timeout = self.timeout if timeout is None else timeout
        deadline = time.time() + timeout
        buf = b''
        while time.time() < deadline:
            buf += self.ser.read(4096)
            if buf.endswith(PROMPT) or PROMPT in buf:
                break
        if required and PROMPT not in buf:
            raise SwitchError(f'timed out waiting for prompt; got {buf!r}')
        return buf.decode('utf-8', 'replace')

    def command(self, line):
        """Send one command; return its response lines.

        Raises SwitchError if the device rejected it.
        """
        self.ser.reset_input_buffer()
        self.ser.write(line.encode() + b'\n')
        self.ser.flush()
        raw = self._read_to_prompt()

        out = []
        echo_seen = False
        for text in raw.replace('\r', '\n').split('\n'):
            text = text.strip()
            if not text or text.startswith('switch>'):
                continue          # blank line or prompt
            if not echo_seen and text == line.strip():
                echo_seen = True  # the echo, not the response
                continue
            if text.startswith('I (') or text.startswith('W (') or text.startswith('E ('):
                continue          # a stray log line, if logging is on
            out.append(text)

        for text in out:
            if 'Unrecognized command' in text:
                raise SwitchError(f'unrecognized command: {line!r}')
            if 'Command returned non-zero error code' in text:
                detail = out[0] if out else ''
                raise SwitchError(f'{line!r} rejected: {detail}')
        return out

    # ---------------------------------------------------------------- commands

    def state(self):
        """Full device state as a dict (same shape as GET /api/state)."""
        for text in self.command('json'):
            if text.startswith('{'):
                return json.loads(text)
        raise SwitchError('no JSON in response to "json"')

    def hold(self, port):
        """Select one port (1-8, or a '011' pattern) and stop iterating."""
        return self.command(f'port {port}')

    def dwell(self, ns):
        """Set the per-step dwell time in nanoseconds."""
        return self.command(f'dwell {int(ns)}')

    def sequence(self, ports):
        """Set the iteration order, e.g. [1,2,3,6,7] or '1-5'."""
        spec = ports if isinstance(ports, str) else ','.join(str(p) for p in ports)
        return self.command(f'seq {spec}')

    def count(self, n):
        """Iterate over the first n ports."""
        return self.command(f'count {int(n)}')

    def iterate(self, on=True):
        return self.command(f'iterate {"on" if on else "off"}')

    def step(self):
        """Advance one place in the sequence and hold."""
        return self.command('step')

    def quiet(self, on=True):
        return self.command(f'log {"off" if on else "on"}')

    def wifi(self, ssid, password=''):
        return self.command(f'wifi {ssid} {password}'.strip())

    def cycle(self, ports, dwell_ns=None):
        """Configure and start iterating in one call.

        `ports` may be an int (first n ports), a list, or a range string.
        """
        if dwell_ns is not None:
            self.dwell(dwell_ns)
        if isinstance(ports, int):
            self.count(ports)
        else:
            self.sequence(ports)
        return self.iterate(True)

    def measure_rate(self, seconds=3.0):
        """Measured step period, from the device's own step counter.

        Returns (steps_per_second, nanoseconds_per_step).
        """
        s0 = self.state()['steps']
        t0 = time.perf_counter()
        time.sleep(seconds)
        s1 = self.state()['steps']
        t1 = time.perf_counter()
        if s1 == s0:
            return 0.0, float('inf')
        rate = (s1 - s0) / (t1 - t0)
        return rate, 1e9 / rate


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--port', default='/dev/ttyACM0')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--reset', action='store_true', help='reboot the device first')
    sub = ap.add_subparsers(dest='cmd', required=True)

    p = sub.add_parser('hold', help='hold one port')
    p.add_argument('target', metavar='PORT', help='1-8 or a 3-bit pattern like 011')

    p = sub.add_parser('cycle', help='iterate over ports')
    p.add_argument('ports', help='count (5), list (1,2,3,6,7) or range (1-5)')
    p.add_argument('--dwell', type=int, help='nanoseconds per step')

    sub.add_parser('stop', help='stop iterating')
    sub.add_parser('step', help='advance one place')
    sub.add_parser('state', help='print state as JSON')

    p = sub.add_parser('dwell', help='set the dwell time')
    p.add_argument('ns', type=int)

    p = sub.add_parser('rate', help='measure the real step period')
    p.add_argument('--seconds', type=float, default=3.0)

    p = sub.add_parser('raw', help='send a literal command')
    p.add_argument('line', nargs='+')

    args = ap.parse_args(argv)

    try:
        with SwitchSerial(args.port, args.baud, reset=args.reset) as sw:
            if args.cmd == 'hold':
                print('\n'.join(sw.hold(args.target)))
            elif args.cmd == 'cycle':
                ports = int(args.ports) if args.ports.isdigit() else args.ports
                print('\n'.join(sw.cycle(ports, args.dwell)))
            elif args.cmd == 'stop':
                print('\n'.join(sw.iterate(False)))
            elif args.cmd == 'step':
                print('\n'.join(sw.step()))
            elif args.cmd == 'dwell':
                print('\n'.join(sw.dwell(args.ns)))
            elif args.cmd == 'state':
                print(json.dumps(sw.state(), indent=2))
            elif args.cmd == 'rate':
                rate, per = sw.measure_rate(args.seconds)
                asked = sw.state()['dwell_ns']
                err = 100 * (per - asked) / asked if asked else 0.0
                print(f'{rate:,.0f} steps/s = {per:,.0f} ns/step '
                      f'(asked {asked:,} ns, {err:+.1f}%)')
            elif args.cmd == 'raw':
                print('\n'.join(sw.command(' '.join(args.line))))
    except SwitchError as err:
        print(f'error: {err}', file=sys.stderr)
        return 1
    except serial.SerialException as err:
        print(f'serial error: {err}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
