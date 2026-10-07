"""Bounded, single-motor IM6014 bench test. Output-shaft units throughout.

No automatic fault clearing, reset, reconnect, or motion restart.
"""
import argparse
import json
import math
from pathlib import Path
import time

import serial
from im6014_codec import POSITION_MAX, POSITION_MIN, RATIO, RPM_MAX, RPM_MIN, command, crc32, feedback

from im6014_serial import exchange

PORT = '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTC2RQ7H-if03-port0'

def healthy(f, mode, allow_timeout=False, speed_limit=0):
    if f['mode'] != mode or f['errors'] or f['warnings'] or f['timeout'] and not allow_timeout:
        raise RuntimeError('Motor mode/fault/warning/timeout gate: ' + json.dumps(f))
    if not 24 <= f['voltage'] <= 72 or not -10 <= f['driver_c'] <= 70 or f['winding_c'] > 90:
        raise RuntimeError('Voltage/temperature outside bench limits: ' + json.dumps(f))
    if speed_limit and abs(f['rpm']) > speed_limit:
        raise RuntimeError('Measured speed exceeds configured limit: ' + json.dumps(f))
    if abs(f['torque_nm']) > 4:
        raise RuntimeError('Torque exceeds bench limit: ' + json.dumps(f))


def run(bus, args, log):
    samples = log['samples']

    def transact(mode=0, rpm=0, kd=0, torque=0, watchdog=True, allow_timeout=False, entering_foc=False):
        f = exchange(bus, command(args.id, mode, rpm, kd, torque, watchdog), args.id)
        f.update(elapsed_s=time.monotonic() - started, target_rpm=rpm)
        samples.append(f)
        healthy(f, f['mode'] if entering_foc else mode, allow_timeout, args.speed_limit)
        return f

    started = time.monotonic()
    first = transact(allow_timeout=args.ack_timeout)
    if first['timeout']:
        # Explicit startup-only acknowledgment while stopped, never during motion/recovery.
        transact(watchdog=False, allow_timeout=True)
    initial = transact()
    print('STOP/status verified:', json.dumps(initial), flush=True)
    if not args.arm:
        return
    # Firmware reports the previous state on its first mode-transition reply.
    # Permit this only for the zero-torque arming exchange, then require FOC.
    transact(1, entering_foc=True)
    initial = transact(1)
    start_position = initial['position_rad']
    start = time.monotonic()
    last_reply = start
    next_print = 0
    while True:
        now = time.monotonic()
        elapsed = now - start
        if elapsed >= args.duration:
            break
        if now - last_reply > 0.1:
            raise RuntimeError('Host feedback watchdog expired; motion remains disarmed')
        # 0.75-second acceleration/deceleration ramps, with zero position stiffness.
        ramp = min(1, elapsed / 0.75, (args.duration - elapsed) / 0.75)
        torque_ff = args.torque_ff * ramp * (1 if args.rpm > 0 else -1) if args.rpm else 0
        f = transact(1, args.rpm * ramp, args.kd, torque_ff)
        last_reply = time.monotonic()
        if args.travel_limit and abs(f['position_rad'] - start_position) > math.radians(args.travel_limit):
            raise RuntimeError('Exceeded configured output travel limit')
        if elapsed >= next_print:
            print(f"t={elapsed:.2f}s target={args.rpm * ramp:.2f} rpm measured={f['rpm']:.3f} rpm torque={f['torque_nm']:.3f} Nm", flush=True)
            next_print += 0.5
        time.sleep(max(0, 0.01 - (time.monotonic() - now)))
    plateau = [f['rpm'] for f in samples if f.get('target_rpm') == args.rpm and f['mode'] == 1]
    mean_rpm = sum(plateau) / len(plateau) if plateau else 0
    log['mean_rpm'] = mean_rpm
    if abs(mean_rpm - args.rpm) > 0.5:
        raise RuntimeError(f'Measured speed {mean_rpm:.3f} rpm did not reach the requested speed')
    print(f'Measured plateau average: {mean_rpm:.3f} rpm', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default=PORT)
    parser.add_argument('--id', type=int, default=0)
    parser.add_argument('--baud', type=int, choices=(4000000, 6000000), default=6000000)
    parser.add_argument('--rpm', type=float, default=5)
    parser.add_argument('--kd', type=float, default=6)
    # Measured unloaded-bench friction compensation; retune for a different motor/load.
    parser.add_argument('--torque-ff', type=float, default=0.16)
    parser.add_argument('--duration', type=float, default=5)
    parser.add_argument('--speed-limit', type=float, default=0, help='Optional measured-speed trip in rpm; 0 disables it')
    parser.add_argument('--travel-limit', type=float, default=0, help='Optional travel trip in degrees from test start; 0 disables it')
    parser.add_argument('--ack-timeout', action='store_true', help='Explicitly acknowledge startup timeout only in Stop, then enable watchdog')
    parser.add_argument('--arm', action='store_true', help='Allow this bounded unloaded-bench motion test')
    parser.add_argument('--log', default='motor_run.json')
    args = parser.parse_args()
    if args.torque_ff < 0:
        parser.error('Friction compensation must be nonnegative; direction follows --rpm')
    try:
        command(args.id, 1, args.rpm, args.kd, args.torque_ff)
    except ValueError as error:
        parser.error(str(error))
    if not math.isfinite(args.duration) or not 2 <= args.duration <= 10:
        parser.error('Duration must be 2–10 seconds')
    if not all(math.isfinite(x) and x >= 0 for x in (args.speed_limit, args.travel_limit)):
        parser.error('Optional speed/travel limits must be finite and nonnegative')
    if args.speed_limit and abs(args.rpm) > args.speed_limit:
        parser.error('Requested speed exceeds the configured measured-speed limit')
    # Confirm the result location is writable before commanding hardware.
    path = Path(args.log)
    log = dict(settings=vars(args), samples=[], stopped=False, error=None)
    path.write_text(json.dumps(log, indent=2) + '\n')
    bus = serial.Serial(port=None, baudrate=args.baud, timeout=0.005, write_timeout=0.05, exclusive=True)
    bus.port = args.port
    bus.dtr = False
    bus.rts = False
    try:
        bus.open()
        run(bus, args, log)
    except (Exception, KeyboardInterrupt) as error:
        log['error'] = str(error) or type(error).__name__
        print('STOPPING:', log['error'], flush=True)
    finally:
        if bus.is_open:
            try:
                for _ in range(3):
                    f = exchange(bus, command(args.id), args.id)
                    log['samples'].append(dict(f, final_stop=True))
                    log['stopped'] = f['mode'] == 0
                    time.sleep(0.02)
                print('Final Stop feedback:', json.dumps(f), flush=True)
            except Exception as error:
                log['stopped'] = False
                log['stop_error'] = str(error)
                print('Stop confirmation unavailable:', error, flush=True)
            bus.close()
        path.write_text(json.dumps(log, indent=2) + '\n')
    if log['error'] or not log['stopped']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
