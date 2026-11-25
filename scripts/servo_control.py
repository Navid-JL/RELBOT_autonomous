#!/usr/bin/env python3
"""
Manual Servo Controller for RELBot ESP32 Manipulator Bridge (/dev/ttyUSB0).
  - Pin 26: SRT DL3020 (1-DOF Arm, -65..+65 deg or 750..2250 us)
  - Pin 25: MG996R (Continuous Gripper, 0..180 deg or 1200..1800 us, 250 ms burst auto-cutoff)

Usage on Raspberry Pi:
  Interactive mode:
    python3 ~/relbot_autonomous/servo_control.py

  One-shot CLI flags:
    python3 ~/relbot_autonomous/servo_control.py --arm 30
    python3 ~/relbot_autonomous/servo_control.py --grip 135
    python3 ~/relbot_autonomous/servo_control.py --arm -20 --grip 45
    python3 ~/relbot_autonomous/servo_control.py --stop
"""

import argparse
import os
import sys
import termios
import time
import serial


def open_serial(port: str, baud: int) -> serial.Serial:
    # Clear HUPCL first so closing /dev/ttyUSB0 never drops DTR/RTS and resets the ESP32.
    # Also keep pyserial's default dtr=True, rts=True on open() because setting dtr=False,
    # rts=False causes pyserial to clear DTR before RTS in two separate USB control packets,
    # briefly creating the (DTR=0, RTS=1) state that pulses EN=LOW and resets the ESP32.
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[2] &= ~termios.HUPCL
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    os.close(fd)

    ser = serial.Serial(port=port, baudrate=baud, timeout=0.15)
    time.sleep(0.05)
    if ser.in_waiting:
        pending = ser.read(ser.in_waiting).decode("utf-8", errors="replace")
        # If the ESP32 just reset, wait for its 300 ms ROM + Arduino boot to finish
        if "rst:0x" in pending or "ets " in pending:
            time.sleep(0.45)
            if ser.in_waiting:
                ser.read(ser.in_waiting)
    return ser


def reset_to_normal_boot(ser: serial.Serial) -> None:
    """Pulse EN=LOW with IO0=HIGH (DTR=False, RTS=True -> DTR=False, RTS=False) to boot application."""
    ser.dtr = False
    ser.rts = True
    time.sleep(0.10)
    ser.rts = False
    time.sleep(0.45)
    while ser.in_waiting:
        ser.read(ser.in_waiting)


def send_and_print(ser: serial.Serial, cmd: str, wait_sec: float = 0.35, allow_retry: bool = True) -> None:
    ser.write((cmd.strip() + "\n").encode("utf-8"))
    ser.flush()
    deadline = time.time() + wait_sec
    got_response = False
    saw_download_boot = False
    while time.time() < deadline:
        line = ser.readline()
        if line:
            text = line.decode("utf-8", errors="replace").rstrip()
            if "DOWNLOAD_BOOT" in text or "waiting for download" in text:
                saw_download_boot = True
            elif not text.startswith("$STAT,"):
                print(text)
                got_response = True

    # Auto-recover if ESP32 was left in ROM DOWNLOAD_BOOT after flashing
    if allow_retry and (saw_download_boot or not got_response):
        reset_to_normal_boot(ser)
        send_and_print(ser, cmd, wait_sec=wait_sec, allow_retry=False)


def interactive_loop(ser: serial.Serial) -> None:
    print("=====================================================================")
    print(" RELBot ESP32 Dual-Servo Manual Control (/dev/ttyUSB0)")
    print("   a <deg>        : Set Arm angle (-65 to +65 deg, 0 = center) or us")
    print("   g <deg>        : Set Gripper angle (0 to 250 deg, 125 = mid) or 500..2500 us")
    print("   <arm> <grip>   : Set BOTH at once (e.g. '25 245' or '0 125')")
    print("   a+ / a-        : Nudge Arm +/- 10 us")
    print("   g+ / g-        : Nudge Gripper +/- 10 us")
    print("   hold / burst   : Toggle Gripper continuous HOLD (default) vs timed BURST")
    print("   t <ms>         : Set Gripper burst duration (default 900 ms)")
    print("   x              : Relax/stop Gripper PWM ('x all' relaxes Arm too)")
    print("   s / help / q   : Status / Help / Quit")
    print("=====================================================================")
    send_and_print(ser, "s", wait_sec=0.25)

    try:
        while True:
            # Drain any async messages (such as auto-cutoff notifications)
            while ser.in_waiting > 0:
                line = ser.readline().decode("utf-8", errors="replace").rstrip()
                if line and not line.startswith("$STAT,"):
                    print(line)

            cmd = input("servo> ").strip()
            if not cmd:
                continue
            if cmd.lower() in ("q", "quit", "exit"):
                break
            # Wait slightly longer if triggering a gripper burst so auto-cutoff prints cleanly
            wait_s = 0.25
            send_and_print(ser, cmd, wait_sec=wait_s)
    except (KeyboardInterrupt, EOFError):
        print("\nExiting (servos holding state; use 'x' or '--stop' to relax)...")


def main() -> None:
    parser = argparse.ArgumentParser(description="RELBot ESP32 Servo Controller (DL3020 Arm + MG996R Gripper)")
    parser.add_argument("-p", "--port", default="/dev/ttyUSB0", help="Serial port (default: /dev/ttyUSB0)")
    parser.add_argument("-b", "--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("-a", "--arm", type=float, default=None, help="Set SRT DL3020 Arm angle (-65..+65 deg) or pulse (750..2250 us)")
    parser.add_argument("-g", "--grip", type=float, default=None, help="Set MG996R Gripper angle (0..250 deg) or pulse (500..2500 us)")
    parser.add_argument("-t", "--duration", type=int, default=None, help="Set Gripper burst duration in ms (50..3000)")
    parser.add_argument("--hold", action="store_true", help="Put Gripper in continuous HOLD mode")
    parser.add_argument("--burst", action="store_true", help="Put Gripper in auto-cutoff BURST mode")
    parser.add_argument("-x", "--stop", action="store_true", help="Stop Gripper PWM ('--stop-all' relaxes Arm too)")
    parser.add_argument("--stop-all", action="store_true", help="Relax both Arm and Gripper PWM")
    parser.add_argument("-s", "--status", action="store_true", help="Print current servo status")
    parser.add_argument("raw_cmd", nargs="*", help="Optional raw command string (e.g. 'a 30' or '20 135')")
    args = parser.parse_args()

    try:
        ser = open_serial(args.port, args.baud)
    except Exception as exc:
        print(f"Error opening {args.port}: {exc}", file=sys.stderr)
        sys.exit(1)

    one_shot = False
    if args.stop_all:
        send_and_print(ser, "x all", wait_sec=0.2)
        one_shot = True
    elif args.stop:
        send_and_print(ser, "x", wait_sec=0.2)
        one_shot = True

    if args.burst:
        send_and_print(ser, "burst", wait_sec=0.2)
        one_shot = True
    elif args.hold:
        send_and_print(ser, "hold", wait_sec=0.2)
        one_shot = True

    if args.duration is not None:
        send_and_print(ser, f"t {args.duration}", wait_sec=0.2)
        one_shot = True

    if args.arm is not None and args.grip is not None:
        send_and_print(ser, f"{args.arm} {args.grip}", wait_sec=0.45)
        one_shot = True
    elif args.arm is not None:
        send_and_print(ser, f"a {args.arm}", wait_sec=0.25)
        one_shot = True
    elif args.grip is not None:
        send_and_print(ser, f"g {args.grip}", wait_sec=0.45)
        one_shot = True

    if args.raw_cmd:
        cmd_str = " ".join(args.raw_cmd)
        send_and_print(ser, cmd_str, wait_sec=0.45)
        one_shot = True

    if args.status:
        send_and_print(ser, "s", wait_sec=0.25)
        one_shot = True

    if not one_shot:
        interactive_loop(ser)

    ser.close()


if __name__ == "__main__":
    main()
