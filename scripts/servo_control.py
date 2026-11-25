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
import sys
import time
import serial


def open_serial(port: str, baud: int) -> serial.Serial:
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = 0.15
    # Prevent DTR/RTS toggle from resetting the ESP32 on every connection
    ser.dtr = False
    ser.rts = False
    ser.open()
    return ser


def send_and_print(ser: serial.Serial, cmd: str, wait_sec: float = 0.35) -> None:
    ser.write((cmd.strip() + "\n").encode("utf-8"))
    ser.flush()
    deadline = time.time() + wait_sec
    while time.time() < deadline:
        line = ser.readline()
        if line:
            text = line.decode("utf-8", errors="replace").rstrip()
            if not text.startswith("$STAT,"):
                print(text)


def interactive_loop(ser: serial.Serial) -> None:
    print("=====================================================================")
    print(" RELBot ESP32 Dual-Servo Manual Control (/dev/ttyUSB0)")
    print("   a <deg>        : Set Arm angle (-65 to +65 deg, 0 = center) or us")
    print("   g <deg>        : Set Gripper angle (0 to 180 deg, 90 = neutral) or us")
    print("   <arm> <grip>   : Set BOTH at once (e.g. '25 135' or '0 90')")
    print("   a+ / a-        : Nudge Arm +/- 10 us")
    print("   g+ / g-        : Nudge Gripper +/- 10 us")
    print("   t <ms>         : Set Gripper burst duration (default 250 ms)")
    print("   burst / hold   : Toggle Gripper burst auto-cutoff vs continuous")
    print("   x              : Emergency stop Gripper ('x all' relaxes Arm too)")
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
                send_and_print(ser, "x", wait_sec=0.15)
                break
            # Wait slightly longer if triggering a gripper burst so auto-cutoff prints cleanly
            wait_s = 0.40 if (cmd.lower().startswith("g") or " " in cmd or "," in cmd) else 0.20
            send_and_print(ser, cmd, wait_sec=wait_s)
    except (KeyboardInterrupt, EOFError):
        print("\nStopping gripper and exiting...")
        send_and_print(ser, "x", wait_sec=0.15)


def main() -> None:
    parser = argparse.ArgumentParser(description="RELBot ESP32 Servo Controller (DL3020 Arm + MG996R Gripper)")
    parser.add_argument("-p", "--port", default="/dev/ttyUSB0", help="Serial port (default: /dev/ttyUSB0)")
    parser.add_argument("-b", "--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("-a", "--arm", type=float, default=None, help="Set SRT DL3020 Arm angle (-65..+65 deg) or pulse (750..2250 us)")
    parser.add_argument("-g", "--grip", type=float, default=None, help="Set MG996R Gripper angle (0..180 deg) or pulse (1200..1800 us)")
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
