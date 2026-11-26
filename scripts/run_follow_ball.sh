#!/usr/bin/env bash
set -eo pipefail

# Launches the full RELbot ball-following & teleop stack on the Raspberry Pi:
#   1. Xenomai 4 / EVL real-time FPGA loop controller (xenoThread)
#   2. ROS 2 <-> Xenomai bridge (RosXenoBridge) + RELbot motor driver (relbot_driver)
#   3. RELbot hardware bridge + Rust-backed live perception, ball follower & manual teleop
#
# Usage on Pi:
#   ~/relbot_autonomous/run_follow_ball.sh              # autonomous ball following
#   ~/relbot_autonomous/run_follow_ball.sh --manual     # interactive manual keyboard control (WASD/Arrows + M to toggle Auto)
#   ENABLE_MOTION=false ~/relbot_autonomous/run_follow_ball.sh  # safe dry-run

MANUAL_MODE="${MANUAL:-false}"
for arg in "$@"; do
  case "${arg}" in
    --manual|-m)
      MANUAL_MODE="true"
      ;;
    --auto|-a)
      MANUAL_MODE="false"
      ;;
  esac
done

ENABLE_MOTION="${ENABLE_MOTION:-true}"
SEARCH_WHEN_LOST="${SEARCH_WHEN_LOST:-true}"
FOLLOW_STANDOFF_M="${FOLLOW_STANDOFF_M:-0.25}"
MAX_WHEEL_VEL="${MAX_WHEEL_VEL:-4.0}"

XENO_WS="${HOME}/xenomai4-framework-for-sdfr"
AUTO_WS="${HOME}/relbot_autonomous"

PIDS=()
XENO_PID=""

cleanup() {
  echo ""
  echo "[run_follow_ball] Stopping ROS 2 nodes and zeroing motors..."
  for pid in "${PIDS[@]:-}"; do
    kill -INT "${pid}" 2>/dev/null || true
  done
  sleep 0.5
  if [[ -n "${XENO_PID}" ]]; then
    sudo kill -INT "${XENO_PID}" 2>/dev/null || true
    wait "${XENO_PID}" 2>/dev/null || true
  fi
  echo "[run_follow_ball] Cleanly stopped."
}
trap cleanup EXIT INT TERM

echo "[run_follow_ball] Cleaning up any previous xenoThread instance..."
sudo pkill -INT xenoThread 2>/dev/null || true
sleep 0.3

echo "[run_follow_ball] Initializing SPI register for Xenomai OOB SPI..."
sudo busybox devmem 0xfe204e00 32 0x03800000

echo "[run_follow_ball] Starting Xenomai 4 real-time motor controller (xenoThread)..."
sudo "${XENO_WS}/build/xenoThread/xenoThread" &
XENO_PID=$!

# Wait for EVL cross-buffers to appear AND verify xenoThread stays alive after FPGA SPI init
sleep 0.6
WAIT_OK=0
if kill -0 "${XENO_PID}" 2>/dev/null && [[ -e /dev/evl/xbuf/Ros-Xeno && -e /dev/evl/xbuf/Xeno-Ros ]]; then
  WAIT_OK=1
fi

if [[ "${WAIT_OK}" -ne 1 ]]; then
  echo "[run_follow_ball] FPGA init retry: flashing /opt/icoboard/ico_io.bin..."
  sudo icoprog -p < /opt/icoboard/ico_io.bin || true
  sleep 0.5
  sudo "${XENO_WS}/build/xenoThread/xenoThread" &
  XENO_PID=$!
  sleep 0.6
  if kill -0 "${XENO_PID}" 2>/dev/null && [[ -e /dev/evl/xbuf/Ros-Xeno && -e /dev/evl/xbuf/Xeno-Ros ]]; then
    WAIT_OK=1
  fi
fi

if [[ "${WAIT_OK}" -ne 1 ]]; then
  echo "[run_follow_ball] ERROR: xenoThread did not stay running. Check FPGA / power switch."
  exit 1
fi

sudo chmod 666 /dev/evl/xbuf/Ros-Xeno /dev/evl/xbuf/Xeno-Ros

source /opt/ros/humble/setup.bash
source "${XENO_WS}/install/setup.bash"
source "${AUTO_WS}/install/setup.bash"

echo "[run_follow_ball] Starting RosXenoBridge and relbot_driver (max_velocity=${MAX_WHEEL_VEL} rad/s)..."
ros2 run ros_xeno_bridge RosXenoBridge &
PIDS+=($!)

ros2 run relbot_ros2_driver relbot_driver --ros-args \
  -p "max_velocity:=${MAX_WHEEL_VEL}" \
  -r /left_motor/setpoint_vel:=/input/left_motor/setpoint_vel \
  -r /right_motor/setpoint_vel:=/input/right_motor/setpoint_vel &
PIDS+=($!)

sleep 0.5

echo "[run_follow_ball] Starting perception + controller (manual_mode=${MANUAL_MODE}, enable_motion=${ENABLE_MOTION})..."
echo "[run_follow_ball] Live video + web WASD controls: http://$(hostname -I | awk '{print $1}'):8080/"

ros2 launch relbot_hardware_cpp relbot_autonomous.launch.py \
  "enable_motion:=${ENABLE_MOTION}" \
  "follow_only:=true" \
  "manual_mode:=${MANUAL_MODE}" \
  "search_when_lost:=${SEARCH_WHEN_LOST}" \
  "follow_standoff_m:=${FOLLOW_STANDOFF_M}" &
PIDS+=($!)

if [[ "${MANUAL_MODE}" == "true" ]]; then
  sleep 1.5
  python3 - << 'PYEOF'
import sys, termios, tty, select, urllib.request, time

print("\n================================================================")
print(" RELbot ArUco Cube Autonomous Pick-Up & Servo Teleop (--manual)")
print("   W / S / A / D   : Drive Forward / Back / Left / Right")
print("   Space / F       : STOP Base")
print("   M               : Toggle MANUAL <-> AUTO (ArUco Cube Pick-Up)")
print("   I / K           : Raise / Lower 1-DOF Arm (+10 / -10 deg)")
print("   U / Y / 0       : Arm to Pickup (-40 deg) / Carry (+20 deg) / 0 deg")
print("   O / P / X       : Gripper OPEN (g 0) / CLOSE (g 180) / OFF (x)")
print("   G / R           : Run Full PICKUP Macro (G) / RELEASE Cube (R)")
print("   + / -           : Increase / Decrease Drive Speed")
print("   Q or Ctrl-C     : Quit and Stop Motors")
print("================================================================\n")

tty_f = open("/dev/tty", "rb+", buffering=0)
fd = tty_f.fileno()
old_settings = termios.tcgetattr(fd)
speed_v = 0.18
speed_w = 0.65
cur_v = 0.0
cur_w = 0.0
last_key_time = 0.0
manual_active = True

def send_cmd(q):
    try:
        urllib.request.urlopen(f"http://127.0.0.1:8080/api/cmd?{q}", timeout=0.25).read()
    except Exception:
        pass

try:
    tty.setraw(fd)
    while True:
        r, _, _ = select.select([tty_f], [], [], 0.15)
        if r:
            ch = tty_f.read(1).decode("latin1", errors="ignore")
            if ch in ('q', 'Q', '\x03'):
                send_cmd("mode=manual&v=0&w=0")
                break
            elif ch == '\x1b':
                seq = tty_f.read(2).decode("latin1", errors="ignore")
                if seq == '[A':
                    cur_v, cur_w = speed_v, 0.0
                elif seq == '[B':
                    cur_v, cur_w = -speed_v * 0.85, 0.0
                elif seq == '[D':
                    cur_v, cur_w = 0.0, speed_w
                elif seq == '[C':
                    cur_v, cur_w = 0.0, -speed_w
                last_key_time = time.time()
            elif ch in ('w', 'W'):
                cur_v, cur_w = speed_v, 0.0
                last_key_time = time.time()
            elif ch in ('s', 'S'):
                cur_v, cur_w = -speed_v * 0.85, 0.0
                last_key_time = time.time()
            elif ch in ('a', 'A'):
                cur_v, cur_w = 0.0, speed_w
                last_key_time = time.time()
            elif ch in ('d', 'D'):
                cur_v, cur_w = 0.0, -speed_w
                last_key_time = time.time()
            elif ch in (' ', 'f', 'F'):
                cur_v, cur_w = 0.0, 0.0
                last_key_time = 0.0
                send_cmd("mode=manual&v=0&w=0")
                sys.stdout.write("\r[MANUAL] STOP (v=0.00, w=0.00)              ")
                sys.stdout.flush()
                continue
            elif ch in ('m', 'M'):
                manual_active = not manual_active
                cur_v, cur_w = 0.0, 0.0
                send_cmd(f"mode={'manual' if manual_active else 'auto'}&v=0&w=0")
                sys.stdout.write(f"\r[MODE] Switched to {'MANUAL' if manual_active else 'AUTO (ArUco Cube Pick-Up)'}          ")
                sys.stdout.flush()
                continue
            elif ch in ('i', 'I'):
                send_cmd("servo=arm_up")
                sys.stdout.write("\r[SERVO] Arm +10 deg (SRT DL3020)                ")
                sys.stdout.flush()
                continue
            elif ch in ('k', 'K'):
                send_cmd("servo=arm_down")
                sys.stdout.write("\r[SERVO] Arm -10 deg (SRT DL3020)                ")
                sys.stdout.flush()
                continue
            elif ch in ('u', 'U'):
                send_cmd("servo=arm_pick")
                sys.stdout.write("\r[SERVO] Arm Lowered to Pickup (-40 deg)         ")
                sys.stdout.flush()
                continue
            elif ch in ('y', 'Y'):
                send_cmd("servo=arm_carry")
                sys.stdout.write("\r[SERVO] Arm Raised to Carry (+20 deg)           ")
                sys.stdout.flush()
                continue
            elif ch == '0':
                send_cmd("servo=arm_zero")
                sys.stdout.write("\r[SERVO] Arm Centered (0 deg)                    ")
                sys.stdout.flush()
                continue
            elif ch in ('o', 'O'):
                send_cmd("servo=grip_open")
                sys.stdout.write("\r[SERVO] Gripper OPEN (g 0, 250ms burst)         ")
                sys.stdout.flush()
                continue
            elif ch in ('p', 'P'):
                send_cmd("servo=grip_close")
                sys.stdout.write("\r[SERVO] Gripper CLOSE (g 180, 250ms burst)      ")
                sys.stdout.flush()
                continue
            elif ch in ('x', 'X'):
                send_cmd("servo=grip_stop")
                sys.stdout.write("\r[SERVO] Gripper PWM OFF (x)                     ")
                sys.stdout.flush()
                continue
            elif ch in ('g', 'G'):
                send_cmd("macro=pickup")
                sys.stdout.write("\r[MACRO] Executing ArUco Cube PICKUP sequence... ")
                sys.stdout.flush()
                continue
            elif ch in ('r', 'R'):
                send_cmd("macro=release")
                sys.stdout.write("\r[MACRO] Executing ArUco Cube RELEASE sequence...")
                sys.stdout.flush()
                continue
            elif ch in ('+', '='):
                speed_v = min(0.22, speed_v + 0.02)
                speed_w = min(1.00, speed_w + 0.08)
                sys.stdout.write(f"\r[SPEED] v={speed_v:.2f} m/s, w={speed_w:.2f} rad/s         ")
                sys.stdout.flush()
                continue
            elif ch in ('-', '_'):
                speed_v = max(0.08, speed_v - 0.02)
                speed_w = max(0.30, speed_w - 0.08)
                sys.stdout.write(f"\r[SPEED] v={speed_v:.2f} m/s, w={speed_w:.2f} rad/s         ")
                sys.stdout.flush()
                continue

            if manual_active:
                send_cmd(f"mode=manual&v={cur_v:.3f}&w={cur_w:.3f}")
                sys.stdout.write(f"\r[MANUAL] Driving v={cur_v:.2f} m/s, w={cur_w:.2f} rad/s   ")
                sys.stdout.flush()
        else:
            # Auto-stop 0.45s after key release so tapping keys moves smoothly without runaway
            if manual_active and (cur_v != 0.0 or cur_w != 0.0) and (time.time() - last_key_time > 0.45):
                cur_v, cur_w = 0.0, 0.0
                send_cmd("mode=manual&v=0&w=0")
                sys.stdout.write("\r[MANUAL] Idle (v=0.00, w=0.00)              ")
                sys.stdout.flush()
finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    tty_f.close()
    print("")
PYEOF
else
  wait "${PIDS[-1]}"
fi
