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

print("\n========================================================")
print(" RELbot Interactive Manual Teleop (--manual)")
print("   W / Up Arrow    : Forward")
print("   S / Down Arrow  : Backward")
print("   A / Left Arrow  : Turn Left")
print("   D / Right Arrow : Turn Right")
print("   Space / F       : STOP")
print("   M               : Toggle MANUAL <-> AUTO (Follow Ball)")
print("   + / -           : Increase / Decrease speed")
print("   Q or Ctrl-C     : Quit and stop motors")
print("========================================================\n")

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
                sys.stdout.write(f"\r[MODE] Switched to {'MANUAL' if manual_active else 'AUTO (Follow Ball)'}          ")
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
