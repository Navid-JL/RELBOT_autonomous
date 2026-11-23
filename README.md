# RELBot Autonomous Object Detection, Pick-Up (1-DOF XL430 + MG996R) & Delivery

A hybrid **Rust + C++ ROS 2** workspace for the **RELBot** mobile manipulator platform:
- **Rust (`relbot_navigation_rs`)**: Real-time camera object detection (`PickupObject` & `DeliveryZone`), monocular 3D distance/bearing projection, PD visual-servoing & waypoint controller, and the autonomous Pick-Up & Delivery Finite State Machine (FSM).
- **C++ (`relbot_hardware_cpp`)**: Low-level hardware interface for the RELBot differential-drive base (inverse/forward kinematics, wheel velocity setpoints, dead-reckoning odometry), a non-blocking 1-DOF arm + gripper macro sequencer, and a POSIX `termios` serial driver communicating with the microcontroller bridge.
- **ROS 2 Interfaces (`relbot_interfaces`)**: Custom messages (`ArmCommand.msg`, `ManipulatorState.msg`, `DetectedObject.msg`, `MissionStatus.msg`) and services (`ExecutePickPlace.srv`).
- **Microcontroller Firmware (`firmware/mcu_manipulator_bridge/`)**: Arduino/ESP32/STM32 firmware using `Dynamixel2Arduino` (Protocol 2.0) to drive the **1-DOF Dynamixel XL430-W250-T** arm and `Servo` PWM to drive the **MG996R** gripper over a checksummed ASCII serial protocol.

---

## Repository Structure

```text
RELBOT_autonomous/
├── Cargo.toml                                  # Rust workspace manifest
├── firmware/
│   └── mcu_manipulator_bridge/
│       └── relbot_manipulator_bridge.ino       # MCU firmware (Dynamixel XL430 + MG996R)
└── src/
    ├── relbot_interfaces/                      # ROS 2 custom msg/srv definitions
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── msg/
    │   │   ├── ArmCommand.msg
    │   │   ├── DetectedObject.msg
    │   │   ├── ManipulatorState.msg
    │   │   └── MissionStatus.msg
    │   └── srv/
    │       └── ExecutePickPlace.srv
    ├── relbot_hardware_cpp/                    # C++ hardware communication & base kinematics
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── config/hardware_params.yaml
    │   ├── launch/relbot_autonomous.launch.py
    │   ├── include/relbot_hardware_cpp/
    │   │   ├── manipulator_sequencer.hpp
    │   │   ├── relbot_base_driver.hpp
    │   │   └── serial_mcu_bridge.hpp
    │   ├── src/
    │   │   ├── manipulator_sequencer.cpp
    │   │   ├── relbot_base_driver.cpp
    │   │   ├── relbot_hardware_node.cpp
    │   │   └── serial_mcu_bridge.cpp
    │   └── test/test_hw_core.cpp
    └── relbot_navigation_rs/                   # Rust perception, visual servoing & mission FSM
        ├── Cargo.toml
        ├── package.xml
        └── src/
            ├── lib.rs
            ├── main.rs
            ├── types.rs
            ├── perception.rs
            ├── controller.rs
            └── state_machine.rs
```

---

## Microcontroller Serial Protocol (C++ <-> MCU Bridge)

Communication runs at **115200 baud (8N1)** using newline-delimited, XOR-checksummed ASCII frames:

- **Command Frame (`Host C++ -> MCU`)**:
  ```text
  $CMD,<dxl_goal_ticks>,<dxl_profile_vel>,<mg996r_pulse_us>,<torque_en>*<HEX_CS>\n
  ```
  - `dxl_goal_ticks`: Dynamixel XL430-W250-T goal position (`0..4095`, `2048` = `0.0 rad` center)
  - `dxl_profile_vel`: Dynamixel XL430 profile velocity (`0..1023`, `1 unit = 0.229 rpm`)
  - `mg996r_pulse_us`: MG996R PWM pulse width in microseconds (`500..2400 us`, mapped from `0..180 deg`)
  - `torque_en`: `1` (Torque On) or `0` (Torque Off)

- **Telemetry Frame (`MCU -> Host C++`, 50 Hz)**:
  ```text
  $STAT,<dxl_curr_ticks>,<dxl_curr_vel>,<dxl_load_ma>,<mg996r_pulse_us>,<grasped>,<err>*<HEX_CS>\n
  ```

---

## Building & Running

### 1. Full ROS 2 Build (`colcon`)
When running in a sourced ROS 2 environment (`humble` / `jazzy` with `colcon-cargo` / `ament_cargo`):
```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch relbot_hardware_cpp relbot_autonomous.launch.py serial_port:=/dev/ttyUSB0
```

### 2. Standalone Build & Verification (Without ROS 2 Sourced)
Both the Rust navigation stack and C++ hardware library can be built and unit-tested directly with `cargo` and `cmake`:

```bash
# Build and run Rust unit tests + closed-loop mission simulation
cargo test
cargo run --bin relbot_navigation_rs

# Build and run C++ hardware driver unit tests
cmake -S src/relbot_hardware_cpp -B build/relbot_hardware_cpp
cmake --build build/relbot_hardware_cpp
ctest --test-dir build/relbot_hardware_cpp --output-on-failure
```
