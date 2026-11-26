/**
 * @file relbot_manipulator_bridge.ino
 * @brief ESP32 Manipulator Firmware for RELBot:
 *   - Pin 26: SRT DL3020 LV digital servo (1-DOF Arm, 330 Hz PWM, -65..+65 deg / 800..2200 us)
 *   - Pin 25: MG996R servo (Continuous Gripper, 50 Hz PWM, 0..180 deg / 1200..1800 us with safe burst auto-cutoff)
 *
 * Supports both:
 *   1. Interactive manual commands over USB Serial (115200 8N1) from the Raspberry Pi:
 *        a <deg_or_us>         -> Set Arm (DL3020, Pin 26) angle (-65..+65 deg) or pulse (750..2250 us)
 *        g <deg_or_us>         -> Trigger Gripper (MG996R, Pin 25) angle (0..180 deg) or pulse (1200..1800 us)
 *        <arm_deg> <grip_deg>  -> Set both Arm and Gripper at once (e.g. "20 135" or "0, 90")
 *        a+ / a- / g+ / g-     -> Nudge Arm or Gripper by +/- 10 us
 *        t <ms>                -> Set Gripper burst duration (50..3000 ms, default 250 ms)
 *        burst / hold          -> Toggle Gripper 250 ms auto-cutoff burst vs continuous hold
 *        x / stop / off        -> Emergency stop Gripper PWM ("x all" relaxes both Arm & Gripper)
 *        s / status / help     -> Print current status and command guide
 *   2. ROS 2 C++ Bridge ASCII checksummed frames:
 *        $CMD,<dxl_goal_ticks>,<dxl_profile_vel>,<mg996r_pulse_us>,<torque_en>*<HEX_CS>\n
 */

#include <Arduino.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// --- Pin & LEDC Hardware Timer Configuration ---
constexpr uint8_t  ARM_PIN          = 26;    // SRT DL3020 (1-DOF Arm)
constexpr uint32_t ARM_FREQ_HZ      = 330;   // 330 Hz digital servo frame rate
constexpr uint32_t ARM_PERIOD_US    = 3030;  // 1,000,000 / 330 Hz
constexpr uint8_t  ARM_LEDC_CH      = 0;     // LEDC Channel 0 (Timer 0)

constexpr uint8_t  GRIP_PIN         = 25;    // MG996R (Continuous Gripper)
constexpr uint32_t GRIP_FREQ_HZ     = 50;    // 50 Hz standard servo frame rate
constexpr uint32_t GRIP_PERIOD_US   = 20000; // 1,000,000 / 50 Hz
constexpr uint8_t  GRIP_LEDC_CH     = 2;     // LEDC Channel 2 (Timer 1 - separate from Timer 0!)

constexpr uint8_t  PWM_RES_BITS     = 16;    // 0..65535 duty resolution

// --- SRT DL3020 (Pin 26) Calibration Constants ---
constexpr float ARM_MIN_DEG         = -65.0f;
constexpr float ARM_MAX_DEG         =  65.0f;
constexpr int   ARM_CAL_MIN_US      = 800;   // Pulse at -65 deg
constexpr int   ARM_CAL_CENTER_US   = 1500;  // Pulse at 0 deg (neutral)
constexpr int   ARM_CAL_MAX_US      = 2200;  // Pulse at +65 deg
constexpr int   ARM_HARD_MIN_US     = 750;
constexpr int   ARM_HARD_MAX_US     = 2250;

// --- MG996R (Pin 25) Calibration & Safety Constants ---
constexpr int   GRIP_SAFE_MIN_US    = 1200;  // Maps to 0 deg (full speed direction A)
constexpr int   GRIP_SAFE_CENTER_US = 1500;  // Maps to 90 deg (neutral / stop)
constexpr int   GRIP_SAFE_MAX_US    = 1800;  // Maps to 180 deg (full speed direction B)

// --- Runtime State ---
int   armCurrentUs       = ARM_CAL_CENTER_US;
bool  armPwmActive       = true;

int   gripCurrentUs      = GRIP_SAFE_CENTER_US;
bool  gripPwmActive      = false;
bool  gripBurstMode      = true;             // Default: auto-cut Gripper PWM after burstDurationMs
unsigned long gripBurstDurationMs = 250;     // Default 250 ms burst window
unsigned long gripPwmStartMs      = 0;

bool  rosBridgeActive    = false;            // Only stream 50 Hz $STAT when ROS $CMD is active
unsigned long lastTelemetryMs     = 0;
int   lastRosMgUs        = 0;

char  rxBuf[96];
size_t rxIdx = 0;

// --- Low-Level LEDC Helpers ---
void writeArmDuty(uint32_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(ARM_PIN, duty);
#else
  ledcWrite(ARM_LEDC_CH, duty);
#endif
}

void writeGripDuty(uint32_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(GRIP_PIN, duty);
#else
  ledcWrite(GRIP_LEDC_CH, duty);
#endif
}

// --- Arm (SRT DL3020, Pin 26) Conversions & Control ---
int armAngleToPulseUs(float deg) {
  deg = constrain(deg, ARM_MIN_DEG, ARM_MAX_DEG);
  if (deg < 0.0f) {
    float ratio = (deg - ARM_MIN_DEG) / (0.0f - ARM_MIN_DEG);
    return static_cast<int>(roundf(ARM_CAL_MIN_US + ratio * (ARM_CAL_CENTER_US - ARM_CAL_MIN_US)));
  } else {
    float ratio = deg / ARM_MAX_DEG;
    return static_cast<int>(roundf(ARM_CAL_CENTER_US + ratio * (ARM_CAL_MAX_US - ARM_CAL_CENTER_US)));
  }
}

float armPulseUsToAngle(int us) {
  if (us < ARM_CAL_CENTER_US) {
    float ratio = static_cast<float>(us - ARM_CAL_MIN_US) / (ARM_CAL_CENTER_US - ARM_CAL_MIN_US);
    return ARM_MIN_DEG + ratio * (0.0f - ARM_MIN_DEG);
  } else {
    float ratio = static_cast<float>(us - ARM_CAL_CENTER_US) / (ARM_CAL_MAX_US - ARM_CAL_CENTER_US);
    return ratio * ARM_MAX_DEG;
  }
}

void setArmPulseUs(int us) {
  us = constrain(us, ARM_HARD_MIN_US, ARM_HARD_MAX_US);
  armCurrentUs = us;
  armPwmActive = true;
  uint32_t duty = (static_cast<uint64_t>(us) * 65535ULL) / ARM_PERIOD_US;
  writeArmDuty(duty);
}

void relaxArm() {
  armPwmActive = false;
  writeArmDuty(0);
}

// --- Gripper (MG996R, Pin 25) Conversions & Control ---
int gripAngleToPulseUs(float deg) {
  deg = constrain(deg, 0.0f, 180.0f);
  return static_cast<int>(roundf(GRIP_SAFE_MIN_US + (deg / 180.0f) * (GRIP_SAFE_MAX_US - GRIP_SAFE_MIN_US)));
}

float gripPulseUsToAngle(int us) {
  return (static_cast<float>(us - GRIP_SAFE_MIN_US) / (GRIP_SAFE_MAX_US - GRIP_SAFE_MIN_US)) * 180.0f;
}

void stopGripPwm(bool autoCutoff, bool quiet = false) {
  gripPwmActive = false;
  writeGripDuty(0);  // Hold signal pin LOW -> 0 torque on continuous servo
  if (!quiet && !rosBridgeActive) {
    if (autoCutoff) {
      Serial.printf("[GRIP] Auto-cutoff (%lu ms) -> PWM OFF (relaxed)\n", gripBurstDurationMs);
    } else {
      Serial.println("[GRIP] STOP -> PWM OFF (relaxed)");
    }
  }
}

void triggerGripPulseUs(int us, bool quiet = false) {
  us = constrain(us, GRIP_SAFE_MIN_US, GRIP_SAFE_MAX_US);
  gripCurrentUs = us;
  gripPwmActive = true;
  gripPwmStartMs = millis();

  uint32_t duty = (static_cast<uint64_t>(us) * 65535ULL) / GRIP_PERIOD_US;
  writeGripDuty(duty);

  if (!quiet && !rosBridgeActive) {
    Serial.printf("[GRIP] Angle: %5.1f deg | Pulse: %4d us | Mode: %s",
                  gripPulseUsToAngle(gripCurrentUs), gripCurrentUs,
                  gripBurstMode ? "BURST" : "HOLD");
    if (gripBurstMode) {
      Serial.printf(" (%lu ms)\n", gripBurstDurationMs);
    } else {
      Serial.println(" (send 'x' to stop)");
    }
  }
}

// --- Status & Help Printing ---
void printStatus() {
  Serial.printf("[STATUS] ARM (Pin 26, DL3020): %+5.1f deg (%4d us, %s) | "
                "GRIP (Pin 25, MG996R): %5.1f deg (%4d us, %s, %s %lu ms)\n",
                armPulseUsToAngle(armCurrentUs), armCurrentUs, armPwmActive ? "ON" : "OFF",
                gripPulseUsToAngle(gripCurrentUs), gripCurrentUs, gripPwmActive ? "ACTIVE" : "OFF",
                gripBurstMode ? "BURST" : "HOLD", gripBurstDurationMs);
}

void printHelp() {
  Serial.println("\n=== RELBot ESP32 Dual-Servo Controller (Pin 26: DL3020 Arm | Pin 25: MG996R Gripper) ===");
  Serial.println("  a <deg_or_us>        : Move Arm (-65..+65 deg, or 750..2250 us), e.g. 'a 0', 'a 30', 'a -45'");
  Serial.println("  g <deg_or_us>        : Move Gripper (0..180 deg [90=neutral], or 1200..1800 us), e.g. 'g 45', 'g 135'");
  Serial.println("  <arm_deg> <grip_deg> : Set BOTH Arm and Gripper angles at once, e.g. '25 135' or '0, 90'");
  Serial.println("  a+ / a-              : Nudge Arm pulse +10 us / -10 us");
  Serial.println("  g+ / g-              : Nudge Gripper pulse +10 us / -10 us");
  Serial.println("  t <ms>               : Set Gripper burst duration (50..3000 ms, default 250 ms)");
  Serial.println("  burst / hold         : Toggle Gripper auto-cutoff BURST mode vs continuous HOLD mode");
  Serial.println("  x / stop             : Stop Gripper PWM immediately ('x all' relaxes Arm too)");
  Serial.println("  s / status / help    : Show current status or this help guide");
  printStatus();
}

bool isValidNumber(const char* s) {
  if (!s || *s == '\0') return false;
  if (*s == '+' || *s == '-') s++;
  bool hasDigit = false;
  while (*s) {
    if (isdigit(static_cast<unsigned char>(*s))) hasDigit = true;
    else if (*s != '.') return false;
    s++;
  }
  return hasDigit;
}

bool applyArmValue(float val) {
  if (val >= ARM_MIN_DEG && val <= ARM_MAX_DEG) {
    setArmPulseUs(armAngleToPulseUs(val));
    return true;
  } else if (val >= ARM_HARD_MIN_US && val <= ARM_HARD_MAX_US) {
    setArmPulseUs(static_cast<int>(roundf(val)));
    return true;
  }
  Serial.printf("[ARM] Out of range (%.1f). Allowed: %.0f..+%.0f deg or %d..%d us.\n",
                val, ARM_MIN_DEG, ARM_MAX_DEG, ARM_HARD_MIN_US, ARM_HARD_MAX_US);
  return false;
}

bool applyGripValue(float val) {
  if (val >= 0.0f && val <= 180.0f) {
    triggerGripPulseUs(gripAngleToPulseUs(val));
    return true;
  } else if (val >= GRIP_SAFE_MIN_US && val <= GRIP_SAFE_MAX_US) {
    triggerGripPulseUs(static_cast<int>(roundf(val)));
    return true;
  }
  Serial.printf("[GRIP] Out of range (%.1f). Allowed: 0..180 deg or %d..%d us.\n",
                val, GRIP_SAFE_MIN_US, GRIP_SAFE_MAX_US);
  return false;
}

// --- ROS 2 Bridge Protocol ($CMD / $STAT) ---
uint8_t computeXorChecksum(const char* s, size_t len) {
  uint8_t cs = 0;
  for (size_t i = 0; i < len; ++i) {
    cs ^= static_cast<uint8_t>(s[i]);
  }
  return cs;
}

void processRosCommandFrame(const char* line) {
  const char* dollar = strchr(line, '$');
  const char* star = strchr(line, '*');
  if (!dollar || !star || star <= dollar + 1 || strlen(star) < 3) {
    return;
  }
  size_t payload_len = static_cast<size_t>(star - (dollar + 1));
  uint8_t expected_cs = computeXorChecksum(dollar + 1, payload_len);
  unsigned int recv_cs = 0;
  if (sscanf(star + 1, "%02x", &recv_cs) != 1 || static_cast<uint8_t>(recv_cs) != expected_cs) {
    return;
  }

  int dxl_ticks = 2048, dxl_vel = 50, mg_us = 1500, torque = 1;
  if (sscanf(dollar + 1, "CMD,%d,%d,%d,%d", &dxl_ticks, &dxl_vel, &mg_us, &torque) == 4) {
    rosBridgeActive = true;
    if (torque != 0) {
      // Map XL430 ticks (2048 = 0 rad, 4096 ticks = 360 deg) to SRT DL3020 degrees (-65..+65 deg)
      float deg = (static_cast<float>(dxl_ticks - 2048) * 360.0f) / 4096.0f;
      setArmPulseUs(armAngleToPulseUs(deg));
    } else {
      relaxArm();
    }
    // Only trigger a new gripper burst when the commanded gripper pulse changes
    int clamped_mg = constrain(mg_us, GRIP_SAFE_MIN_US, GRIP_SAFE_MAX_US);
    if (abs(clamped_mg - lastRosMgUs) >= 8) {
      lastRosMgUs = clamped_mg;
      triggerGripPulseUs(clamped_mg, true);
    }
  }
}

void publishRosTelemetryFrame() {
  float arm_deg = armPulseUsToAngle(armCurrentUs);
  int curr_ticks = constrain(static_cast<int>(roundf(2048.0f + (arm_deg / 360.0f) * 4096.0f)), 0, 4095);
  int grasped = (gripCurrentUs >= 1550) ? 1 : 0;

  char payload[96];
  int len = snprintf(payload, sizeof(payload), "STAT,%d,0,120.0,%d,%d,0",
                     curr_ticks, gripCurrentUs, grasped);
  if (len <= 0) return;
  uint8_t cs = computeXorChecksum(payload, static_cast<size_t>(len));
  Serial.printf("$%s*%02X\n", payload, cs);
}

// --- Interactive Command Parser ---
void handleLine(char* line) {
  while (*line == ' ' || *line == '\t') line++;
  size_t len = strlen(line);
  while (len > 0 && (line[len - 1] == ' ' || line[len - 1] == '\t')) {
    line[--len] = '\0';
  }
  if (len == 0) return;

  // ROS 2 frame
  if (line[0] == '$') {
    processRosCommandFrame(line);
    return;
  }
  rosBridgeActive = false;

  // Help / Status
  if (strcasecmp(line, "s") == 0 || strcasecmp(line, "status") == 0) {
    printStatus();
    return;
  }
  if (strcasecmp(line, "h") == 0 || strcasecmp(line, "help") == 0 || strcmp(line, "?") == 0) {
    printHelp();
    return;
  }

  // Emergency Stop
  if (strcasecmp(line, "x") == 0 || strcasecmp(line, "stop") == 0 || strcasecmp(line, "off") == 0) {
    stopGripPwm(false);
    return;
  }
  if (strcasecmp(line, "x all") == 0 || strcasecmp(line, "off all") == 0) {
    stopGripPwm(false);
    relaxArm();
    Serial.println("[ALL] Arm & Gripper PWM OFF (relaxed).");
    return;
  }

  // Gripper mode toggles
  if (strcasecmp(line, "burst") == 0) {
    gripBurstMode = true;
    if (gripPwmActive) stopGripPwm(false);
    Serial.printf("[GRIP] Switched to BURST mode (%lu ms auto-cutoff).\n", gripBurstDurationMs);
    return;
  }
  if (strcasecmp(line, "hold") == 0) {
    gripBurstMode = false;
    Serial.println("[GRIP] Switched to HOLD mode (continuous PWM until 'x').");
    return;
  }

  // Burst duration: "t 250"
  if ((line[0] == 't' || line[0] == 'T') && (line[1] == ' ' || isdigit(static_cast<unsigned char>(line[1])))) {
    char* arg = line + 1;
    while (*arg == ' ') arg++;
    if (isValidNumber(arg)) {
      gripBurstDurationMs = static_cast<unsigned long>(constrain(atol(arg), 50L, 3000L));
      Serial.printf("[GRIP] Burst duration set to %lu ms.\n", gripBurstDurationMs);
    } else {
      Serial.println("Usage: t <ms> (50..3000)");
    }
    return;
  }

  // Quick nudges
  if (strcasecmp(line, "a+") == 0) {
    setArmPulseUs(armCurrentUs + 10);
    Serial.printf("[ARM] Angle: %+5.1f deg | Pulse: %4d us\n", armPulseUsToAngle(armCurrentUs), armCurrentUs);
    return;
  }
  if (strcasecmp(line, "a-") == 0) {
    setArmPulseUs(armCurrentUs - 10);
    Serial.printf("[ARM] Angle: %+5.1f deg | Pulse: %4d us\n", armPulseUsToAngle(armCurrentUs), armCurrentUs);
    return;
  }
  if (strcasecmp(line, "g+") == 0) {
    triggerGripPulseUs(gripCurrentUs + 10);
    return;
  }
  if (strcasecmp(line, "g-") == 0) {
    triggerGripPulseUs(gripCurrentUs - 10);
    return;
  }

  // Arm prefix: "a <val>" or "arm <val>"
  if (strncasecmp(line, "arm", 3) == 0 && (line[3] == ' ' || line[3] == ':' || line[3] == '=')) {
    char* arg = line + 4;
    while (*arg == ' ' || *arg == ':' || *arg == '=') arg++;
    if (strcasecmp(arg, "off") == 0) {
      relaxArm();
      Serial.println("[ARM] PWM OFF (relaxed).");
    } else if (isValidNumber(arg) && applyArmValue(atof(arg))) {
      Serial.printf("[ARM] Angle: %+5.1f deg | Pulse: %4d us\n", armPulseUsToAngle(armCurrentUs), armCurrentUs);
    }
    return;
  }
  if ((line[0] == 'a' || line[0] == 'A') && (line[1] == ' ' || line[1] == ':' || line[1] == '=' || isdigit(static_cast<unsigned char>(line[1])) || line[1] == '-' || line[1] == '+')) {
    char* arg = line + 1;
    while (*arg == ' ' || *arg == ':' || *arg == '=') arg++;
    if (strcasecmp(arg, "off") == 0) {
      relaxArm();
      Serial.println("[ARM] PWM OFF (relaxed).");
    } else if (isValidNumber(arg) && applyArmValue(atof(arg))) {
      Serial.printf("[ARM] Angle: %+5.1f deg | Pulse: %4d us\n", armPulseUsToAngle(armCurrentUs), armCurrentUs);
    }
    return;
  }

  // Gripper prefix: "g <val>" or "grip <val>"
  if (strncasecmp(line, "grip", 4) == 0 && (line[4] == ' ' || line[4] == ':' || line[4] == '=')) {
    char* arg = line + 5;
    while (*arg == ' ' || *arg == ':' || *arg == '=') arg++;
    if (strcasecmp(arg, "off") == 0 || strcasecmp(arg, "stop") == 0 || strcasecmp(arg, "x") == 0) {
      stopGripPwm(false);
    } else if (strcasecmp(arg, "c") == 0) {
      triggerGripPulseUs(GRIP_SAFE_CENTER_US);
    } else if (isValidNumber(arg)) {
      applyGripValue(atof(arg));
    }
    return;
  }
  if ((line[0] == 'g' || line[0] == 'G') && (line[1] == ' ' || line[1] == ':' || line[1] == '=' || isdigit(static_cast<unsigned char>(line[1])))) {
    char* arg = line + 1;
    while (*arg == ' ' || *arg == ':' || *arg == '=') arg++;
    if (strcasecmp(arg, "off") == 0 || strcasecmp(arg, "stop") == 0 || strcasecmp(arg, "x") == 0) {
      stopGripPwm(false);
    } else if (strcasecmp(arg, "c") == 0) {
      triggerGripPulseUs(GRIP_SAFE_CENTER_US);
    } else if (isValidNumber(arg)) {
      applyGripValue(atof(arg));
    }
    return;
  }

  // Two numbers "<arm_val> <grip_val>" or "<arm_val>,<grip_val>"
  char* sep = strchr(line, ',');
  if (!sep) sep = strchr(line, ' ');
  if (sep) {
    *sep = '\0';
    char* second = sep + 1;
    while (*second == ' ' || *second == ',') second++;
    if (isValidNumber(line) && isValidNumber(second)) {
      if (applyArmValue(atof(line))) {
        Serial.printf("[ARM] Angle: %+5.1f deg | Pulse: %4d us\n", armPulseUsToAngle(armCurrentUs), armCurrentUs);
      }
      applyGripValue(atof(second));
      return;
    }
  }

  Serial.println("Unknown command. Type 'help' (or 'a <deg>', 'g <deg>', '<arm> <grip>', 'x').");
}

void setup() {
  Serial.begin(115200);

  // Hold Gripper pin LOW before attaching LEDC so continuous servo doesn't twitch at boot
  pinMode(GRIP_PIN, OUTPUT);
  digitalWrite(GRIP_PIN, LOW);
  pinMode(ARM_PIN, OUTPUT);
  digitalWrite(ARM_PIN, LOW);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcAttachChannel(ARM_PIN, ARM_FREQ_HZ, PWM_RES_BITS, ARM_LEDC_CH);
  ledcAttachChannel(GRIP_PIN, GRIP_FREQ_HZ, PWM_RES_BITS, GRIP_LEDC_CH);
#else
  ledcSetup(ARM_LEDC_CH, ARM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(ARM_PIN, ARM_LEDC_CH);
  ledcSetup(GRIP_LEDC_CH, GRIP_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(GRIP_PIN, GRIP_LEDC_CH);
#endif

  writeGripDuty(0);                  // Gripper starts passive (PWM OFF)
  setArmPulseUs(ARM_CAL_CENTER_US);  // Arm starts at 0.0 deg (1500 us)
  delay(200);

  printHelp();
}

void loop() {
  // Auto-cutoff Gripper PWM after burstDurationMs in BURST mode
  if (gripPwmActive && gripBurstMode && (millis() - gripPwmStartMs >= gripBurstDurationMs)) {
    stopGripPwm(true);
  }

  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      if (rxIdx > 0) {
        rxBuf[rxIdx] = '\0';
        handleLine(rxBuf);
        rxIdx = 0;
      }
    } else if (rxIdx < sizeof(rxBuf) - 1) {
      rxBuf[rxIdx++] = c;
    }
  }

  if (rosBridgeActive && (millis() - lastTelemetryMs >= 20)) {
    lastTelemetryMs = millis();
    publishRosTelemetryFrame();
  }
}
