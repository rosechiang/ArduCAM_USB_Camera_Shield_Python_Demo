#include <Arduino.h>
#include <SPI.h>
#include <stdlib.h>

// =====================
// Pin Definitions
// =====================
#define SYNC_PIN 10              // DAC SYNC pin
#define JOYSTICK_BUTTON_PIN A0   // (optional) analog trigger
#define STROBE_IN_PIN 2          // Camera 4 DIR_GPIO1 -> Arduino D2 (interrupt-capable)

// On Arduino Mega: PORTA corresponds to digital pins 22..29
static const uint8_t CAM_MASK = 0xFF; // trigger all 8 cameras on PORTA

// =====================
// LED/DAC Parameters
// =====================
// DAC channels: 0=780, 1=NIR, 2=VIS, 3=450, 4=560, 5=730 (adjust if needed)
const int dacChannels[6] = {0, 1, 2, 3, 4, 5};

// Legacy SET format: 6 numbers only
// pwms[i] = 0..255, where 0 = OFF
int pwms[6] = {0};

// =====================
// ARM buffers (NEW)
// =====================
enum ArmMode : uint8_t { MODE_SNAP = 0, MODE_CAPTURE = 1 };
ArmMode armMode = MODE_SNAP;

// SNAP armed PWMs (one-shot)
int armed_snap[6] = {0};

// CAPTURE armed sequence: 6 steps, each step has 6-channel pwm
static const int STEPS = 6;
int armed_steps[STEPS][6] = {0};

// Active run-state (gated by strobe)
volatile bool snapActive    = false;   // one exposure gated by strobe
volatile bool captureActive = false;   // 6 exposures gated by strobe
volatile int  captureStep   = -1;      // increments on each strobe rising edge

// =====================
// Mode Control (legacy behavior)
// =====================
// strobeEnabled: LED follows camera strobe edges (HIGH=on, LOW=off)
// manualOverride: after ledon/ledoff, ignore strobe until strobe_on
bool strobeEnabled  = true;
bool manualOverride = false;

// =====================
// Strobe edge handling
// =====================
volatile bool strobeLevel   = false;
volatile bool strobeChanged = false;
volatile bool strobeRising  = false;

void onStrobeChange() {
  bool newLevel = (digitalRead(STROBE_IN_PIN) == HIGH);
  // detect rising edge using previous strobeLevel
  strobeRising  = (newLevel && !strobeLevel);
  strobeLevel   = newLevel;
  strobeChanged = true;
}

// =====================
// DAC Functions
// =====================
void setGainAndRef(bool refOn, bool gain2x) {
  uint8_t command  = 0x07 << 4;
  uint8_t dataHigh = (refOn << 7) | (gain2x << 6);
  uint8_t dataLow  = 0x00;

  digitalWrite(SYNC_PIN, LOW);
  delayMicroseconds(1);
  SPI.transfer(command);
  SPI.transfer(dataHigh);
  SPI.transfer(dataLow);
  delayMicroseconds(1);
  digitalWrite(SYNC_PIN, HIGH);
}

void writeAndUpdateDAC(uint8_t channel, uint16_t value) {
  uint8_t command  = (0x03 << 4) | (channel & 0x0F);
  uint8_t highByte = (value >> 8) & 0xFF;
  uint8_t lowByte  = value & 0xFF;

  digitalWrite(SYNC_PIN, LOW);
  delayMicroseconds(1);
  SPI.transfer(command);
  SPI.transfer(highByte);
  SPI.transfer(lowByte);
  delayMicroseconds(1);
  digitalWrite(SYNC_PIN, HIGH);
}

// =====================
// Debug print
// =====================
void printArray6(const int a[6]) {
  for (int i = 0; i < 6; i++) {
    Serial.print(a[i]);
    Serial.print(" ");
  }
}

void printCurrentSettings() {
  Serial.print("Legacy PWMs (SET)  : ");
  printArray6(pwms);
  Serial.print("| ARM_MODE: ");
  Serial.print(armMode == MODE_SNAP ? "SNAP" : "CAPTURE");
  Serial.print(" | strobeEnabled: ");
  Serial.print(strobeEnabled ? "YES" : "NO");
  Serial.print(" | manualOverride: ");
  Serial.println(manualOverride ? "YES" : "NO");
}

// =====================
// Apply LEDs
// =====================
// Apply LED values from a given pwm[6] array (on=true => use array, on=false => zeros)
void applyLED_from(const int pwm_src[6], bool on) {
  // prevent partial update if strobe edge happens mid-write
  noInterrupts();
  for (int i = 0; i < 6; i++) {
    uint16_t v = 0;
    int p = pwm_src[i];
    if (on && p > 0) {
      v = (uint16_t)map(p, 0, 255, 0, 65535);
    }
    writeAndUpdateDAC(dacChannels[i], v);
  }
  interrupts();
}

// Legacy apply uses pwms[]
void applyLED(bool on) {
  applyLED_from(pwms, on);
}

// =====================
// Serial Commands (legacy)
// =====================
void cmd_ledon() {
  manualOverride = true;
  strobeEnabled  = false;
  snapActive = false;
  captureActive = false;
  Serial.println("Manual override: LED ON (continuous) using legacy SET PWMs");
  printCurrentSettings();
  applyLED(true);
}

void cmd_ledoff() {
  manualOverride = true;
  strobeEnabled  = false;
  snapActive = false;
  captureActive = false;
  Serial.println("Manual override: LED OFF (continuous)");
  printCurrentSettings();
  applyLED(false);
}

void cmd_strobe_on() {
  manualOverride = false;
  strobeEnabled  = true;
  Serial.println("Strobe enabled: LED follows camera strobe edges (legacy SET PWMs unless CAPTURE/SNAP active)");
  printCurrentSettings();
}

// Trigger cameras: a short pulse is usually enough for rising-edge trigger mode
void cmd_virtual_arduino_trigger() {
  PORTA |= CAM_MASK;
  delayMicroseconds(200);   // adjust if needed (e.g., 50–500 us)
  PORTA &= ~CAM_MASK;
}

// =====================
// NEW: ARM commands
// =====================
void cmd_arm_mode_snap() {
  armMode = MODE_SNAP;
  Serial.println("ARM_MODE set to SNAP");
}

void cmd_arm_mode_capture() {
  armMode = MODE_CAPTURE;
  Serial.println("ARM_MODE set to CAPTURE (expects step0..5 armed)");
}

void cmd_fire() {
  // Safety: stop manual override so we can gate with strobe
  manualOverride = false;

  if (armMode == MODE_SNAP) {
    // One-shot: gate armed_snap with exactly one strobe pulse
    snapActive = true;
    captureActive = false;
    captureStep = -1;

    // Trigger cameras once
    cmd_virtual_arduino_trigger();

    // Note: LED will actually turn on/off in the strobe-follow section
    Serial.println("FIRE (SNAP): triggered once, waiting for strobe to gate LED");
  } else {
    // CAPTURE: gate armed_steps[0..5] with 6 strobe pulses
    captureActive = true;
    snapActive = false;
    captureStep = -1; // will become 0 on first strobe rising

    // Trigger cameras once (camera sequencer / multi-snap should produce 6 exposures)
    cmd_virtual_arduino_trigger();

    Serial.println("FIRE (CAPTURE): triggered once, will advance step on each strobe rising edge");
  }
}

// =====================
// Setup
// =====================
void setup() {
  Serial.begin(115200);

  // DAC/SPI
  pinMode(SYNC_PIN, OUTPUT);
  digitalWrite(SYNC_PIN, HIGH);
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2));
  setGainAndRef(true, true);  // internal ref ON, gain=2

  // Camera trigger bus
  DDRA  = 0xFF;
  PORTA = 0x00;

  // Strobe input
  // If Camera GPIO output is open-drain, switch to INPUT_PULLUP
  pinMode(STROBE_IN_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(STROBE_IN_PIN), onStrobeChange, CHANGE);

  printCurrentSettings();
}

// =====================
// Helpers for parsing
// =====================
static inline int clamp255(int v) {
  if (v < 0) return 0;
  if (v > 255) return 255;
  return v;
}

void set_array6(int dst[6], int a0,int a1,int a2,int a3,int a4,int a5) {
  dst[0]=clamp255(a0); dst[1]=clamp255(a1); dst[2]=clamp255(a2);
  dst[3]=clamp255(a3); dst[4]=clamp255(a4); dst[5]=clamp255(a5);
}

// =====================
// Loop
// =====================
void loop() {
  // ---- Serial parsing ----
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    // -------- Legacy SET (backward compatible) --------
    if (input.startsWith("SET")) {
      int p0, p1, p2, p3, p4, p5;
      int n = sscanf(input.c_str(), "SET %d %d %d %d %d %d", &p0, &p1, &p2, &p3, &p4, &p5);
      if (n == 6) {
        set_array6(pwms, p0,p1,p2,p3,p4,p5);
        Serial.println("SET parsed (legacy).");
        printCurrentSettings();
      } else {
        Serial.print("SET parse failed. Fields read = ");
        Serial.println(n);
      }

    } else if (input == "virtual_arduino_trigger") {
      cmd_virtual_arduino_trigger();

    } else if (input == "ledon") {
      cmd_ledon();

    } else if (input == "ledoff") {
      cmd_ledoff();

    } else if (input == "strobe_on") {
      cmd_strobe_on();

    // -------- NEW: ARM_MODE --------
    } else if (input.startsWith("ARM_MODE")) {
      // ARM_MODE SNAP | ARM_MODE CAPTURE
      if (input.indexOf("SNAP") >= 0) {
        cmd_arm_mode_snap();
      } else if (input.indexOf("CAPTURE") >= 0) {
        cmd_arm_mode_capture();
      } else {
        Serial.println("ARM_MODE parse failed. Use: ARM_MODE SNAP | ARM_MODE CAPTURE");
      }

    // -------- NEW: ARM_LED --------
    } else if (input.startsWith("ARM_LED")) {
      // SNAP form:    ARM_LED p0 p1 p2 p3 p4 p5
      // CAPTURE form: ARM_LED step p0 p1 p2 p3 p4 p5   where step=0..5
      int step, p0, p1, p2, p3, p4, p5;

      // Try CAPTURE form first (7 ints after ARM_LED)
      int n1 = sscanf(input.c_str(), "ARM_LED %d %d %d %d %d %d %d", &step, &p0, &p1, &p2, &p3, &p4, &p5);
      if (n1 == 7) {
        if (step < 0 || step >= STEPS) {
          Serial.println("ARM_LED step out of range. step must be 0..5");
        } else {
          set_array6(armed_steps[step], p0,p1,p2,p3,p4,p5);
          Serial.print("ARM_LED set CAPTURE step ");
          Serial.print(step);
          Serial.print(": ");
          printArray6(armed_steps[step]);
          Serial.println();
        }
      } else {
        // Try SNAP form (6 ints after ARM_LED)
        int m = sscanf(input.c_str(), "ARM_LED %d %d %d %d %d %d", &p0, &p1, &p2, &p3, &p4, &p5);
        if (m == 6) {
          set_array6(armed_snap, p0,p1,p2,p3,p4,p5);
          Serial.print("ARM_LED set SNAP: ");
          printArray6(armed_snap);
          Serial.println();
        } else {
          Serial.println("ARM_LED parse failed.");
          Serial.println("Use SNAP:    ARM_LED p0 p1 p2 p3 p4 p5");
          Serial.println("Use CAPTURE: ARM_LED step p0 p1 p2 p3 p4 p5   (step=0..5)");
        }
      }

    // -------- NEW: FIRE --------
    } else if (input == "FIRE") {
      cmd_fire();

    } else {
      Serial.println("Unknown command.");
    }
  }

  // ---- Strobe following / gating ----
  if (strobeChanged) {
    bool level;
    bool rising;
    noInterrupts();
    level = strobeLevel;
    rising = strobeRising;
    strobeChanged = false;
    strobeRising = false;
    interrupts();

    // Priority 1: CAPTURE active (step-advance on rising)
    if (captureActive) {
      if (rising) {
        captureStep++;
        if (captureStep >= STEPS) captureStep = STEPS - 1; // clamp
      }

      if (level) {
        int step = captureStep;
        if (step < 0) step = 0; // first high may happen without rising flag in rare cases
        applyLED_from(armed_steps[step], true);
      } else {
        applyLED_from(armed_steps[max(0, captureStep)], false);

        // If we've completed step 5 and strobe fell, stop captureActive
        if (captureStep >= (STEPS - 1)) {
          captureActive = false;
          captureStep = -1;
        }
      }
      return;
    }

    // Priority 2: SNAP active (one-shot gate)
    if (snapActive) {
      if (level) {
        applyLED_from(armed_snap, true);
      } else {
        applyLED_from(armed_snap, false);
        snapActive = false;
      }
      return;
    }

    // Priority 3: legacy strobe-follow (only when enabled & not manualOverride)
    if (strobeEnabled && !manualOverride) {
      applyLED(level);
    }
  }
}
