#define CUSTOM_SETTINGS
#define INCLUDE_TERMINAL_MODULE
#include <DabbleESP32.h>
#include <math.h>

/*
  SOIL BUDDY RE v3.3 — ESP32-WROOM + resistive soil sensor + HW-038 + TIP120
  ========================================================================
  v3.3 CHANGES (water level in cm; pump/soil/water-guard logic untouched)
    - The reservoir is about 4 cm high x 15 cm long, so the water reading is
      now a LEVEL IN CM (0..RESERVOIR_HEIGHT_CM) instead of a 500 mL estimate.
    - Two-point ADC to cm map: WATER_ADC_AT_EMPTY / WATER_ADC_AT_FULL.
      The defaults (0 and 4095) are the same plain linear guess as before.
    - Field 3 of the compact line is now the water level in cm, 2 decimals:
        #S,soilIndex,soilAdc,waterCm,waterAdc,mode,pump,demand,guard
      Update the web app (index.html v3.1) together with this sketch.
    - The AUTO low-water ADC cutoff (500 / 750) is unchanged. Its cm value is
      printed by 'water', 'threshold' and at boot.

  v3.2 CHANGES (Terminal + Bluetooth fixes; pump/soil/water logic untouched)
    - DabbleESP32 sends every byte as its own BLE notification with a 12 ms
      delay (10 ms in esp32BLEUtilities + 2 ms in appWrite). The old 'status'
      reply (~470 bytes) therefore blocked loop() for ~6 s, which also delays
      the pump timers and the water interlock. Replies are now kept short.
    - NEW command 's' (used by the web app): ONE short machine line, ~0.4 s:
        #S,soilIndex,soilAdc,waterMl,waterAdc,mode,pump,demand,guard   (v3.2; v3.3 sends cm)
        mode 0=OFF/STANDBY 1=AUTO 2=TIMED TEST; pump/demand 1=on; guard 1=PASS
      It is not echoed and is not logged as a command.
    - 'chat on' now sends one brief line every 10 s instead of the full status.
    - The library never restarts BLE advertising after a client disconnects
      (its prevDeviceConnected flag is never set), so after leaving the web
      app the board vanished until reset. loop() now restarts advertising.
    - Removed the 15 ms per-line delay added in v3.1 (the library already
      delays far more than that).
    - BLE MTU raised so a command frame is not split across two BLE writes.
    - Dabble Terminal module ID is 0x02; text from the phone is function 0x01.
    NOTE: DabbleESP32 is BLE only. The board will NOT appear in the phone's
    normal Bluetooth settings list. Use the Soil Buddy web app (Connect),
    the Dabble app, or nRF Connect.

  CONFIRMED CONNECTIONS
    Soil sensor AO        -> GPIO35 (ADC1)
    HW-038 analog signal  -> GPIO34 (ADC1)
    GPIO26                -> 330 ohm resistor -> TIP120 BASE
    TIP120 EMITTER        -> GND
    TIP120 COLLECTOR      -> pump negative
    5V regulated supply   -> pump positive
    ESP32 GND              -> 5V supply GND (common ground)

  RECOMMENDED HARDWARE PROTECTION
    - 10k pull-down from TIP120 BASE to EMITTER to keep pump OFF during reset.
    - Flyback diode across the DC pump: striped cathode to +5V, anode to
      pump-negative/TIP120-collector.
    - Power both analog sensors from 3.3V so their AO outputs stay within the
      ESP32 input range. Do not feed 5V analog output directly to an ESP32 pin.
    - A 100k pull-down from GPIO34 to GND can help make an unplugged water
      sensor fail toward a low-water reading; verify it does not distort the
      sensor's normal output.

  IMPORTANT MEASUREMENT LIMITATIONS (USER REQUEST: NO CALIBRATION)
    1. Soil percentage is an UNCALIBRATED relative ADC index, not VWC and not
       an accurate measured soil-moisture percentage. The number is shown in
       parentheses as requested. The mapping assumes a common resistive-probe
       behavior: higher ADC = drier, lower ADC = wetter. Change the direction
       flag below if this specific board behaves oppositely.
    2. Water level in cm is an UNCALIBRATED ADC estimate. The HW-038 is a
       resistive sensor, so its reading is not truly linear with depth and it
       shifts with water conductivity and sensor mounting. By default 0..4095
       maps to 0..4 cm (RESERVOIR_HEIGHT_CM). For real cm, set
       WATER_ADC_AT_EMPTY / WATER_ADC_AT_FULL from two ruler checks, and
       validate with more depths before SOP 3 (water-level MAE in cm).
    3. AUTO low-water protection uses a fixed raw ADC threshold. It is a pilot
       threshold, NOT a validated value for this particular tank. Physically
       observe the HW-038 reading at the minimum safe pump-submersion level and
       adjust WATER_LOW_ADC_THRESHOLD before unattended AUTO trials. No
       software-only threshold can guarantee dry-run protection from a failed
       or misleading resistive sensor.
    4. The 0/50 soil-index thresholds and 1-second AUTO pulse are pilot
       settings only. Validate the control behavior on your actual chili plant
       and garden soil before reporting them as optimal.
    5. pump test deliberately bypasses the water interlock and runs for 30s.
       It MUST be supervised. A small 4 cm reservoir can be emptied before 30s by
       some mini pumps. Stop early with 'pump off' if the water gets low.

  COMMANDS
    help, soil/sms, water, status, chat on/off, pump test, pump off,
    pump auto, threshold
    No calibration commands, no Preferences storage, no camera functions.
*/

// ------------------------- PINS -----------------------------------------
const uint8_t SOIL_PIN = 35;
const uint8_t WATER_PIN = 34;
const uint8_t PUMP_PIN = 26;
const uint8_t PUMP_ON_LEVEL = HIGH; // TIP120 NPN low-side driver
const uint8_t PUMP_OFF_LEVEL = LOW;
const char* BLUETOOTH_NAME = "Soil Buddy RE";

// ------------------------- DISPLAY / SENSOR ASSUMPTIONS -----------------
const bool SOIL_WET_READING_IS_LOWER = true;
const bool WATER_ADC_INCREASES_WITH_LEVEL = true;
// Reservoir: about 4 cm high x 15 cm long. The water reading is a level in cm.
const float RESERVOIR_HEIGHT_CM = 4.0f;
const float RESERVOIR_LENGTH_CM = 15.0f; // reference only, not used in the maths

// ADC is 12-bit on classic ESP32: 0..4095.
const int ADC_MAX_VALUE = 4095;
const uint8_t ADC_SAMPLE_COUNT = 17; // trimmed average removes one high + one low

// ADC to water level (cm): two-point linear map, no commands, no storage.
//   level_cm = (adc - EMPTY) / (FULL - EMPTY) * RESERVOIR_HEIGHT_CM
// The defaults 0 and 4095 are only a rough guess (0..4095 = 0..4 cm). For real cm:
//   1) send 'water' with the sensor dry and note "HW-038 ADC"  -> WATER_ADC_AT_EMPTY
//   2) fill to the 4 cm mark, send 'water' again                -> WATER_ADC_AT_FULL
// If this sensor reads LOWER when wetter, swap the two numbers AND set
// WATER_ADC_INCREASES_WITH_LEVEL to false (that flag only feeds the safety guard).
const int WATER_ADC_AT_EMPTY = 0;
const int WATER_ADC_AT_FULL = ADC_MAX_VALUE;

// FIXED HW-038 LOW-WATER INTERLOCK (pilot constant; verify physically).
// For WATER_ADC_INCREASES_WITH_LEVEL=true, readings BELOW this value block AUTO.
// This initial value is only a starting point, not a validated sensor threshold.
const int WATER_LOW_ADC_THRESHOLD = 500;
const int WATER_RECOVERY_ADC_THRESHOLD = 750; // hysteresis before low-water alert clears

// ------------------------- AUTO IRRIGATION PILOT SETTINGS ----------------
const float SOIL_START_INDEX = 0.0f;   // demand starts at/below this index (user request: start at 0)
const float SOIL_STOP_INDEX  = 50.0f;  // demand clears at/above this index
const uint8_t REQUIRED_SOIL_READINGS = 3;
const unsigned long SAMPLE_INTERVAL_MS = 1000UL;
const unsigned long WATER_SAFETY_SAMPLE_MS = 250UL;
const unsigned long LIVE_INTERVAL_MS = 10000UL;  // brief live line; Dabble sends ~12 ms per byte
const unsigned long AUTO_PULSE_MS = 1000UL;       // pilot pulse
const unsigned long AUTO_SOAK_MS = 90000UL;       // 90 seconds
const unsigned long AUTO_LOCKOUT_MS = 300000UL;   // 5 minutes
const uint8_t MAX_AUTO_PULSES = 2;

// Manual hardware test: exactly 30 seconds maximum, non-blocking.
const unsigned long PUMP_TEST_MS = 30000UL;

// Set by the DabbleESP32 library (esp32BLEUtilities.cpp) when a BLE client connects.
extern bool isDeviceConnected;
bool bleWasConnected = false;

// ------------------------- STATE ----------------------------------------
bool pumpIsOn = false;
bool pumpTestActive = false;
bool autoPumpEnabled = false; // OFF after boot; send 'pump auto'
bool autoReport = false;
bool wateringDemand = false;
bool lowWaterLatched = false;
bool pumpLockoutActive = false;

int soilRaw = 0;
int waterRaw = 0;
float soilIndexPercent = 0.0f;
float waterLevelCm = 0.0f;

uint8_t belowCount = 0;
uint8_t aboveCount = 0;
uint8_t completedAutoPulses = 0;

unsigned long pumpStartedAt = 0;
unsigned long lastSampleAt = 0;
unsigned long lastWaterSafetyAt = 0;
unsigned long lastLiveAt = 0;
unsigned long nextPumpAllowedAt = 0;
unsigned long lockoutStartedAt = 0;

// ------------------------- MESSAGING ------------------------------------
void say(const String& message) {
  Serial.println(message);
  Terminal.println(message);
}

void sayHeader(const String& title) {
  say("");
  say("========== " + title + " ==========");
}

bool elapsed(unsigned long since, unsigned long duration) {
  return (unsigned long)(millis() - since) >= duration;
}

// ------------------------- SENSOR READS ---------------------------------
int readTrimmedAverage(uint8_t pin) {
  uint32_t total = 0;
  int lowValue = 5000;
  int highValue = -1;

  for (uint8_t i = 0; i < ADC_SAMPLE_COUNT; ++i) {
    int value = analogRead(pin);
    total += (uint32_t)value;
    if (value < lowValue) lowValue = value;
    if (value > highValue) highValue = value;
    delayMicroseconds(250);
  }

  // Trim one low and one high sample to reduce isolated ADC spikes.
  return (int)((total - (uint32_t)lowValue - (uint32_t)highValue) /
               (ADC_SAMPLE_COUNT - 2));
}

float calculateSoilRelativeIndex(int raw) {
  float pct = 100.0f * (float)raw / (float)ADC_MAX_VALUE;
  if (SOIL_WET_READING_IS_LOWER) pct = 100.0f - pct;
  return constrain(pct, 0.0f, 100.0f);
}

float calculateWaterLevelCm(int raw) {
  float span = (float)(WATER_ADC_AT_FULL - WATER_ADC_AT_EMPTY);
  if (span == 0.0f) return 0.0f;
  float fraction = (float)(raw - WATER_ADC_AT_EMPTY) / span;
  fraction = constrain(fraction, 0.0f, 1.0f);
  return fraction * RESERVOIR_HEIGHT_CM;
}

bool waterSensorLooksSafeForAuto() {
  // Sensor near ADC top rail may be wiring saturation/short; fail safe.
  if (waterRaw >= 4085) return false;

  if (WATER_ADC_INCREASES_WITH_LEVEL) {
    return waterRaw >= WATER_LOW_ADC_THRESHOLD;
  }
  return waterRaw <= WATER_LOW_ADC_THRESHOLD;
}

bool waterHasRecovered() {
  if (waterRaw >= 4085) return false;
  if (WATER_ADC_INCREASES_WITH_LEVEL) {
    return waterRaw >= WATER_RECOVERY_ADC_THRESHOLD;
  }
  return waterRaw <= WATER_RECOVERY_ADC_THRESHOLD;
}

void refreshSensors() {
  soilRaw = readTrimmedAverage(SOIL_PIN);
  waterRaw = readTrimmedAverage(WATER_PIN);
  soilIndexPercent = calculateSoilRelativeIndex(soilRaw);
  waterLevelCm = calculateWaterLevelCm(waterRaw);
}

// ------------------------- PUMP CONTROL ---------------------------------
void setPump(bool on) {
  digitalWrite(PUMP_PIN, on ? PUMP_ON_LEVEL : PUMP_OFF_LEVEL);
}

void stopPump(const String& reason, bool completedAutoPulse = false) {
  if (!pumpIsOn) return;

  setPump(false);
  pumpIsOn = false;
  bool wasTest = pumpTestActive;
  pumpTestActive = false;

  say("🛑🚿 PUMP OFF — " + reason);

  if (wasTest) {
    say("🧪✅ The 30-second pump test has ended. The pump is OFF.");
    return;
  }

  if (completedAutoPulse) {
    ++completedAutoPulses;
    nextPumpAllowedAt = millis() + AUTO_SOAK_MS;

    if (completedAutoPulses >= MAX_AUTO_PULSES) {
      pumpLockoutActive = true;
      lockoutStartedAt = millis();
      say("🛡️⏸️ Two AUTO pulses completed. Taking a 5-minute protection break.");
    } else {
      say("💧⏳ Soak time! Letting water spread through the garden soil for 90 seconds.");
    }
  } else if (wateringDemand) {
    nextPumpAllowedAt = millis() + AUTO_SOAK_MS;
    say("⏳ AUTO will pause before attempting another watering pulse.");
  }
}

void startPumpTest() {
  if (pumpIsOn) {
    stopPump("Switching to the requested supervised pump test.");
  }

  autoPumpEnabled = false;
  wateringDemand = false;
  belowCount = 0;
  aboveCount = 0;
  pumpLockoutActive = false;

  setPump(true);
  pumpIsOn = true;
  pumpTestActive = true;
  pumpStartedAt = millis();

  say("🧪🚿 SUPERVISED 30-SECOND PUMP TEST STARTED!");
  say("⚠️ This test bypasses the HW-038 low-water interlock.");
  say("🪣 Confirm the reservoir is filled (about 4 cm deep) and the pump intake is submerged.");
  say("👀 Stay beside the device. Send 'pump off' to stop early. It turns OFF automatically after 30 seconds.");
}

String autoBlockReason() {
  if (!waterSensorLooksSafeForAuto()) {
    return "LOW WATER OR HW-038 SIGNAL FAULT — AUTO PUMP BLOCKED";
  }
  return "";
}

void startAutoPump() {
  String block = autoBlockReason();
  if (block.length()) {
    say("🛡️ AUTO watering paused: " + block + ".");
    return;
  }

  setPump(true);
  pumpIsOn = true;
  pumpTestActive = false;
  pumpStartedAt = millis();
  say("🌱💧 AUTO watering started! One short 1-second pilot pulse.");
}

// ------------------------- SOIL DEMAND / AUTO CONTROL -------------------
void updateSoilDemand() {
  // The index is intentionally relative/un-calibrated, not an absolute
  // moisture-content measurement. These thresholds are initial pilot values.
  if (soilIndexPercent <= SOIL_START_INDEX) {
    aboveCount = 0;
    if (belowCount < REQUIRED_SOIL_READINGS) ++belowCount;

    if (belowCount >= REQUIRED_SOIL_READINGS && !wateringDemand) {
      wateringDemand = true;
      completedAutoPulses = 0;
      pumpLockoutActive = false;
      say("🥺🌵 Soil is in the relatively-dry zone (" +
          String(soilIndexPercent, 1) + "% relative index). Watering demand activated.");
    }
  } else if (soilIndexPercent >= SOIL_STOP_INDEX) {
    belowCount = 0;
    if (aboveCount < REQUIRED_SOIL_READINGS) ++aboveCount;

    if (aboveCount >= REQUIRED_SOIL_READINGS && wateringDemand) {
      wateringDemand = false;
      completedAutoPulses = 0;
      pumpLockoutActive = false;
      nextPumpAllowedAt = 0;
      say("😊🌿 Soil index is in the wetter zone (" +
          String(soilIndexPercent, 1) + "% relative index). Watering demand cleared.");
    }
  } else {
    // Hysteresis band: keep previous demand state.
    belowCount = 0;
    aboveCount = 0;
  }
}

void servicePump() {
  unsigned long now = millis();

  // 30-second non-blocking test timeout. It intentionally bypasses HW-038.
  if (pumpIsOn && pumpTestActive) {
    if (elapsed(pumpStartedAt, PUMP_TEST_MS)) {
      stopPump("30-second supervised test duration reached.");
    }
    return;
  }

  if (!autoPumpEnabled) return;

  String block = autoBlockReason();
  if (block.length()) {
    if (pumpIsOn && !pumpTestActive) stopPump("SAFETY STOP: " + block);
    if (!lowWaterLatched) {
      lowWaterLatched = true;
      say("🚨🪣 " + block + ". Refill/check the tank and HW-038.");
    }
    return;
  }

  if (lowWaterLatched && waterHasRecovered()) {
    lowWaterLatched = false;
    say("✅💧 Water-level signal recovered above the fixed AUTO safety threshold.");
  }

  if (pumpIsOn) {
    if (soilIndexPercent >= SOIL_STOP_INDEX) {
      stopPump("Soil index reached the adequate-moisture zone.");
      return;
    }

    if (elapsed(pumpStartedAt, AUTO_PULSE_MS)) {
      stopPump("1-second AUTO pilot pulse completed.", true);
    }
    return;
  }

  if (!wateringDemand) return;

  if (pumpLockoutActive) {
    if (!elapsed(lockoutStartedAt, AUTO_LOCKOUT_MS)) return;
    pumpLockoutActive = false;
    completedAutoPulses = 0;
    say("⏰🌱 Protection break complete. Checking the soil again.");
  }

  if ((int32_t)(now - nextPumpAllowedAt) < 0) return;
  if (soilIndexPercent <= SOIL_START_INDEX) startAutoPump();
}

void updateWaterAlert() {
  bool safe = waterSensorLooksSafeForAuto();
  if (!safe && !lowWaterLatched) {
    lowWaterLatched = true;
    say("🚨🪣 Water estimate is low or the HW-038 signal is abnormal. AUTO irrigation is blocked.");
  } else if (safe && lowWaterLatched && waterHasRecovered()) {
    lowWaterLatched = false;
    say("✅🪣 Water signal recovered. AUTO water guard is clear.");
  }
}

// ------------------------- REPORTS --------------------------------------
String soilMood() {
  if (soilIndexPercent <= SOIL_START_INDEX) return "🥺 Relatively dry";
  if (soilIndexPercent >= SOIL_STOP_INDEX) return "😊 Relatively wet";
  return "🙂 Middle range";
}

String waterMood() {
  if (!waterSensorLooksSafeForAuto()) return "🚨 LOW / CHECK SENSOR";
  if (waterLevelCm < 1.2f) return "🪣 Getting low (rough estimate)";
  return "💧 Water signal sufficient for AUTO";
}

void reportSoil() {
  sayHeader("🌱 SOIL CHECK-IN");
  say("Raw ADC: " + String(soilRaw) + " / 4095");
  say(soilMood() + " (" + String(soilIndexPercent, 1) + "% relative index)");
  say("ℹ️ The percentage is an uncalibrated ADC index, NOT verified actual soil-water content.");
  say("🌶️ Plant: sili / chili pepper | Medium: garden soil");
}

void reportWater() {
  sayHeader("🪣 RESERVOIR CHECK-IN");
  say("Water level: " + String(waterLevelCm, 1) + " cm / " +
      String(RESERVOIR_HEIGHT_CM, 1) + " cm");
  say("HW-038 ADC: " + String(waterRaw) + " / 4095");
  say(waterMood());
  say("⚠️ cm is an uncalibrated ADC-based estimate (linear map, not a measured depth). Sensor position and water conductivity affect it.");
  say("🛡️ AUTO low-water ADC cutoff: " + String(WATER_LOW_ADC_THRESHOLD) + " counts (about " +
      String(calculateWaterLevelCm(WATER_LOW_ADC_THRESHOLD), 1) + " cm).");
}

void reportStatus() {
  sayHeader("🤖 SOIL BUDDY RE | LIVE STATUS");
  say("🌱 Soil: " + String(soilIndexPercent, 1) + "% relative index (ADC " + String(soilRaw) + ")");
  say("🪣 Water: ~" + String(waterLevelCm, 1) + " cm / " + String(RESERVOIR_HEIGHT_CM, 1) + " cm (ADC " + String(waterRaw) + ")");
  say("🎛️ Pump mode: " + String(pumpTestActive ? "TIMED TEST" : (autoPumpEnabled ? "AUTO" : "OFF / STANDBY")));
  say("🚿 Pump state: " + String(pumpIsOn ? "ON" : "OFF"));
  say("🌿 Watering demand: " + String(wateringDemand ? "ACTIVE" : "CLEAR"));
  say("🛡️ Water guard: " + String(waterSensorLooksSafeForAuto() ? "PASS (pilot threshold)" : "LOW / FAULT — AUTO BLOCKED"));
  say("💬 Live chat: " + String(autoReport ? "ON ✨" : "OFF 🤫"));
  say("ℹ️ Soil % and water cm are relative/estimated values (no calibration).");
}

// ONE short machine-readable line for the web app (about 38 bytes, ~0.5 s to send).
void sendCompact() {
  uint8_t mode = pumpTestActive ? 2 : (autoPumpEnabled ? 1 : 0);
  String line = "#S," + String(soilIndexPercent, 1) + "," + String(soilRaw) + "," +
                String(waterLevelCm, 2) + "," + String(waterRaw) + "," +
                String(mode) + "," + String(pumpIsOn ? 1 : 0) + "," +
                String(wateringDemand ? 1 : 0) + "," +
                String(waterSensorLooksSafeForAuto() ? 1 : 0);
  Serial.println(line);
  Terminal.println(line);
}

// One readable line for 'chat on' (keeps loop() responsive).
void reportBrief() {
  String line = "Soil " + String(soilIndexPercent, 0) + "% | Water ~" +
                String(waterLevelCm, 1) + " cm | Pump " + String(pumpIsOn ? "ON" : "OFF") +
                " | Guard " + String(waterSensorLooksSafeForAuto() ? "PASS" : "LOW");
  Serial.println(line);
  Terminal.println(line);
}

void reportThresholds() {
  sayHeader("🌿 IRRIGATION RULES");
  say("🥺 Start demand: " + String(SOIL_START_INDEX, 1) + "% relative index or lower for 3 readings.");
  say("😊 Clear demand: " + String(SOIL_STOP_INDEX, 1) + "% relative index or higher for 3 readings.");
  say("🚿 AUTO pulse: 1 second | soak: 90 seconds | max 2 pulses before a 5-minute break.");
  say("🧪 Pump test: 30 seconds maximum, non-blocking, automatically OFF.");
  say("🪣 AUTO low-water cutoff: raw ADC threshold " + String(WATER_LOW_ADC_THRESHOLD) +
      " (about " + String(calculateWaterLevelCm(WATER_LOW_ADC_THRESHOLD), 1) + " cm).");
  say("⚠️ All soil thresholds and water ADC cutoff are pilot settings and must be validated on the physical setup.");
}

void help() {
  sayHeader("🕹️ SOIL BUDDY RE COMMAND DECK");
  say("help                 - show commands");
  say("soil / sms           - check soil reading + relative index (%)");
  say("water                - show water level in cm (reservoir about 4 cm deep)");
  say("status               - show complete system status");
  say("chat on / chat off   - enable/disable lively updates");
  say("pump test            - run pump for 30 seconds, then auto OFF");
  say("pump off             - stop pump and disable AUTO immediately");
  say("pump auto            - enable automatic soil-based watering");
  say("threshold            - explain watering and safety settings");
  say("🌱 No calibration commands. Percent and cm are estimates, not validated measurements.");
  say("⚠️ 30-second pump test bypasses the water sensor. Fill tank, submerge pump, supervise, and stop early if needed.");
}

// ------------------------- COMMAND HANDLING ----------------------------
void processCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();
  if (!cmd.length()) return;

  // Fast poll from the web app: no echo, one short line.
  if (cmd == "s") {
    refreshSensors();
    sendCompact();
    return;
  }

  say("💬 > " + cmd);

  if (cmd == "help" || cmd == "?" || cmd == "commands") {
    help();
  }
  else if (cmd == "soil" || cmd == "sms" || cmd == "soil moisture") {
    refreshSensors();
    reportSoil();
  }
  else if (cmd == "water" || cmd == "water level" || cmd == "water status" || cmd == "cm" || cmd == "ml") {
    refreshSensors();
    reportWater();
  }
  else if (cmd == "status" || cmd == "all") {
    refreshSensors();
    reportStatus();
  }
  else if (cmd == "chat on" || cmd == "live on" || cmd == "auto on") {
    autoReport = true;
    say("💬✨ Live chat ON! I'll send a short update every 10 seconds.");
  }
  else if (cmd == "chat off" || cmd == "live off" || cmd == "auto off") {
    autoReport = false;
    say("🤫💤 Quiet mode ON. Important safety alerts and your commands still get replies.");
  }
  else if (cmd == "pump test" || cmd == "test" || cmd == "pump on" || cmd == "on") {
    startPumpTest();
  }
  else if (cmd == "pump off" || cmd == "off") {
    if (pumpIsOn) stopPump("User pressed the stop command.");
    autoPumpEnabled = false;
    wateringDemand = false;
    belowCount = 0;
    aboveCount = 0;
    pumpLockoutActive = false;
    say("🛑 Pump is OFF and AUTO is disabled. Send 'pump auto' to resume automatic watering.");
  }
  else if (cmd == "pump auto" || cmd == "auto pump" || cmd == "auto") {
    if (pumpIsOn && pumpTestActive) stopPump("Leaving test mode and switching to AUTO.");
    autoPumpEnabled = true;
    refreshSensors();
    if (waterSensorLooksSafeForAuto()) {
      say("🤖🌱 AUTO mode enabled! I'll use soil index and the fixed HW-038 low-water guard.");
    } else {
      say("🛡️ AUTO enabled but pump is currently blocked: water is below the pilot threshold or sensor signal is abnormal.");
    }
  }
  else if (cmd == "threshold") {
    reportThresholds();
  }
  else if (cmd == "pump") {
    say("🚿 Commands: 'pump test' = timed 30s test; 'pump off' = stop; 'pump auto' = automatic irrigation.");
  }
  else {
    say("🤔 I didn't recognize that command. Send 'help' for the command deck.");
  }
}

void pollTerminal() {
  if (Terminal.available()) {
    String command = Terminal.readString();
    command.trim();
    if (command.length()) processCommand(command);
  }
}

void pollSerial() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.length()) processCommand(command);
  }
}

// ------------------------- SETUP / LOOP ---------------------------------
void setup() {
  // Set output latch LOW before changing pin mode to minimize boot glitches.
  digitalWrite(PUMP_PIN, PUMP_OFF_LEVEL);
  pinMode(PUMP_PIN, OUTPUT);
  setPump(false);

  Serial.begin(115200);
  Serial.setTimeout(20); // Bound serial command wait so timed pump shutdown stays responsive.

  // Start BLE FIRST so the board is advertising as soon as possible.
  Dabble.begin(BLUETOOTH_NAME);
  Serial.println();
  BLEDevice::setMTU(185); // lets a whole command frame arrive in ONE BLE write
  Serial.print("BLE advertising as: ");
  Serial.println(BLUETOOTH_NAME);

  pinMode(SOIL_PIN, INPUT);
  pinMode(WATER_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(SOIL_PIN, ADC_11db);
  analogSetPinAttenuation(WATER_PIN, ADC_11db);

  delay(400);

  refreshSensors();
  unsigned long now = millis();
  lastSampleAt = now;
  lastWaterSafetyAt = now;
  lastLiveAt = now;

  // Serial only: no BLE phone is connected yet, so Terminal lines here were lost.
  Serial.println("========================================");
  Serial.println("   SOIL BUDDY RE v3.3 | GROW MODE ONLINE");
  Serial.println("   ESP32-WROOM + resistive probe + HW-038");
  Serial.println("   TIP120 pump driver | GPIO26");
  Serial.println("   Pump OFF at boot | AUTO disabled at boot");
  Serial.println("   BLE only: find it in the app / Dabble / nRF Connect,");
  Serial.println("   not in the phone's Bluetooth settings list.");
  Serial.print("   AUTO low-water ADC threshold (pilot): ");
  Serial.println(WATER_LOW_ADC_THRESHOLD);
  Serial.print("   Water level map: ADC ");
  Serial.print(WATER_ADC_AT_EMPTY);
  Serial.print(" = 0 cm, ADC ");
  Serial.print(WATER_ADC_AT_FULL);
  Serial.print(" = ");
  Serial.print(RESERVOIR_HEIGHT_CM, 1);
  Serial.println(" cm");
  Serial.println("========================================");
}

void loop() {
  Dabble.processInput();

  // Library bug workaround: advertise again after a client disconnects.
  bool bleNow = isDeviceConnected;
  if (bleWasConnected && !bleNow) {
    delay(300);
    BLEDevice::startAdvertising();
    Serial.println("BLE client left: advertising again as Soil Buddy RE");
  }
  bleWasConnected = bleNow;

  pollTerminal();
  pollSerial();

  unsigned long now = millis();

  // Normal soil and water sampling.
  if (elapsed(lastSampleAt, SAMPLE_INTERVAL_MS)) {
    lastSampleAt = now;
    refreshSensors();
    updateSoilDemand();
    updateWaterAlert();
  }

  // Faster water interlock sampling while AUTO pump is running.
  if (pumpIsOn && !pumpTestActive && elapsed(lastWaterSafetyAt, WATER_SAFETY_SAMPLE_MS)) {
    lastWaterSafetyAt = now;
    waterRaw = readTrimmedAverage(WATER_PIN);
    waterLevelCm = calculateWaterLevelCm(waterRaw);
    if (!waterSensorLooksSafeForAuto()) {
      stopPump("🛡️ Water safety interlock: low/abnormal HW-038 reading.");
      autoPumpEnabled = false;
    }
  }

  servicePump();

  if (autoReport && elapsed(lastLiveAt, LIVE_INTERVAL_MS)) {
    lastLiveAt = now;
    reportBrief();
  }

  delay(3);
}
