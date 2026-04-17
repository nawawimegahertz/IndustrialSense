/**
 * ============================================================================
 * 02_Advanced_Calibration.ino — Full-Featured IndustrialSense Demo
 * ============================================================================
 * 
 * LEVEL 2: Pro-Level Features
 * 
 * This example demonstrates ALL advanced features of IndustrialSense:
 *   ✓ Runtime serial command calibration (processCommand)
 *   ✓ Kalman filter tuning for high-noise environments
 *   ✓ Multi-point non-linear voltage calibration
 *   ✓ Persistent EEPROM/Preferences auto-load on boot
 *   ✓ Custom ADC function injection (ADS1115 example)
 *   ✓ Non-blocking loop with millis()-based timing
 * 
 * SERIAL COMMANDS:
 *   Send these via Serial Monitor (115200 baud, NL+CR):
 * 
 *   ┌──────────┬──────────────────────────────────────────────────────────┐
 *   │ Command  │ Action                                                  │
 *   ├──────────┼──────────────────────────────────────────────────────────┤
 *   │ v12.5    │ Read current ADC, map it to 12.5V, save to EEPROM      │
 *   │ v0       │ Read current ADC, map it to 0.0V (ground calibration)  │
 *   │ v24.0    │ Read current ADC, map it to 24.0V (full-scale cal)     │
 *   │ auto     │ Force current sensor auto-zero, save offset            │
 *   │ reset    │ Clear ALL calibration data from RAM and EEPROM         │
 *   └──────────┴──────────────────────────────────────────────────────────┘
 * 
 * CALIBRATION WORKFLOW:
 *   1. Connect a known voltage source (e.g., 12.50V from a bench PSU).
 *   2. Type "v12.5" in the Serial Monitor and press Enter.
 *   3. Connect a different voltage (e.g., 5.00V), type "v5.0".
 *   4. Repeat for more points (up to 10). More points = higher accuracy.
 *   5. Calibration is saved automatically — it persists across reboots.
 * 
 * WIRING: Same as 01_Basic_Usage.ino
 *   ACS712-20A VOUT → A0
 *   Voltage Divider  → A1  (R1=30kΩ, R2=7.5kΩ)
 * 
 * LICENSE: MIT
 * ============================================================================
 */

#include <IndustrialSense.h>

// ============================================================================
//  SENSOR DECLARATIONS
// ============================================================================
IndustrialCurrent currentSensor(ACS712_20A, A0);
IndustrialVoltage voltageSensor(A1, 30000.0, 7500.0);

// ============================================================================
//  TIMING (non-blocking)
// ============================================================================
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL_MS = 250;  // 4 Hz update rate

// ============================================================================
//  SETUP — Advanced configuration
// ============================================================================
void setup() {
    Serial.begin(115200);
    while (!Serial) { ; }

    Serial.println(F("================================================"));
    Serial.println(F("  IndustrialSense — Advanced Calibration Demo"));
    Serial.println(F("================================================"));
    Serial.println();

    // -----------------------------------------------------------------------
    //  Initialize sensors (this auto-loads saved calibration).
    // -----------------------------------------------------------------------
    currentSensor.begin();
    voltageSensor.begin();

    // -----------------------------------------------------------------------
    //  KALMAN TUNING — for high-EMI / FOC inverter environments.
    //
    //  Process Noise (Q):
    //    Default: 0.01. Increase for fast-changing loads (e.g., Q=0.05).
    //    Too high → noisy output. Too low → sluggish response.
    //
    //  Measurement Noise (R):
    //    Default: 0.5. Increase in electrically noisy environments (e.g., R=2.0).
    //    Higher R = smoother output but slower tracking.
    //
    //  Innovation Threshold (rho):
    //    Default: 0.5. The "adaptive" part — if the change in ADC reading
    //    is smaller than rho, we skip the full Kalman update.
    //    Set to 0.0 to disable adaptive behavior (always full update).
    //    Increase for noisier environments to get more zero-stability.
    // -----------------------------------------------------------------------
    currentSensor.setKalmanProcessNoise(0.02f);       // Slightly more responsive
    currentSensor.setKalmanMeasurementNoise(1.0f);     // More smoothing for noisy env
    currentSensor.setKalmanInnovationThreshold(0.8f);  // Wider adaptive band

    // -----------------------------------------------------------------------
    //  OVERSAMPLING — increase for better noise floor, at cost of latency.
    //  128 samples × 10μs each = ~1.3ms per read() call.
    // -----------------------------------------------------------------------
    currentSensor.setOversampleCount(128);
    voltageSensor.setOversampleCount(64);

    // -----------------------------------------------------------------------
    //  DEADZONE — tune based on your sensor's noise floor.
    //  0.15A is good for ACS712-20A with moderate EMI.
    // -----------------------------------------------------------------------
    currentSensor.setDeadzone(0.15f);

    // -----------------------------------------------------------------------
    //  AUTO-ZERO INTERVAL — how long (ms) of stable 0A before auto-recal.
    //  10 seconds for production, 3 seconds for bench testing.
    // -----------------------------------------------------------------------
    currentSensor.setAutoZeroInterval(10000);

    // -----------------------------------------------------------------------
    //  EMA ALPHA — tune for display update rate.
    //  0.10 = very smooth (good for LCD/OLED at 5Hz refresh).
    //  0.30 = snappy (good for responsive data logging).
    // -----------------------------------------------------------------------
    voltageSensor.setEmaAlpha(0.12f);

    // -----------------------------------------------------------------------
    //  CUSTOM ADC EXAMPLE (commented out — uncomment if using ADS1115):
    //
    //  #include <Adafruit_ADS1X15.h>
    //  Adafruit_ADS1115 ads;
    //
    //  int customADCRead(uint8_t pin) {
    //      return ads.readADC_SingleEnded(pin);
    //  }
    //
    //  In setup():
    //    ads.begin();
    //    currentSensor.setCustomADCFunction(customADCRead);
    //    currentSensor.setAdcResolution(16);  // ADS1115 is 16-bit
    //    currentSensor.setAdcVoltageRef(6.144f);  // ±6.144V PGA
    // -----------------------------------------------------------------------

    // -----------------------------------------------------------------------
    //  Print loaded calibration info.
    // -----------------------------------------------------------------------
    Serial.print(F("[INFO] Voltage calibration points loaded: "));
    Serial.println(voltageSensor.getCalibrationPointCount());
    Serial.println();
    Serial.println(F("COMMANDS:  v<volts>  |  auto  |  reset"));
    Serial.println(F("Example:   v12.5  (maps current ADC to 12.5V)"));
    Serial.println(F("------------------------------------------"));
    Serial.println();

    lastPrintTime = millis();
}

// ============================================================================
//  LOOP — Non-blocking read + serial command processing
// ============================================================================
void loop() {
    // -----------------------------------------------------------------------
    //  SERIAL COMMAND PROCESSING
    //  Check if the user has sent a calibration command via Serial.
    //  Commands are processed by BOTH sensors — the first one that recognizes
    //  the command will handle it.
    // -----------------------------------------------------------------------
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();

        if (cmd.length() > 0) {
            Serial.print(F("\n>>> Command received: \""));
            Serial.print(cmd);
            Serial.println(F("\""));

            bool handled = false;

            // Try voltage sensor first (handles "v<value>" and "reset").
            if (voltageSensor.processCommand(cmd)) {
                handled = true;
                Serial.println(F("[OK] Voltage sensor processed command."));
                Serial.print(F("[INFO] Calibration points: "));
                Serial.println(voltageSensor.getCalibrationPointCount());
            }

            // Try current sensor (handles "auto" and "reset").
            if (currentSensor.processCommand(cmd)) {
                handled = true;
                Serial.println(F("[OK] Current sensor processed command."));
            }

            if (!handled) {
                Serial.println(F("[ERR] Unknown command. Use: v<volts>, auto, reset"));
            }

            Serial.println();
        }
    }

    // -----------------------------------------------------------------------
    //  PERIODIC SENSOR READING (non-blocking, millis()-based)
    // -----------------------------------------------------------------------
    unsigned long now = millis();
    if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
        lastPrintTime = now;

        float current = currentSensor.read();
        float voltage = voltageSensor.read();
        float rawADC  = voltageSensor.readRaw();

        // Formatted output suitable for Serial Plotter or data logging.
        Serial.print(F("I="));
        Serial.print(current, 3);
        Serial.print(F("A  V="));
        Serial.print(voltage, 2);
        Serial.print(F("V  RAW="));
        Serial.print(rawADC, 1);
        Serial.print(F("  CAL_PTS="));
        Serial.println(voltageSensor.getCalibrationPointCount());
    }
}
