/**
 * ============================================================================
 * 01_Basic_Usage.ino — IndustrialSense Quick Start
 * ============================================================================
 * 
 * LEVEL 1: "3 Lines of Code" Simplicity
 * 
 * This example demonstrates the simplest possible usage of the
 * IndustrialSense library. Just create the sensor objects, call begin(),
 * and read() — the library handles everything else internally:
 *   ✓ Oversampling & decimation for noise reduction
 *   ✓ Adaptive Kalman filtering (current sensor)
 *   ✓ EMA smoothing (voltage sensor)
 *   ✓ Auto-zero thermal compensation (current sensor)
 *   ✓ Deadzone for near-zero readings
 * 
 * WIRING:
 *   ACS712-20A VOUT → A0
 *   Voltage Divider  → A1  (R1 = 30kΩ to Vin, R2 = 7.5kΩ to GND)
 *   
 *   Voltage Divider Schematic:
 *     Vin ───[R1=30kΩ]───┬───[R2=7.5kΩ]───GND
 *                        │
 *                        └──→ A1 (to MCU ADC)
 *   
 *   Max measurable voltage = Vref × (R1+R2)/R2 = 5.0 × 5.0 = 25.0V
 *   (For 3.3V MCUs like ESP32: max = 3.3 × 5.0 = 16.5V)
 * 
 * LICENSE: MIT
 * ============================================================================
 */

#include <IndustrialSense.h>

// ============================================================================
//  SENSOR DECLARATIONS
//  This is all you need — the library auto-loads sensitivity, offset,
//  filter parameters, and platform-specific defaults.
// ============================================================================

// Current sensor: ACS712-20A variant, connected to analog pin A0.
IndustrialCurrent currentSensor(ACS712_20A, A0);

// Voltage sensor: Resistor divider with R1=30kΩ, R2=7.5kΩ, on pin A1.
//   Divider ratio = (30000 + 7500) / 7500 = 5.0x
IndustrialVoltage voltageSensor(A1, 30000.0, 7500.0);

// ============================================================================
//  SETUP
// ============================================================================
void setup() {
    Serial.begin(115200);
    while (!Serial) { ; }  // Wait for Serial on USB-native boards (Leonardo, etc.)

    Serial.println(F("========================================"));
    Serial.println(F("  IndustrialSense — Basic Usage Demo"));
    Serial.println(F("========================================"));
    Serial.println();

    // Initialize both sensors.
    // begin() will:
    //   - Auto-detect ADC resolution & Vref for your board
    //   - Seed the Kalman filter / EMA with an initial reading
    //   - Load any saved calibration from EEPROM/Preferences
    currentSensor.begin();
    voltageSensor.begin();

    Serial.println(F("[OK] Sensors initialized."));
    Serial.println(F("[INFO] Reading current (A0) and voltage (A1) every 500ms."));
    Serial.println();
}

// ============================================================================
//  LOOP — Read and print sensor values every 500ms
// ============================================================================
void loop() {
    // read() performs the ENTIRE pipeline internally:
    //   Oversample → Filter → Convert → Deadzone/Smooth
    float current = currentSensor.read();
    float voltage = voltageSensor.read();

    // Print with formatted output.
    Serial.print(F("Current: "));
    Serial.print(current, 2);   // 2 decimal places
    Serial.print(F(" A   |   Voltage: "));
    Serial.print(voltage, 2);
    Serial.println(F(" V"));

    // Non-blocking delay — in a real application you would use millis()
    // based timing instead of delay(), but for this basic demo it's fine.
    delay(500);
}
