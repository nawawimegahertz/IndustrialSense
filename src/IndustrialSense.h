/**
 * ============================================================================
 * IndustrialSense.h — Core Header File
 * ============================================================================
 * 
 * Industrial-grade power monitoring library for Arduino & ESP32.
 * 
 * DESIGN PHILOSOPHY: "Progressive Disclosure"
 *   Level 1 — Beginners:  3 lines of code to get a reading.
 *   Level 2 — Pros:       Full access to Kalman tuning, non-linear calibration,
 *                          custom ADC injection, and runtime serial commands.
 * 
 * ARCHITECTURE:
 *   ┌─────────────────────────────────────────────────────────────────────┐
 *   │                      IndustrialSense.h (you are here)              │
 *   │  Aggregates all public-facing classes into a single include.       │
 *   ├─────────────────────┬───────────────────────┬───────────────────────┤
 *   │ IndustrialCurrent   │  IndustrialVoltage    │  StorageManager.h     │
 *   │ (Adaptive Kalman,   │  (Multi-point NL cal, │  (ESP32 Preferences / │
 *   │  Oversampling,      │   Segmented Linear    │   AVR EEPROM, with    │
 *   │  Auto-Zero,         │   Interpolation, EMA) │   wear-level logic)   │
 *   │  Deadzone)          │                       │                       │
 *   └─────────────────────┴───────────────────────┴───────────────────────┘
 * 
 * QUICK START:
 *   #include <IndustrialSense.h>
 * 
 *   IndustrialCurrent current(ACS712_20A, A0);
 *   IndustrialVoltage  voltage(A1, 30000.0, 7500.0);  // R1=30k, R2=7.5k
 * 
 *   void setup() { current.begin(); voltage.begin(); }
 *   void loop()  { Serial.println(current.read()); Serial.println(voltage.read()); }
 * 
 * LICENSE: MIT
 * ============================================================================
 */

#ifndef INDUSTRIAL_SENSE_H
#define INDUSTRIAL_SENSE_H

#include <Arduino.h>
#include "StorageManager.h"

// ============================================================================
//  Sensor Model Enumeration
// ============================================================================
/**
 * Predefined current sensor profiles.
 * Each entry auto-loads the correct sensitivity (mV/A) and nominal
 * zero-current offset voltage.
 * 
 * SENSITIVITY VALUES (from datasheets):
 *   ACS712-05A  →  185 mV/A   (most sensitive, for small currents)
 *   ACS712-20A  →  100 mV/A
 *   ACS712-30A  →   66 mV/A
 *   ACS758-50B  →   40 mV/A   (bidirectional ±50A)
 *   ACS758-100U →   20 mV/A   (unidirectional 0..100A)
 */
enum SensorModel : uint8_t {
    ACS712_05A,
    ACS712_20A,
    ACS712_30A,
    ACS758_50B,
    ACS758_100U
};


// ============================================================================
//  Type alias for custom ADC read function injection.
//  Signature: int myCustomRead(uint8_t pin)
//  This allows users to swap in external ADC chips (e.g., ADS1115).
// ============================================================================
typedef int (*CustomADCReadFunc)(uint8_t pin);


// ############################################################################
//  CLASS: IndustrialCurrent
// ############################################################################
/**
 * High-fidelity current measurement with:
 *   • Adaptive Kalman Filter (simplified EKF-inspired)
 *   • Configurable oversampling + decimation
 *   • Smart deadzone (readings < threshold → forced 0.0 A)
 *   • Auto-zero thermal drift compensation
 *   • Persistent offset storage
 * 
 * KALMAN FILTER THEORY (simplified):
 *   The standard 1-D Kalman equations are:
 * 
 *     Predict:
 *       x̂⁻  = x̂(k-1)               (state estimate doesn't change, no model)
 *       P⁻   = P(k-1) + Q            (error covariance grows by process noise Q)
 * 
 *     Update:
 *       innovation = z(k) - x̂⁻       (measurement residual)
 *       K = P⁻ / (P⁻ + R)            (Kalman gain)
 *       x̂  = x̂⁻ + K * innovation    (corrected state)
 *       P  = (1 - K) * P⁻             (corrected covariance)
 * 
 *   ADAPTIVE EXTENSION:
 *     If |innovation| < rho (a user-tunable threshold), we SKIP the heavy
 *     covariance update. This:
 *       1. Saves CPU cycles on stable (0A) readings.
 *       2. Prevents the filter from "chasing" noise near zero.
 *       3. Dramatically stabilizes idle current display.
 */
class IndustrialCurrent {
public:
    // --------------------------------------------------------------------
    //  Constructor
    //  model:  One of the SensorModel enum values (auto-loads sensitivity).
    //  pin:    The analog input pin connected to the sensor's VOUT.
    // --------------------------------------------------------------------
    IndustrialCurrent(SensorModel model, uint8_t pin);

    // --------------------------------------------------------------------
    //  begin()
    //  Initializes the sensor. Call this in setup().
    //  Loads the persisted zero-offset from EEPROM/Preferences if available.
    // --------------------------------------------------------------------
    void begin();

    // --------------------------------------------------------------------
    //  read()
    //  Performs a full measurement cycle:
    //    1. Oversample N raw ADC readings and average (decimation).
    //    2. Run through the Adaptive Kalman Filter.
    //    3. Convert to Amperes.
    //    4. Apply deadzone.
    //    5. Check auto-zero conditions.
    //  Returns the filtered current in Amperes.
    // --------------------------------------------------------------------
    float read();

    // --------------------------------------------------------------------
    //  readRaw()
    //  Returns the raw oversampled+averaged ADC value (before Kalman).
    //  Useful for diagnostics and calibration.
    // --------------------------------------------------------------------
    float readRaw();

    // --------------------------------------------------------------------
    //  getCurrent()
    //  Returns the last computed current value without performing a new read.
    // --------------------------------------------------------------------
    float getCurrent() const;

    // ======================== ADVANCED CONFIGURATION ========================

    /**
     * Inject a custom ADC reading function.
     * Example for ADS1115:
     *   int myRead(uint8_t pin) { return ads.readADC_SingleEnded(pin); }
     *   current.setCustomADCFunction(myRead);
     */
    void setCustomADCFunction(CustomADCReadFunc func);

    /** Set ADC resolution in bits (default 10 for AVR, 12 for ESP32). */
    void setAdcResolution(uint8_t bits);

    /** Set ADC reference voltage in Volts (default 5.0 for AVR, 3.3 for ESP32). */
    void setAdcVoltageRef(float vRef);

    // ----- Kalman Tuning -----
    /** Process noise Q — higher = trust measurements more, noisier output. Default: 0.01 */
    void setKalmanProcessNoise(float Q);

    /** Measurement noise R — higher = trust model more, smoother but slower. Default: 0.5 */
    void setKalmanMeasurementNoise(float R);

    /**
     * Innovation threshold (rho).
     * If |innovation| < rho, we skip the covariance update to save CPU.
     * Default: 0.5 ADC units. Set to 0 to disable adaptive behavior.
     */
    void setKalmanInnovationThreshold(float rho);

    // ----- Oversampling -----
    /** Set the number of ADC samples to average per read() call. Default: 64. */
    void setOversampleCount(uint16_t count);

    // ----- Deadzone -----
    /** Set the deadzone threshold in Amperes. Readings below this → 0.0A. Default: 0.20 */
    void setDeadzone(float amps);

    // ----- Auto-Zero -----
    /** Set the duration (ms) of stable 0A readings before auto-zero triggers. Default: 5000 */
    void setAutoZeroInterval(uint32_t ms);

    /** Force an immediate auto-zero calibration (reads current ADC as zero offset). */
    void forceAutoZero();

    // ----- Persistence -----
    /** Save the current zero-offset to non-volatile memory. */
    void saveCalibration();

    /** Load the zero-offset from non-volatile memory. */
    void loadCalibration();

    /** Reset/erase the persisted zero-offset. */
    void resetCalibration();

    // ----- Command Parser -----
    /**
     * processCommand() — parse a string command for runtime calibration.
     * Supported commands:
     *   "auto"   → Forces auto-zero and saves offset.
     *   "reset"  → Clears the persisted offset.
     * Returns true if the command was recognized and processed.
     */
    bool processCommand(const String& cmd);

private:
    // Hardware
    uint8_t         _pin;
    SensorModel     _model;
    float           _sensitivity;     // mV/A (from datasheet)
    float           _zeroOffset;      // ADC value at 0A (calibrated)
    CustomADCReadFunc _customADC;     // nullptr = use analogRead()

    // ADC parameters
    uint8_t         _adcBits;         // Resolution in bits
    float           _adcVRef;         // Reference voltage
    uint16_t        _adcMaxValue;     // 2^bits - 1

    // Oversampling
    uint16_t        _oversampleCount;

    // Kalman filter state
    float           _kalmanX;         // State estimate (ADC units)
    float           _kalmanP;         // Error covariance
    float           _kalmanQ;         // Process noise
    float           _kalmanR;         // Measurement noise
    float           _kalmanRho;       // Innovation threshold for adaptive skip

    // Deadzone
    float           _deadzone;        // Amperes — below this → 0.0

    // Auto-zero
    uint32_t        _autoZeroInterval; // Duration of stable zero before auto-cal
    uint32_t        _lastNonZeroTime;  // millis() of last non-zero reading
    bool            _autoZeroEnabled;

    // Output cache
    float           _lastCurrent;

    // Internal helpers
    int             _readADC();                       // Single ADC read (custom or analogRead)
    float           _oversampleAndDecimate();          // Average N readings
    float           _applyKalman(float measurement);   // Adaptive Kalman step
    float           _adcToCurrent(float adcFiltered);  // Convert filtered ADC to Amps
    void            _checkAutoZero(float current);     // Auto-zero state machine
    void            _loadModelDefaults();              // Load sensitivity from enum
};


// ############################################################################
//  CLASS: IndustrialVoltage
// ############################################################################
/**
 * High-precision voltage measurement with:
 *   • Resistor divider math (configurable R1/R2)
 *   • Multi-point non-linear calibration (up to 10 points)
 *   • Segmented linear interpolation for measured→real voltage mapping
 *   • Exponential Moving Average (EMA) for display anti-flicker
 *   • Persistent calibration map storage
 * 
 * RESISTOR DIVIDER MATH:
 *   Vout = Vin × R2 / (R1 + R2)
 *   Therefore:  Vin = Vout × (R1 + R2) / R2
 *   In ADC terms: Vout = (ADC / maxADC) × Vref
 *                 Vin  = Vout × dividerRatio
 *   Where dividerRatio = (R1 + R2) / R2
 * 
 * SEGMENTED LINEAR INTERPOLATION:
 *   Given sorted calibration points [(adc₁,v₁), (adc₂,v₂), ...],
 *   for a measured ADC value x between adcᵢ and adcᵢ₊₁:
 *     voltage = vᵢ + (x - adcᵢ) × (vᵢ₊₁ - vᵢ) / (adcᵢ₊₁ - adcᵢ)
 * 
 *   Below the first point → linear extrapolation from first two points.
 *   Above the last point  → linear extrapolation from last two points.
 * 
 * EMA (Exponential Moving Average):
 *   EMA(k) = α × sample(k) + (1 - α) × EMA(k-1)
 *   Where α ∈ (0, 1]. Higher α = less smoothing, faster response.
 *   Default α = 0.15 — tuned for ~60Hz display refresh without flicker.
 */
class IndustrialVoltage {
public:
    // --------------------------------------------------------------------
    //  Constructor
    //  pin:  Analog input pin.
    //  R1:   Top resistor of the voltage divider (Ohms).
    //  R2:   Bottom resistor of the voltage divider (Ohms).
    // --------------------------------------------------------------------
    IndustrialVoltage(uint8_t pin, float R1, float R2);

    // --------------------------------------------------------------------
    //  begin()
    //  Initializes the sensor. Call this in setup().
    //  Loads any persisted calibration map from EEPROM/Preferences.
    // --------------------------------------------------------------------
    void begin();

    // --------------------------------------------------------------------
    //  read()
    //  Performs a full measurement cycle:
    //    1. Oversample raw ADC.
    //    2. If calibration points exist → segmented linear interpolation.
    //    3. Else → standard resistor divider math.
    //    4. Apply EMA smoothing.
    //  Returns the smoothed voltage in Volts.
    // --------------------------------------------------------------------
    float read();

    // --------------------------------------------------------------------
    //  readRaw()
    //  Returns the raw oversampled+averaged ADC value.
    // --------------------------------------------------------------------
    float readRaw();

    // --------------------------------------------------------------------
    //  getVoltage()
    //  Returns the last computed voltage without performing a new read.
    // --------------------------------------------------------------------
    float getVoltage() const;

    // --------------------------------------------------------------------
    //  getFilteredVoltage()
    //  Returns the EMA-smoothed voltage (same as last read() result).
    // --------------------------------------------------------------------
    float getFilteredVoltage() const;

    // ======================== ADVANCED CONFIGURATION ========================

    /** Inject a custom ADC reading function (e.g., for ADS1115). */
    void setCustomADCFunction(CustomADCReadFunc func);

    /** Set ADC resolution in bits. */
    void setAdcResolution(uint8_t bits);

    /** Set ADC reference voltage in Volts. */
    void setAdcVoltageRef(float vRef);

    /** Set oversampling count. Default: 32. */
    void setOversampleCount(uint16_t count);

    // ----- Calibration -----
    /**
     * addCalibrationPoint()
     * Reads the current raw ADC value and maps it to the user-supplied
     * real voltage. The internal map is automatically sorted by rawADC
     * after every insertion.
     * 
     * Returns false if the map is full (MAX_CAL_POINTS reached).
     */
    bool addCalibrationPoint(float realVoltage);

    /**
     * addCalibrationPoint() — explicit raw value version.
     * Allows specifying both rawADC and realVoltage directly.
     */
    bool addCalibrationPoint(float rawADC, float realVoltage);

    /** Clear all calibration points from RAM (does NOT erase storage). */
    void clearCalibration();

    /** Get the number of active calibration points. */
    uint8_t getCalibrationPointCount() const;

    // ----- EMA Tuning -----
    /**
     * Set the EMA smoothing factor α.
     * Range: (0.0, 1.0]. Higher = less smoothing.
     * Default: 0.15
     */
    void setEmaAlpha(float alpha);

    // ----- Persistence -----
    /** Save the calibration map to non-volatile memory. */
    void saveCalibration();

    /** Load the calibration map from non-volatile memory. */
    void loadCalibration();

    /** Clear calibration from both RAM and persistent storage. */
    void resetCalibration();

    // ----- Command Parser -----
    /**
     * processCommand() — parse a string command for runtime calibration.
     * Supported commands:
     *   "v<value>"  → e.g., "v12.5" — reads current ADC, maps to 12.5V, saves.
     *   "reset"     → Clears calibration map from RAM and storage.
     * Returns true if the command was recognized and processed.
     */
    bool processCommand(const String& cmd);

private:
    // Hardware
    uint8_t             _pin;
    float               _R1, _R2;           // Divider resistors (Ohms)
    float               _dividerRatio;       // (R1 + R2) / R2
    CustomADCReadFunc   _customADC;

    // ADC parameters
    uint8_t             _adcBits;
    float               _adcVRef;
    uint16_t            _adcMaxValue;

    // Oversampling
    uint16_t            _oversampleCount;

    // Calibration map
    CalibrationPoint    _calPoints[MAX_CAL_POINTS];
    uint8_t             _calCount;

    // EMA state
    float               _emaAlpha;
    float               _emaValue;
    bool                _emaInitialized;

    // Output cache
    float               _lastVoltage;

    // Internal helpers
    int                 _readADC();
    float               _oversampleAndDecimate();
    void                _sortCalibrationPoints();
    float               _interpolate(float rawADC);
    float               _dividerMath(float rawADC);
    float               _applyEMA(float value);
};

#endif // INDUSTRIAL_SENSE_H
