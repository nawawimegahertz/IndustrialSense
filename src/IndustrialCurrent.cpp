/**
 * ============================================================================
 * IndustrialCurrent.cpp — Current Sensor Implementation
 * ============================================================================
 * 
 * Implements the IndustrialCurrent class with:
 *   1. Adaptive Kalman Filter (simplified EKF-inspired)
 *   2. Oversampling with decimation (default 64 samples)
 *   3. Smart deadzone for near-zero readings
 *   4. Auto-zero thermal drift compensation
 *   5. Cross-platform persistent offset storage
 * 
 * LICENSE: MIT
 * ============================================================================
 */

#include "IndustrialSense.h"

// ============================================================================
//  Constructor
// ============================================================================
IndustrialCurrent::IndustrialCurrent(SensorModel model, uint8_t pin)
    : _pin(pin)
    , _model(model)
    , _sensitivity(0.0f)
    , _zeroOffset(0.0f)
    , _customADC(nullptr)
    , _adcBits(10)
    , _adcVRef(5.0f)
    , _adcMaxValue(1023)
    , _oversampleCount(64)
    , _kalmanX(0.0f)
    , _kalmanP(1.0f)       // Initial covariance — moderate uncertainty
    , _kalmanQ(0.01f)      // Process noise — small for slow-changing systems
    , _kalmanR(0.5f)       // Measurement noise — moderate for noisy ADC
    , _kalmanRho(0.5f)     // Innovation threshold for adaptive skip
    , _deadzone(0.20f)     // 200 mA deadzone
    , _autoZeroInterval(5000)
    , _lastNonZeroTime(0)
    , _autoZeroEnabled(true)
    , _lastCurrent(0.0f)
{
    _loadModelDefaults();
}

// ============================================================================
//  begin() — Initialize the sensor
// ============================================================================
void IndustrialCurrent::begin() {
    // Auto-detect platform defaults for ADC resolution and Vref.
#if defined(ESP32)
    _adcBits     = 12;
    _adcVRef     = 3.3f;
    _adcMaxValue = 4095;
    // ESP32 ADC resolution is set globally; we configure it here for safety.
    analogReadResolution(_adcBits);
#else
    // AVR / default: 10-bit ADC, 5V reference.
    _adcBits     = 10;
    _adcVRef     = 5.0f;
    _adcMaxValue = 1023;
#endif

    // Seed the Kalman filter with the first raw reading so the filter
    // doesn't need to "catch up" from zero on the first call.
    float initial = _oversampleAndDecimate();
    _kalmanX = initial;
    _zeroOffset = initial;  // Assume 0A at power-on (will be overridden by loadCalibration)

    // Attempt to load a previously-saved offset from persistent storage.
    loadCalibration();

    _lastNonZeroTime = millis();
}

// ============================================================================
//  read() — Full measurement pipeline
// ============================================================================
float IndustrialCurrent::read() {
    // STEP 1: Oversample and decimate — take N raw ADC samples and average.
    float rawAvg = _oversampleAndDecimate();

    // STEP 2: Adaptive Kalman filter — smooth the oversampled reading.
    float filtered = _applyKalman(rawAvg);

    // STEP 3: Convert filtered ADC value to current in Amperes.
    float current = _adcToCurrent(filtered);

    // STEP 4: Deadzone — force near-zero readings to exactly 0.0.
    if (abs(current) < _deadzone) {
        current = 0.0f;
    }

    // STEP 5: Auto-zero check — if stable at 0A for long enough, recalibrate.
    _checkAutoZero(current);

    // Cache and return.
    _lastCurrent = current;
    return current;
}

// ============================================================================
//  readRaw() — Raw oversampled ADC value (no Kalman, no conversion)
// ============================================================================
float IndustrialCurrent::readRaw() {
    return _oversampleAndDecimate();
}

// ============================================================================
//  getCurrent() — Return last computed value without new measurement
// ============================================================================
float IndustrialCurrent::getCurrent() const {
    return _lastCurrent;
}

// ============================================================================
//  Configuration Setters
// ============================================================================
void IndustrialCurrent::setCustomADCFunction(CustomADCReadFunc func) {
    _customADC = func;
}

void IndustrialCurrent::setAdcResolution(uint8_t bits) {
    _adcBits = bits;
    _adcMaxValue = (1 << bits) - 1;
#if defined(ESP32)
    analogReadResolution(bits);
#endif
}

void IndustrialCurrent::setAdcVoltageRef(float vRef) {
    _adcVRef = vRef;
}

void IndustrialCurrent::setKalmanProcessNoise(float Q) {
    _kalmanQ = Q;
}

void IndustrialCurrent::setKalmanMeasurementNoise(float R) {
    _kalmanR = R;
}

void IndustrialCurrent::setKalmanInnovationThreshold(float rho) {
    _kalmanRho = rho;
}

void IndustrialCurrent::setOversampleCount(uint16_t count) {
    _oversampleCount = (count > 0) ? count : 1;
}

void IndustrialCurrent::setDeadzone(float amps) {
    _deadzone = abs(amps);
}

void IndustrialCurrent::setAutoZeroInterval(uint32_t ms) {
    _autoZeroInterval = ms;
}

// ============================================================================
//  forceAutoZero() — Immediately recalibrate the zero offset
// ============================================================================
void IndustrialCurrent::forceAutoZero() {
    // Take a fresh oversampled reading and declare it as the new zero point.
    _zeroOffset = _oversampleAndDecimate();
    // Reset the Kalman state to the new offset to avoid transient.
    _kalmanX = _zeroOffset;
    _kalmanP = 1.0f;
}

// ============================================================================
//  Persistence
// ============================================================================
void IndustrialCurrent::saveCalibration() {
    IndustrialSenseStorage::saveCurrentOffset(_zeroOffset);
}

void IndustrialCurrent::loadCalibration() {
    // Use the current _zeroOffset as the default if nothing is stored.
    _zeroOffset = IndustrialSenseStorage::loadCurrentOffset(_zeroOffset);
}

void IndustrialCurrent::resetCalibration() {
    IndustrialSenseStorage::resetAllCalibration();
    // Re-read the current ADC as the new zero offset.
    _zeroOffset = _oversampleAndDecimate();
    _kalmanX = _zeroOffset;
    _kalmanP = 1.0f;
}

// ============================================================================
//  processCommand() — Dynamic command parser for runtime calibration
// ============================================================================
bool IndustrialCurrent::processCommand(const String& cmd) {
    String trimmed = cmd;
    trimmed.trim();
    trimmed.toLowerCase();

    if (trimmed == "auto") {
        // ----- AUTO-ZERO COMMAND -----
        // Read the current ADC value as the new zero offset and persist it.
        forceAutoZero();
        saveCalibration();
        return true;
    }
    else if (trimmed == "reset") {
        // ----- RESET COMMAND -----
        // Erase persisted calibration data and re-baseline.
        resetCalibration();
        return true;
    }

    return false;  // Command not recognized
}

// ============================================================================
//  PRIVATE: _readADC() — Single ADC read (custom function or analogRead)
// ============================================================================
int IndustrialCurrent::_readADC() {
    if (_customADC != nullptr) {
        return _customADC(_pin);
    }
    return analogRead(_pin);
}

// ============================================================================
//  PRIVATE: _oversampleAndDecimate()
//  Collects N ADC samples and returns the arithmetic mean.
// 
//  WHY OVERSAMPLE?
//    A single ADC read on most microcontrollers has ±2 LSB noise.
//    By averaging 64 samples, we effectively gain ~3 extra bits of
//    resolution (log₂(√64) = 3). The cost is ~640μs at 10μs/sample,
//    which is negligible for most power monitoring applications.
// ============================================================================
float IndustrialCurrent::_oversampleAndDecimate() {
    uint32_t accumulator = 0;
    for (uint16_t i = 0; i < _oversampleCount; i++) {
        accumulator += (uint32_t)_readADC();
        // Minimal inter-sample delay to let the ADC sample-and-hold settle.
        // 10μs is sufficient for most SAR ADCs (ESP32, AVR).
        delayMicroseconds(10);
    }
    return (float)accumulator / (float)_oversampleCount;
}

// ============================================================================
//  PRIVATE: _applyKalman() — Adaptive Kalman Filter (1-D)
// 
//  This is the heart of the noise rejection system.
// 
//  STANDARD KALMAN EQUATIONS:
//    Predict:
//      x̂⁻ = x̂(k-1)           // Our system has no dynamics model, so
//      P⁻  = P(k-1) + Q        // the prediction is just the previous state.
// 
//    Update:
//      innovation = z - x̂⁻     // Difference between measurement and prediction
//      K = P⁻ / (P⁻ + R)       // Kalman gain ∈ [0, 1]
//      x̂ = x̂⁻ + K * innovation // Corrected state
//      P = (1 - K) * P⁻         // Corrected covariance
// 
//  ADAPTIVE EXTENSION:
//    If |innovation| < rho:
//      → We consider the measurement "consistent" with the current state.
//      → We still update x̂ but with a SMALLER effective K (clamped to 0.01).
//      → We do NOT grow P, preventing the filter from becoming responsive
//        to noise during quiet periods.
//    This dramatically improves 0A stability while preserving transient response.
// ============================================================================
float IndustrialCurrent::_applyKalman(float measurement) {
    // --- PREDICT STEP ---
    // x̂⁻ remains _kalmanX (no dynamics model).
    // P⁻ = P + Q (uncertainty grows due to process noise).
    float P_predicted = _kalmanP + _kalmanQ;

    // --- INNOVATION ---
    float innovation = measurement - _kalmanX;

    // --- ADAPTIVE LOGIC ---
    if (abs(innovation) < _kalmanRho) {
        // Small innovation → measurement is consistent with state.
        // Apply a very gentle correction to avoid drift, but don't
        // grow the covariance (keeps the filter "locked" during stable periods).
        _kalmanX += 0.01f * innovation;
        // P stays at _kalmanP (no update) — intentionally keeping it low.
    }
    else {
        // Significant innovation → full Kalman update.
        float K = P_predicted / (P_predicted + _kalmanR);  // Kalman gain
        _kalmanX = _kalmanX + K * innovation;              // Corrected state
        _kalmanP = (1.0f - K) * P_predicted;               // Corrected covariance
    }

    return _kalmanX;
}

// ============================================================================
//  PRIVATE: _adcToCurrent() — Convert filtered ADC value to Amperes
// 
//  MATH:
//    sensorVoltage = (filteredADC / maxADC) × Vref     [Volts at sensor VOUT]
//    offsetVoltage = (zeroOffset  / maxADC) × Vref     [Volts at 0A]
//    current       = (sensorVoltage - offsetVoltage) / (sensitivity / 1000)
// 
//  We combine the operations to minimize floating-point divisions:
//    current = ((filteredADC - zeroOffset) × Vref) / (maxADC × sensitivity_V)
// ============================================================================
float IndustrialCurrent::_adcToCurrent(float adcFiltered) {
    float sensitivity_V = _sensitivity / 1000.0f;  // Convert mV/A to V/A
    float deltaADC = adcFiltered - _zeroOffset;
    float deltaVolts = (deltaADC * _adcVRef) / (float)_adcMaxValue;
    return deltaVolts / sensitivity_V;
}

// ============================================================================
//  PRIVATE: _checkAutoZero() — Auto-zero thermal drift compensation
// 
//  STRATEGY:
//    Track how long the current has been within the deadzone (effectively 0A).
//    If it stays at 0A for longer than _autoZeroInterval milliseconds,
//    we assume the load is truly disconnected and any non-zero ADC reading
//    is due to thermal drift of the sensor's internal offset.
//    We then recalibrate the zero offset automatically.
// ============================================================================
void IndustrialCurrent::_checkAutoZero(float current) {
    if (!_autoZeroEnabled) return;

    if (abs(current) > _deadzone) {
        // Non-zero current detected — reset the timer.
        _lastNonZeroTime = millis();
    }
    else {
        // Current is in the deadzone (effectively 0A).
        uint32_t elapsed = millis() - _lastNonZeroTime;
        if (elapsed >= _autoZeroInterval) {
            // Stable at 0A for long enough — recalibrate.
            forceAutoZero();
            saveCalibration();
            // Reset the timer to prevent continuous re-calibration.
            _lastNonZeroTime = millis();
        }
    }
}

// ============================================================================
//  PRIVATE: _loadModelDefaults() — Initialize sensitivity from enum
// 
//  Sensitivity values are from the official datasheets:
//    ACS712-05A:  185 mV/A   (highest sensitivity, lowest current range)
//    ACS712-20A:  100 mV/A
//    ACS712-30A:   66 mV/A
//    ACS758-50B:   40 mV/A   (bidirectional ±50A variant)
//    ACS758-100U:  20 mV/A   (unidirectional 0-100A variant)
// ============================================================================
void IndustrialCurrent::_loadModelDefaults() {
    switch (_model) {
        case ACS712_05A:   _sensitivity = 185.0f;  break;
        case ACS712_20A:   _sensitivity = 100.0f;  break;
        case ACS712_30A:   _sensitivity =  66.0f;  break;
        case ACS758_50B:   _sensitivity =  40.0f;  break;
        case ACS758_100U:  _sensitivity =  20.0f;  break;
        default:           _sensitivity = 100.0f;  break;  // Safe fallback
    }
}
