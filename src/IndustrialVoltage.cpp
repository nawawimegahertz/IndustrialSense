/**
 * ============================================================================
 * IndustrialVoltage.cpp — Voltage Sensor Implementation
 * ============================================================================
 * 
 * Implements the IndustrialVoltage class with:
 *   1. Resistor divider math (configurable R1/R2)
 *   2. Multi-point non-linear calibration (up to 10 points)
 *   3. Segmented linear interpolation for ADC→Voltage mapping
 *   4. Exponential Moving Average (EMA) for display anti-flicker
 *   5. Cross-platform persistent calibration storage
 * 
 * LICENSE: MIT
 * ============================================================================
 */

#include "IndustrialSense.h"

// ============================================================================
//  Constructor
// ============================================================================
IndustrialVoltage::IndustrialVoltage(uint8_t pin, float R1, float R2)
    : _pin(pin)
    , _R1(R1)
    , _R2(R2)
    , _dividerRatio((R1 + R2) / R2)    // Pre-compute for efficiency
    , _customADC(nullptr)
    , _adcBits(10)
    , _adcVRef(5.0f)
    , _adcMaxValue(1023)
    , _oversampleCount(32)
    , _calCount(0)
    , _emaAlpha(0.15f)
    , _emaValue(0.0f)
    , _emaInitialized(false)
    , _lastVoltage(0.0f)
{
    // Zero-initialize the calibration array for safety.
    memset(_calPoints, 0, sizeof(_calPoints));
}

// ============================================================================
//  begin() — Initialize the sensor
// ============================================================================
void IndustrialVoltage::begin() {
    // Auto-detect platform defaults.
#if defined(ESP32)
    _adcBits     = 12;
    _adcVRef     = 3.3f;
    _adcMaxValue = 4095;
    analogReadResolution(_adcBits);
#else
    _adcBits     = 10;
    _adcVRef     = 5.0f;
    _adcMaxValue = 1023;
#endif

    // Attempt to load persisted calibration data.
    loadCalibration();

    // Seed the EMA with an initial reading to avoid cold-start transients.
    float rawAvg = _oversampleAndDecimate();
    if (_calCount >= 2) {
        _emaValue = _interpolate(rawAvg);
    } else {
        _emaValue = _dividerMath(rawAvg);
    }
    _emaInitialized = true;
}

// ============================================================================
//  read() — Full measurement pipeline
// ============================================================================
float IndustrialVoltage::read() {
    // STEP 1: Oversample and decimate.
    float rawAvg = _oversampleAndDecimate();

    // STEP 2: Convert to voltage — use calibration if available, else divider math.
    float voltage;
    if (_calCount >= 2) {
        // We have enough calibration points for segmented linear interpolation.
        voltage = _interpolate(rawAvg);
    } else {
        // Fall back to ideal resistor divider math.
        voltage = _dividerMath(rawAvg);
    }

    // Clamp negative voltages to zero (physically impossible for our use case).
    if (voltage < 0.0f) voltage = 0.0f;

    // STEP 3: Apply EMA smoothing.
    voltage = _applyEMA(voltage);

    // Cache and return.
    _lastVoltage = voltage;
    return voltage;
}

// ============================================================================
//  readRaw() — Raw oversampled ADC value
// ============================================================================
float IndustrialVoltage::readRaw() {
    return _oversampleAndDecimate();
}

// ============================================================================
//  getVoltage() / getFilteredVoltage()
// ============================================================================
float IndustrialVoltage::getVoltage() const {
    return _lastVoltage;
}

float IndustrialVoltage::getFilteredVoltage() const {
    return _lastVoltage;  // The stored value is already EMA-filtered.
}

// ============================================================================
//  Configuration Setters
// ============================================================================
void IndustrialVoltage::setCustomADCFunction(CustomADCReadFunc func) {
    _customADC = func;
}

void IndustrialVoltage::setAdcResolution(uint8_t bits) {
    _adcBits = bits;
    _adcMaxValue = (1 << bits) - 1;
#if defined(ESP32)
    analogReadResolution(bits);
#endif
}

void IndustrialVoltage::setAdcVoltageRef(float vRef) {
    _adcVRef = vRef;
}

void IndustrialVoltage::setOversampleCount(uint16_t count) {
    _oversampleCount = (count > 0) ? count : 1;
}

void IndustrialVoltage::setEmaAlpha(float alpha) {
    // Clamp to valid range to prevent NaN or runaway values.
    if (alpha < 0.001f) alpha = 0.001f;
    if (alpha > 1.0f)   alpha = 1.0f;
    _emaAlpha = alpha;
}

// ============================================================================
//  addCalibrationPoint(realVoltage)
//  Auto-reads the current ADC value and maps it to the user-supplied voltage.
// ============================================================================
bool IndustrialVoltage::addCalibrationPoint(float realVoltage) {
    if (_calCount >= MAX_CAL_POINTS) return false;  // Map is full

    float rawADC = _oversampleAndDecimate();
    return addCalibrationPoint(rawADC, realVoltage);
}

// ============================================================================
//  addCalibrationPoint(rawADC, realVoltage) — Explicit version
// ============================================================================
bool IndustrialVoltage::addCalibrationPoint(float rawADC, float realVoltage) {
    if (_calCount >= MAX_CAL_POINTS) return false;

    // Insert the new point at the end.
    _calPoints[_calCount].rawADC    = rawADC;
    _calPoints[_calCount].realValue = realVoltage;
    _calCount++;

    // Sort the array by rawADC (ascending) after every insertion.
    // This is critical for the binary search in _interpolate().
    _sortCalibrationPoints();

    return true;
}

// ============================================================================
//  clearCalibration() — Remove all points from RAM only
// ============================================================================
void IndustrialVoltage::clearCalibration() {
    _calCount = 0;
    memset(_calPoints, 0, sizeof(_calPoints));
}

uint8_t IndustrialVoltage::getCalibrationPointCount() const {
    return _calCount;
}

// ============================================================================
//  Persistence
// ============================================================================
void IndustrialVoltage::saveCalibration() {
    IndustrialSenseStorage::saveVoltageCalibration(_calPoints, _calCount);
}

void IndustrialVoltage::loadCalibration() {
    _calCount = IndustrialSenseStorage::loadVoltageCalibration(_calPoints, MAX_CAL_POINTS);
    if (_calCount > 1) {
        _sortCalibrationPoints();  // Ensure loaded data is sorted.
    }
}

void IndustrialVoltage::resetCalibration() {
    clearCalibration();
    IndustrialSenseStorage::resetAllCalibration();
}

// ============================================================================
//  processCommand() — Dynamic command parser
// 
//  FORMAT:
//    "v12.5"  → Reads current ADC, maps it to 12.5V, saves to storage.
//    "reset"  → Clears calibration from RAM and persistent storage.
// ============================================================================
bool IndustrialVoltage::processCommand(const String& cmd) {
    String trimmed = cmd;
    trimmed.trim();

    // ----- VOLTAGE CALIBRATION COMMAND: "v<value>" -----
    if (trimmed.charAt(0) == 'v' || trimmed.charAt(0) == 'V') {
        String valueStr = trimmed.substring(1);
        float realVoltage = valueStr.toFloat();

        // Sanity check: toFloat() returns 0.0 on failure,
        // but "v0" is a valid command (calibrate 0V point).
        // We accept all values including 0.0.
        if (addCalibrationPoint(realVoltage)) {
            saveCalibration();
            return true;
        }
        return false;  // Map is full
    }

    // ----- RESET COMMAND -----
    String lower = trimmed;
    lower.toLowerCase();
    if (lower == "reset") {
        resetCalibration();
        return true;
    }

    return false;  // Command not recognized
}

// ============================================================================
//  PRIVATE: _readADC() — Single ADC read
// ============================================================================
int IndustrialVoltage::_readADC() {
    if (_customADC != nullptr) {
        return _customADC(_pin);
    }
    return analogRead(_pin);
}

// ============================================================================
//  PRIVATE: _oversampleAndDecimate()
//  Collects N samples and returns the arithmetic mean.
//  32 samples gives ~2.5 extra effective bits of resolution.
// ============================================================================
float IndustrialVoltage::_oversampleAndDecimate() {
    uint32_t accumulator = 0;
    for (uint16_t i = 0; i < _oversampleCount; i++) {
        accumulator += (uint32_t)_readADC();
        delayMicroseconds(10);
    }
    return (float)accumulator / (float)_oversampleCount;
}

// ============================================================================
//  PRIVATE: _sortCalibrationPoints()
//  Insertion sort — optimal for nearly-sorted, small arrays (N ≤ 10).
//  Sorts ascending by rawADC value.
// ============================================================================
void IndustrialVoltage::_sortCalibrationPoints() {
    for (uint8_t i = 1; i < _calCount; i++) {
        CalibrationPoint key = _calPoints[i];
        int8_t j = i - 1;
        while (j >= 0 && _calPoints[j].rawADC > key.rawADC) {
            _calPoints[j + 1] = _calPoints[j];
            j--;
        }
        _calPoints[j + 1] = key;
    }
}

// ============================================================================
//  PRIVATE: _interpolate() — Segmented Linear Interpolation
// 
//  Given N calibration points sorted by rawADC:
//    [(adc₀,v₀), (adc₁,v₁), ..., (adcₙ₋₁,vₙ₋₁)]
// 
//  For a measured ADC value x:
//    1. Find the segment [i, i+1] where adcᵢ ≤ x ≤ adcᵢ₊₁
//    2. Linearly interpolate:
//       voltage = vᵢ + (x - adcᵢ) × (vᵢ₊₁ - vᵢ) / (adcᵢ₊₁ - adcᵢ)
// 
//  EDGE CASES:
//    x < adc₀     → Extrapolate using the first segment (points 0,1).
//    x > adcₙ₋₁   → Extrapolate using the last segment (points n-2, n-1).
// 
//  WHY SEGMENTED LINEAR?
//    Real-world ADC non-linearity is piece-wise smooth. A polynomial fit
//    would oscillate (Runge's phenomenon) with 10 points, while segmented
//    linear is monotonic, fast, and deterministic.
// ============================================================================
float IndustrialVoltage::_interpolate(float rawADC) {
    if (_calCount < 2) {
        // Not enough points for interpolation — fall back to divider math.
        return _dividerMath(rawADC);
    }

    // ----- Below the lowest calibration point → extrapolate from first segment -----
    if (rawADC <= _calPoints[0].rawADC) {
        float adcSpan = _calPoints[1].rawADC - _calPoints[0].rawADC;
        if (abs(adcSpan) < 0.001f) return _calPoints[0].realValue;  // Degenerate case
        float slope = (_calPoints[1].realValue - _calPoints[0].realValue) / adcSpan;
        return _calPoints[0].realValue + slope * (rawADC - _calPoints[0].rawADC);
    }

    // ----- Above the highest calibration point → extrapolate from last segment -----
    if (rawADC >= _calPoints[_calCount - 1].rawADC) {
        uint8_t last = _calCount - 1;
        float adcSpan = _calPoints[last].rawADC - _calPoints[last - 1].rawADC;
        if (abs(adcSpan) < 0.001f) return _calPoints[last].realValue;
        float slope = (_calPoints[last].realValue - _calPoints[last - 1].realValue) / adcSpan;
        return _calPoints[last].realValue + slope * (rawADC - _calPoints[last].rawADC);
    }

    // ----- Within calibration range → find the correct segment and interpolate -----
    for (uint8_t i = 0; i < _calCount - 1; i++) {
        if (rawADC >= _calPoints[i].rawADC && rawADC <= _calPoints[i + 1].rawADC) {
            float adcSpan = _calPoints[i + 1].rawADC - _calPoints[i].rawADC;
            if (abs(adcSpan) < 0.001f) return _calPoints[i].realValue;
            float t = (rawADC - _calPoints[i].rawADC) / adcSpan;
            return _calPoints[i].realValue + t * (_calPoints[i + 1].realValue - _calPoints[i].realValue);
        }
    }

    // Fallback (should never reach here with sorted data).
    return _dividerMath(rawADC);
}

// ============================================================================
//  PRIVATE: _dividerMath() — Standard resistor divider calculation
// 
//  Vin = (ADC / maxADC) × Vref × dividerRatio
//  Where dividerRatio = (R1 + R2) / R2
// ============================================================================
float IndustrialVoltage::_dividerMath(float rawADC) {
    float vADC = (rawADC / (float)_adcMaxValue) * _adcVRef;
    return vADC * _dividerRatio;
}

// ============================================================================
//  PRIVATE: _applyEMA() — Exponential Moving Average
// 
//  EMA(k) = α × sample(k) + (1 - α) × EMA(k-1)
// 
//  Properties:
//    - α close to 1.0: Fast response, minimal smoothing (noisy display).
//    - α close to 0.0: Heavy smoothing, slow response (laggy display).
//    - Default α = 0.15 gives a good balance for ~10Hz LCD refresh.
// 
//  TIME CONSTANT:
//    The "half-life" of the EMA (samples to reach 50% of a step change)
//    is approximately: τ ≈ -1 / ln(1 - α)
//    For α = 0.15: τ ≈ 6.2 samples → at 10 reads/sec, about 620ms response.
// ============================================================================
float IndustrialVoltage::_applyEMA(float value) {
    if (!_emaInitialized) {
        _emaValue = value;
        _emaInitialized = true;
        return value;
    }
    _emaValue = _emaAlpha * value + (1.0f - _emaAlpha) * _emaValue;
    return _emaValue;
}
