/**
 * ============================================================================
 * StorageManager.h — Cross-Platform Persistent Storage Abstraction
 * ============================================================================
 * 
 * Part of the IndustrialSense Library.
 * 
 * PURPOSE:
 *   Provides a unified API for saving/loading calibration data to non-volatile
 *   memory, without the user ever needing to write platform-specific code.
 * 
 * PLATFORM STRATEGY:
 *   - ESP32:  Uses the Preferences library (NVS — Non-Volatile Storage).
 *             This is wear-leveled by the ESP-IDF framework automatically.
 *   - AVR:    Uses the EEPROM library. We implement manual wear-leveling
 *             by only writing bytes that have actually changed (EEPROM.update).
 *   - Other:  Falls back to a no-op stub so the library compiles everywhere,
 *             but persistent storage is simply disabled.
 * 
 * MEMORY LAYOUT (AVR EEPROM):
 *   Addr 0:    Magic byte (0xA5) — indicates valid data exists.
 *   Addr 1:    uint8_t — number of voltage calibration points (N, max 10).
 *   Addr 2:    float[10] — rawADC values  (40 bytes, addr 2..41).
 *   Addr 42:   float[10] — realVoltage values (40 bytes, addr 42..81).
 *   Addr 82:   float — current sensor zero-offset.
 *   Addr 86:   (end, total = 86 bytes used)
 * 
 * LICENSE: MIT
 * ============================================================================
 */

#ifndef STORAGE_MANAGER_H
#define STORAGE_MANAGER_H

#include <Arduino.h>

// ---------------------------------------------------------------------------
//  Maximum calibration points. This is shared across all sensor classes.
// ---------------------------------------------------------------------------
static const uint8_t MAX_CAL_POINTS = 10;

// ---------------------------------------------------------------------------
//  Calibration point structure — a single (rawADC → realValue) mapping.
// ---------------------------------------------------------------------------
struct CalibrationPoint {
    float rawADC;       // The raw ADC reading at the calibration moment
    float realValue;    // The user-provided "true" value (e.g., 12.50 V)
};

// ============================================================================
//  Platform-specific includes
// ============================================================================
#if defined(ESP32)
    #include <Preferences.h>
#elif defined(ARDUINO_ARCH_AVR)
    #include <EEPROM.h>
#endif

namespace IndustrialSenseStorage {

// ============================================================================
//  AVR EEPROM Address Map Constants
// ============================================================================
#if defined(ARDUINO_ARCH_AVR)
    static const uint16_t EEPROM_MAGIC_ADDR       = 0;
    static const uint8_t  EEPROM_MAGIC_VALUE       = 0xA5;
    static const uint16_t EEPROM_CAL_COUNT_ADDR    = 1;
    static const uint16_t EEPROM_CAL_RAW_ADDR      = 2;    // float[10] = 40 bytes
    static const uint16_t EEPROM_CAL_REAL_ADDR     = 42;   // float[10] = 40 bytes
    static const uint16_t EEPROM_CURRENT_OFF_ADDR  = 82;   // float     =  4 bytes
#endif

// ============================================================================
//  saveVoltageCalibration()
//  Persists the voltage calibration map (up to MAX_CAL_POINTS entries).
// ============================================================================
inline void saveVoltageCalibration(const CalibrationPoint* points, uint8_t count) {
    if (count > MAX_CAL_POINTS) count = MAX_CAL_POINTS;

#if defined(ESP32)
    // -----------------------------------------------------------------------
    //  ESP32: Preferences (NVS) — wear-leveled by the IDF flash layer.
    //  We store the count as a uint8, and the arrays as raw byte blobs.
    // -----------------------------------------------------------------------
    Preferences prefs;
    prefs.begin("isense_vcal", false);  // Read-Write mode
    prefs.putUChar("count", count);
    prefs.putBytes("raw",  (const void*)points, sizeof(CalibrationPoint) * count);
    prefs.end();

#elif defined(ARDUINO_ARCH_AVR)
    // -----------------------------------------------------------------------
    //  AVR: EEPROM.update() — only writes bytes that differ, reducing wear.
    // -----------------------------------------------------------------------
    EEPROM.update(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VALUE);
    EEPROM.update(EEPROM_CAL_COUNT_ADDR, count);

    for (uint8_t i = 0; i < count; i++) {
        // Write rawADC float (4 bytes each)
        uint16_t rawAddr  = EEPROM_CAL_RAW_ADDR  + (i * sizeof(float));
        uint16_t realAddr = EEPROM_CAL_REAL_ADDR  + (i * sizeof(float));
        const uint8_t* pRaw  = (const uint8_t*)&(points[i].rawADC);
        const uint8_t* pReal = (const uint8_t*)&(points[i].realValue);
        for (uint8_t b = 0; b < sizeof(float); b++) {
            EEPROM.update(rawAddr  + b, pRaw[b]);
            EEPROM.update(realAddr + b, pReal[b]);
        }
    }
#else
    // No-op on unsupported platforms — data is volatile only.
    (void)points;
    (void)count;
#endif
}

// ============================================================================
//  loadVoltageCalibration()
//  Loads the voltage calibration map from persistent storage.
//  Returns the number of valid points loaded (0 if empty/invalid).
// ============================================================================
inline uint8_t loadVoltageCalibration(CalibrationPoint* points, uint8_t maxCount) {
    if (maxCount > MAX_CAL_POINTS) maxCount = MAX_CAL_POINTS;

#if defined(ESP32)
    Preferences prefs;
    prefs.begin("isense_vcal", true);  // Read-only mode
    uint8_t count = prefs.getUChar("count", 0);
    if (count > maxCount) count = maxCount;
    if (count > 0) {
        prefs.getBytes("raw", (void*)points, sizeof(CalibrationPoint) * count);
    }
    prefs.end();
    return count;

#elif defined(ARDUINO_ARCH_AVR)
    // Check magic byte — if absent, no valid data has ever been saved.
    if (EEPROM.read(EEPROM_MAGIC_ADDR) != EEPROM_MAGIC_VALUE) {
        return 0;
    }
    uint8_t count = EEPROM.read(EEPROM_CAL_COUNT_ADDR);
    if (count > maxCount) count = maxCount;

    for (uint8_t i = 0; i < count; i++) {
        uint16_t rawAddr  = EEPROM_CAL_RAW_ADDR  + (i * sizeof(float));
        uint16_t realAddr = EEPROM_CAL_REAL_ADDR  + (i * sizeof(float));
        uint8_t* pRaw  = (uint8_t*)&(points[i].rawADC);
        uint8_t* pReal = (uint8_t*)&(points[i].realValue);
        for (uint8_t b = 0; b < sizeof(float); b++) {
            pRaw[b]  = EEPROM.read(rawAddr  + b);
            pReal[b] = EEPROM.read(realAddr + b);
        }
    }
    return count;

#else
    (void)points;
    (void)maxCount;
    return 0;
#endif
}

// ============================================================================
//  saveCurrentOffset()
//  Persists the current sensor's zero-offset calibration value.
// ============================================================================
inline void saveCurrentOffset(float offset) {
#if defined(ESP32)
    Preferences prefs;
    prefs.begin("isense_ical", false);
    prefs.putFloat("offset", offset);
    prefs.end();

#elif defined(ARDUINO_ARCH_AVR)
    EEPROM.update(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VALUE);
    const uint8_t* p = (const uint8_t*)&offset;
    for (uint8_t b = 0; b < sizeof(float); b++) {
        EEPROM.update(EEPROM_CURRENT_OFF_ADDR + b, p[b]);
    }
#else
    (void)offset;
#endif
}

// ============================================================================
//  loadCurrentOffset()
//  Loads the current sensor's zero-offset from persistent storage.
//  Returns `defaultVal` if no valid data is found.
// ============================================================================
inline float loadCurrentOffset(float defaultVal) {
#if defined(ESP32)
    Preferences prefs;
    prefs.begin("isense_ical", true);
    float val = prefs.getFloat("offset", defaultVal);
    prefs.end();
    return val;

#elif defined(ARDUINO_ARCH_AVR)
    if (EEPROM.read(EEPROM_MAGIC_ADDR) != EEPROM_MAGIC_VALUE) {
        return defaultVal;
    }
    float val;
    uint8_t* p = (uint8_t*)&val;
    for (uint8_t b = 0; b < sizeof(float); b++) {
        p[b] = EEPROM.read(EEPROM_CURRENT_OFF_ADDR + b);
    }
    return val;

#else
    return defaultVal;
#endif
}

// ============================================================================
//  resetAllCalibration()
//  Erases all IndustrialSense persistent data.
// ============================================================================
inline void resetAllCalibration() {
#if defined(ESP32)
    Preferences prefs;
    prefs.begin("isense_vcal", false);
    prefs.clear();
    prefs.end();
    prefs.begin("isense_ical", false);
    prefs.clear();
    prefs.end();

#elif defined(ARDUINO_ARCH_AVR)
    // Invalidate the magic byte — effectively "erasing" our data.
    EEPROM.update(EEPROM_MAGIC_ADDR, 0xFF);
#else
    // No-op
#endif
}

} // namespace IndustrialSenseStorage

#endif // STORAGE_MANAGER_H
