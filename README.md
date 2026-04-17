# IndustrialSense

**Industrial-grade power monitoring for Arduino & ESP32.**

An "Apple-style" easy-to-use, yet industrially robust Arduino library for voltage and current sensing. Bridges the gap between simple hardware (voltage dividers, ACS712/ACS758) and complex industrial environments (high EMI, FOC inverter noise, BLDC motor transients).

---

## ✨ Features

| Feature | Current Sensor | Voltage Sensor |
|---|---|---|
| **Signal Processing** | Adaptive Kalman Filter | Segmented Linear Interpolation |
| **Smoothing** | Oversampling (64×) + Decimation | EMA Anti-Flicker |
| **Calibration** | Auto-Zero Thermal Compensation | Multi-Point Non-Linear (up to 10 pts) |
| **Noise Rejection** | Innovation Gating + Deadzone | — |
| **Persistence** | EEPROM / NVS auto-save | EEPROM / NVS auto-save |
| **Runtime Commands** | `auto`, `reset` | `v<value>`, `reset` |

## 🚀 Quick Start (3 Lines of Code)

```cpp
#include <IndustrialSense.h>

IndustrialCurrent current(ACS712_20A, A0);
IndustrialVoltage voltage(A1, 30000.0, 7500.0);

void setup() {
    current.begin();
    voltage.begin();
}

void loop() {
    Serial.println(current.read());  // Filtered Amps
    Serial.println(voltage.read());  // Smoothed Volts
}
```

## 📦 Supported Sensors

| Enum | Sensor | Sensitivity | Range |
|---|---|---|---|
| `ACS712_05A` | ACS712 5A | 185 mV/A | ±5A |
| `ACS712_20A` | ACS712 20A | 100 mV/A | ±20A |
| `ACS712_30A` | ACS712 30A | 66 mV/A | ±30A |
| `ACS758_50B` | ACS758 50B | 40 mV/A | ±50A |
| `ACS758_100U` | ACS758 100U | 20 mV/A | 0–100A |

## 🔧 Serial Calibration Commands

Send via Serial Monitor at 115200 baud:

```
v12.5   → Map current ADC reading to 12.5V and save
v0      → Calibrate the 0V point
auto    → Force current sensor auto-zero
reset   → Clear all calibration data
```

## 🏗️ Architecture

```
IndustrialSense/
├── library.properties
├── keywords.txt
├── src/
│   ├── IndustrialSense.h        ← Core header (include this)
│   ├── IndustrialCurrent.cpp    ← Adaptive Kalman + Auto-Zero
│   ├── IndustrialVoltage.cpp    ← NL Calibration + EMA
│   └── StorageManager.h         ← ESP32 Preferences / AVR EEPROM
└── examples/
    ├── 01_Basic_Usage/
    └── 02_Advanced_Calibration/
```

## 📄 License

MIT
# IndustrialSense
