/*************************************************************************
* UDS DID table for VAG ECU 4G2 907 311 B
* (Audi A6 Allroad 3.0 TDI EA897 evo, Bosch/Continental MD1).
*
* Each entry knows its UDS DID, which ECU to ask (request CAN ID), the
* synthetic PID to write into the VorsprungLogger ring buffer, the expected
* payload byte count, a polling cadence (cycles between reads), a decoder
* that turns the raw bytes into a float in engineering units, and a
* human-readable unit string.
*
* Two ECUs live on the powertrain bus (see SPDD.md):
*   - engine ECU    request 0x7E0 / response 0x7E8  (UDS_ECU_ENGINE)
*   - SCR/reductant request 0x7EA / response 0x794  (UDS_ECU_SCR)
* CUDS selects the request ID per DID; a wide receive filter catches both
* response IDs.
*
* Synthetic PIDs occupy the 0x200+ range so they do not collide with
* either OBD-II PIDs (which get OR'd with 0x100 by processOBD() — so
* 0x101..0x1FF) or the internal GPS/MEMS PIDs in FreematicsBase.h.
* 0x200..0x209 are the Phase 4 baseline; 0x20A+ are the Phase 6 expansion.
*
* All DIDs and formulas were validated live on the target car (Phase 2/3
* capture-analyse-validate, then three Phase 5 expansion rounds). Decoders
* are grouped by encoding family — do NOT extrapolate a formula from a
* signal's physical category; the families overlap by coincidence only.
*
* To bring up a new signal: append a row and a PID #define. Nothing else
* needs to change.
*************************************************************************/

#ifndef UDS_DIDS_H_INCLUDED
#define UDS_DIDS_H_INCLUDED

#include <stdint.h>

// ---- ECU selectors (UDS request CAN IDs) ----
#define UDS_ECU_ENGINE 0x7E0
#define UDS_ECU_SCR    0x7EA

// ---- polling cadence tiers (cycles between reads) ----
#define UDS_FAST     1   // every process() cycle — fast dynamics
#define UDS_SLOW    10   // drifting temps/pressures, soot, efficiency
#define UDS_GLACIAL 60   // odometer, regen counters, tank/fluid levels
// Sentinel tier (not a cadence): membership of the "derived-inputs" group.
// These DIDs are skipped by the round-robin and co-read atomically on a shared
// cadence by readGroup() in main.cpp, so they always share a packet with RPM.
// See SPDD.md §8 (The Derived-Inputs Group).
#define UDS_GROUP    0

// Synthetic PIDs — Phase 4 baseline (0x200..0x209)
#define PID_UDS_COOLANT_TEMP        0x200
#define PID_UDS_EGT_B1S1            0x201
#define PID_UDS_EGT_B1S2            0x202
#define PID_UDS_EGT_SENSOR1_RAW     0x203
#define PID_UDS_OIL_TEMP            0x204
#define PID_UDS_FUEL_PRESS_REG      0x205
#define PID_UDS_OIL_PRESS_ACTUAL    0x206
#define PID_UDS_OIL_PRESS_SETPOINT  0x207
#define PID_UDS_IMAP_CALCULATED     0x208
#define PID_UDS_AMBIENT_PRESS       0x209

// Synthetic PIDs — Phase 6 expansion (0x20A+, sequential in table order)
#define PID_UDS_IAT                 0x20A
#define PID_UDS_CTRL_MOD_TEMP       0x20B
#define PID_UDS_FUEL_TEMP           0x20C
#define PID_UDS_CHARGECOOL_TEMP     0x20D
#define PID_UDS_TURBO_OUT_TEMP      0x20E
#define PID_UDS_SCR_CAT_TEMP        0x20F
#define PID_UDS_REDUCT_TANK_TEMP    0x210
#define PID_UDS_REDUCT_PUMP_TEMP    0x211
#define PID_UDS_BARO_PRESS          0x212
#define PID_UDS_CHARGE_PRESS_SPEC   0x213
#define PID_UDS_CHARGE_PRESS_ACT    0x214
#define PID_UDS_FUEL_LOW_PRESS      0x215
#define PID_UDS_FUEL_RAIL_PRESS     0x216
#define PID_UDS_FUEL_METERING       0x217
#define PID_UDS_DPF_DP_OFFSET       0x218
#define PID_UDS_REDUCT_LINE_PRESS   0x219
#define PID_UDS_MAF                 0x21A
#define PID_UDS_AIR_MASS_METER      0x21B
#define PID_UDS_AIR_MASS_SPEC       0x21C
#define PID_UDS_FUEL_RATE           0x21D
#define PID_UDS_FUEL_LEVEL          0x21E
#define PID_UDS_ENGINE_TORQUE       0x21F
#define PID_UDS_ODOMETER            0x220
#define PID_UDS_LAMBDA_B1           0x221
#define PID_UDS_LAMBDA_B2           0x222
#define PID_UDS_HPEGR_ACTIVATION    0x223
#define PID_UDS_EGR_CMD             0x224
#define PID_UDS_EGR_ACT             0x225
#define PID_UDS_DPF_ASH             0x226
#define PID_UDS_DPF_SOOT_CALC       0x227
#define PID_UDS_DPF_SOOT_MEAS       0x228
#define PID_UDS_REGEN_TIME          0x229
#define PID_UDS_REGEN_DISTANCE      0x22A
#define PID_UDS_FUEL_SINCE_REGEN    0x22B
#define PID_UDS_NOX1                0x22C
#define PID_UDS_NOX2                0x22D
#define PID_UDS_SCR_DOSAGE_ADAPT    0x22E
#define PID_UDS_SCR_CAT_EFF         0x22F
#define PID_UDS_REDUCT_LEVEL_PCT    0x230
#define PID_UDS_REDUCT_LEVEL_VOLTS  0x231
#define PID_UDS_REDUCT_PUMP_SPEED   0x232

struct UDSDIDEntry {
	uint16_t did;
	uint16_t reqId;          // ECU request CAN ID (UDS_ECU_ENGINE / UDS_ECU_SCR)
	uint16_t syntheticPID;
	const char* label;
	uint8_t bytes;           // expected payload byte count
	uint8_t intervalCycles;  // polling cadence tier (UDS_FAST/SLOW/GLACIAL)
	float (*decode)(const uint8_t* d);
	const char* unit;
};

// A DID tagged UDS_GROUP belongs to the derived-inputs group: it is co-read
// atomically rather than scheduled by the round-robin (see processUDS).
static inline bool udsIsGroupMember(const UDSDIDEntry& e) { return e.intervalCycles == UDS_GROUP; }

// ---- decoders, grouped by encoding family (formulas validated Phase 2/3/5) ----

// Temperatures (the only families that aren't pure scaling)
static inline float udsDecodeTempJ1979_1b(const uint8_t* d) {
	return (float)d[0] - 40.0f;
}
static inline float udsDecodeTempJ1979_2b(const uint8_t* d) {
	return (float)(((uint16_t)d[0] << 8) | d[1]) / 10.0f - 40.0f;
}
static inline float udsDecodeTempDeciKelvin_2b(const uint8_t* d) {
	return (float)(((uint16_t)d[0] << 8) | d[1]) / 10.0f - 273.1f;
}

// Pure 16-bit scalings (unit is conveyed by the table's unit column, not here)
static inline float udsDecodeU16Div1000_2b(const uint8_t* d) {   // bar, V, lambda, ratio
	return (float)(((uint16_t)d[0] << 8) | d[1]) / 1000.0f;
}
static inline float udsDecodeU16Div100_2b(const uint8_t* d) {    // %, g/s, L, g, ratio
	return (float)(((uint16_t)d[0] << 8) | d[1]) / 100.0f;
}
static inline float udsDecodeU16Div20_2b(const uint8_t* d) {     // L/h
	return (float)(((uint16_t)d[0] << 8) | d[1]) / 20.0f;
}
static inline float udsDecodeU16Div12_8_2b(const uint8_t* d) {   // hPa (binary 1/128)
	return (float)(((uint16_t)d[0] << 8) | d[1]) / 12.8f;
}
static inline float udsDecodeU16Div10_2b(const uint8_t* d) {     // kPa, kg/h, mg/stroke, Nm
	return (float)(((uint16_t)d[0] << 8) | d[1]) / 10.0f;
}
static inline float udsDecodeU16Mul100_2b(const uint8_t* d) {    // hPa (rail pressure)
	return (float)(((uint16_t)d[0] << 8) | d[1]) * 100.0f;
}
static inline float udsDecodeU16Raw_2b(const uint8_t* d) {       // hPa, ppm, rpm
	return (float)(((uint16_t)d[0] << 8) | d[1]);
}
static inline float udsDecodeU8Raw_1b(const uint8_t* d) {        // kPa (1-byte baro)
	return (float)d[0];
}
static inline float udsDecodeBinFracPct_2b(const uint8_t* d) {   // % (binary fraction /8192)
	return (float)(((uint16_t)d[0] << 8) | d[1]) / 8192.0f * 100.0f;
}

// Signed families — cast to int16_t before scaling or negatives wrap to ~65500
static inline float udsDecodeSignedMbar_2b(const uint8_t* d) {       // signed mbar
	return (float)(int16_t)(((uint16_t)d[0] << 8) | d[1]);
}
static inline float udsDecodeSignedPctDiv100_2b(const uint8_t* d) {  // signed %
	return (float)(int16_t)(((uint16_t)d[0] << 8) | d[1]) / 100.0f;
}

// 4-byte unsigned, big-endian (single-frame, DLC 07). Values stay well within
// float's exact-integer range (< 2^24) on this vehicle (km, seconds, metres).
static inline float udsDecodeU32_4b(const uint8_t* d) {
	return (float)(((uint32_t)d[0] << 24) | ((uint32_t)d[1] << 16) |
	               ((uint32_t)d[2] << 8) | d[3]);
}

// ---- the table ----
//   did,    ECU,            synthetic PID,             label,                bytes, tier,        decoder,                     unit
static const UDSDIDEntry UDS_DID_TABLE[] = {
	// Phase 4 baseline (0x200..0x209)
	{ 0xF405, UDS_ECU_ENGINE, PID_UDS_COOLANT_TEMP,       "coolant_temp",       1, UDS_SLOW,    udsDecodeTempJ1979_1b,       "C"   },
	{ 0xF43C, UDS_ECU_ENGINE, PID_UDS_EGT_B1S1,           "egt_b1s1",           2, UDS_FAST,    udsDecodeTempJ1979_2b,       "C"   },
	{ 0xF43E, UDS_ECU_ENGINE, PID_UDS_EGT_B1S2,           "egt_b1s2",           2, UDS_FAST,    udsDecodeTempJ1979_2b,       "C"   },
	{ 0x10FB, UDS_ECU_ENGINE, PID_UDS_EGT_SENSOR1_RAW,    "egt_sensor1_raw",    2, UDS_FAST,    udsDecodeTempDeciKelvin_2b,  "C"   },
	{ 0x11BE, UDS_ECU_ENGINE, PID_UDS_OIL_TEMP,           "oil_temp",           2, UDS_SLOW,    udsDecodeTempDeciKelvin_2b,  "C"   },
	{ 0x11BF, UDS_ECU_ENGINE, PID_UDS_FUEL_PRESS_REG,     "fuel_press_reg",     2, UDS_SLOW,    udsDecodeU16Div100_2b,       "%"   },
	{ 0x1B04, UDS_ECU_ENGINE, PID_UDS_OIL_PRESS_ACTUAL,   "oil_press_actual",   2, UDS_SLOW,    udsDecodeU16Div1000_2b,      "bar" },
	{ 0x25CF, UDS_ECU_ENGINE, PID_UDS_OIL_PRESS_SETPOINT, "oil_press_setpt",    2, UDS_SLOW,    udsDecodeU16Div1000_2b,      "bar" },
	{ 0x103C, UDS_ECU_ENGINE, PID_UDS_IMAP_CALCULATED,    "imap_calculated",    2, UDS_SLOW,    udsDecodeU16Div1000_2b,      "bar" },
	{ 0x1627, UDS_ECU_ENGINE, PID_UDS_AMBIENT_PRESS,      "ambient_press",      2, UDS_GLACIAL, udsDecodeU16Div1000_2b,      "bar" },

	// Phase 6 expansion — temperatures
	{ 0xF40F, UDS_ECU_ENGINE, PID_UDS_IAT,                "iat",                1, UDS_GROUP,   udsDecodeTempJ1979_1b,       "C"   },
	{ 0x12FB, UDS_ECU_ENGINE, PID_UDS_CTRL_MOD_TEMP,      "ctrl_mod_temp",      2, UDS_SLOW,    udsDecodeTempDeciKelvin_2b,  "C"   },
	{ 0x162D, UDS_ECU_ENGINE, PID_UDS_FUEL_TEMP,          "fuel_temp",          2, UDS_SLOW,    udsDecodeTempDeciKelvin_2b,  "C"   },
	{ 0x1628, UDS_ECU_ENGINE, PID_UDS_CHARGECOOL_TEMP,    "chargecool_temp",    2, UDS_SLOW,    udsDecodeTempDeciKelvin_2b,  "C"   },
	{ 0x1798, UDS_ECU_ENGINE, PID_UDS_TURBO_OUT_TEMP,     "turbo_out_temp",     2, UDS_SLOW,    udsDecodeTempDeciKelvin_2b,  "C"   },
	{ 0x1527, UDS_ECU_ENGINE, PID_UDS_SCR_CAT_TEMP,       "scr_cat_temp",       2, UDS_SLOW,    udsDecodeTempDeciKelvin_2b,  "C"   },
	{ 0x29AA, UDS_ECU_SCR,    PID_UDS_REDUCT_TANK_TEMP,   "reduct_tank_temp",   1, UDS_GLACIAL, udsDecodeTempJ1979_1b,       "C"   },
	{ 0x422F, UDS_ECU_SCR,    PID_UDS_REDUCT_PUMP_TEMP,   "reduct_pump_temp",   1, UDS_SLOW,    udsDecodeTempJ1979_1b,       "C"   },

	// Phase 6 expansion — pressures
	{ 0xF433, UDS_ECU_ENGINE, PID_UDS_BARO_PRESS,         "baro_press",         1, UDS_SLOW,    udsDecodeU8Raw_1b,           "kPa"  },
	{ 0x1149, UDS_ECU_ENGINE, PID_UDS_CHARGE_PRESS_SPEC,  "charge_press_spec",  2, UDS_SLOW,    udsDecodeU16Raw_2b,          "hPa"  },
	{ 0x1B0E, UDS_ECU_ENGINE, PID_UDS_CHARGE_PRESS_ACT,   "charge_press_act",   2, UDS_GROUP,   udsDecodeU16Div12_8_2b,      "hPa"  },
	{ 0x1AB8, UDS_ECU_ENGINE, PID_UDS_FUEL_LOW_PRESS,     "fuel_low_press",     2, UDS_SLOW,    udsDecodeU16Div10_2b,        "kPa"  },
	{ 0x1169, UDS_ECU_ENGINE, PID_UDS_FUEL_RAIL_PRESS,    "fuel_rail_press",    2, UDS_SLOW,    udsDecodeU16Mul100_2b,       "hPa"  },
	{ 0x1148, UDS_ECU_ENGINE, PID_UDS_FUEL_METERING,      "fuel_metering",      2, UDS_SLOW,    udsDecodeU16Div100_2b,       "%"    },
	{ 0x10F1, UDS_ECU_ENGINE, PID_UDS_DPF_DP_OFFSET,      "dpf_dp_offset",      2, UDS_SLOW,    udsDecodeU16Raw_2b,          "hPa"  },
	{ 0x29A7, UDS_ECU_SCR,    PID_UDS_REDUCT_LINE_PRESS,  "reduct_line_press",  2, UDS_SLOW,    udsDecodeSignedMbar_2b,      "mbar" },

	// Phase 6 expansion — air / fuel / flow
	{ 0xF410, UDS_ECU_ENGINE, PID_UDS_MAF,                "maf",                2, UDS_GROUP,   udsDecodeU16Div100_2b,       "g/s"  },
	{ 0x1024, UDS_ECU_ENGINE, PID_UDS_AIR_MASS_METER,     "air_mass_meter",     2, UDS_SLOW,    udsDecodeU16Div10_2b,        "kg/h" },
	{ 0x104C, UDS_ECU_ENGINE, PID_UDS_AIR_MASS_SPEC,      "air_mass_spec",      2, UDS_SLOW,    udsDecodeU16Div10_2b,        "mg"   },
	{ 0xF45E, UDS_ECU_ENGINE, PID_UDS_FUEL_RATE,          "fuel_rate",          2, UDS_GROUP,   udsDecodeU16Div20_2b,        "L/h"  },
	{ 0x100C, UDS_ECU_ENGINE, PID_UDS_FUEL_LEVEL,         "fuel_level",         2, UDS_GLACIAL, udsDecodeU16Div100_2b,       "L"    },

	// Phase 6 expansion — mechanical / electrical / engine control
	{ 0x1047, UDS_ECU_ENGINE, PID_UDS_ENGINE_TORQUE,      "engine_torque",      2, UDS_GROUP,   udsDecodeU16Div10_2b,        "Nm"   },
	{ 0x16A9, UDS_ECU_ENGINE, PID_UDS_ODOMETER,           "odometer",           4, UDS_GLACIAL, udsDecodeU32_4b,             "km"   },
	{ 0x113D, UDS_ECU_ENGINE, PID_UDS_LAMBDA_B1,          "lambda_b1",          2, UDS_SLOW,    udsDecodeU16Div1000_2b,      "lambda" },
	{ 0x12F8, UDS_ECU_ENGINE, PID_UDS_LAMBDA_B2,          "lambda_b2",          2, UDS_SLOW,    udsDecodeU16Div1000_2b,      "lambda" },
	{ 0x1334, UDS_ECU_ENGINE, PID_UDS_HPEGR_ACTIVATION,   "hpegr_activation",   2, UDS_SLOW,    udsDecodeSignedPctDiv100_2b, "%"    },
	{ 0x10C2, UDS_ECU_ENGINE, PID_UDS_EGR_CMD,            "egr_cmd",            2, UDS_SLOW,    udsDecodeBinFracPct_2b,      "%"    },
	{ 0x132F, UDS_ECU_ENGINE, PID_UDS_EGR_ACT,            "egr_act",            2, UDS_SLOW,    udsDecodeBinFracPct_2b,      "%"    },

	// Phase 6 expansion — DPF (engine ECU)
	{ 0x1153, UDS_ECU_ENGINE, PID_UDS_DPF_ASH,            "dpf_ash",            2, UDS_GLACIAL, udsDecodeU16Div100_2b,       "L"    },
	{ 0x114F, UDS_ECU_ENGINE, PID_UDS_DPF_SOOT_CALC,      "dpf_soot_calc",      2, UDS_SLOW,    udsDecodeU16Div100_2b,       "g"    },
	{ 0x114E, UDS_ECU_ENGINE, PID_UDS_DPF_SOOT_MEAS,      "dpf_soot_meas",      2, UDS_SLOW,    udsDecodeU16Div100_2b,       "g"    },
	{ 0x115E, UDS_ECU_ENGINE, PID_UDS_REGEN_TIME,         "regen_time",         4, UDS_GLACIAL, udsDecodeU32_4b,             "s"    },
	{ 0x1156, UDS_ECU_ENGINE, PID_UDS_REGEN_DISTANCE,     "regen_distance",     4, UDS_GLACIAL, udsDecodeU32_4b,             "m"    },
	{ 0x115A, UDS_ECU_ENGINE, PID_UDS_FUEL_SINCE_REGEN,   "fuel_since_regen",   2, UDS_GLACIAL, udsDecodeU16Div100_2b,       "L"    },

	// Phase 6 expansion — SCR / NOx
	{ 0x13BD, UDS_ECU_ENGINE, PID_UDS_NOX1,               "nox1",               2, UDS_FAST,    udsDecodeU16Raw_2b,          "ppm"   },
	{ 0x13BC, UDS_ECU_ENGINE, PID_UDS_NOX2,               "nox2",               2, UDS_FAST,    udsDecodeU16Raw_2b,          "ppm"   },
	{ 0x13C3, UDS_ECU_ENGINE, PID_UDS_SCR_DOSAGE_ADAPT,   "scr_dosage_adapt",   2, UDS_GLACIAL, udsDecodeU16Div100_2b,       "ratio" },
	{ 0x13CB, UDS_ECU_ENGINE, PID_UDS_SCR_CAT_EFF,        "scr_cat_eff",        2, UDS_SLOW,    udsDecodeU16Div1000_2b,      "ratio" },
	{ 0x16F7, UDS_ECU_ENGINE, PID_UDS_REDUCT_LEVEL_PCT,   "reduct_level_pct",   2, UDS_GLACIAL, udsDecodeBinFracPct_2b,      "%"     },
	{ 0x29B7, UDS_ECU_SCR,    PID_UDS_REDUCT_LEVEL_VOLTS, "reduct_level_volts", 2, UDS_GLACIAL, udsDecodeU16Div1000_2b,      "V"     },
	{ 0x3F4D, UDS_ECU_SCR,    PID_UDS_REDUCT_PUMP_SPEED,  "reduct_pump_speed",  2, UDS_SLOW,    udsDecodeU16Raw_2b,          "rpm"   },
};

static const int UDS_DID_COUNT = sizeof(UDS_DID_TABLE) / sizeof(UDS_DID_TABLE[0]);

#endif // UDS_DIDS_H_INCLUDED
