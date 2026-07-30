# ARCHITECTURE SPEC: Monotonic Back-Stamping & Dual Time Anchors

## Executive Summary & Architectural Pivot

**Context & Abandoned Approach:** 
Active CAN bus polling on Module 17 (Dashboard/Instruments) for wall-clock time has been **abandoned**. Module 17 polling adds significant UDS/ISO-TP filter complexity, carries high hardware debugging friction, provides no GPS coordinates, and returns local time rather than true UTC.

**New Architecture:**
To solve **Finding A** (spool-drain mis-dating where off-line or cold-start records are stamped with server-receive time instead of capture time), we are implementing an in-RAM **Monotonic Back-Stamping System** paired with a **Tiered UTC Time Anchor**.

This eliminates flash wear, handles garage cold-starts with no GPS signal, and guarantees 100% accurate time-stamping for cold-engine metrics.

---

## Technical Design

### 1. Monotonic Time Base (`esp_timer_get_time`)
Do not use `millis()` (32-bit, wraps every ~49 days). 

Use native ESP-IDF `esp_timer_get_time()`, which returns a 64-bit integer of microseconds since boot (`uint64_t`). It is monotonic, never wraps during the hardware lifetime, and is unaffected by system clock adjustments.

### 2. Spool Buffer Record Changes
Every telemetry record generated prior to establishing a valid UTC time lock must store its raw microsecond tick timestamp.

Modify the internal sensor record / spool entry struct:

```cpp
struct TelemetryRecord {
    uint64_t tick_us;        // esp_timer_get_time() at capture instant
    uint32_t utc_timestamp;  // Resolved Unix epoch (0 if unresolved)
    bool time_resolved;      // false = needs back-stamping; true = ready for transport
    // ... remaining OBD/telemetry payload fields
};
```

### 3. Tiered UTC Time Anchor Hierarchy
A global state structure maintains the current baseline anchor:

```cpp
struct TimeAnchor {
    uint32_t anchor_utc_sec; // Unix epoch in UTC seconds
    uint64_t anchor_tick_us; // esp_timer_get_time() at the instant UTC was acquired
    bool is_set;             // true if at least one time source has fired
};
```

The system resolves time in the following order of availability:

1. **Cellular NITZ Time (Primary Cold-Start Source):**
   * Upon modem attach, query cellular network time (e.g., via `AT+CCLK?` or network registration callback).
   * Populates `TimeAnchor` within seconds of boot, even in underground garages with zero GPS line-of-sight.
2. **GPS UTC (High Precision Source):**
   * Upon receiving a valid NMEA time sentence or `$GNIFO` string from the u-blox module.
   * Overwrites/updates `TimeAnchor` with millisecond-accurate UTC time.
3. **RTC RAM (`RTC_DATA_ATTR`) / NVS (Fallback):**
   * Optional: Preserves last-known timestamp across software restarts (`ESP.restart()`) during vehicle motion wakeups.

---

## The Reconciler Algorithm

When a telemetry frame is generated, calculate its timestamp as follows:

### Scenario A: Time Anchor is already set (`is_set == true`)
Stamp record immediately at generation:

`T_event = T_anchor + ((M_event - M_anchor) / 1000000)`

* `T_event`: Calculated Unix epoch in seconds.
* `T_anchor`: `anchor_utc_sec`.
* `M_event`: Current `esp_timer_get_time()`.
* `M_anchor`: `anchor_tick_us`.
* Set `time_resolved = true`.

### Scenario B: Time Anchor is NOT yet set (`is_set == false`)
1. Record frame with `tick_us = esp_timer_get_time()`.
2. Set `utc_timestamp = 0` and `time_resolved = false`.
3. Push to spool buffer (RAM ring buffer or SD queue).

### Scenario C: Anchor Event Trigger (`onTimeAnchorEstablished`)
The exact moment NITZ or GPS establishes the *first* time anchor:

```cpp
void reconcileSpooledRecords(uint32_t utc_sec, uint64_t tick_us) {
    for (auto& record : spoolBuffer) {
        if (!record.time_resolved) {
            int64_t delta_us = (int64_t)tick_us - (int64_t)record.tick_us;
            uint32_t delta_sec = (uint32_t)(delta_us / 1000000ULL);
            
            record.utc_timestamp = utc_sec - delta_sec;
            record.time_resolved = true;
        }
    }
    // Unblock cellular transport worker to begin despooling
}
```

---

## Action Items for Implementation

1. **Deprecate Module 17 Listeners:**
   * Remove/disable active OBD requests targeted at Module 17 (`0x77E` / `0x17`).
   * Revert any custom CAN/UDS receive filters expanded specifically for Module 17.

2. **Implement Monotonic Ticking in Data Structures:**
   * Replace `millis()` calls in frame generation with `esp_timer_get_time()`.
   * Update the buffer packet serialization format to ensure `PID_ABS_TIME` / UTC timestamps are always included when `time_resolved == true`.

3. **Wire Up NITZ Engine:**
   * In `initCell()` or network management loop, extract time from `AT+CCLK?` after registration.
   * Call `setTimeAnchor(nitz_utc_sec, esp_timer_get_time())`.

4. **Wire Up GPS Time Lock:**
   * In GPS parsing logic, upon standard time fix, call `setTimeAnchor(gps_utc_sec, esp_timer_get_time())`.

5. **Update Despool Criteria:**
   * Ensure cellular despool worker skips records where `time_resolved == false`.
   * Confirm despooling is permitted even if GPS position (Lat/Lng) is missing—Traccar handles time-stamped telemetry frames without coordinates cleanly.