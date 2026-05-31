# VorsprungLogger — Software Project Design Document

An Audi-focused vehicle telemetry logger for the Freematics ONE+ Model B (ESP32).
Derived from the Freematics **Telelogger** by Stanley Huang (BSD; attribution
retained), narrowed to one vehicle and one job: log a 2017 Audi A6 Allroad
(3.0 TDI, EA897 evo) over OBD-II + UDS, plus GPS/MEMS, to SD and to a Traccar
server over cellular.

This document is the single source of design truth. It supersedes the earlier
SPDD, `PROJECT_CONTEXT.md`, `UDS_Implementation_Plan.md`, and the derived-inputs
brief, which have been folded in here.

---

## 1. Purpose

Turn a Freematics ONE+ Model B into a vehicle telemetry logger that:

- reads OBD-II live PIDs from the engine ECU;
- reads ~50 manufacturer-specific **UDS DIDs** (ISO 14229, service 0x22) from the
  VAG EA897 powertrain — EGTs, pressures, air/fuel flow, torque, DPF, SCR/NOx;
- records position via the onboard u-blox GNSS and motion via the MEMS;
- logs everything to SD as CSV and streams it over UDP to a Traccar endpoint.

The point of the UDS work is the rich signal set behind VAG's diagnostic space:
generic OBD-II (Mode 0x01) exposes ~20 PIDs; the interesting diesel signals live
in the ECU's manufacturer DID space and are the reason this project exists.

---

## 2. Hardware Platform

This unit reports `TYPE:14` at boot (`sys.devType`, set from `ATI`) — a ONE+
**Model B+**.

| Component | Detail |
|-----------|--------|
| MCU | ESP32 (Xtensa LX6, 160 MHz, no PSRAM) |
| OBD co-processor | Freematics co-processor over UART2 (`LINK_UART_NUM`), ELM327-compatible CAN |
| Cellular modem | **SIM7670G-LN** (auto-detected at boot via `ATI`; SIM7600/7070/5360 also handled) |
| GNSS | Onboard u-blox, reached **through the OBD co-processor** (`ATGPSON`/`ATGPS`) — see §9 |
| MEMS | ICM-42627 (primary) or ICM-20948 (fallback), auto-detected over I2C |
| Storage | MicroSD, FAT32, files at `/DATA/<n>.CSV` |
| Power | Vehicle OBD-II port (12 V) |

The SIM7670G's own GNSS engine is **not** used (the fork's driver lacks a 7670
branch — §9). The u-blox standalone path is strictly better on this board.

---

## 3. Firmware Architecture

Two cooperating execution contexts on core 1, communicating only through a
shared ring buffer — no shared mutable device state, so no locks.

```
 loop() / process()                    telemetry()  [FreeRTOS task, prio 2, 8 KB]
 ─────────────────────                 ──────────────────────────────────────────
 processOBD()   OBD-II PIDs            cellular connect / reconnect
 processUDS()   UDS DIDs               pull newest buffer → CStorageRAM → UDP
 processMEMS()  accel / gyro           RSSI monitoring, ping-back in standby
 processGPS()   u-blox via link        server-sync watchdog
 SD write (CSVlogger)
        │                                       ▲
        └──────────►  CBufferManager  ──────────┘
                      (32 slots × 256 B ring)
```

`process()` fills one `CBuffer` per cycle and marks it `FILLED`. `telemetry()`
takes the **newest** filled buffer, serialises it, transmits, and frees it; if
production outruns transmission, older buffers are dropped rather than queued
unbounded.

### Source layout

```
src/
  main.cpp        entry point: setup/loop, process(), the OBD+UDS schedulers, standby
  teleclient.cpp  TeleClientUDP, CBuffer / CBufferManager
  telestore.cpp   SDLogger (CSV) + CStorageRAM (TX serialisation)
  dataserver.cpp  stub (serverProcess = delay; HTTPD removed)
include/
  config.h        all build-time configuration
  uds_dids.h      the per-vehicle UDS DID table + decoders (the swap-a-car file)
  teleclient.h    CBuffer, CBufferManager, TeleClientUDP
  telestore.h     CStorage, SDLogger, CStorageRAM
libraries/
  FreematicsPlus/ upstream Freematics SDK (HAL, OBD, MEMS, network, UDS) — used as-is
platformio.ini    build: env esp32dev, 160 MHz, huge_app partition
```

---

## 4. Data Flow & Wire Format

Each transmitted packet is one text frame built by `CStorageRAM`:

```
<devid>#<PID>:<value>,<PID>:<value>,...*<checksum>
```

- PIDs are **uppercase hex**; values are decimal (UDS DIDs emit as `FLOAT_D2`).
- The leading `0:<ms>` is the timestamp PID; `*<checksum>` is a byte-sum hex tail.
- The same serialisation feeds both the UDP path and the SD CSV — UDS-derived
  values flow through unchanged, no downstream special-casing.

Example (GPS PIDs visible as `A`/`B`/`C`/`D`/`E`/`F`/`12`/`10`):

```
ABCD1234#0:82838,...,10:11193940,A:-37.813600,B:144.963100,C:91,D:0.1,E:0,F:8,12:102,...
```

---

## 5. Configuration (`config.h`)

Build-time options live in `config.h`; a runtime NVS override (namespace
`storage`) takes precedence where present.

| Setting | Current value | Notes |
|---------|---------------|-------|
| `CELL_APN` | `"hologram"` | `iot.1nce.net` staged (commented) for the pending 1NCE migration |
| `SERVER_HOST` | `traccar.example.com` | Traccar endpoint |
| `SERVER_PORT` / `SERVER_PROTOCOL` | `5170` / UDP | UDP only |
| `SIM_CARD_PIN` | `""` | `checkSIM()` has a lockout guard to avoid PIN-retry exhaustion |
| `GNSS` | `GNSS_STANDALONE` | onboard u-blox via co-processor — see §9 before changing |
| `STORAGE` | `STORAGE_SD` | FAT32 MicroSD |
| `ENABLE_MEMS` / `ENABLE_OBD` | `1` / `1` | MEMS required for motion wakeup |
| `ENABLE_WIFI/OLED/HTTPD/BLE/ORIENTATION` | `0` | trimmed off |
| `BUFFER_SLOTS` × `BUFFER_LENGTH` | `32` × `256 B` | ring depth (no PSRAM) |
| `DATA_INTERVAL_TABLE` | `{1000,2000,5000}` ms | cycle period by stationary tier |
| `STATIONARY_TIME_TABLE` | `{10,60,180}` s | motionless thresholds → interval/standby |
| `MOTION_THRESHOLD` | `0.4 G` | wakeup sensitivity |
| `MAX_OBD_ERRORS` | `3` | re-init OBD after this many |
| `MAX_CONN_ERRORS_RECONNECT` | `5` | then power-cycle cellular + 3 min cool-off |
| `SERVER_SYNC_INTERVAL` | `120 s` | poor-connection watchdog |
| `SIGNAL_CHECK_INTERVAL` / `PING_BACK_INTERVAL` | `10 s` / `900 s` | RSSI poll / standby keepalive |
| `COOLING_DOWN_TEMP` | `75 °C` | purge buffers above this device temp |

Runtime-overridable via NVS: `CELL_APN` (key `CELL_APN`).

---

## 6. OBD-II Acquisition (`processOBD`)

A small tiered table (`obdData[]`): tier-1 PIDs (speed, RPM, throttle, engine
load) are read **every cycle**; tier-2/3 PIDs rotate one-per-cycle. A failed
read breaks the cycle's OBD pass (errors counted; after `MAX_OBD_ERRORS` the ECU
link is re-initialised, then `uds.applyHeaderConfig()` re-applied).

`processOBD()` sets the module-level `obdRpmValid` flag when the RPM read
succeeds; the UDS scheduler consults it for derived-inputs-group atomicity (§8).

---

## 7. UDS Subsystem

### 7.1 Protocol stack

```
UDS (ISO 14229)      service 0x22 ReadDataByIdentifier        ← what we use
ISO-TP (ISO 15765-2) single/first/consecutive frame + flow   ← co-processor handles SF
CAN (ISO 11898)      8-byte frames @ 500 kbps                 ← co-processor handles
```

Request is `22 <DID-hi> <DID-lo>`; positive response is `62 <DID echo> <data…>`;
negative is `7F 22 <NRC>`.

### 7.2 Confirmed link-API behaviour (the facts the implementation relies on)

- **Outbound single-frame ISO-TP is automatic.** Pass raw UDS bytes
  (`{0x22,hi,lo}`) to the co-processor; it prepends the single-frame length.
  Never build ISO-TP framing in ESP32 code.
- **Responses come back as ASCII hex text**, e.g. `"62 F4 05 64 \r\r"` — the
  co-processor has already stripped the length byte and CAN-ID prefix. The
  parser walks 2-digit hex tokens and checks for `0x62` / `0x7F`.
- **No session management needed.** Every DID we care about answers in the
  default session — zero `7F 22 7E` across the whole validation run. No extended
  session, no tester-present keepalive. (Retrofit per-DID if a future signal
  ever needs it; don't build it preemptively.)
- **Dual-ECU addressing.** Two ECUs matter on the powertrain bus:
  - **Engine ECU** — request `0x7E0`, response `0x7E8` — engine, NOx, DPF, most data.
  - **SCR / reductant module** — request `0x7EA`, response `0x794` — AdBlue
    tank/pump/pressure/temperature.
  `CUDS` selects the request ID per DID (`UDS_ECU_ENGINE` / `UDS_ECU_SCR`). A
  **wide receive filter** (mask `0xFFFFF800` + filter `0x7E0`) catches both
  response IDs; `uds.applyHeaderConfig()` installs it after each OBD init.
- **Multi-frame ISO-TP is not auto-assembled** by the co-processor, but is
  **avoided entirely**: every multi-frame composite DID (`0xF483`, `0xF485`) has
  single-frame equivalents already in the table. The flow-control recipe is
  documented in git history if ever needed (e.g. part-number string `0xF187`).

### 7.3 Negative-response codes worth knowing

| NRC | Meaning | Notes |
|-----|---------|-------|
| `0x12` | sub-function not supported | DID not implemented |
| `0x13` | incorrect length | malformed request |
| `0x22` | conditions not correct | e.g. needs engine running |
| `0x31` | request out of range | DID exists, value not available now |
| `0x33` | security access denied | DID locked (out of scope) |
| `0x78` | response pending | keep waiting; not an error |
| `0x7E`/`0x7F` | not supported in active session | would need extended session |

### 7.4 The DID table (`include/uds_dids.h`)

The per-vehicle artifact — swapping cars is a one-file change. Each row binds a
UDS DID to: the ECU request ID, a **synthetic PID** for the ring buffer, expected
payload byte count, a cadence tier (§7.5), a decoder, and a unit string.

Synthetic PIDs occupy `0x200+` so they never collide with OBD-II PIDs (which are
OR'd with `0x100` → `0x101..0x1FF`) or internal GPS/MEMS PIDs. The full current
map (`0x200..0x232`, 51 signals) lives in `uds_dids.h`; it is the canonical list.

### 7.5 The scheduler (`processUDS`)

A per-DID countdown scheduler with a per-cycle read budget:

- Each DID has a tier — `UDS_FAST` (1, every cycle), `UDS_SLOW` (10),
  `UDS_GLACIAL` (60) — plus the special `UDS_GROUP` tier (§8).
- A `wait[]` counter per DID counts down; at 0 it's due and is rescheduled to
  its tier on read. At most `UDS_MAX_PER_CYCLE` (**8**) reads happen per cycle,
  bounding added latency (each read ≤ ~100 ms).
- **Work-conserving:** a rotating cursor (`rr`) gives the leftovers first refusal
  next cycle when more DIDs are due than the budget allows — nothing starves.
- Group members (`UDS_GROUP`) are **exempt** from the round-robin; `readGroup()`
  co-reads them on a shared cadence (§8).

Non-group fast-tier DIDs are the 3 EGTs + nox1 + nox2 (5), comfortably under the
budget on non-group cycles. Tiers, `UDS_GROUP_PERIOD` and `UDS_MAX_PER_CYCLE` are
all one-line tunables.

### 7.6 Decoder families — and the rule that governs them

Decoders in `uds_dids.h` are grouped by **encoding family**, validated from real
captures. The hard-won rule: **never extrapolate a DID's formula from its
physical category.** Pressures alone use ≥7 different encodings; temperatures
split between J1979 `B−40` and VAG deciKelvin; percentages come in three
flavours. Every new DID needs its value captured and its formula fitted.

| Family | Bytes | Formula | Example signals |
|--------|-------|---------|-----------------|
| J1979 temp (1-byte) | 1 | `B − 40` °C | coolant, IAT, reductant tank/pump |
| J1979 temp (2-byte) | 2 | `u16/10 − 40` °C | EGT `0xF43C/0xF43E` |
| VAG deciKelvin | 2 | `u16/10 − 273.1` °C | oil, EGT-raw, ctrl-mod, fuel, charge-cooler, turbo-out, SCR cat |
| VAG mbar→bar | 2 | `u16/1000` bar | oil press, IMAP, ambient |
| Direct hPa | 2 | `u16` hPa | charge spec, DPF Δp |
| Binary 1/128 hPa | 2 | `u16/12.8` hPa | charge actual `0x1B0E` |
| kPa ×10 | 2 | `u16/10` kPa | fuel low press |
| hPa ×100 | 2 | `u16×100` hPa | rail press |
| 1-byte kPa | 1 | `B` kPa | baro `0xF433` |
| **signed** mbar | 2 | `(int16_t)` mbar | reductant line press (swings through 0) |
| percent ×100 | 2 | `u16/100` % | fuel reg, metering |
| **signed** percent ×100 | 2 | `(int16_t)/100` % | HP-EGR `0x1334` (reads negative warm) |
| binary-fraction % | 2 | `u16/8192 × 100` % | EGR cmd/act, reductant level |
| ratio ×100 / ×1000 | 2 | `u16/100` or `/1000` | SCR adaptation / efficiency, lambda |
| Nm ×10 | 2 | `u16/10` Nm | engine torque |
| g/s ×100 | 2 | `u16/100` g/s | MAF |
| kg/h ×10, mg ×10 | 2 | `u16/10` | air-mass meter / spec |
| L/h ×20 | 2 | `u16/20` L/h | fuel rate |
| L ×100, g ×100 | 2 | `u16/100` | fuel level, DPF soot/ash |
| V ×1000 | 2 | `u16/1000` V | reductant level voltage |
| raw u16 | 2 | `u16` | NOx ppm, reductant pump rpm |
| **u32 (4-byte)** | 4 | `u32` (single-frame, DLC 07) | odometer (km), regen time (s) / distance (m) |

Gotchas baked into the decoders: cast signed families to `int16_t` before
scaling (else negatives wrap to ~65500); 4-byte values arrive big-endian in a
single frame; prefer the ECU's interpreted value over raw-voltage twins.

---

## 8. The Derived-Inputs Group (atomic co-emission)

Downstream analytics — volumetric efficiency, power, AFR/lambda, BSFC, brake
thermal efficiency — each need several signals sampled at the **same instant**
so a single packet becomes a self-contained row (no SQL windowing). Six signals
form the group:

| Signal | PID | Source | Used by |
|--------|-----|--------|---------|
| `rpm` | `0x10C` | OBD live | power, VE |
| `maf` | `0x21A` | UDS | VE, AFR, BSFC |
| `charge_press_act` | `0x214` | UDS | VE (manifold density) |
| `iat` | `0x20A` | UDS | VE (manifold density) |
| `fuel_rate` | `0x21D` | UDS | AFR, BSFC, brake thermal eff. |
| `engine_torque` | `0x21F` | UDS | power, BSFC, brake thermal eff. |

**Why a group, not just "all fast":** under the budget'd round-robin, members
drift apart and rarely land in the same packet. The five UDS members are tagged
`UDS_GROUP` and co-read by `readGroup()` every `UDS_GROUP_PERIOD` (**3**) cycles
(~0.33 Hz — atomicity matters, 1 Hz does not).

**Atomicity is all-or-nothing.** `readGroup()` stages all five into a temporary
and commits to the packet only if every member **and** `rpm` (`obdRpmValid`) read
cleanly that cycle; on any failure it emits none, so a packet never carries a
partial group. On a group cycle the five reads take budget priority (5 of 8,
leaving 3 for the round-robin); the other two cycles give the full budget of 8 to
everything else.

**Validation invariant:** in a capture, the five UDS tokens
(`20A 214 21A 21D 21F`) are strictly all-present or all-absent, and appear with
`10C`, on ~every third data packet. `rpm`-only packets on the off-cycles are
normal. Other PIDs (NOx, EGT, DPF, SCR, odometer) keep their own tiers — they
tolerate mild misalignment and are not in the group.

---

## 9. GNSS Subsystem

**Active mode: `GNSS_STANDALONE`.** The onboard u-blox is reached **through the
OBD co-processor**, not a dedicated ESP32 UART:

- `initGPS()` tries `gpsBeginExt()` (dedicated UART) first; on this TYPE-14 board
  that times out, so `gpsBegin()` takes over — it issues `ATGPSON` to the
  co-processor and reads fixes via `ATGPS` (`$GNIFO` parsed by `gpsGetData()`).
  Boot therefore prints **`GNSS:OK(I)`** (internal-link path).
- Consequence: **if the co-processor link drops, GPS drops with it.**

**Why not cellular GNSS:** `CellSIMCOM::setGPS()` in the fork only branches for
SIM7070 (`AT+CGNSPWR`) vs everything-else (`AT+CGPS`, correct for SIM7600). The
**SIM7670G needs `AT+CGNSSPWR`/`+CGNSSINFO`**, which is absent — so in
`GNSS_CELLULAR` the modem never powers GNSS, `getLocation()` returns false, and
no GPS PIDs are emitted. The dead path is left intact; don't switch back without
first patching `FreematicsNetwork.cpp` for a `CELL_SIM7670` GNSS branch.

**Reading `processGPS()` output:**
- Time-only `[GNSS]` lines = tracking but no fix (lat/lng still 0); no PIDs added.
- HDOP on the wire is **HDOP × 10** (`102` = 10.2, marginal; roof antenna → 1–2).
- `gd->speed` is knots in the raw struct; `processGPS()` writes km/h (× 1.852).
- `lastMotionTime` only updates at `kph ≥ 2`, feeding stationary-shutdown.
- Proven cold-start TTFF on a window sill ≈ 69 s; position correct end-to-end to
  Traccar.

**Stale path to fix if cellular GNSS ever returns:** the `GNSS_RESET_TIMEOUT`
block in `process()` unconditionally calls the standalone reset routines; it
would need a `GNSS_CELLULAR` branch.

---

## 10. MEMS / Motion

ICM-42627 or ICM-20948, auto-detected. `processMEMS()` averages accelerometer
samples per cycle (bias-corrected via `calibrateMEMS()`), emits `PID_ACC`, and
tracks device temperature. Motion feeds both the standby decision and wakeup.

---

## 11. Standby / Wakeup

```
vehicle stops → motionless timer expires → standby()
  ├── SD file closed, OBD co-processor slept, MEMS re-calibrated
  └── waitMotion(-1) polls accel until motion ≥ MOTION_THRESHOLD (0.4 G)
        └── ESP.restart()   ← full reset (RESET_AFTER_WAKEUP = 1)
```

Full reset on wake is intentional: clean cellular re-registration, no stale state
machine. Stationary thresholds step the data interval 1 s → 2 s → 5 s as the
vehicle sits (`STATIONARY_TIME_TABLE` / `DATA_INTERVAL_TABLE`). During standby the
telemetry task pings every `PING_BACK_INTERVAL` (900 s).

---

## 12. Cellular / Network Behaviour

- Modem auto-detected via `ATI` (SIM7670G-LN on this unit).
- `initCell()` does APN setup, registration, IP acquisition.
- RSSI polled every `SIGNAL_CHECK_INTERVAL` (10 s), logged as `PID_CSQ`.
- After `MAX_CONN_ERRORS_RECONNECT` (5) consecutive failures the modem is powered
  off with a **3-minute cool-off** to avoid carrier banning.
- A poor-connection watchdog reconnects if no server sync within
  `SERVER_SYNC_INTERVAL` (120 s).
- `VERBOSE_LINK 1` in `FreematicsPlus.cpp` enables AT-command tracing for SIM
  debugging.

---

## 13. SIM Card Notes

- **Hologram** (`hologram`): confirmed working — connects, gets IP, transmits.
- **1NCE** (`iot.1nce.net`): migration pending; APN staged but commented. Earlier
  testing didn't obtain an IP — revisit APN/data-plan config during the swap.
- SIM PIN: `checkSIM()` includes a lockout guard to prevent PIN-retry exhaustion.

---

## 14. Storage

MicroSD, FAT32, `/DATA/<n>.CSV`. The same serialised frames written to UDP are
appended to the CSV and flushed every ~1 KB.

---

## 15. Validation (on-car)

Timing/behaviour is verified by capture, not value assertions:

1. **Group atomicity (§8):** over ~60 s of `[DAT]` lines, confirm the five UDS
   tokens co-occur (all-or-nothing) with `10C` on ~every third packet, and that
   EGT/NOx still appear at roughly their prior rate on off-cycles.
2. **SCR addressing:** the five `scr`-ECU DIDs (request `0x7EA`/response `0x794`)
   are the only novel request path; `[UDS] reduct_* FAIL` points at the wide
   filter or the `0x7EA` header.
3. **Cycle time:** 8 UDS reads + the OBD poll can push a cycle past the 1 s
   floor; the loop tolerates it (end-of-cycle delay vanishes). If sample rate
   suffers, lower `UDS_MAX_PER_CYCLE` or demote DIDs a tier.
4. **Decoder sanity:** `hpegr_activation` reads negative warm (~−11 %),
   `reduct_line_press` swings through 0, binary-fraction signals read ~99–100 %.
5. **Engine-off placeholders** (not bugs): lambda DIDs pin ~31.99 and
   `scr_cat_eff` reads 0.000 until sensors/CAT are hot.

**Reference signatures** (useful sanity checks): at key-on→start, oil pressure
jumps ~1.0→2.6 bar and fuel-reg duty ~9→19 %, while coolant/EGT lag (thermal
mass); ambient pressure is unchanged. `0xF43C` (standardised EGT) and `0x10FB`
(VAG raw) read the same sensor differently — capture both, they're not redundant.
On this throttle-less diesel, `0x103C` IMAP ≈ ambient at idle is correct.

---

## 16. Build & Flash

PlatformIO:

```sh
pio run -e esp32dev             # compile
pio run -e esp32dev -t upload   # flash
```

**Serial-monitor gotcha (CH340 on this unit, `/dev/cu.usbserial-*`):** for a
clean reset, **pulse RTS only — never DTR.** On this wiring DTR drives IO0;
pulsing it during the EN-release window drops the ESP32 into download mode
("waiting for download"). `pio device monitor` needs a real TTY; for scripted
capture, drive the port via pyserial with `dtr=False` and an RTS-only pulse, or —
if the unit is already running — just open with `dtr=False, rts=False` and read
without resetting.

---

## 17. Known Warnings (non-fatal)

Appear every build, from transitively-included ESP-IDF headers; ignore:

```
esp_spi_flash.h is deprecated → use spi_flash_mmap.h
legacy adc driver is deprecated → use esp_adc/adc_oneshot.h
```

A benign `task_wdt: TWDT already initialized` may print during co-processor
link-up.

---

## 18. Non-Goals / Out of Scope

- **No UDS writes** (0x2E), **no SecurityAccess** (0x27), **no UDS DTC** (0x19) —
  read-only; OBD-II Mode 0x03 DTCs suffice.
- **No co-processor firmware changes** — pure ESP32-side C++.
- **No multi-frame ISO-TP** — single-frame equivalents cover every needed signal.
- **No 29-bit extended CAN IDs** — this car's diagnostic stack is 11-bit.
- **Don't change the wire format** — synthetic UDS PIDs (`0x200+`) ride the
  existing CSV/UDP path.
- **Don't rename the upstream SDK** under `libraries/` — it's Freematics' code.
- **Don't strip Stanley Huang's BSD copyright** from derived files.

---

## 19. References

- ISO 14229-1 (UDS services), ISO 15765-2 (ISO-TP), ISO 11898 (CAN).
- Vehicle: Audi A6 Allroad 3.0 TDI, EA897 evo, ECU `4G2 907 311 B`
  (Bosch/Continental MD1). DIDs validated against VCDS + live CAN captures.
- Upstream: [Freematics](https://github.com/stanleyhuangyc/Freematics) by Stanley
  Huang — the Telelogger this project derives from.
