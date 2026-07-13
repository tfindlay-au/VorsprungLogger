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
ZKUCAQHM#0:82838,...,10:11193940,A:-37.710648,B:144.906204,C:91,D:0.1,E:0,F:8,12:102,...
```

---

## 5. Configuration (`config.h`)

Build-time options live in `config.h`; a runtime NVS override (namespace
`storage`) takes precedence where present.

| Setting | Current value | Notes |
|---------|---------------|-------|
| `CELL_APN` | `"hologram"` | `iot.1nce.net` staged (commented) for the pending 1NCE migration |
| `SERVER_HOST` | `traccar.bielenberg.id.au` | Traccar endpoint |
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

## 16. Field Observations — GNSS Reliability Baseline (pre-external-antenna)

> **Snapshot: 2026-06-01.** This is a *measured baseline*, not a design decision —
> it records the GNSS behaviour of the **internal antenna** (co-processor
> `GNSS:OK(I)` path, §9) so a future re-log after the external antenna is fitted
> can be compared against the same numbers. Re-run the analysis after ~1 week of
> driving and update this section with an "after" column.

**Dataset:** full SD card dump (`/DATA/*.CSV`), 104 of 121 files non-empty,
**52,346 logged cycles**. The 17 empty/near-empty files are reboot/wake churn
(each file is one session between `ESP.restart()` events, §11).

**Method (reproducible):** flat `PID,value` lines are grouped into per-cycle
records (a new cycle begins whenever a PID repeats, since no PID appears twice in
a cycle). A record "has a fix" if it carries both `A` (lat) and `B` (lng). The
key trick is using **OBD vehicle speed `10D` as an independent motion reference**
— it tells us the car was moving even when GNSS produced nothing, which
separates "no fix while driving" from "parked / cold-starting."

**Findings:**

- **39.9 %** of all cycles (20,870 / 52,346) carried **no position**, while OBD
  logged normally — so the device was alive and writing to SD throughout.
- **35.7 %** of *moving* cycles (OBD speed ≥ 5 km/h) had **no fix** (13,899 /
  38,921). This is position loss while genuinely driving, not while stationary.
- The failure is **bimodal, not gradual.** Of 58 real drives (files with ≥ 20
  OBD-speed samples): **33 good** (≥ 50 % coverage), **9 partial**, **16 with
  zero fix for the entire drive** (files 12–15, 18, 19, 21, 23, 25–28, 36, 38, 66,
  120). Examples: `21.CSV` — 1848 records, 0 fixes, up to 82 km/h; `25.CSV` — up
  to 111 km/h, 0 fixes.
- **When it does lock, reception is healthy:** satellite count peaks at 9–12
  (18,539 samples at 12 sats), HDOP good (remember the wire value is HDOP × 10,
  §9). Gradual signal weakness would show many marginal 3–4-sat fixes; we don't
  see that. "Solid lock or total silence" is the signature of an **intermittent
  antenna / connector (or GNSS failing to init on some boots)** — not a
  reception-quality problem. The zero-fix drives cluster (files 12–28), consistent
  with a physical connection bad for a stretch.
- **Standby churn amplifies it.** With `GNSS_ALWAYS_ON = 0` (§11), GPS cold-starts
  on every wake (TTFF ≈ 69 s per §9). Several files take 400–600 cycles to their
  first fix, and short post-wake segments end before a fix ever arrives.

**Conclusion — the core issue is lost *fix*, not lost *transmission*.** The gaps
seen at the Traccar server are also gaps on the SD card: the position was never
captured, so there was nothing to transmit. This was the question that drove the
analysis, and it settles the solution space:

- **Cellular GNSS would not help** — it is the modem's own *satellite* receiver
  (and unimplemented for the SIM7670G anyway, §9); same failure mode, different
  antenna.
- **Store-and-forward / SD backfill would not have helped** — nothing was logged
  to forward.
- **External GNSS antenna (pending)** targets the root cause directly; expected to
  be the primary fix. Check the physical connector too — the bimodal pattern
  smells like a loose u.FL.
- **CAN-bus GNSS from the car's nav unit is the chosen plan-B** — the only
  genuinely *independent* source (car roof antenna + its own receiver). Deferred
  until the external antenna is evaluated; if pursued, it needs (1) VCDS capture
  of the nav-unit request/response CAN IDs and lat/lng/time DIDs + scaling, and
  (2) a reachability spike confirming those DIDs are routable through the gateway
  from the OBD port. Integration would follow the atomic `readGroup()` pattern
  (§8) with a staleness gate so it only fills when the u-blox is stale; coordinates
  need a dedicated scaled-`int32`→float decoder (the generic `udsDecodeU32_4b`
  overflows float's exact-integer range for degrees × 1e7).
- **Standby / `GNSS_ALWAYS_ON` tuning** is a cheap, independent win that removes
  the cold-start amplifier regardless of the antenna outcome.

### 16.1 After external antenna fitted — 2026-06-19

> **Snapshot: 2026-06-19.** External u-blox antenna fitted (loose-laid, not yet
> mounted/dressed) and the car driven for a long afternoon trip. Same comparison
> method as above, but against Traccar's Postgres (`tc_positions`) rather than
> the SD dump, since the device is back online and transmitting.

**Dataset:** 36-hour window ending 2026-06-19 11:16 UTC, device `Allroad`
(`uniqueid ZKUCAQHM`), **17,307 rows** across 11 drive sessions — including a
170-minute afternoon drive (session #10, 05:40–08:30 UTC = 15:40–18:30 AEST,
9,882 rows) and a 94-minute return (session #11, 5,328 rows). OBD speed `10D`
used as the independent motion reference as before.

| Metric | Pre (internal antenna) | Post (external antenna) |
|---|---|---|
| Rows with **no fix**, overall | 39.9 % | **0.0 %** (0 / 17,307) |
| Rows with **no fix**, while moving (OBD ≥ 5 km/h) | 35.7 % | **0.0 %** (0 / 13,949) |
| Drives with zero fix end-to-end | 16 / 58 | **0 / 11** |
| Satellite count (median) | mixed, often 0 | **12** |
| Inter-row cadence — p50 / p90 / p99 | — | 0.98 s / 2.0 s / 4.5 s |

**Conclusion — the external antenna resolves the GNSS reliability issue.** The
bimodal "solid lock or total silence" failure has not occurred in any session in
this window; every moving sample carried a valid fix. Position track over the
afternoon drive spans ~59 km N–S and 1.6° of longitude with GPS speed tracking
OBD speed cleanly (e.g. 110 km/h GPS vs 110 km/h OBD on the freeway leg).

**Caveats noted but not blocking:**

- `hdop` reports `255` on a handful of rows — sentinel for "no DOP available yet"
  while `valid=true` from a cached fix. Median hdop is 5 (i.e. 0.5 once the
  ×10 wire scaling is reversed, §9).
- `sat` outliers (min 1 during warm-up, max 59) suggest the new module is
  reporting combined GNSS-system count in a single field; doesn't affect
  position quality.
- Antenna is **not yet permanently mounted** — wire run still needs to be hidden
  and the puck fixed down. Re-confirm after mounting to make sure the cable
  dressing doesn't introduce a connector flex problem.
- **Plan-B (CAN-bus nav GNSS)** is therefore parked, not abandoned. No further
  VCDS / gateway spike needed unless the external antenna regresses after
  mounting.

### 16.2 Post-v1.1.0 field check — 2026-07-13

> **Snapshot: 2026-07-13.** First field data review since the v1.1.0 flash
> (2026-07-11: hardened NMEA parser, date PID, external GPS on UART0 RX).
> Same Postgres method as §16.1. Window: everything since the flash —
> **25,961 rows** over ~60 h, 11 drive sessions.

**Verified good:**

- **Phantom-midnight corruption has not recurred.** No `.000`-ms phantom rows,
  no corrupt `io768` dates. Heartbeats and a drive segment cross UTC midnight
  cleanly. The §16-era parser fix is holding in the field.
- **Transport is healthy.** Inter-row cadence p50 0.87 s / p99 5.2 s. One real
  cellular outage in the whole window (2026-07-12 03:27:49→03:28:51 UTC, 63 s at
  ~100 km/h): RAM-buffered rows replayed with correct devicetimes (servertime
  lag draining 126 s → 4 s), confirming the §14 spool/replay path works at the
  transport level. The 63 s hole itself contains no rows — consistent with the
  blocking modem reconnect stalling record production (known design tradeoff,
  acceptable).

**Finding A — records without GPS payload carry no timestamp; spool drains
mis-date them.** `processGPS()` gates PID `0x10`/`0x11`/`PID_ABS_TIME` on
`gd->date`, so a record built between GPS updates has *no time information at
all* and Traccar stamps it with **arrival time**. Exact partition confirmed:
the 4,399 rows (17 %) with servertime ≡ devicetime are precisely the rows
lacking `sat`/`io768`. Live this is harmless (<1 s error); on spool drain each
GPS-less record lands at *drain* time, not capture time. Observed damage:

- Old idle-engine content (OBD 0–4 km/h, rpm ~780, 12.0 V battery) interleaved
  into a live 85 km/h stream at reconnect (7/12 03:20:30, 7/11 04:42:30) —
  wrong time *and* wrong position (Traccar copies the last live location onto
  timestampless packets).
- A **phantom 1-minute "session"** at 2026-07-12 00:02:54: 281 rows in 60 s
  (4.7 rows/s — faster than live production), 0 % GPS payload, moving-OBD
  content from an earlier capture, all collapsed onto the drain moment.
- Server-side `totalDistance` corrupted by the out-of-order arrivals
  (kilometre-scale backwards jumps around every drain).

**Finding B — a real ~25-min GNSS payload blackout on 2026-07-13, masked by
the fix metric.** Drive 06:22–06:51 UTC (real drive; OBD shows the full
accelerate/decelerate/park profile): GPS payload in only **3.6 %** of records —
nothing at all 06:22:52→06:37:39 and again 06:39:49→06:49:40. Track frozen at
three coordinates. `stats` still reported **100 % fix** because Traccar carries
the last known position forward and marks it `valid`. Healthy sessions run
72–97 % GPS payload. First blackout since the antenna install; it sits on the
new UART0 RX path. Recovered on its own (next drive 87 %). Cause unknown —
needs a serial capture (does `[GNSS]` output stop, or does data flow but fail
the stale/jump checks?).

**Proposed fixes — PENDING, not yet implemented.** Plan: collect more drive
data through the week, re-analyse ~2026-07-16 (Thu); if the diagnosis stands
up, implement + flash then.

1. **Timestamp every record** (fixes Finding A). Bake a device-side epoch into
   every serialized record, not just GPS-carrying ones: hold a time anchor
   (last GPS epoch, else network/LOGIN time) plus a `millis()` delta at record
   build. `PID_ABS_TIME` exists precisely for this; emit it unconditionally
   once any anchor is known. GPS `0x10`/`0x11` stay gated on `gd->date` as
   now. Requires Traccar to honour it — the server decoder maps `0x300` to
   attribute `io768` only, so either (a) also emit `0x10`/`0x11` derived from
   the anchor epoch, which the stock decoder already turns into devicetime, or
   (b) patch the server decoder. Option (a) is firmware-only and preferred.
2. **Make the baseline script measure real GNSS availability** (exposes
   Finding B instead of masking it). `traccar_baseline.py` must count GPS
   payload presence (`sat`/`io768` in attributes), not Traccar `valid`; report
   per-session payload %, and flag zero-lag row bursts (drain signatures).
   This was already a §16 TODO; Finding B upgrades it to required.
3. **Instrument the GNSS path for the blackout** (diagnoses Finding B).
   Serial capture on a real drive; if not reproducible on demand, add a
   lightweight counter PID (NMEA sentences seen / checksum failures /
   stale-jump rejections per minute) so the next blackout self-diagnoses from
   the Traccar side.
4. *(Optional, lower priority)* Consider draining spool records oldest-first
   only while a monotonic guard holds, or tagging drained packets, so any
   future timestampless stragglers can't forge phantom sessions.

---

## 17. Build & Flash

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

## 18. Known Warnings (non-fatal)

Appear every build, from transitively-included ESP-IDF headers; ignore:

```
esp_spi_flash.h is deprecated → use spi_flash_mmap.h
legacy adc driver is deprecated → use esp_adc/adc_oneshot.h
```

A benign `task_wdt: TWDT already initialized` may print during co-processor
link-up.

---

## 19. Non-Goals / Out of Scope

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

## 20. References

- ISO 14229-1 (UDS services), ISO 15765-2 (ISO-TP), ISO 11898 (CAN).
- Vehicle: Audi A6 Allroad 3.0 TDI, EA897 evo, ECU `4G2 907 311 B`
  (Bosch/Continental MD1). DIDs validated against VCDS + live CAN captures.
- Upstream: [Freematics](https://github.com/stanleyhuangyc/Freematics) by Stanley
  Huang — the Telelogger this project derives from.
