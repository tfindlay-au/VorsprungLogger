# VorsprungLogger — Software Project Design Document

An Audi-focused vehicle telemetry logger for the Freematics ONE+ Model B (ESP32).
Derived from the Freematics **Telelogger** by Stanley Huang (BSD; attribution
retained), narrowed to one vehicle and one job: log a 2017 Audi A6 Allroad
(3.0 TDI, EA897 evo) over OBD-II + UDS, plus GPS/MEMS, to SD and to a Traccar
server over cellular.

This document is the design reference. `TIME.md` covers record timestamping in
depth, `CHANGELOG.md` the per-release history, and §16 here is the dated field
log with a current status table at its head.

---

## 1. Purpose

Turn a Freematics ONE+ Model B into a vehicle telemetry logger that:

- reads OBD-II live PIDs from the engine ECU;
- reads 51 manufacturer-specific **UDS DIDs** (ISO 14229, service 0x22) from the
  VAG EA897 powertrain — EGTs, pressures, air/fuel flow, torque, DPF, SCR/NOx;
- records position via a u-blox GNSS receiver (external module preferred,
  onboard receiver as fallback) and motion via the MEMS;
- streams every record over UDP to a Traccar endpoint, stamped with its own
  capture time; anything unsent during a cell outage is spooled to SD and
  replayed when the link returns.

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
| GNSS (primary) | **External u-blox module** on the molex header: module TX → GPIO34 (UART0 RX, shared with the console), module RX ← GPIO26 (bit-banged UBX commands), VCC switched by GPIO12. External antenna fitted 2026-06-19. See §9 |
| GNSS (fallback) | Onboard u-blox, reached **through the OBD co-processor** (`ATGPSON`/`ATGPS`); used only when no external module answers at boot |
| MEMS | ICM-42627 (primary) or ICM-20948 (fallback), auto-detected over I2C |
| Storage | MicroSD, FAT32, used only as the outage spool: `/SPOOL/<seq>.PKT` (§14) |
| Buzzer | GPIO25; connect and GNSS-source chimes (§9, §12) |
| Power | Vehicle OBD-II port (12 V) |

The SIM7670G's own GNSS engine is **not** used (the fork's driver lacks a 7670
branch — §9). The u-blox standalone path is strictly better on this board.

Because the external module's receive line is UART0 RX, **the console runs at
38400 baud** (`Serial.begin(38400)` in `main.cpp`, `monitor_speed` in
`platformio.ini`), the rate the module is configured to at boot.

---

## 3. Firmware Architecture

Two cooperating execution contexts on core 1, communicating through a shared
ring buffer plus one small shared value: the UTC time anchor, written by the
main loop and read by the telemetry task under a spinlock (`TIME.md`). No other
shared mutable device state, so no other locks.

```
 loop() / process()                    telemetry()  [FreeRTOS task, prio 2, 8 KB]
 ─────────────────────                 ──────────────────────────────────────────
 capture tick (timeTicks)              cellular connect / reconnect, AT+CCLK → anchor
 processOBD()   OBD-II PIDs            pull newest buffer → buildPacket() → UDP
 processUDS()   UDS DIDs                 └ on send failure → RecordSpool (SD)
 processMEMS()  accel, device temp     while disconnected: spool queued buffers
 processGPS()   GNSS → anchor + PIDs   idle gaps: drain one spooled record
 GNSS watchdog                         RSSI monitoring, ping-back in standby
        │                              server-sync watchdog
        └──────────►  CBufferManager  ──────────┘
                      (32 slots × 256 B ring)

 third context: taskGPS (prio 1) decodes NMEA from UART0 RX into GPS_DATA
```

`process()` fills one `CBuffer` per cycle and marks it `FILLED`. While the link
is up, `telemetry()` takes the **newest** filled buffer, serialises it,
transmits, and frees it; if production outruns transmission, older buffers are
dropped rather than queued unbounded. While the link is down, queued buffers
are written oldest-first to the SD spool before and during each reconnect
attempt, so nothing is lost across an outage (§14).

### Source layout

```
src/
  main.cpp        entry point: setup/loop, process(), the OBD+UDS schedulers,
                  initGPS/processGPS, buildPacket(), telemetry task, standby
  teleclient.cpp  TeleClientUDP, CellUDPTime::networkTime, CBuffer / CBufferManager
  telestore.cpp   CStorageRAM (packet serialisation) + RecordSpool (SD outage spool)
  timeanchor.cpp  monotonic ticks, tiered UTC anchor, time-PID stamping (TIME.md)
  dataserver.cpp  stub (serverProcess = delay; HTTPD removed)
include/
  config.h        all build-time configuration, FIRMWARE_VERSION
  secrets.h       SERVER_HOST — gitignored; copy from secrets.h.example
  uds_dids.h      the per-vehicle UDS DID table + decoders (the swap-a-car file)
  teleclient.h    CBuffer, CBufferManager, TeleClientUDP, CellUDPTime
  telestore.h     CStorage, CStorageRAM, RecordSpool
  timeanchor.h    TIME_SRC_* ranks and the anchor API
libraries/
  FreematicsPlus/ upstream Freematics SDK (HAL, OBD, MEMS, network, UDS).
                  Modified in this fork: FreematicsPlus.cpp gpsBegin/gpsBeginExt/
                  gpsEnd (external-module detection on UART0 RX, baud sweep,
                  CFG-VALSET, probe log) and FreematicsGPS.cpp (NMEA parser
                  hardening). Everything else is upstream as shipped.
  FreematicsONE/, FreematicsOLED/, httpd/, TinyGPS/  upstream, unused by this build
tools/
  traccar_baseline.py  Postgres queries behind the §16 field checks
platformio.ini    build: env esp32dev, 160 MHz, huge_app partition, monitor 38400
CHANGELOG.md      per-release notes, keyed to FIRMWARE_VERSION
```

Not in the public repo (gitignored, deployment-specific): `homeassistant/`
(Home Assistant dashboard and template sensors) and `sql/` (the `vorsprung`
Postgres reporting schema over Traccar's `tc_positions`). The deployed Traccar
is a local fork whose Freematics decoder maps the `0x200+` PIDs to named
attributes; stock Traccar stores them as `io<decimal>`.

---

## 4. Data Flow & Wire Format

Each transmitted packet is one text frame built by `CStorageRAM`:

```
<devid>#<PID>:<value>,<PID>:<value>,...*<checksum>
```

- PIDs are **uppercase hex**; values are decimal (UDS DIDs emit as `FLOAT_D2`).
- The leading `0:<ms>` is the relative timestamp PID (capture tick in ms since
  boot); `*<checksum>` is a byte-sum hex tail.
- **Every packet ends with the time triplet** `10:<HHMMSScc>`, `11:<DDMMYY>`,
  `300:<epoch>`, written by `buildPacket()` from the record's capture tick and
  the UTC anchor — not from GNSS directly, so it is present with or without a
  fix. Traccar rebuilds `devicetime` from `10`/`11`; `300` lands as `io768`.
  See `TIME.md`.
- The exact bytes that would have gone on the wire are what the SD spool
  stores on a failed send (§14) — UDS-derived values flow through unchanged, no
  downstream special-casing.

Example (GPS PIDs `A` lat, `B` lng, `C` alt, `D` speed km/h, `E` heading,
`F` sats, `12` HDOP; `82` device temp in tenths; `24` battery in centivolts):

```
ABCD1234#0:82838,10D:0,10C:812,...,A:-37.813600,B:144.963100,C:91,D:0.1,E:0,F:8,12:102,82:310,10:11193940,11:180926,300:1789730379*7B
```

(values illustrative; the checksum is not computed for this line)

Two wire-scale traps that have each cost a diagnosis: `12` (HDOP) is HDOP×10
from the external receiver but HDOP×100 from the internal one (§9), and `82`
(device temperature) is tenths of a degree because that is how PID `0x82` is
defined and how Traccar divides it (§16.4, defect 5).

---

## 5. Configuration (`config.h`)

Build-time options live in `config.h`; a runtime NVS override (namespace
`storage`) takes precedence where present.

| Setting | Current value | Notes |
|---------|---------------|-------|
| `FIRMWARE_VERSION` | `"v1.3.2"` | printed at boot with the build timestamp; bump with every behaviour change and add a `CHANGELOG.md` entry |
| `CELL_APN` | `"hologram"` | `iot.1nce.net` staged (commented) for the pending 1NCE migration |
| `SERVER_HOST` | set in `secrets.h` | Traccar endpoint; see below |
| `SERVER_PORT` / `SERVER_PROTOCOL` | `5170` / UDP | UDP only |
| `SIM_CARD_PIN` | `""` | `checkSIM()` has a lockout guard to avoid PIN-retry exhaustion |
| `GNSS` | `GNSS_STANDALONE` | u-blox, external module preferred — see §9 before changing |
| `GNSS_RESET_TIMEOUT` | `300 s` | no GPS data for this long → warm re-init, then power cycle (§9) |
| `STORAGE` | `STORAGE_SD` | FAT32 MicroSD, spool only (§14) |
| `ENABLE_MEMS` / `ENABLE_OBD` | `1` / `1` | MEMS required for motion wakeup |
| `ENABLE_WIFI/OLED/HTTPD/BLE/ORIENTATION` | `0` | trimmed off |
| `BUFFER_SLOTS` × `BUFFER_LENGTH` | `32` × `256 B` | ring depth (no PSRAM) |
| `SERIALIZE_BUFFER_SIZE` | `1024 B` | one wire packet; also the spool record ceiling |
| `DATA_INTERVAL_TABLE` | `{1000,2000,5000}` ms | cycle period by stationary tier |
| `STATIONARY_TIME_TABLE` | `{10,60,180}` s | motionless thresholds → interval/standby |
| `MOTION_THRESHOLD` | `0.4 G` | wakeup sensitivity |
| `MAX_OBD_ERRORS` | `3` | re-init OBD after this many |
| `MAX_CONN_ERRORS_RECONNECT` | `5` | then power-cycle cellular + 3 min cool-off |
| `SERVER_SYNC_INTERVAL` | `120 s` | poor-connection watchdog |
| `SIGNAL_CHECK_INTERVAL` / `PING_BACK_INTERVAL` | `10 s` / `900 s` | RSSI poll / standby keepalive |
| `COOLING_DOWN_TEMP` | `75 °C` | purge buffers above this device temp |

Runtime-overridable via NVS (namespace `storage`): `CELL_APN` (key `CELL_APN`).

Defined in `config.h` but **referenced nowhere** (inherited from upstream, kept
only so the file still reads like a Telelogger config): `RESET_AFTER_WAKEUP`,
`JUMPSTART_VOLTAGE`, `MAX_CONN_TIME`, `DATA_RECEIVING_TIMEOUT`, `PIN_SENSOR1`,
`PIN_SENSOR2`. Changing them does nothing; in particular the restart at the end
of standby is unconditional (§11).

`SERVER_HOST` is deployment-specific and lives in `include/secrets.h`, which
is gitignored; `config.h` includes it unconditionally, so a missing file is a
compile error rather than a build that flashes and transmits nowhere. Copy
`include/secrets.h.example` to create it. The split exists because the Traccar
UDP protocol authenticates by device unique ID alone — there is no shared
secret on the wire — so host plus device ID is enough for anyone to inject
positions. Keep both out of the repo; the ID is derived from the ESP32 eFuse
MAC (`genDeviceID`) and is not guessable unless published.

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

**Plausibility gate (v1.2.1).** `udsValuePlausible()` rejects a decoded value
outside physical range, keyed off the table's `unit` column so no per-DID
bounds exist. Only temperatures are bounded today (−60 … 1500 °C), because an
ECU fault code came through `egt_b1s1` as exactly 3003.6 °C (§16.4, defect 4).
A rejected value is treated like a failed read: the PID is absent from that
cycle's packet and a `[UDS] <label> implausible` line is printed. The bound is
deliberately loose so genuine DPF-regeneration EGTs above 1000 °C pass.

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

**Active mode: `GNSS_STANDALONE`.** Two receivers exist and `initGPS()` picks
one per boot, external first:

| | External module (primary) | Internal receiver (fallback) |
|---|---|---|
| Hardware | u-blox module on the molex header, external antenna | u-blox on the ONE+ board, internal antenna |
| Data path | NMEA on UART0 RX (GPIO34), decoded by `taskGPS` into `GPS_DATA` | `ATGPSON`/`ATGPS` to the OBD co-processor, `$GNIFO` reply parsed by `gpsGetData()` |
| Commands to module | UBX bit-banged on GPIO26 (`gpsSendAt`, under a critical section) | none |
| Power | GPIO12 | co-processor |
| Boot line | `GNSS:EXTERNAL` | `GNSS:INTERNAL  <-- external module NOT detected, external antenna is unused` |
| Chime | short rising two-note ping (G6→D7) | low descending three-note grumble (G5→E5→A4) |
| `hdop` on the wire | **HDOP × 10** (`TinyGPS::hdop()/10`) | **HDOP × 100** (`$GNIFO` field unscaled) |
| Fails with | nothing else | the co-processor link — if it drops, GPS drops with it |

**Detection (`gpsBeginExt()`, v1.3.2).** The module ships at 115200 and must
end up at 38400 because UART0 is shared with the console. Each boot:
samples the RX pin idle level before and after power (a live UART idles high);
waits 1.5 s for the module to boot; **sweeps the receive rate** by re-clocking
UART0 through 115200, 38400, 9600, 57600, 19200, 4800, 230400, listening up to
2 s each and stopping at the first checksum-valid sentence; prints one probe
line per rate (byte count, good/bad sentences, first 24 bytes); then, if the
rate found is not 38400, sends u-blox `CFG-VALSET` `CFG-UART1-BAUDRATE`=38400
on the **RAM layer** (so nothing is written to the module's flash) at the
module's rate and verifies at 38400. A module that refuses to move keeps the
console at its rate: GNSS data is the product, log lines are not. Detection
counts checksum-valid NMEA sentences, which a u-blox emits with or without a
fix, so **sky view cannot cause a fallback**; only an absent, unpowered or
unwired module can. How this was arrived at: §16.4 defect 3, §16.6, §16.7.

**The fallback is latched for the trip.** `initGPS()` runs only while
`STATE_GPS_READY` is clear, and the watchdog below only fires with *no* GPS data,
which the internal receiver prevents. Every parked period ends in
`ESP.restart()`, so each trip re-probes from scratch. The chime fires on a
*change* of source, so it announces at boot and on a mid-drive fallback but is
silent on a watchdog re-init that lands on the same receiver.

**Never powered down.** Since v1.3.0 the receiver stays on and tracking through
standby, so wake-up is a hot start (TTFF 0 s measured, §16.7). Cutting power in
standby forced a 94–278 s cold start on every trip (§16.5).

**Why not cellular GNSS:** `CellSIMCOM::setGPS()` in the fork only branches for
SIM7070 (`AT+CGNSPWR`) vs everything-else (`AT+CGPS`, correct for SIM7600). The
**SIM7670G needs `AT+CGNSSPWR`/`+CGNSSINFO`**, which is absent — so in
`GNSS_CELLULAR` the modem never powers GNSS, `getLocation()` returns false, and
no GPS PIDs are emitted. The dead path is left intact; don't switch back without
first patching `FreematicsNetwork.cpp` for a `CELL_SIM7670` GNSS branch.

**What `processGPS()` does with a reading:**
- A reading with a date **feeds the time anchor first**, before any position
  check (`TIME.md`). Time-only `[GNSS] <iso-time>` lines mean tracking but no
  fix: the session is dated, but no position PIDs are added.
- A reading whose `time` field equals the previous one is skipped (no new fix).
  A jump of more than 0.001° from the previous accepted position is rejected
  once and resets the reference, so a single corrupt sentence cannot teleport
  the track.
- Position PIDs added: lat, lng, altitude, speed (knots × 1.852 → km/h),
  heading, and `sat`/`hdop` when non-zero. `sat` is capped at 12 by NMEA 0183's
  GGA field, so it cannot distinguish the receivers; `hdop` scale can (table
  above). `hdop = 255` is the external path's "no DOP yet" sentinel.
- `lastMotionTime` only updates at `kph ≥ 2`, feeding stationary-shutdown.
- **A GPS-less record is not a gap.** Traccar back-fills the last known location
  and marks the row `valid`, so it plots at a stale coordinate. Detect these with
  `fixtime < devicetime`, never with `valid` (§16.5).

**The `GNSS_RESET_TIMEOUT` recovery escalates:** first a warm re-init
(`sys.gpsEnd(false)` — power and ephemeris retained), then a power cycle if that
did not revive it. `gpsEnd()`'s `powerOff` argument defaults to `true`, which is
what made the old unqualified `sys.gpsEnd()` call self-defeating (§16.5) — pass
it explicitly.

**Stale path to fix if cellular GNSS ever returns:** the `GNSS_RESET_TIMEOUT`
block in `process()` unconditionally calls the standalone reset routines; it
would need a `GNSS_CELLULAR` branch.

---

## 10. MEMS / Motion

ICM-42627 or ICM-20948, auto-detected. `processMEMS()` averages accelerometer
samples per cycle (bias-corrected via `calibrateMEMS()`), emits `PID_ACC`, and
tracks device temperature. Motion feeds both the standby decision and wakeup.
Gyro and magnetometer are read but not emitted.

Device temperature (`deviceTemp`, whole °C internally, compared against
`COOLING_DOWN_TEMP`) goes on the wire as **tenths** in PID `0x82`, because that
is how the Freematics protocol defines it and Traccar divides by 10. It was
sent as whole degrees until v1.2.1 and read as 2.8–4.3 °C on the server
(§16.4, defect 5). Without a MEMS the ESP32's own chip temperature is used.

---

## 11. Standby / Wakeup

```
vehicle stops → motionless timer expires → standby()
  ├── telemetry task: logout, modem off, spool files flushed and closed,
  │   RAM buffers purged (orphan spool files are re-found at next boot)
  ├── OBD co-processor slept, MEMS re-calibrated
  ├── GNSS stays powered and tracking  ← never switched off (§16.5)
  └── waitMotion(-1) polls accel until motion ≥ MOTION_THRESHOLD (0.4 G)
        └── ESP.restart()   ← unconditional full reset
```

Full reset on wake is intentional: clean cellular re-registration, no stale state
machine, and a fresh GNSS probe (§9). It is unconditional — the
`RESET_AFTER_WAKEUP` macro in `config.h` is not consulted (§5). Stationary thresholds step the data interval 1 s → 2 s → 5 s as the
vehicle sits (`STATIONARY_TIME_TABLE` / `DATA_INTERVAL_TABLE`). During standby the
telemetry task pings every `PING_BACK_INTERVAL` (900 s).

The device does **not** sleep in standby — `waitMotion()` busy-polls the MEMS —
so leaving GNSS powered costs ~25 mA on top of an already-awake ESP32 and modem.
That is why the receiver is no longer switched off here: the cold start it forced
on every trip cost far more than the current does (§16.5). Note the wake-up
`ESP.restart()` still blips GPIO12 (a strapping pin with an internal pull-down)
for a few ms, which is a residual cold-start risk for the external module.

---

## 12. Cellular / Network Behaviour

- Modem auto-detected via `ATI` (SIM7670G-LN on this unit).
- `initCell()` does APN setup, registration, IP acquisition — and, once
  registered, reads the network's clock with `AT+CCLK?` and offers it to the
  time anchor as `TIME_SRC_MODEM` (`TIME.md`). This modem returns UTC in the
  time field despite reporting a `+40` zone; the value is taken as UTC.
- A successful connect plays `chimeConnected()` (rising C-major arpeggio).
- The Traccar login/reconnect/ping/logout exchange is the `EV=` event packet
  (`notify()`); the login reply's `TM=` would set `TIME_SRC_SERVER` but the
  deployed server does not send it (§16.4).
- **`transmit()` is not an end-to-end acknowledgement.** It returns whether the
  modem confirmed `AT+CIPSEND` within 1 s. `false` often means the datagram
  already left, and the record is spooled and later replayed anyway, so some
  rows reach Traccar twice (18 % on 2026-09-05, §16.5). Traccar's decoder
  does not acknowledge data packets; `EVENT_ACK` exists in `teleclient.h` and
  is never handled. The UDP socket is closed and reopened every 64 packets.
- RSSI polled every `SIGNAL_CHECK_INTERVAL` (10 s), logged as `PID_CSQ` when it
  changes.
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

## 14. Storage — the outage spool (`RecordSpool`)

The SD card is **not a log**. Since 2026-07-02 (CSV logger removed) it holds
exactly one thing: packets that could not be sent, so that a cell outage or a
pre-connect cold-start window leaves no permanent gap at the server.

**Files are the only state.** `/SPOOL/<seq>.PKT`, one file per outage, `seq`
monotonic and chosen at boot by scanning the directory (`max + 1`). No NVS, no
high-water mark. Files left over from a previous boot are orphans awaiting
drain, reported as `[SPOOL] n orphan file(s) pending drain`. A crash mid-drain
leaves the file in place and the next attempt replays it from byte zero;
Traccar accepts the resulting duplicates (no unique constraint on
`(deviceid, devicetime)`).

**Record format.** 2-byte big-endian length, then the exact wire bytes of the
packet. A record built before the session had a UTC anchor is written by
`appendUnstamped()` with a 13-byte prefix inside the length — marker `0x00`,
boot ID (4), capture tick (8), big-endian — and is back-stamped on the way out
(`TIME.md` §4). A stamped record begins with the device ID, which is always
alphanumeric, so the marker is unambiguous. The outage file is `flush()`ed
every 30 records, bounding power-cut loss while keeping the FAT entry honest.

**When records enter the spool** (telemetry task, `main.cpp`):

- a live `transmit()` returns false → the packet just built is appended, and
  the outage file opens if none is open (`[SPOOL] outage start`);
- the link is down → `spoolQueuedBuffers()` moves every filled RAM buffer to
  the spool, oldest first, before each connect attempt and once a second during
  the 3-minute reconnect cool-off, so the 32-slot ring never has to drop a
  record while offline;
- standby → `closeForStandby()` flushes and closes both handles before the
  restart.

**When records leave.** The first successful live transmit after an outage
calls `endOutage()`, closing the active file (`[SPOOL] outage end n record(s)`).
`drainOneRecord()` runs only when the live producer has nothing queued, so live
data keeps first refusal on the cell link: oldest file first, one record per
call, ~10 records/s when fully idle, naturally interleaved with live packets
otherwise. The file is unlinked when its last record has gone. Preconditions
and drop rules:

| Condition | Action | Log line |
|---|---|---|
| no time anchor yet | wait (nothing is sent untimed) | — |
| unstamped record from a different boot | drop the record | `stale-boot record dropped` |
| unstamped record, anchor present | back-stamp, send | — |
| transmit fails | leave the record, close the handle, retry next pass | — |
| truncated header/payload or length > `SERIALIZE_BUFFER_SIZE + 13` | drop the **file** | `short header` / `short payload` / `bad length` |
| end of file | unlink | `drained` |

Record production stalls during the blocking modem reconnect, so an outage
window holds fewer rows than the same time live; duplicates are expected (§12).

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
6. **Time stamping:** every `[DAT]` line ends with `10:`/`11:`/`300:`. On a
   cold start expect `[TIME] anchor MODEM …` within seconds of registration,
   then `[TIME] anchor GNSS … (MODEM was off by Ns)` with N in single digits;
   tens of thousands means the modem's CCLK zone convention has changed
   (§16.4). Spool drains should show `[SPOOL] draining` only after an anchor
   line.
7. **GNSS source:** the boot line and the chime agree (`GNSS:EXTERNAL` with the
   rising ping); the probe table shows a checksum-valid sentence at some rate
   and `external module at 38400`. In Traccar, `hdop` p50 around 5 means the
   external path, around 80–100 means internal (§9).

**Reference signatures** (useful sanity checks): at key-on→start, oil pressure
jumps ~1.0→2.6 bar and fuel-reg duty ~9→19 %, while coolant/EGT lag (thermal
mass); ambient pressure is unchanged. `0xF43C` (standardised EGT) and `0x10FB`
(VAG raw) read the same sensor differently — capture both, they're not redundant.
On this throttle-less diesel, `0x103C` IMAP ≈ ambient at idle is correct.

---

## 16. Field Observations

What the deployed device actually did, measured from the SD card (§16.0) and
then from Traccar's Postgres (§16.1 onward, `tools/traccar_baseline.py` and
ad-hoc SQL). Each dated subsection keeps its method and numbers so a future
re-run is comparable; the reasoning that led to each fix lives with the fix
(§9, §12, §14, `TIME.md`, `CHANGELOG.md`) and is not repeated here.

**Status as of 2026-09-09 (v1.3.2 in the car):**

| Area | State | Evidence |
|---|---|---|
| GNSS fix availability | 100 % of rows, moving or stopped; TTFF 0 s since the receiver stopped being powered down | §16.1, §16.7 |
| External module / antenna | Detected and carrying the fix: true HDOP p50 0.50 / p90 0.60 vs 0.86 / 1.58 internal | §16.7 |
| Record timestamps | Every row carries capture time; zero-lag rows 33 % → ~0; spool drains replay at capture time | §16.3, §16.4, §16.5 |
| Spool / backfill | Working through cell outages (lag up to 1738 s replayed) | §16.5 |
| Finding B (payload blackout) | Did not recur in 44+ sessions; closed as a one-off | §16.4 |

**Open items, in rough priority:**

1. **Duplicate deliveries, ~18 % of rows** — no end-to-end ack (§12); handled
   in the `vorsprung` SQL views for now. Firmware fix would be a per-record
   sequence number. (§16.5)
2. **`egt_b1s2` scaling** — 1000–1300 °C during DPF regeneration is real
   behaviour but above what that sensor should reach; formula unverified. (§16.4)
3. **Traccar `totalDistance` is unreliable** — out-of-order replay arrivals
   corrupt it server-side; the reporting layer uses the odometer DID instead.
   Should have improved after v1.2.1 but was not re-measured. (§16.4)
4. **`TM=` in the login reply** — a four-line Traccar change would give the
   device an authoritative server clock one round-trip after modem-up. (§16.4)
5. **`traccar_baseline.py cmd_stats` measures `valid`**, which Traccar sets on
   back-filled positions too; it overstates fix availability. Measure payload
   presence (`sat`/`io768`) instead. (§16.4)
6. **One 140 s mid-drive gap on 2026-09-09 was not backfilled.** Cause unknown,
   seen once. (§16.7)
7. **Wake-up GPIO12 blip** — `ESP.restart()` interrupts the external module's
   power for a few ms; TTFF 0 s says the module rides it out. (§16.5)
8. **External antenna mounting** — fitted loose-laid on 2026-06-19; re-confirm
   HDOP after it is permanently dressed. (§16.1)
9. **1NCE SIM migration** pending. (§13)

**How to measure, learned the hard way:**

- Use **OBD speed `10D` as the motion reference**. It says the car was moving
  even when GNSS produced nothing, which separates "no fix while driving" from
  "parked".
- **Never use Traccar's `valid` for fix availability.** The decoder back-fills a
  GPS-less row with the last known position and marks it valid. Count GNSS
  payload presence (`sat`/`io768`), or `fixtime < devicetime`.
- **`hdop` scale depends on the receiver** (×10 external, ×100 internal, §9);
  it is the receiver fingerprint. **`sat` is not**: NMEA caps it at 12.
- **A mis-stamped live row and a real spool drain both show large
  `servertime − devicetime` lag.** A drain's lag spread matches its capture
  window (ratio ~0.75–0.94); a mis-stamp's collapses (~0.1–0.3). `TIME.md`.
- **Zero-lag rows (`devicetime = servertime`) are the arrival-stamp signature.**
  They should be ~0 % since v1.2.1.

### 16.0 GNSS reliability baseline, internal antenna — 2026-06-01

Full SD dump, 52,346 logged cycles across 104 files (one file per session
between restarts). A cycle "has a fix" if it carries both `A` and `B`.

- **39.9 %** of all cycles carried no position while OBD logged normally.
- **35.7 %** of *moving* cycles (OBD ≥ 5 km/h) had no fix.
- **Bimodal:** of 58 real drives, 33 good (≥ 50 % coverage), 9 partial, **16
  with zero fix end-to-end** (e.g. 1848 records at up to 82 km/h, 0 fixes). When
  locked, reception was healthy (9–12 sats). Solid lock or total silence is an
  antenna/connector signature, not weak reception.
- Standby power-down cold-started GNSS on every wake; several files took
  400–600 cycles to first fix. (Confirmed as the dominant cause in §16.5.)

**Conclusion:** the gaps at the server were gaps on the SD card too — lost
*fix*, not lost *transmission*. Cellular GNSS and store-and-forward would not
have helped; an external antenna would. A CAN-bus nav-unit GNSS plan-B was
parked after §16.1 (it would need a VCDS capture of the nav unit's DIDs, a
gateway-reachability spike, and a scaled-`int32` decoder).

### 16.1 After external antenna fitted — 2026-06-19

External antenna fitted loose-laid. 36 h, 17,307 rows, 11 sessions including a
170-minute drive; Postgres `tc_positions`, same motion reference.

| Metric | Internal antenna | External antenna |
|---|---|---|
| Rows with no fix, overall | 39.9 % | **0.0 %** (0 / 17,307) |
| Rows with no fix, moving (OBD ≥ 5 km/h) | 35.7 % | **0.0 %** (0 / 13,949) |
| Drives with zero fix end-to-end | 16 / 58 | **0 / 11** |
| Satellite count (median) | mixed, often 0 | 12 |
| Inter-row cadence p50 / p90 / p99 | — | 0.98 / 2.0 / 4.5 s |

Resolved the bimodal failure. Median stored `hdop` 5 (true 0.5). `hdop = 255`
on a few rows is the "no DOP yet" sentinel. The `sat` values up to 59 seen here
were later shown to be NMEA corruption artefacts, not a constellation count
(§16.7). Antenna still to be permanently mounted (open item 8).

### 16.2 Post-v1.1.0 field check — 2026-07-13, re-checked 2026-07-25

25,961 rows over ~60 h after the v1.1.0 flash (hardened NMEA parser, date PID,
external GPS on UART0 RX). Re-check: 288 h, 41,199 rows.

**Verified:** no phantom-midnight rows; transport healthy (cadence p50 0.87 s);
one real 63 s outage replayed with correct devicetimes. The outage hole itself
holds no rows because the blocking modem reconnect stalls record production —
accepted tradeoff.

**Finding A — GPS-less records carried no timestamp.** `processGPS()` wrote the
time PIDs only when it had a date, so a record built between GNSS updates had
no time at all and Traccar arrival-stamped it. Exact partition: the rows with
`servertime ≡ devicetime` were precisely the rows lacking `sat`/`io768`.
Harmless live (< 1 s); on a spool drain every such record landed at drain time
— old idle-engine content interleaved into a live 85 km/h stream, a
281-row "session" in 60 s, `totalDistance` jumping backwards by kilometres.
17 % of rows on 07-13, **33 %** by 07-25, a burst roughly once a day. Fixed in
v1.2.0 (§16.3, `TIME.md`).

**Finding B — a ~25-min GNSS payload blackout on 2026-07-13**, masked by the
fix metric (Traccar carried the position forward and reported 100 % valid).
GPS payload in 3.6 % of records against 72–97 % in healthy sessions. Did not
recur in the next 32 sessions, nor in the 44 of §16.4; closed as a one-off.

### 16.3 Monotonic back-stamping — implemented 2026-07-29 (v1.2.0)

Fixes Finding A. Design and mechanism: `TIME.md`. As shipped, the anchor had
two ranks (GPS over a single network source); v1.2.1 split the network source
(§16.4). The CAN-clock alternative was dropped: local time with DST, no
position, itself GPS-sourced, and UDS filter work to fix only the timestamp
half. No Module 17 code ever existed in this firmware.

**Verified 2026-07-31:** zero-lag rows 33 % → 1.4 %. One claim from that check
("back-stamping proven across an overnight park, max lag 36,030 s") was the
CCLK bug of §16.4, not a drain, and is retracted; back-stamping is proven by
other events (227 rows, lag 51957→52130 s, spread 172 s against a 232 s
window). Position for GPS-less rows is unchanged by design: they sort
correctly now but still carry Traccar's copied-forward fix.

### 16.4 Fourteen-day health check — 2026-08-20 (v1.2.1)

336 h, 41,488 rows, 44 drive sessions. v1.2.0 confirmed: zero-lag rows 92.5 %
pre-fix → 3.8 %. GNSS payload presence (the right metric, replacing `valid`):
97.2 % overall, per-session min 96.7 %. Five defects:

1. **`+CCLK` zone double-count.** The modem puts UTC in the time field *and*
   reports `+40`; subtracting the offset per 3GPP landed the anchor 36000 s
   early for every record between modem-up and GNSS lock — 525 rows (1.27 %),
   none with `sat`, and 43 % of the `totalDistance` corruption. Now taken as
   UTC; `TIME_SRC_NET` split into `MODEM` < `SERVER`; every anchor handover
   prints the outgoing source's error. Found via the lag-spread tell above.
   Separately: the login reply carries no `TM=`, so `TIME_SRC_SERVER` never
   fires; GNSS acquiring *time* ~30–45 s in is what corrects the modem anchor
   (open item 4).
2. **VIN rows were junk and arrival-stamped.** `getVIN()` returns the
   placeholder `EEPR0M-READ-ERR0R` on failure; it reached Traccar on 1,114 of
   1,179 VIN rows, and those rows were the *entire* residue of zero-lag rows.
   Traccar's `decodeEvent()` ignores PID tokens and makes no row without a VIN,
   so validating the VIN alphabet removes the rows rather than fixing them.
3. **External GNSS module unused since v1.1.0.** Stored `hdop` p50 was 5–6 in
   June (external, ×10) and 81–106 from 07-13 (internal, ×100); fix-to-fix
   jitter confirmed HDOP ~1. The antenna had done nothing since the flash and
   nobody noticed because the internal receiver is good enough. Boot line made
   loud here; cause in §16.6–§16.7.
4. **Fault code decoded as a temperature.** `egt_b1s1` = 3003.6 °C on 20 rows
   (raw 0x76E4). Plausibility gate added (§7.6), loose enough to keep the
   genuine 1000–1300 °C `egt_b1s2` regeneration readings (open item 2).
5. **`deviceTemp` sent as whole degrees** where PID 0x82 is tenths; a 28–43 °C
   device read 2.8–4.3 °C. Fixed at the PID (§10).

`totalDistance`: 737 backward jumps / 392 km in 14 days, 474 of them pairs that
arrived out of devicetime order; defects 1 and 2 account for most. Not
re-measured after the fix (open item 3).

### 16.5 GNSS cold-start churn — 2026-09-05 (v1.3.0)

9,131 rows, seven sessions in one day, prompted by gaps on the Grafana map.

**The spool was not the problem:** every outage was backfilled with correct
devicetime (lag up to 1738 s). The gaps were rows with **no position**, which
Traccar plots at the last known location. Root cause: the receiver was powered
down in standby (`sys.gpsEnd(true)` on GPIO12), so every trip began with a
full cold start — **94–278 s blind at the start of all seven sessions**. The
300 s watchdog made it worse: its recovery also power-cycled (`gpsEnd()`
defaults `powerOff` to true), forcing another 2–4 min cold start inside a
300 s timeout; the 16.9-minute hole at 06:12–06:29 is that loop.

| Metric | Value |
|---|---|
| GNSS-blind time | 32.2 of 388.9 min logged (**8.3 %**) |
| Straight-line "teleport" drawn on the map | **28.6 km** of 152.0 km (~16 %) |
| Duplicate rows | 1,642 extra on 7,489 distinct timestamps (**18 %**) |

Duplicates come from `transmit()` reporting the modem's 1 s `AT+CIPSEND`
confirmation, not delivery (§12); 977 duplicate pairs *disagree on position*
because a GPS-less packet arriving twice gets last-known-location substituted
twice, producing a 19.5 km saw-tooth. Open item 1.

**Fixed in v1.3.0:** receiver never powered down (`GNSS_ALWAYS_ON` removed);
watchdog warm-restarts first and power-cycles only on the second attempt;
boot chime reports which receiver won; external detection window widened to
3 s. Verified in §16.7: TTFF 0 s. The wake-up GPIO12 blip (open item 7) is the
residual risk.

### 16.6 Why the external module was never detected — 2026-09-05 (v1.3.1)

The v1.3.0 chime reported the *internal* receiver on the first drive with the
external module plugged in. Detection counts checksum-valid NMEA sentences,
which a u-blox emits with or without a fix, so sky view was not the cause.
Diffing `gpsBeginExt()` against upstream: upstream sets the module's baud rate
*before* listening; the fork, since moving receive onto UART0 in v1.1.0, sent
nothing and listened at 38400. A module at any other rate was indistinguishable
from an empty socket. v1.3.1 bit-banged `CFG-VALSET` at four rates before
listening, open-loop, at 0.3 s and 1.07 s after power. **It did not work**
(2026-09-07: descending chime every start, 17 raw bytes seen in 3 s). The
claim made here that the rate "cannot be discovered by listening" was wrong.

### 16.7 Bench diagnosis and fix — 2026-09-07 (v1.3.2), verified 2026-09-09

Device and module on the desk (valid without OBD: the co-processor enumerates
over USB, only OBD/UDS rows are absent). First v1.3.2 boot:

```
[GNSS] rx pin idle-high: 0% unpowered, 98% powered
[GNSS] probe  38400 bps  bytes=10    ok=0  bad=0  <C3><FD><C3><FD><C3><FD>...
[GNSS] probe   9600 bps  bytes=40    ok=0  bad=0  <17><AE><FE><17><AE><02>L<A3>...
[GNSS] probe 115200 bps  bytes=27    ok=1  bad=0  $GNRMC,,V,,,,,,,,,,N,V*3
[GNSS] probe  38400 bps  bytes=27    ok=1  bad=0  $GNRMC,,V,,,,,,,,,,N,V*3
[GNSS] external module at 38400
GNSS:EXTERNAL
```

Wiring and power were fine (pin idles high only when powered). **The module
ships at 115200**, upstream's own `gpsBeginExt()` default. Idling without a fix
it sends one ~27-byte `$GNRMC` per second, which at 38400 decodes to ~10 bytes
of framing garbage — so v1.3.1's "17 bytes" *was* the wrong-rate signature.
v1.3.1's coercion missed because a cold u-blox needs ~1 s before its UART
accepts input and an interrupt mid-byte at 115200 corrupts the UBX checksum;
it never checked. v1.3.2 listens first, then configures at the rate found
(mechanism in §9). Two state leaks fixed in passing (`gpsBegin()` zeroing the
power pin; `FLAG_GNSS_USE_LINK` not cleared on re-probe). Ruled out at the
desk: `F_CPU` vs real clock, `analogRead(A0)` (GPIO36, not 34),
`Serial.setPins(34, -1)`.

**Field verified 2026-09-09**, two drives, 3,045 rows:

| | internal (v1.3.1, 17 drives) | external (v1.3.2, 2 drives) |
|---|---|---|
| stored `hdop` p50 / p90 | 86 / 158 | 5 / 6 |
| **true HDOP** p50 / p90 | **0.86 / 1.58** | **0.50 / 0.60** |
| fix availability, moving-row fix, TTFF | 100 %, 100 %, 0 s | 100 %, 100 %, 0 s |

The stored column is not comparable across receivers (×100 vs ×10). The real
gain is a ~42 % tighter and far more consistent fix (p90 −62 %); fix
availability was already 100 % on both since v1.3.0, so **re-baseline on DOP,
not fix rate**. The `sat > 12` criterion proposed in §16.6 is withdrawn (NMEA
caps the field; the June rows above 12 were corruption artefacts). One 140 s
mid-drive gap in the second drive was a cell dropout the spool did not backfill
(open item 6).

---

## 17. Build & Flash

PlatformIO:

```sh
cp include/secrets.h.example include/secrets.h   # once; set SERVER_HOST
pio run -e esp32dev                              # compile
pio run -e esp32dev -t upload                    # flash
pio device monitor                               # 38400 baud
```

The console is **38400 baud** (§2); `monitor_speed` in `platformio.ini` and
`Serial.begin()` in `main.cpp` must move together. Bump `FIRMWARE_VERSION`
and add a `CHANGELOG.md` entry with any behaviour change; the version and
build timestamp printed at boot are how a flashed binary is identified in a
serial log. After a flash, bump `data_from` in the `vorsprung` reporting
schema if the change affects data trust (see `homeassistant/README.md`,
local only).

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
- **Don't change the wire format** — synthetic UDS PIDs (`0x200+`) and the time
  triplet ride the stock Freematics UDP protocol, so a stock Traccar decoder
  still files every row with the right `devicetime`.
- **Don't rename the upstream SDK** under `libraries/` — it's Freematics' code.
  Local changes are confined to the GNSS path (§3) and are marked in place.
- **Don't strip Stanley Huang's BSD copyright** from derived files.

---

## 20. References

- ISO 14229-1 (UDS services), ISO 15765-2 (ISO-TP), ISO 11898 (CAN).
- Vehicle: Audi A6 Allroad 3.0 TDI, EA897 evo, ECU `4G2 907 311 B`
  (Bosch/Continental MD1). DIDs validated against VCDS + live CAN captures.
- Upstream: [Freematics](https://github.com/stanleyhuangyc/Freematics) by Stanley
  Huang — the Telelogger this project derives from.
