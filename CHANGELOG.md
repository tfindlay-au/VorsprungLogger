# Changelog

Firmware releases of VorsprungLogger. The version here is `FIRMWARE_VERSION` in
`include/config.h`, printed at boot alongside the build timestamp — that pair is
what identifies a flashed binary in a serial log.

Versioning started at v1.1.0; earlier work is summarised at the bottom for
continuity. Design rationale lives in `SPDD.md`, referenced by section.

## v1.3.2 — 2026-09-07

**Field verified 2026-09-09:** external module carrying the fix over two
drives — true HDOP p50 0.86 → 0.50, p90 1.58 → 0.60. Fix availability was
already 100% and is unchanged. See SPDD §16.7 for the scale caveat (the raw
`hdop` column is ×100 on internal, ×10 on external — not comparable directly).

**The external GNSS module is finally detected.** v1.3.1's baud coercion never
landed: it fired at 0.3 s and 1.07 s after power-on, before a cold u-blox will
accept input, with interrupts enabled, and never checked. Bench diagnosis with
the module on the desk found it alive, correctly wired, and talking at
**115200** — upstream's own `gpsBeginExt()` default. See SPDD §16.7.

Fixed

- **Closed-loop detection.** `gpsBeginExt()` sweeps the *receive* rate
  (115200, 38400, 9600, 57600, 19200, 4800, 230400) by re-clocking UART0, and
  stops at the first checksum-valid sentence. Once the rate is known it sends
  `CFG-VALSET` at that rate and verifies the module moved to 38400. A module
  that will not move keeps the console at its rate — data over log lines.
- **Every boot prints a probe table**: receive-pin idle level before and after
  power, then per rate the byte count, good/bad sentence counts and a sample of
  what arrived. A dead module, an unwired pin, a wrong rate and a wrong
  protocol each now look different in the log.
- **Bit-banged UBX commands go out under a critical section** (`gpsSendAt()`),
  so a tick interrupt cannot corrupt a packet mid-byte.
- `gpsBegin()` no longer zeroes `m_pinGPSPower`, and `gpsBeginExt()` clears
  `FLAG_GNSS_USE_LINK` on entry — an in-boot re-probe after an internal
  fallback previously ran unpowered and leaked the decode task.
- `taskGPS.create()` failure is reported instead of reading as "no module".

## v1.3.1 — 2026-09-05

**The external GNSS module is told what baud rate to speak.** v1.3.0 shipped the
chime that reports which receiver won, and it reported the internal one — the
external module fitted 2026-06-19 still was not being detected. Cause found by
diffing `gpsBeginExt()` against upstream Freematics: the fork listens at 38400
and sent the module *no* configuration at all before deciding whether it was
there. Upstream always sends a baud-set command first. A module that came up at
any other rate produced bytes that failed every checksum, so it was
indistinguishable from an empty socket. See SPDD §16.6.

Fixed

- **Baud coercion before detection.** `gpsBeginExt()` now bit-bangs a u-blox
  `CFG-VALSET` (RAM layer, `CFG-UART1-BAUDRATE` = 38400) at 38400, 115200, 9600
  and 57600 before it starts listening. A packet sent at the wrong rate is
  framing noise the module discards; the one at the right rate lands. It is sent
  twice — once after 300 ms for a module that never lost power on a warm
  re-init, once at ~1 s for one that is still booting from cold.
- **The failure is now legible.** On no detection the boot log prints the count
  of raw bytes seen on the receive pin. Non-zero means the module is wired and
  talking at a rate the sweep did not reach; zero means nothing is on the pin.

Known limitation

- Fallback to the internal receiver is still **latched for the trip**.
  `initGPS()` runs from `initialize()` only while `STATE_GPS_READY` is clear,
  and the `GNSS_RESET_TIMEOUT` watchdog only fires when there is no GPS data at
  all — which the internal receiver prevents. Deliberate: detection counts
  checksum-valid NMEA sentences, which a u-blox emits with or without a fix, so
  sky view (a garage, a tunnel) cannot cause the fallback. Re-probing would cost
  a live gap to fix a case that does not arise.

## v1.3.0 — 2026-09-05

**The GNSS receiver is no longer powered down.** Fixes the cold-start churn that
made every trip begin blind. Full analysis: SPDD §16.5 (9,131 rows,
2026-09-05 00:14 → 06:43 UTC, seven drive sessions), prompted by visible gaps
between points on the Grafana map.

The gaps were not lost data — the v1.2.0 spool and backfill are working, and no
rows were missing. They were rows carrying **no position**: `processGPS()`
correctly emits no GPS PIDs without a fix, and Traccar's decoder then back-fills
the last known location and marks the row `valid`. Standby was cutting GNSS
power, so every trip cold-started. Measured cost on the day: **94–278 s blind at
the start of all seven sessions, 8.3 % of logged time with no real fix, and
28.6 km of the 152 km track drawn as straight lines — about 16 % of the plotted
distance.**

Fixed

- **GNSS powered down in standby.** `standby()` called `sys.gpsEnd(true)`,
  driving `PIN_GPS_POWER` (GPIO12) low for the whole parked period, so the
  receiver lost its ephemeris and cold-started on every wake. It now stays
  powered and tracking; the NMEA decode task keeps running, so the module holds
  a live fix across the park. Costs ~25 mA on a device that does not sleep in
  standby anyway — `waitMotion()` busy-polls the MEMS and the modem pings every
  `PING_BACK_INTERVAL`. SPDD §16 predicted this in June as a "cheap, independent
  win"; it was not a minor amplifier but the dominant cause.
- **The GNSS watchdog power-cycled on first failure.** The `GNSS_RESET_TIMEOUT`
  recovery called `sys.gpsEnd()`, whose `powerOff` argument defaults to `true`
  — so recovery forced a cold start needing 2–4 minutes, longer than the 300 s
  timeout that triggered it, and the device just cycled. That loop is the
  16.9-minute hole at 06:12:20–06:29:17. It now warm-restarts first
  (`sys.gpsEnd(false)`, power and ephemeris retained) and escalates to a power
  cycle only if that did not revive the module.
- **External-module detection window too short.** `gpsBeginExt()` allowed 300 ms
  of settle plus 1.1 s of listening, but a cold-powered module needs about a
  second to boot before its first NMEA sentence. Widened to 3 s — a contributing
  cause of the silent fallback tracked since SPDD §16.4. The extra 1.9 s is only
  ever spent when no external module answers.

Added

- **The boot chime now reports which receiver won.** `chimeGnssExternal()` is a
  short rising two-note ping (G6→D7); `chimeGnssInternal()` is a low descending
  three-note grumble (G5→E5→A4), pitched well below it and below
  `chimeConnected()` so the three cannot be confused at road speed. It fires on
  a *change* of source, so it announces at boot and on a mid-drive fallback but
  stays silent on a watchdog re-init that lands on the same receiver. This gives
  an in-car feedback loop for unplugging and replugging the external module,
  which previously required a console.

Changed

- `GNSS_ALWAYS_ON` is **removed**, not set to `1`. The receiver is on,
  unconditionally; the dead `#if` and the branch it guarded are gone.

Known issues

- **The wake-up blip is not addressed.** `standby()` still ends in
  `ESP.restart()`, and GPIO12 is an ESP32 strapping pin with an internal
  pull-down, so power to the external module is interrupted for a few
  milliseconds on every wake regardless of this change. Whether the module
  rides that out depends on its VBCKP backup. If TTFF after a stop is still
  minutes rather than seconds, this is the cause; the next step would be holding
  GPIO12 through reset or dropping `RESET_AFTER_WAKEUP`.
- **Duplicate transmissions are unchanged**, at 18 % of rows on 2026-09-05.
  `transmit()` returns whether the *modem* confirmed `AT+CIPSEND` within 1 s,
  not whether the server received anything — there is no end-to-end
  acknowledgement in this design at all. A false return usually means the
  datagram already went out, and the record is spooled and replayed anyway.
  Being handled server-side in the `vorsprung` views for now; a per-record
  sequence number is the firmware-side fix if it becomes worth doing.
- **Traccar's last-known-location substitution** means a GPS-less row is
  indistinguishable from a real fix by the `valid` column. Detect these with
  `fixtime < devicetime`. `tools/traccar_baseline.py` still has this bug and
  overstates fix availability.

## v1.2.1 — 2026-08-20

**Four data-integrity fixes from a 14-day Postgres health check.** Full
analysis: SPDD §16.4 (41,488 rows, 2026-08-07 → 08-20, 44 drive sessions).
v1.2.0 itself is confirmed working — zero-lag rows fell 92.5 % → 3.8 % and
spool replays land at true capture time.

Fixed

- **`+CCLK` timezone double-count.** `networkTime()` subtracted the reply's
  `+zz` offset per 3GPP TS 27.007, but this modem reports UTC in the time field
  *and* `+40`, so the anchor landed exactly 36000 s in the past. Every record
  built between modem-up and GNSS lock was stamped 10 hours early: 11 events /
  525 rows (1.27 %) over 14 days, none carrying `sat`, and 43 % of the
  server-side `totalDistance` corruption. Taken as UTC now.
- **VIN rows.** `obd.getVIN()` signals failure in band with the 17-character
  placeholder `EEPR0M-READ-ERR0R`, which passed every length check and reached
  Traccar on 1,114 of 1,179 VIN rows. Now validated against the real VIN
  alphabet (17 chars, digits plus A–Z less I/O/Q). Because Traccar's
  `decodeEvent()` returns null with no VIN present, this also closes SPDD
  §16.3's open item 1: those rows *were* the entire residue of arrival-stamped
  rows (1,179 of 1,179, an exact partition), and they now stop being created
  rather than being stamped wrong.
- **ECU fault code decoded as a temperature.** `egt_b1s1` reported exactly
  3003.6 °C on 20 rows (raw 0x76E4 through the J1979 scaling). Added a
  plausibility bound keyed off the DID table's `unit` column, temperatures
  only, treated as a failed read. Bound left loose (−60…1500 °C) so it does not
  swallow the 1000–1300 °C `egt_b1s2` readings, which are genuine DPF
  regenerations.
- **`deviceTemp` off by 10×.** PID 0x82 is defined in tenths of a degree and
  Traccar's decoder divides by 10, but the firmware sent whole °C — so a device
  running at 28–43 °C reported 2.8–4.3 °C. Now sent as deci-degrees; internal
  semantics (and the `COOLING_DOWN_TEMP` comparison) unchanged.

Changed

- `TIME_SRC_NET` split into `TIME_SRC_MODEM` (1) < `TIME_SRC_SERVER` (2), with
  GNSS at 3. The two network sources shared a rank, making the anchor
  last-writer-wins between them.
- `timeAnchorSet()` prints how far out the outgoing anchor was on every
  handover. The CCLK error hid for a month because that number was never shown.
- `initGPS()` boot line is now `GNSS:EXTERNAL` / `GNSS:INTERNAL` / `GNSS:NONE`
  (was `OK(E)`/`OK(I)`/`NO`), and the internal case says plainly that the
  external module was not detected and its antenna is unused — which has been
  true and silent since the v1.1.0 flash on 2026-07-11. Why detection fails is
  not yet diagnosed; see SPDD §16.4.

## v1.2.0 — 2026-07-29

**Monotonic back-stamping: every record now carries its own capture time.**
Fixes SPDD §16.2 Finding A, where records built without GPS payload carried no
time at all, so Traccar stamped them with arrival time and each spool drain
forged a phantom session — 33 % of rows by the 2026-07-25 field check, with a
burst roughly once a day. Design doc: `TIME.md`. Implementation notes:
SPDD §16.3.

Added

- `timeanchor.{h,cpp}`: a single tiered UTC anchor ("at tick M the clock read
  T"), fed by GNSS (rank 2) and cellular (rank 1). A higher rank always wins; a
  lower one waits until the incumbent is 120 s stale. Readings outside
  2025–2050 are rejected, which catches both a mis-parsed NMEA date and an
  un-provisioned modem's 1980 default (whose two-digit year reads back as 2080).
- `CBuffer::tick` — `esp_timer_get_time()` sampled once at the top of each
  `process()` cycle. 64-bit monotonic microseconds: no 49-day `millis()` wrap,
  unaffected by `settimeofday()`. `buffer->timestamp` is now derived from it.
- `CellUDPTime::networkTime()` — reads the modem's NITZ clock via `AT+CCLK?`
  once registration completes. On a cold start this lands before the first GNSS
  fix, and it is the only source that works underground.
- Spool back-stamping. Records built before the session's first anchor are
  written with a 13-byte prefix (marker, boot ID, capture tick); the drain
  resolves the tick against the anchor and splices the time PIDs in on the way
  out, so they land at capture time rather than drain time.

Changed

- `buildPacket()` is now the only writer of the `0x10`/`0x11`/`0x300` time PIDs,
  and writes them into **every** packet. `processGPS()` no longer stamps
  packets directly — it feeds the anchor instead, and does so above the position
  checks, so a time-only fix still dates the whole session. Per SPDD §16.2
  item 1 option (a), Traccar's stock decoder rebuilds `devicetime` from the
  `0x10`/`0x11` pair, so no server change is needed.
- The Traccar login `TM=` reply now sets the anchor rather than only calling
  `settimeofday()`.
- `RecordSpool::drainOneRecord()` takes a `CStorageRAM&` for back-stamping, and
  returns early while the device has no clock rather than spending records on
  arrival stamps.

Notes

- **Spool records from a previous boot are dropped**, with a log line. A tick is
  only meaningful within its own boot, so unstamped records carry a boot ID;
  sending a mismatched one untimed would forge exactly the phantom session being
  eliminated. Only affects a session that never obtained any time source before
  rebooting. This also closes SPDD §16.2 item 4.
- Spool files written by v1.1.0 still drain: a stamped record begins with the
  device ID, always alphanumeric, so the `0x00` marker is unambiguous.
- Epoch↔GPS-format conversion is hand-rolled (Hinnant civil/days) to keep the
  ESP32's configured timezone out of it. Round-trip verified against `gmtime`
  over 316,483 samples spanning 2025–2035, zero mismatches.
- The CAN/UDS vehicle-clock route (Module 17, request `0x714`) was evaluated and
  **dropped** — see SPDD §16.3. No Module 17 code ever existed in this firmware.
- Not addressed: the position half of Finding A (GPS-less rows still get
  Traccar's copied-forward fix, they just sort correctly now), and Finding B.

## v1.1.0 — 2026-07-13

**Phantom-midnight rows.** Bit-banged soft-serial RX corrupted NMEA
mid-sentence; `atol`-truncated time fields slipped past the XOR checksum and
landed as devicetime 00:0x UTC.

- NMEA parser rejects malformed date/time terms.
- The date PID (`0x11`) rides with the time PID so Traccar rebuilds full
  devicetime — without it the server supplied its own date, mis-dating any spool
  replay crossing UTC midnight.
- External GNSS moved to UART0's unused RX channel; the soft-serial decode path
  is deleted. **Console baud is now 38400** to match the module — update
  `monitor_speed` in lockstep.

Field-verified 2026-07-13: no recurrence.

## Earlier (pre-versioning)

- **2026-07-02** — Replaced the CSV logger with `RecordSpool`: per-outage
  `/SPOOL/<seq>.PKT` files capturing the exact wire bytes of unsent packets,
  drained on live-producer idle gaps so live data keeps first refusal on the
  cell link. Files-are-state — no NVS, orphans recovered on next boot,
  duplicate replays acceptable to Traccar. `PID_ABS_TIME` (`0x300`) introduced.
  `SDLogger`/`FileLogger`/`SPIFFSLogger` removed. (SPDD §14)
- **2026-07-02** — GNSS reliability baseline documented (SPDD §16/§16.1) and
  `tools/traccar_baseline.py` added.
- **2026-06-18** — Fixed a WDT reboot loop with the external GPS module fitted:
  the soft-decode task's start-bit wait starved IDLE0 at priority 1.
- **2026-05-31** — Connect chime; transmit-LED blink and dead `beep()` removed.
- **2026-05-31** — Initial commit: Audi EA897 telemetry logger (OBD-II + UDS
  DIDs + GPS/MEMS over cellular UDP to Traccar), derived from the Freematics
  Telelogger.
