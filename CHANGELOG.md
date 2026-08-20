# Changelog

Firmware releases of VorsprungLogger. The version here is `FIRMWARE_VERSION` in
`include/config.h`, printed at boot alongside the build timestamp — that pair is
what identifies a flashed binary in a serial log.

Versioning started at v1.1.0; earlier work is summarised at the bottom for
continuity. Design rationale lives in `SPDD.md`, referenced by section.

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
