# Changelog

Firmware releases of VorsprungLogger. The version here is `FIRMWARE_VERSION` in
`include/config.h`, printed at boot alongside the build timestamp — that pair is
what identifies a flashed binary in a serial log.

Versioning started at v1.1.0; earlier work is summarised at the bottom for
continuity. Design rationale lives in `SPDD.md`, referenced by section.

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
