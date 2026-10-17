# TIME.md — How every record gets its timestamp

As built in v1.2.0 (2026-07-29) and corrected in v1.2.1 (2026-08-20). Code:
`include/timeanchor.h`, `src/timeanchor.cpp`, `buildPacket()` in
`src/main.cpp`, and the spool paths in `src/telestore.cpp`. Field history and
the measurements behind each decision are in `SPDD.md` §16.2–§16.4.

## The problem this solves

Until v1.2.0 the only time a packet carried was GNSS time, written by
`processGPS()` only when the receiver had a date. A record built between GNSS
updates, or before the first fix, had **no time at all**, and Traccar filed it
at arrival time. Live, that cost under a second. On a spool drain after a cell
outage or a cold start, every such record collapsed onto the drain instant and
forged a phantom session, with old idle-engine content interleaved into a live
motorway stream. By 2026-07-25 that was 33 % of all rows and a burst roughly
once a day (SPDD §16.2, Finding A).

## Design

Two separate ideas, kept separate on purpose: a **monotonic tick** that says
*when* a record was captured relative to boot, and a **UTC anchor** that says
what the clock read at one known tick. A record's UTC time is always
`anchor_utc + (record_tick − anchor_tick)`. The subtraction works just as well
when the record is older than the anchor, which is what lets records captured
before any clock source existed be stamped correctly later.

### 1. Capture tick

`process()` calls `timeTicks()` once at the top of each cycle and stores it in
`CBuffer::tick`. It wraps `esp_timer_get_time()`: 64-bit microseconds since
boot, monotonic, never wraps in practice, and unaffected by `settimeofday()`.
`buffer->timestamp` (PID `0x00`, and the buffer manager's ordering key) is
derived from the same tick as milliseconds, so there is one notion of "when"
per record. `millis()` is not used for anything time-of-capture related.

### 2. The anchor

One global triple in `timeanchor.cpp`: `(source, utc_sec, tick_us)`, read and
written under a spinlock because the main loop writes it (GNSS) and the
telemetry task reads it (packet build, spool drain).

| Rank | Source | Set from | Notes |
|---|---|---|---|
| 3 | `TIME_SRC_GPS` | `processGPS()`, on any valid `gd->date` | Sub-second. Set **above** the position checks, so a time-only fix (no lat/lng yet) still dates the session. Re-anchors every second while tracking. |
| 2 | `TIME_SRC_SERVER` | Traccar login reply, `TM=<epoch>` | Authoritative by definition: it is the clock that stamps `servertime`. **Currently never fires** — the deployed decoder's login reply carries no `TM=` (SPDD §16.4). Kept because adding it server-side is a four-line change. |
| 1 | `TIME_SRC_MODEM` | `AT+CCLK?` in `initCell()`, right after registration | NITZ-fed, second-level. Lands first on a cold start and is the only source that works underground. The weakest source: its zone convention is modem-specific (see "CCLK" below). |

**Replacement rule** (`timeAnchorSet`): a reading from a source of equal or
higher rank always replaces the incumbent; a lower-ranked source is accepted
only once the incumbent is more than **120 s** old (`ANCHOR_STALE_US`), so a
GNSS lock that goes away mid-drive hands back to network time after a couple
of minutes rather than holding a drifting anchor forever.

**Plausibility window:** readings outside 2025-01-01 … 2050-01-01 are
rejected. The lower bound catches an un-provisioned SIMCOM modem's 1980
power-on default and mis-parsed NMEA dates; the upper bound catches the same
1980 default when its two-digit year reads back as 2080.

**Side effects on first anchor:** the first accepted reading of a session also
calls `settimeofday()`, so SD directory entries and `printTime()` are honest.
Every **handover** between sources prints a `[TIME] anchor …` line that
includes how far out the outgoing source was. A handover should move the clock
by a second or two; anything larger means the previous source was lying and
every record stamped under it is wrong by that amount. The CCLK bug below ran
unnoticed for a month because that number was not printed.

### 3. Stamping at serialisation

`buildPacket()` is the **only** writer of the time PIDs. For every packet, live
or replayed, GPS payload or not, it resolves the record's tick against the
anchor and appends three PIDs:

| PID | Value | Why |
|---|---|---|
| `0x10` | `HHMMSScc` UTC | Traccar's stock Freematics decoder rebuilds `devicetime` from this pair, so no server change is needed |
| `0x11` | `DDMMYY` | |
| `0x300` | Unix epoch seconds (`PID_ABS_TIME`) | Convenience for analysis; lands as attribute `io768` |

The epoch to civil conversion is hand-rolled (Hinnant's `civil_from_days`) so
the ESP32's configured timezone can never leak in. It was round-tripped against
`gmtime` over 316,483 samples spanning 2025–2035 with zero mismatches.
`timeAnchorResolve()` floors a negative delta rather than truncating toward
zero, so a pre-anchor record cannot be pulled forward across a second boundary.

`buildPacket()` returns `false` when no anchor exists yet. The caller must not
send that packet; it goes to the spool unstamped (next section).

### 4. Back-stamping through the spool

Records built before any source has reported cannot be stamped at build time.
On a cold start they are produced for tens of seconds before the modem is even
up, and the telemetry task spools them before its first connect attempt.

`RecordSpool::appendUnstamped()` prefixes such a record with a 13-byte header,
all big-endian: marker `0x00`, 32-bit **boot ID**, 64-bit capture tick. A
stamped record starts with the device ID, which is always alphanumeric, so the
marker is unambiguous and spool files written by v1.1.0 still read back.

`drainOneRecord()`:

1. Returns early while **no anchor exists**. Draining without a clock could
   only spend records on arrival stamps, which is the defect being fixed.
2. For an unstamped record, compares its boot ID with this boot's. A mismatch
   means the tick was measured on a monotonic clock that no longer exists, and
   the record is **dropped** with a `[SPOOL] stale-boot record dropped` line.
   Sending it untimed would forge exactly the phantom session being
   eliminated. This only affects a session that rebooted without ever
   obtaining a time source.
3. Otherwise resolves the tick, re-opens the packet (`CStorageRAM::load` →
   `untailer`), writes the three time PIDs, re-checksums (`tailer`), and sends.

The boot ID is `esp_random()` mixed with a counter kept in RTC no-init RAM
that increments every restart, so consecutive boots differ even though the RF
is never up when `esp_random()` is called.

## Considered and rejected

- **Vehicle CAN clock (instrument cluster, module 17).** Local time with DST,
  no position, itself GPS-sourced with holdover on this platform, and it would
  have cost UDS/ISO-TP filter work and a timezone correction to fix only the
  timestamp half of the problem. Dropped before any code existed (SPDD §16.3).
- **RTC-RAM holdover of the anchor across standby.** Standby ends in
  `ESP.restart()` after an unknown parked duration, so ticks reset while the
  gap stays unknown; a carried-over anchor would be wrong by exactly the gap it
  was meant to bridge. The boot-ID drop in the spool handles the same case
  honestly.

## Gotchas

- **CCLK zone handling is modem-specific.** 3GPP TS 27.007 says the `+CCLK`
  time field is local and `+zz` is the offset in quarter-hours, so subtracting
  `zz` should give UTC. The SIM7670G on this unit puts **UTC** in the time field
  *and* reports `+40`. v1.2.0 subtracted and landed the anchor exactly 36000 s
  early for every record built between modem-up and GNSS lock (1.27 % of rows
  over 14 days). v1.2.1 takes the field as UTC. A different modem may need the
  subtraction back; the handover delta line will say so on the first boot.
- **Telling a CCLK mis-stamp from a real spool drain** in the Postgres data:
  both show a large `servertime − devicetime` lag. A real drain delivers
  records spanning the capture window at one instant, so the lag spread
  matches the window (ratio ~0.75–0.94 observed). Mis-stamped live rows track
  servertime, so the spread collapses (ratio ~0.1–0.3). Details in SPDD §16.4.
- **Position is not fixed by any of this.** A GPS-less record now sorts into
  the right place, but it still carries no coordinates and Traccar still copies
  the last known fix onto it. Detect those rows with `fixtime < devicetime`,
  never with `valid` (SPDD §16.5).
- **Event packets (`EV=`, the VIN/login/ping path) carry no time PIDs** and
  cannot: Traccar's `decodeEvent()` parses exactly four keys and arrival-stamps
  the row. The firmware's answer is to never send a VIN that is not a real VIN,
  so no such row is created (SPDD §16.4, defect 2).

## Verification

- 2026-07-31: zero-lag rows (`devicetime = servertime`) fell from 33 % to
  1.4 % after the v1.2.0 flash.
- 2026-08-20: over 336 h / 41,488 rows, 92.5 % pre-fix → 3.8 %; genuine
  drains replay at capture time (e.g. 227 rows, lag 51957→52130 s, spread 172 s
  against a 232 s window). The residual zero-lag rows were all VIN event rows,
  fixed in v1.2.1 (SPDD §16.4).
- 2026-09-05: every cell outage in a seven-session day fully back-filled with
  correct `devicetime`, lag up to 1738 s (SPDD §16.5).
