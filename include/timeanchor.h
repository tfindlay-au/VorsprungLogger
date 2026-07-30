/*************************************************************************
* Monotonic capture ticks and the tiered UTC time anchor.
*
* Every record is built against a 64-bit monotonic microsecond tick
* (esp_timer_get_time) taken at the capture instant. Wall-clock UTC comes
* separately, from whichever source got there first, and is held as a single
* anchor pair: "at tick M the clock read T". Any record's UTC time is then
* T + (its tick - M), so a record captured before the anchor existed resolves
* just as well as one captured after it (delta is simply negative).
*
* This is what fixes SPDD §16.2 Finding A: records built with no GPS payload
* used to carry no time at all and were filed by the server at arrival time,
* which turned every spool drain into a phantom session. Now the anchor stamps
* them, GPS payload or not.
*
* Sources, in rank order (see timeAnchorSet for the replacement rule):
*   TIME_SRC_NET  cellular network time (AT+CCLK, NITZ-fed) and the Traccar
*                 login reply. Second-level; available in an underground
*                 garage where GNSS never locks.
*   TIME_SRC_GPS  GNSS UTC. Sub-second, and the reference every "good" row in
*                 Traccar already carries.
*
* See SPDD.md §16.2 and TIME.md.
*************************************************************************/

#ifndef TIMEANCHOR_H_INCLUDED
#define TIMEANCHOR_H_INCLUDED

#include <stdint.h>

class CStorage;

#define TIME_SRC_NONE 0
#define TIME_SRC_NET  1
#define TIME_SRC_GPS  2

// Monotonic microseconds since boot. 64-bit, so unlike millis() it does not
// wrap (~585,000 years), and it is unaffected by settimeofday().
uint64_t timeTicks();

// Randomised per boot. Ticks are only comparable within one boot, so a spooled
// record that outlives its session carries this to prove its tick is still
// meaningful (see RecordSpool::appendUnstamped).
void timeAnchorInit();
uint32_t timeBootId();

// Offer a UTC reading taken at `tickUs`. Returns true if it became the anchor.
// Rejected silently when the value is implausible or a better source holds.
bool timeAnchorSet(uint8_t source, uint32_t utcSec, uint64_t tickUs);
bool timeAnchorIsSet();

// Resolve a capture tick to UTC seconds. False when no anchor exists yet.
bool timeAnchorResolve(uint64_t tickUs, uint32_t& utcSec);

// Write the absolute-time PID triplet into a packet: 0x10/0x11 (the pair
// Traccar's stock decoder rebuilds devicetime from) plus 0x300 for the raw
// epoch. Emitted for every record, which is the whole point.
void timeWriteStamp(CStorage& store, uint32_t utcSec);

// Civil UTC <-> Unix epoch, and the GPS wire encodings the PIDs use:
//   date = DDMMYY as a decimal integer, time = HHMMSScc (cc = hundredths).
// Done by hand rather than via mktime/gmtime so the conversion never picks up
// whatever timezone the ESP32 happens to have configured.
uint32_t timeCivilToEpoch(int y, int m, int d, int hh, int mm, int ss);
void timeEpochToGps(uint32_t utcSec, uint32_t& gpsDate, uint32_t& gpsTime);

#endif // TIMEANCHOR_H_INCLUDED
