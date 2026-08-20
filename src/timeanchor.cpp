/******************************************************************************
* VorsprungLogger — Audi-focused vehicle telemetry logger
* Monotonic capture ticks and the tiered UTC time anchor. See timeanchor.h.
******************************************************************************/

#include <FreematicsPlus.h>
#include <esp_timer.h>
#include <esp_system.h>
#include <esp_attr.h>
#include <sys/time.h>
#include "telestore.h"
#include "uds_dids.h"
#include "timeanchor.h"

// A lower-ranked source may take over once the incumbent anchor is this old.
// Sized so a GNSS lock that goes away mid-drive hands back to network time
// after a couple of minutes rather than holding a drifting anchor forever.
#define ANCHOR_STALE_US (120ULL * 1000000ULL)

// Plausibility window: 2025-01-01 to 2050-01-01. A reading outside it is not a
// clock reading at all. Both ends earn their keep — a mis-parsed NMEA date can
// land anywhere, and an un-provisioned SIMCOM modem answers AT+CCLK? with its
// 1980 default, whose two-digit year reads back as 2080.
#define ANCHOR_MIN_EPOCH 1735689600UL
#define ANCHOR_MAX_EPOCH 2524608000UL

// The anchor is written from the main loop (GNSS) and read from the telemetry
// task (packet build, spool drain), so the 32+64-bit pair needs a lock to be
// read consistently.
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_bootId = 0;
static uint32_t s_utc = 0;
static uint64_t s_tick = 0;
static uint8_t  s_src = TIME_SRC_NONE;

// Deliberately in the no-init RTC segment: the bootloader leaves it alone, so
// it survives the ESP.restart() that ends standby (an RTC_DATA_ATTR would be
// reloaded from flash and defeat the point). A power cut leaves garbage, which
// is harmless — it only ever goes in as entropy.
static RTC_NOINIT_ATTR uint32_t s_bootSeq;

static const char* sourceName(uint8_t source)
{
  switch (source) {
    case TIME_SRC_GPS:    return "GNSS";
    case TIME_SRC_SERVER: return "SERVER";
    case TIME_SRC_MODEM:  return "MODEM";
    default:              return "NONE";
  }
}

uint64_t timeTicks()
{
  return (uint64_t)esp_timer_get_time();
}

void timeAnchorInit()
{
  // esp_random() is only a true random source with RF up, which it never is
  // here. Mixing in a counter that increments every restart guarantees
  // consecutive boots differ regardless — which is the property the spool's
  // stale-boot check actually depends on.
  s_bootSeq++;
  s_bootId = esp_random() ^ (s_bootSeq * 2654435761UL);
}

uint32_t timeBootId()
{
  return s_bootId;
}

bool timeAnchorSet(uint8_t source, uint32_t utcSec, uint64_t tickUs)
{
  if (utcSec < ANCHOR_MIN_EPOCH || utcSec > ANCHOR_MAX_EPOCH) return false;

  portENTER_CRITICAL(&s_mux);
  uint8_t  prev     = s_src;
  uint32_t prevUtc  = s_utc;
  uint64_t prevTick = s_tick;
  bool accept = source >= s_src || tickUs > s_tick + ANCHOR_STALE_US;
  if (accept) {
    s_src = source;
    s_utc = utcSec;
    s_tick = tickUs;
  }
  portEXIT_CRITICAL(&s_mux);
  if (!accept) return false;

  if (prev == TIME_SRC_NONE) {
    // First reading of the session — give the system clock the same value so
    // SD directory entries and printTime() are honest too.
    struct timeval tv = { .tv_sec = (time_t)utcSec, .tv_usec = 0 };
    settimeofday(&tv, NULL);
  }
  if (prev != source) {
    // GNSS re-anchors every second; only the handovers are worth printing.
    Serial.print("[TIME] anchor ");
    Serial.print(sourceName(source));
    Serial.print(" epoch ");
    Serial.print(utcSec);
    if (prev != TIME_SRC_NONE) {
      // What the outgoing anchor said this same instant was. A handover should
      // move the clock by a second or two at most; anything larger means the
      // source we had been trusting was lying, and every record stamped since
      // it took over is wrong by this much. Printing it is what turns a silent
      // corruption into an obvious one — the CCLK zone bug (SPDD §16.4) ran for
      // a month precisely because this number was never shown.
      int64_t predicted = (int64_t)prevUtc
                        + ((int64_t)tickUs - (int64_t)prevTick) / 1000000LL;
      Serial.print(" (");
      Serial.print(sourceName(prev));
      Serial.print(" was off by ");
      Serial.print((long)((int64_t)utcSec - predicted));
      Serial.print("s)");
    }
    Serial.println();
  }
  return true;
}

bool timeAnchorIsSet()
{
  portENTER_CRITICAL(&s_mux);
  bool set = s_src != TIME_SRC_NONE;
  portEXIT_CRITICAL(&s_mux);
  return set;
}

bool timeAnchorResolve(uint64_t tickUs, uint32_t& utcSec)
{
  portENTER_CRITICAL(&s_mux);
  uint8_t src = s_src;
  uint32_t utc = s_utc;
  uint64_t tick = s_tick;
  portEXIT_CRITICAL(&s_mux);
  if (src == TIME_SRC_NONE) return false;

  int64_t deltaUs = (int64_t)tickUs - (int64_t)tick;
  int64_t deltaSec = deltaUs / 1000000LL;
  // Truncation runs toward zero, which would pull a pre-anchor record forward
  // in time; floor it so those records stay on the correct side of the anchor.
  if (deltaUs % 1000000LL < 0) deltaSec--;

  int64_t sec = (int64_t)utc + deltaSec;
  if (sec < (int64_t)ANCHOR_MIN_EPOCH) return false;
  utcSec = (uint32_t)sec;
  return true;
}

void timeWriteStamp(CStorage& store, uint32_t utcSec)
{
  uint32_t gpsDate, gpsTime;
  timeEpochToGps(utcSec, gpsDate, gpsTime);
  store.log(PID_GPS_TIME, &gpsTime, 1);
  store.log(PID_GPS_DATE, &gpsDate, 1);
  store.log(PID_ABS_TIME, &utcSec, 1);
}

// Howard Hinnant's days_from_civil. Years/months are signed for the era math.
uint32_t timeCivilToEpoch(int y, int m, int d, int hh, int mm, int ss)
{
  y -= (m <= 2);
  int era = (y >= 0 ? y : y - 399) / 400;
  unsigned yoe = (unsigned)(y - era * 400);
  unsigned doy = (153 * (m > 2 ? m - 3 : m + 9) + 2) / 5 + d - 1;
  unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  long days = (long)era * 146097L + (long)doe - 719468L;
  return (uint32_t)(days * 86400L + hh * 3600 + mm * 60 + ss);
}

// civil_from_days, the exact inverse of the above.
void timeEpochToGps(uint32_t utcSec, uint32_t& gpsDate, uint32_t& gpsTime)
{
  long z = (long)(utcSec / 86400UL) + 719468L;
  unsigned long sod = utcSec % 86400UL;

  long era = (z >= 0 ? z : z - 146096L) / 146097L;
  unsigned doe = (unsigned)(z - era * 146097L);
  unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  long y = (long)yoe + era * 400;
  unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  unsigned mp = (5 * doy + 2) / 153;
  unsigned d = doy - (153 * mp + 2) / 5 + 1;
  unsigned m = mp + (mp < 10 ? 3 : -9);
  y += (m <= 2);

  gpsDate = (uint32_t)d * 10000UL + (uint32_t)m * 100UL + (uint32_t)(((y % 100) + 100) % 100);
  gpsTime = (uint32_t)(sod / 3600) * 1000000UL
          + (uint32_t)((sod % 3600) / 60) * 10000UL
          + (uint32_t)(sod % 60) * 100UL;
}
