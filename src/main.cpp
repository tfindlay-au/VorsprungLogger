/******************************************************************************
* VorsprungLogger — Audi-focused vehicle telemetry logger
* Main firmware: data collection, tiered OBD/UDS scheduling, telemetry loop.
*
* Runs on Freematics ONE+ Model B hardware. Logs OBD-II live PIDs plus VAG
* EA897 UDS DIDs and GPS/MEMS, and transmits to a Traccar server over
* cellular UDP.
*
* Derived from the Freematics Telelogger, developed by
* Stanley Huang <stanley@freematics.com.au>. Original copyright retained
* with thanks. Distributed under BSD license.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
******************************************************************************/

#include <FreematicsPlus.h>
#include <FreematicsUDS.h>
#include "config.h"
#include "telestore.h"
#include "teleclient.h"
#include "uds_dids.h"
#include "driver/adc.h"
#include "nvs_flash.h"
#include "nvs.h"

// states
#define STATE_STORAGE_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_READY 0x4
#define STATE_MEMS_READY 0x8
#define STATE_NET_READY 0x10
#define STATE_GPS_ONLINE 0x20
#define STATE_CELL_CONNECTED 0x40
#define STATE_WORKING 0x100
#define STATE_STANDBY 0x200

typedef struct {
  byte pid;
  byte tier;
  int value;
  uint32_t ts;
} PID_POLLING_INFO;

PID_POLLING_INFO obdData[]= {
  {PID_SPEED, 1},
  {PID_RPM, 1},
  {PID_THROTTLE, 1},
  {PID_ENGINE_LOAD, 1},
  {PID_FUEL_PRESSURE, 2},
  {PID_TIMING_ADVANCE, 2},
  {PID_COOLANT_TEMP, 3},
  {PID_INTAKE_TEMP, 3},
};

CBufferManager bufman;
Task subtask;

float accBias[3] = {0};
float accSum[3] = {0};
float acc[3] = {0};
float gyr[3] = {0};
float mag[3] = {0};
uint8_t accCount = 0;
int deviceTemp = 0;

// config data
char apn[32];
nvs_handle_t nvs;

// live data
String netop;
String ip;
int16_t rssi = 0;
int16_t rssiLast = 0;
char vin[18] = {0};
uint16_t dtc[6] = {0};
float batteryVoltage = 0;
GPS_DATA* gd = 0;

char devid[12] = {0};
char isoTime[32] = {0};

// stats data
uint32_t lastMotionTime = 0;
uint32_t timeoutsOBD = 0;
uint32_t timeoutsNet = 0;
uint32_t lastStatsTime = 0;

int32_t syncInterval = SERVER_SYNC_INTERVAL * 1000;
int32_t dataInterval = 1000;

byte ledMode = 0;

// Set by processOBD() each cycle: true when this cycle's RPM read succeeded.
// processUDS() reads it to know whether the derived-inputs group can be complete
// (RPM is a group member, read on the OBD side rather than via UDS).
bool obdRpmValid = false;

void serverProcess(int timeout);
void processMEMS(CBuffer* buffer);
bool processGPS(CBuffer* buffer);
void processBLE(int timeout);

class State {
public:
  bool check(uint16_t flags) { return (m_state & flags) == flags; }
  void set(uint16_t flags) { m_state |= flags; }
  void clear(uint16_t flags) { m_state &= ~flags; }
  uint16_t m_state = 0;
};

FreematicsESP32 sys;

class OBD : public COBD
{
protected:
  void idleTasks()
  {
    processMEMS(0);
  }
};

OBD obd;
CUDS uds;

MEMS_I2C* mems = 0;

RecordSpool spool;

TeleClientUDP teleClient;

State state;

void printTimeoutStats()
{
  Serial.print("Timeouts: OBD:");
  Serial.print(timeoutsOBD);
  Serial.print(" Network:");
  Serial.println(timeoutsNet);
}

// Play a short melody from an array of {frequency, duration} pairs.
// A frequency of 0 produces a silent rest of the given duration.
void playTune(const int notes[][2], int count)
{
    for (int i = 0; i < count; i++) {
        sys.buzzer(notes[i][0]);
        delay(notes[i][1]);
    }
    sys.buzzer(0);
}

// Cheerful rising arpeggio played when the cellular link comes up.
// Notes: C6, E6, G6, C7 (a C-major chord) with a final bright accent.
void chimeConnected()
{
    static const int tune[][2] = {
        {1047, 70},   // C6
        {1319, 70},   // E6
        {1568, 70},   // G6
        {2093, 130},  // C7 (held a touch longer)
    };
    playTune(tune, sizeof(tune) / sizeof(tune[0]));
}

/*******************************************************************************
  Reading and processing OBD data
*******************************************************************************/
void processOBD(CBuffer* buffer)
{
  static int idx[2] = {0, 0};
  int tier = 1;
  obdRpmValid = false;
  for (byte i = 0; i < sizeof(obdData) / sizeof(obdData[0]); i++) {
    if (obdData[i].tier > tier) {
        idx[tier - 2] = 0;
        tier = obdData[i].tier;
        i += idx[tier - 2]++;
        if (obdData[i].tier != tier) {
            idx[tier - 2]= 0;
            i--;
            continue;
        }
    }
    byte pid = obdData[i].pid;
    if (!obd.isValidPID(pid)) continue;
    int value;
    if (obd.readPID(pid, value)) {
        obdData[i].ts = millis();
        obdData[i].value = value;
        buffer->add((uint16_t)pid | 0x100, ELEMENT_INT32, &value, sizeof(value));
        if (pid == PID_RPM) obdRpmValid = true;
    } else {
        timeoutsOBD++;
        printTimeoutStats();
        break;
    }
    if (tier > 1) break;
  }
  int kph = obdData[0].value;
  if (kph >= 2) lastMotionTime = millis();
}

/*******************************************************************************
  Reading and processing UDS DIDs

  The DID table is large (~50 signals across two ECUs), so it is polled with
  a per-DID countdown scheduler instead of a flat round-robin:
    - each DID carries a cadence tier (UDS_FAST=every cycle, UDS_SLOW=10,
      UDS_GLACIAL=60); fast-changing signals are sampled often, slow ones rarely;
    - at most UDS_MAX_PER_CYCLE reads happen per cycle, bounding the time UDS
      adds to a cycle (each read is ~one ELM327 round-trip, <=100 ms);
    - the scheduler is work-conserving: when more DIDs are due than the budget
      allows, a rotating cursor serves the rest over the next few cycles rather
      than dropping them;
    - DIDs tagged UDS_GROUP are exempt from this round-robin. They form the
      "derived-inputs" group and are co-read atomically by readGroup() on a
      shared cadence so they always share a packet (see readGroup below).
  A failed DID is logged and skipped. See SPDD.md / uds_dids.h.
*******************************************************************************/
#define UDS_MAX_PER_CYCLE 8   // tunable: cap on UDS reads per process() cycle
#define UDS_GROUP_PERIOD  3   // co-read the derived-inputs group every Nth cycle

// Read and decode one DID into `value` (engineering units), logging the result.
// Returns false (and logs the cause) on transport failure or short payload, and
// leaves the buffer untouched so the caller decides whether/when to emit.
static bool readUDSValue(const UDSDIDEntry& e, float& value)
{
  uint8_t payload[UDS_MAX_PAYLOAD];
  uint8_t payloadLen = 0;
  if (!uds.readDID(e.did, payload, payloadLen, e.reqId)) {
    Serial.print("[UDS] ");
    Serial.print(e.label);
    Serial.print(" FAIL");
    uint8_t nrc = uds.lastNRC();
    if (nrc) {
      Serial.print(" NRC=0x");
      Serial.println(nrc, HEX);
    } else {
      Serial.println();
    }
    return false;
  }
  if (payloadLen < e.bytes) {
    Serial.print("[UDS] ");
    Serial.print(e.label);
    Serial.print(" short payload ");
    Serial.println(payloadLen);
    return false;
  }

  value = e.decode(payload);
  Serial.print("[UDS] ");
  Serial.print(e.label);
  Serial.print('=');
  Serial.print(value, 2);
  Serial.print(' ');
  Serial.println(e.unit);
  return true;
}

// Round-robin path: read one DID and, on success, add it straight to the packet.
static void pollUDS(CBuffer* buffer, const UDSDIDEntry& e)
{
  float value;
  if (readUDSValue(e, value))
    buffer->add(e.syntheticPID, ELEMENT_FLOAT_D2, &value, sizeof(value));
}

/*******************************************************************************
  The "derived-inputs" group (DIDs tagged UDS_GROUP). Downstream analytics
  (volumetric efficiency, power, AFR/lambda, BSFC, brake thermal efficiency) need
  these sampled at the same instant, so they are co-read here as one atomic unit
  and always land in the same packet as the RPM read — instead of drifting apart
  under the round-robin's per-cycle budget contention.

  Atomicity is all-or-nothing: members are staged into a temporary and committed
  to the packet only if every member — RPM included — reads cleanly this cycle.
  On any failure nothing is emitted, so a packet never carries a partial group.

  Returns the number of UDS reads attempted, so the caller can debit the cycle's
  read budget accurately even when the group is suppressed mid-way.
*******************************************************************************/
static int readGroup(CBuffer* buffer, bool rpmValid)
{
  // RPM is a group member (read on the OBD side); without it the group can't be
  // complete this cycle, so skip the bus traffic entirely.
  if (!rpmValid) return 0;

  // Sized to the whole table as a safe upper bound; only UDS_GROUP rows are used.
  float values[UDS_DID_COUNT];
  const UDSDIDEntry* members[UDS_DID_COUNT];
  int staged = 0;
  int reads = 0;
  for (int i = 0; i < UDS_DID_COUNT; i++) {
    if (!udsIsGroupMember(UDS_DID_TABLE[i])) continue;
    reads++;
    if (!readUDSValue(UDS_DID_TABLE[i], values[staged]))
      return reads;                   // member failed — suppress the whole group
    members[staged++] = &UDS_DID_TABLE[i];
  }
  for (int i = 0; i < staged; i++)
    buffer->add(members[i]->syntheticPID, ELEMENT_FLOAT_D2, &values[i], sizeof(float));
  return reads;
}

void processUDS(CBuffer* buffer)
{
  if (UDS_DID_COUNT <= 0) return;

  // wait[i] = cycles remaining before DID i is due. A DID is polled when its
  // wait hits 0, then rescheduled to its cadence tier. rr rotates so that, when
  // the budget is exhausted, the DIDs left over get first refusal next cycle.
  // Group members (UDS_GROUP) are excluded here — readGroup() handles them.
  static uint16_t wait[UDS_DID_COUNT] = {0};
  static int rr = 0;
  static uint8_t groupWait = 0;   // cycles until the derived-inputs group is due

  int budget = UDS_MAX_PER_CYCLE;

  // The group takes priority on the cycles it fires, spending its reads from the
  // shared budget so the round-robin gets the remainder. On the other
  // (UDS_GROUP_PERIOD - 1) cycles the full budget serves the rest as before.
  if (groupWait > 0) {
    groupWait--;
  } else {
    budget -= readGroup(buffer, obdRpmValid);
    if (budget < 0) budget = 0;
    groupWait = UDS_GROUP_PERIOD - 1;
  }

  int lastServed = -1;
  for (int n = 0; n < UDS_DID_COUNT && budget > 0; n++) {
    int i = (rr + n) % UDS_DID_COUNT;
    if (udsIsGroupMember(UDS_DID_TABLE[i])) continue;  // co-read above
    if (wait[i]) continue;            // not due yet
    pollUDS(buffer, UDS_DID_TABLE[i]);
    wait[i] = UDS_DID_TABLE[i].intervalCycles;
    lastServed = i;
    budget--;
  }
  if (lastServed >= 0) rr = (lastServed + 1) % UDS_DID_COUNT;

  for (int i = 0; i < UDS_DID_COUNT; i++) {
    if (udsIsGroupMember(UDS_DID_TABLE[i])) continue;
    if (wait[i]) wait[i]--;
  }
}

// Convert a UTC civil date/time to Unix epoch seconds without leaning on
// mktime / TZ, which would otherwise interpret the GPS broadcast through
// whatever local timezone the ESP32 happens to have configured. Algorithm:
// Howard Hinnant's days_from_civil. Years/months are signed for the era math.
static uint32_t gpsCivilToEpoch(int y, int m, int d, int h, int mi, int s)
{
  y -= (m <= 2);
  int era = (y >= 0 ? y : y - 399) / 400;
  unsigned yoe = (unsigned)(y - era * 400);
  unsigned doy = (153 * (m > 2 ? m - 3 : m + 9) + 2) / 5 + d - 1;
  unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  long days = (long)era * 146097L + (long)doe - 719468L;
  return (uint32_t)(days * 86400L + h * 3600 + mi * 60 + s);
}

bool initGPS()
{
  if (sys.gpsBeginExt()) {
    Serial.println("GNSS:OK(E)");
  } else if (sys.gpsBegin()) {
    Serial.println("GNSS:OK(I)");
  } else {
    Serial.println("GNSS:NO");
    return false;
  }
  return true;
}

bool processGPS(CBuffer* buffer)
{
  static uint32_t lastGPStime = 0;
  static float lastGPSLat = 0;
  static float lastGPSLng = 0;

  if (!gd) {
    lastGPStime = 0;
    lastGPSLat = 0;
    lastGPSLng = 0;
  }
#if GNSS == GNSS_STANDALONE
  if (state.check(STATE_GPS_READY)) {
    if (!sys.gpsGetData(&gd)) {
      return false;
    }
  }
#else
    if (!teleClient.cell.getLocation(&gd)) {
      return false;
    }
#endif
  if (!gd || lastGPStime == gd->time) return false;
  if (gd->date) {
    char *p = isoTime + sprintf(isoTime, "%04u-%02u-%02uT%02u:%02u:%02u",
        (unsigned int)(gd->date % 100) + 2000, (unsigned int)(gd->date / 100) % 100, (unsigned int)(gd->date / 10000),
        (unsigned int)(gd->time / 1000000), (unsigned int)(gd->time % 1000000) / 10000, (unsigned int)(gd->time % 10000) / 100);
    unsigned char tenth = (gd->time % 100) / 10;
    if (tenth) p += sprintf(p, ".%c00", '0' + tenth);
    *p = 'Z';
    *(p + 1) = 0;
  }
  if (gd->lng == 0 && gd->lat == 0) {
    if (gd->date) {
      Serial.print("[GNSS] ");
      Serial.println(isoTime);
    }
    return false;
  }
  if ((lastGPSLat || lastGPSLng) && (abs(gd->lat - lastGPSLat) > 0.001 || abs(gd->lng - lastGPSLng) > 0.001)) {
    lastGPSLat = 0;
    lastGPSLng = 0;
    return false;
  }
  lastGPSLat = gd->lat;
  lastGPSLng = gd->lng;

  float kph = gd->speed * 1.852f;
  if (kph >= 2) lastMotionTime = millis();

  if (buffer) {
    if (gd->date) {
      // Time PIDs only ride along with a valid GPS date. Traccar rebuilds
      // devicetime from the time PID plus the date PID; without the date it
      // stamps the packet with the server's current date, which mis-dates any
      // spool replay that crosses UTC midnight. A time without a date isn't
      // trustworthy anyway - leave both off and let the server use receive
      // time for the (live) packet.
      buffer->add(PID_GPS_TIME, ELEMENT_UINT32, &gd->time, sizeof(uint32_t));
      buffer->add(PID_GPS_DATE, ELEMENT_UINT32, &gd->date, sizeof(uint32_t));
      // Bake the absolute capture time into the packet so spool replay across
      // reboots resolves to the original moment, not the next session's anchor.
      uint32_t epoch = gpsCivilToEpoch(
        2000 + (int)(gd->date % 100),
        (int)((gd->date / 100) % 100),
        (int)(gd->date / 10000),
        (int)(gd->time / 1000000),
        (int)((gd->time % 1000000) / 10000),
        (int)((gd->time % 10000) / 100));
      buffer->add(PID_ABS_TIME, ELEMENT_UINT32, &epoch, sizeof(epoch));
    }
    buffer->add(PID_GPS_LATITUDE, ELEMENT_FLOAT, &gd->lat, sizeof(float));
    buffer->add(PID_GPS_LONGITUDE, ELEMENT_FLOAT, &gd->lng, sizeof(float));
    buffer->add(PID_GPS_ALTITUDE, ELEMENT_FLOAT_D1, &gd->alt, sizeof(float)); /* m */
    buffer->add(PID_GPS_SPEED, ELEMENT_FLOAT_D1, &kph, sizeof(kph));
    buffer->add(PID_GPS_HEADING, ELEMENT_UINT16, &gd->heading, sizeof(uint16_t));
    if (gd->sat) buffer->add(PID_GPS_SAT_COUNT, ELEMENT_UINT8, &gd->sat, sizeof(uint8_t));
    if (gd->hdop) buffer->add(PID_GPS_HDOP, ELEMENT_UINT8, &gd->hdop, sizeof(uint8_t));
  }

  Serial.print("[GNSS] ");
  Serial.print(gd->lat, 6);
  Serial.print(' ');
  Serial.print(gd->lng, 6);
  Serial.print(' ');
  Serial.print((int)kph);
  Serial.print("km/h");
  Serial.print(" SATS:");
  Serial.print(gd->sat);
  Serial.print(" HDOP:");
  Serial.print(gd->hdop);
  Serial.print(" Course:");
  Serial.println(gd->heading);
  lastGPStime = gd->time;
  return true;
}

bool waitMotionGPS(int timeout)
{
  unsigned long t = millis();
  lastMotionTime = 0;
  do {
    serverProcess(100);
    if (!processGPS(0)) continue;
    if (lastMotionTime) return true;
  } while (millis() - t < timeout);
  return false;
}

void processMEMS(CBuffer* buffer)
{
  if (!state.check(STATE_MEMS_READY)) return;

  float temp;
  if (!mems->read(acc, gyr, mag, &temp)) return;
  deviceTemp = (int)temp;

  accSum[0] += acc[0];
  accSum[1] += acc[1];
  accSum[2] += acc[2];
  accCount++;

  if (buffer) {
    if (accCount) {
      float value[3];
      value[0] = accSum[0] / accCount - accBias[0];
      value[1] = accSum[1] / accCount - accBias[1];
      value[2] = accSum[2] / accCount - accBias[2];
      buffer->add(PID_ACC, ELEMENT_FLOAT_D2, value, sizeof(value), 3);
    }
    accSum[0] = 0;
    accSum[1] = 0;
    accSum[2] = 0;
    accCount = 0;
  }
}

void calibrateMEMS()
{
  if (state.check(STATE_MEMS_READY)) {
    accBias[0] = 0;
    accBias[1] = 0;
    accBias[2] = 0;
    int n;
    unsigned long t = millis();
    for (n = 0; millis() - t < 1000; n++) {
      float acc[3];
      if (!mems->read(acc)) continue;
      accBias[0] += acc[0];
      accBias[1] += acc[1];
      accBias[2] += acc[2];
      delay(10);
    }
    accBias[0] /= n;
    accBias[1] /= n;
    accBias[2] /= n;
    Serial.print("ACC BIAS:");
    Serial.print(accBias[0]);
    Serial.print('/');
    Serial.print(accBias[1]);
    Serial.print('/');
    Serial.println(accBias[2]);
  }
}

void printTime()
{
  time_t utc;
  time(&utc);
  struct tm *btm = gmtime(&utc);
  if (btm->tm_year > 100) {
    char buf[64];
    sprintf(buf, "%04u-%02u-%02u %02u:%02u:%02u",
      1900 + btm->tm_year, btm->tm_mon + 1, btm->tm_mday, btm->tm_hour, btm->tm_min, btm->tm_sec);
    Serial.print("UTC:");
    Serial.println(buf);
  }
}

/*******************************************************************************
  Initializing all data logging components
*******************************************************************************/
void initialize()
{
  bufman.purge();

  if (state.check(STATE_MEMS_READY)) {
    calibrateMEMS();
  }

#if GNSS == GNSS_STANDALONE
  if (!state.check(STATE_GPS_READY)) {
    if (initGPS()) {
      state.set(STATE_GPS_READY);
    }
  }
#endif

  if (!state.check(STATE_OBD_READY)) {
    timeoutsOBD = 0;
    if (obd.init()) {
      Serial.println("OBD:OK");
      state.set(STATE_OBD_READY);
      uds.applyHeaderConfig();
    } else {
      Serial.println("OBD:NO");
    }
  }

  if (!state.check(STATE_STORAGE_READY)) {
    if (spool.init()) {
      state.set(STATE_STORAGE_READY);
    }
  }

  if (state.check(STATE_OBD_READY)) {
    char buf[128];
    if (obd.getVIN(buf, sizeof(buf))) {
      memcpy(vin, buf, sizeof(vin) - 1);
      Serial.print("VIN:");
      Serial.println(vin);
    }
    int dtcCount = obd.readDTC(dtc, sizeof(dtc) / sizeof(dtc[0]));
    if (dtcCount > 0) {
      Serial.print("DTC:");
      Serial.println(dtcCount);
    }
  }

  printTime();

  lastMotionTime = millis();
  state.set(STATE_WORKING);
}

void showStats()
{
  uint32_t t = millis() - teleClient.startTime;
  char buf[32];
  sprintf(buf, "%02u:%02u.%c ", t / 60000, (t % 60000) / 1000, (t % 1000) / 100 + '0');
  Serial.print("[NET] ");
  Serial.print(buf);
  Serial.print("| Packet #");
  Serial.print(teleClient.txCount);
  Serial.print(" | Out: ");
  Serial.print(teleClient.txBytes >> 10);
  Serial.print(" KB | In: ");
  Serial.print(teleClient.rxBytes);
  Serial.print(" bytes | ");
  Serial.print((unsigned int)((uint64_t)(teleClient.txBytes + teleClient.rxBytes) * 3600 / (millis() - teleClient.startTime)));
  Serial.println(" KB/h");
}

bool waitMotion(long timeout)
{
  unsigned long t = millis();
  if (state.check(STATE_MEMS_READY)) {
    do {
      float motion = 0;
      float acc[3];
      if (!mems->read(acc)) continue;
      if (accCount == 10) {
        accCount = 0;
        accSum[0] = 0;
        accSum[1] = 0;
        accSum[2] = 0;
      }
      accSum[0] += acc[0];
      accSum[1] += acc[1];
      accSum[2] += acc[2];
      accCount++;
      for (byte i = 0; i < 3; i++) {
        float m = (acc[i] - accBias[i]);
        motion += m * m;
      }
      if (motion >= MOTION_THRESHOLD * MOTION_THRESHOLD) {
        Serial.println(motion);
        return true;
      }
    } while (state.check(STATE_STANDBY) && ((long)(millis() - t) < timeout || timeout == -1));
    return false;
  }
  serverProcess(timeout);
  return false;
}

/*******************************************************************************
  Collecting and processing data
*******************************************************************************/
void process()
{
  static uint32_t lastGPStick = 0;
  uint32_t startTime = millis();

  CBuffer* buffer = bufman.getFree();
  buffer->state = BUFFER_STATE_FILLING;

  if (state.check(STATE_OBD_READY)) {
    processOBD(buffer);
    if (obd.errors >= MAX_OBD_ERRORS) {
      if (!obd.init()) {
        Serial.println("[OBD] ECU OFF");
        state.clear(STATE_OBD_READY | STATE_WORKING);
        return;
      }
      uds.applyHeaderConfig();
    }
    processUDS(buffer);
  } else if (obd.init(PROTO_AUTO, true)) {
    state.set(STATE_OBD_READY);
    uds.applyHeaderConfig();
    Serial.println("[OBD] ECU ON");
  }

  if (rssi != rssiLast) {
    int val = (rssiLast = rssi);
    buffer->add(PID_CSQ, ELEMENT_INT32, &val, sizeof(val));
  }
  if (sys.devType > 12) {
    batteryVoltage = (float)(analogRead(A0) * 45) / 4095;
  } else {
    batteryVoltage = obd.getVoltage();
  }
  if (batteryVoltage) {
    uint16_t v = batteryVoltage * 100;
    buffer->add(PID_BATTERY_VOLTAGE, ELEMENT_UINT16, &v, sizeof(v));
  }

  processMEMS(buffer);

  bool success = processGPS(buffer);
#if GNSS_RESET_TIMEOUT
  if (success) {
    lastGPStick = millis();
    state.set(STATE_GPS_ONLINE);
  } else {
    if (millis() - lastGPStick > GNSS_RESET_TIMEOUT * 1000) {
      sys.gpsEnd();
      state.clear(STATE_GPS_ONLINE | STATE_GPS_READY);
      delay(20);
      if (initGPS()) state.set(STATE_GPS_READY);
      lastGPStick = millis();
    }
  }
#endif

  if (!state.check(STATE_MEMS_READY)) {
    deviceTemp = readChipTemperature();
  }
  buffer->add(PID_DEVICE_TEMP, ELEMENT_INT32, &deviceTemp, sizeof(deviceTemp));

  buffer->timestamp = millis();
  buffer->state = BUFFER_STATE_FILLED;

  if (startTime - lastStatsTime >= 3000) {
    bufman.printStats();
    lastStatsTime = startTime;
  }

  const int dataIntervals[] = DATA_INTERVAL_TABLE;
  const uint16_t stationaryTime[] = STATIONARY_TIME_TABLE;
  unsigned int motionless = (millis() - lastMotionTime) / 1000;
  bool stationary = true;
  for (byte i = 0; i < sizeof(stationaryTime) / sizeof(stationaryTime[0]); i++) {
    dataInterval = dataIntervals[i];
    if (motionless < stationaryTime[i] || stationaryTime[i] == 0) {
      stationary = false;
      break;
    }
  }
  if (stationary) {
    Serial.print("Stationary for ");
    Serial.print(motionless);
    Serial.println(" secs");
    state.clear(STATE_WORKING);
    return;
  }
  do {
    long t = dataInterval - (millis() - startTime);
    delay(t > 0 ? t : 0);
  } while (millis() - startTime < dataInterval);
}

bool initCell(bool quick = false)
{
  Serial.println("[CELL] Activating...");
  if (!teleClient.cell.begin(&sys)) {
    Serial.println("[CELL] No supported module");
    return false;
  }
  if (quick) return true;
  Serial.print("CELL:");
  Serial.println(teleClient.cell.deviceName());
  if (!teleClient.cell.checkSIM(SIM_CARD_PIN)) {
    Serial.println("NO SIM CARD");
  }
  Serial.print("IMEI:");
  Serial.println(teleClient.cell.IMEI);
  Serial.println("[CELL] Searching...");
  if (*apn) {
    Serial.print("APN:");
    Serial.println(apn);
  }
  if (teleClient.cell.setup(apn, APN_USERNAME, APN_PASSWORD)) {
    netop = teleClient.cell.getOperatorName();
    if (netop.length()) {
      Serial.print("Operator:");
      Serial.println(netop);
    }

#if GNSS == GNSS_CELLULAR
    if (teleClient.cell.setGPS(true)) {
      Serial.println("CELL GNSS:OK");
    }
#endif

    ip = teleClient.cell.getIP();
    if (ip.length()) {
      Serial.print("[CELL] IP:");
      Serial.println(ip);
    }
    state.set(STATE_CELL_CONNECTED);
  } else {
    char *p = strstr(teleClient.cell.getBuffer(), "+CPSI:");
    if (p) {
      char *q = strchr(p, '\r');
      if (q) *q = 0;
      Serial.print("[CELL] ");
      Serial.println(p + 7);
    } else {
      Serial.print(teleClient.cell.getBuffer());
    }
  }
  timeoutsNet = 0;
  return state.check(STATE_CELL_CONNECTED);
}

/*******************************************************************************
  Serializing queued live buffers into the SD spool

  Called whenever the cell link is unusable (mid-drive outage or the back-off
  between reconnect attempts) so records survive on SD instead of rotating out
  of the RAM buffers. Oldest-first keeps the spool file chronological.
*******************************************************************************/
void spoolQueuedBuffers(CStorageRAM& store)
{
  CBuffer* buffer;
  while ((buffer = bufman.getOldest())) {
    store.header(devid);
    store.timestamp(buffer->timestamp);
    buffer->serialize(store);
    bufman.free(buffer);
    store.tailer();
    spool.append(store.buffer(), store.length());
    store.purge();
  }
}

/*******************************************************************************
  Initializing network, maintaining connection and doing transmissions
*******************************************************************************/
void telemetry(void* inst)
{
  uint32_t lastRssiTime = 0;
  uint8_t connErrors = 0;
  CStorageRAM store;
  store.init(
    (char*)malloc(SERIALIZE_BUFFER_SIZE),
    SERIALIZE_BUFFER_SIZE
  );
  teleClient.reset();

  for (;;) {
    if (state.check(STATE_STANDBY)) {
      if (state.check(STATE_CELL_CONNECTED)) {
        teleClient.shutdown();
        netop = "";
        ip = "";
        rssi = 0;
      }
      state.clear(STATE_NET_READY | STATE_CELL_CONNECTED);
      teleClient.reset();
      // Close any in-progress spool file so its FAT directory entry is
      // committed before the device restarts on wake. Orphan files are
      // re-discovered by spool.init() on next boot.
      spool.closeForStandby();
      bufman.purge();

      uint32_t t = millis();
      do {
        delay(1000);
      } while (state.check(STATE_STANDBY) && millis() - t < 1000L * PING_BACK_INTERVAL);
      if (state.check(STATE_STANDBY)) {
        if (initCell()) {
          Serial.println("[CELL] Ping...");
          teleClient.ping();
        }
        teleClient.shutdown();
        state.clear(STATE_CELL_CONNECTED);
      }
      continue;
    }

    while (state.check(STATE_WORKING)) {
      if (!state.check(STATE_CELL_CONNECTED)) {
        connErrors = 0;
        // Records queued up since the link dropped (or since boot) go to the
        // spool now, before the blocking connect attempt below.
        spoolQueuedBuffers(store);
        if (!initCell() || !teleClient.connect()) {
          teleClient.cell.end();
          state.clear(STATE_NET_READY | STATE_CELL_CONNECTED);
          Serial.println("[CELL] Deactivated");
          // avoid turning on/off cellular module too frequently to avoid operator banning
          uint32_t t = millis();
          do {
            spoolQueuedBuffers(store);
            delay(1000);
          } while (state.check(STATE_WORKING) && millis() - t < 60000 * 3);
          break;
        }
        Serial.println("[CELL] In service");
        state.set(STATE_NET_READY);
        chimeConnected();
      }

      if (millis() - lastRssiTime > SIGNAL_CHECK_INTERVAL * 1000) {
        rssi = teleClient.cell.RSSI();
        if (rssi) {
          Serial.print("RSSI:");
          Serial.print(rssi);
          Serial.println("dBm");
        }
        lastRssiTime = millis();
      }

      CBuffer* buffer = bufman.getNewest();
      if (!buffer) {
        // Live producer is idle. Use the gap to drain one spooled record —
        // older outages bleed back to the server in the background while the
        // 1 Hz live path keeps first refusal on the cell. ~10 records/s when
        // the live producer is fully idle, naturally interleaved with live
        // transmits when it isn't.
        if (spool.drainOneRecord(teleClient)) {
          delay(100);
        } else {
          delay(50);
        }
        continue;
      }
      store.header(devid);
      store.timestamp(buffer->timestamp);
      buffer->serialize(store);
      bufman.free(buffer);
      store.tailer();
      Serial.print("[DAT] ");
      Serial.println(store.buffer());

      if (teleClient.transmit(store.buffer(), store.length())) {
        connErrors = 0;
        // The link is good again — close any open outage file so the drain
        // path picks it up on the next idle gap (no-op when none is open).
        spool.endOutage();
        showStats();
      } else {
        timeoutsNet++;
        connErrors++;
        printTimeoutStats();
        spool.append(store.buffer(), store.length());
        if (connErrors < MAX_CONN_ERRORS_RECONNECT) {
          teleClient.connect(true);
        }
      }
      store.purge();

      teleClient.inbound();

      if (state.check(STATE_CELL_CONNECTED) && !teleClient.cell.check(1000)) {
        Serial.println("[CELL] Not in service");
        state.clear(STATE_NET_READY | STATE_CELL_CONNECTED);
        break;
      }

      if (syncInterval > 10000 && millis() - teleClient.lastSyncTime > syncInterval) {
        Serial.println("[NET] Poor connection");
        timeoutsNet++;
        if (!teleClient.connect()) {
          connErrors++;
        }
      }

      if (connErrors >= MAX_CONN_ERRORS_RECONNECT) {
        if (state.check(STATE_CELL_CONNECTED)) {
          teleClient.cell.end();
          state.clear(STATE_NET_READY | STATE_CELL_CONNECTED);
          break;
        }
      }

      if (deviceTemp >= COOLING_DOWN_TEMP) {
        Serial.print("HIGH DEVICE TEMP: ");
        Serial.println(deviceTemp);
        bufman.purge();
      }
    }
  }
}

/*******************************************************************************
  Implementing stand-by mode
*******************************************************************************/
void standby()
{
  state.set(STATE_STANDBY);
  // The telemetry task observes STATE_STANDBY and flushes/closes its spool
  // handles itself (see telemetry()); nothing to do here for storage.

#if !GNSS_ALWAYS_ON && GNSS == GNSS_STANDALONE
  if (state.check(STATE_GPS_READY)) {
    Serial.println("[GNSS] OFF");
    sys.gpsEnd(true);
    state.clear(STATE_GPS_READY | STATE_GPS_ONLINE);
    gd = 0;
  }
#endif

  state.clear(STATE_WORKING | STATE_OBD_READY | STATE_STORAGE_READY);
  Serial.println("STANDBY");
  obd.enterLowPowerMode();
  calibrateMEMS();
  waitMotion(-1);
  Serial.println("WAKEUP");
  sys.resetLink();
  if (mems) mems->end();
  ESP.restart();
  state.clear(STATE_STANDBY);
}

void genDeviceID(char* buf)
{
    uint64_t seed = ESP.getEfuseMac() >> 8;
    for (int i = 0; i < 8; i++, seed >>= 5) {
      byte x = (byte)seed & 0x1f;
      if (x >= 10) {
        x = x - 10 + 'A';
        switch (x) {
          case 'B': x = 'W'; break;
          case 'D': x = 'X'; break;
          case 'I': x = 'Y'; break;
          case 'O': x = 'Z'; break;
        }
      } else {
        x += '0';
      }
      buf[i] = x;
    }
    buf[8] = 0;
}

void showSysInfo()
{
  Serial.println("VorsprungLogger " FIRMWARE_VERSION " - Audi telemetry logger");
  Serial.println("Build: " __DATE__ " " __TIME__);
  Serial.print("CPU:");
  Serial.print(ESP.getCpuFreqMHz());
  Serial.print("MHz FLASH:");
  Serial.print(ESP.getFlashChipSize() >> 20);
  Serial.println("MB");
  Serial.print("IRAM:");
  Serial.print(ESP.getHeapSize() >> 10);
  Serial.println("KB");

  int rtc = rtc_clk_slow_freq_get();
  if (rtc) {
    Serial.print("RTC:");
    Serial.println(rtc);
  }

  Serial.print("DEVICE ID:");
  Serial.println(devid);
}

void loadConfig()
{
  size_t len = sizeof(apn);
  apn[0] = 0;
  nvs_get_str(nvs, "CELL_APN", apn, &len);
  if (!apn[0]) {
    strcpy(apn, CELL_APN);
  }
}

void processBLE(int timeout)
{
  if (timeout) delay(timeout);
}

void setup()
{
  delay(500);

  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);
  err = nvs_open("storage", NVS_READWRITE, &nvs);
  if (err == ESP_OK) {
    loadConfig();
  }

  // 38400 to match the external GNSS module: the module's NMEA output shares
  // UART0's receive channel (see gpsBeginExt), and one UART runs one baud
  // for both directions. Update monitor_speed in platformio.ini in lockstep.
  Serial.begin(38400);

#ifdef PIN_LED
  pinMode(PIN_LED, OUTPUT);
  if (ledMode == 0) digitalWrite(PIN_LED, HIGH);
#endif

  genDeviceID(devid);
  showSysInfo();

  bufman.init();

  if (sys.begin()) {
    Serial.print("TYPE:");
    Serial.println(sys.devType);
    obd.begin(sys.link);
    uds.begin(&obd);
  }

  if (!state.check(STATE_MEMS_READY)) do {
    Serial.print("MEMS:");
    mems = new ICM_42627;
    byte ret = mems->begin();
    if (ret) {
      state.set(STATE_MEMS_READY);
      Serial.println("ICM-42627");
      break;
    }
    delete mems;
    mems = new ICM_20948_I2C;
    ret = mems->begin();
    if (ret) {
      state.set(STATE_MEMS_READY);
      Serial.println("ICM-20948");
      break;
    }
    delete mems;
    mems = 0;
    Serial.println("NO");
  } while (0);

  state.set(STATE_WORKING);

  initialize();

  subtask.create(telemetry, "telemetry", 2, 8192);

#ifdef PIN_LED
  digitalWrite(PIN_LED, LOW);
#endif
}

void loop()
{
  if (!state.check(STATE_WORKING)) {
    standby();
#ifdef PIN_LED
    if (ledMode == 0) digitalWrite(PIN_LED, HIGH);
#endif
    initialize();
#ifdef PIN_LED
    digitalWrite(PIN_LED, LOW);
#endif
    return;
  }

  process();
}
