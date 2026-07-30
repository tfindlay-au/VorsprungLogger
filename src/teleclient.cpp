/******************************************************************************
* VorsprungLogger — Audi-focused vehicle telemetry logger
* Telemetry client: UDP over cellular to a Traccar server.
*
* Runs on Freematics ONE+ Model B hardware.
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
#include "telestore.h"
#include "teleclient.h"
#include "timeanchor.h"
#include "config.h"

extern int16_t rssi;
extern char devid[];
extern char vin[];
extern GPS_DATA* gd;
extern char isoTime[];

CBuffer::CBuffer(uint8_t* mem)
{
  m_data = mem;
  purge();
}

void CBuffer::add(uint16_t pid, uint8_t type, void* values, int bytes, uint8_t count)
{
  if (offset < BUFFER_LENGTH - sizeof(ELEMENT_HEAD) - bytes) {
    ELEMENT_HEAD hdr = {pid, type, count};
    *(ELEMENT_HEAD*)(m_data + offset) = hdr;
    offset += sizeof(ELEMENT_HEAD);
    memcpy(m_data + offset, values, bytes);
    offset += bytes;
    total++;
  } else {
    Serial.println("FULL");
  }
}

void CBuffer::purge()
{
  state = BUFFER_STATE_EMPTY;
  tick = 0;
  timestamp = 0;
  offset = 0;
  total = 0;
}

void CBuffer::serialize(CStorage& store)
{
  uint16_t of = 0;
  for (int n = 0; n < total && of < offset; n++) {
    ELEMENT_HEAD* hdr = (ELEMENT_HEAD*)(m_data + of);
    of += sizeof(ELEMENT_HEAD);
    switch (hdr->type) {
    case ELEMENT_UINT8:
      store.log(hdr->pid, (uint8_t*)(m_data + of), hdr->count);
      of += (uint16_t)hdr->count * sizeof(uint8_t);
      break;
    case ELEMENT_UINT16:
      store.log(hdr->pid, (uint16_t*)(m_data + of), hdr->count);
      of += (uint16_t)hdr->count * sizeof(uint16_t);
      break;
    case ELEMENT_UINT32:
      store.log(hdr->pid, (uint32_t*)(m_data + of), hdr->count);
      of += (uint16_t)hdr->count * sizeof(uint32_t);
      break;
    case ELEMENT_INT32:
      store.log(hdr->pid, (int32_t*)(m_data + of), hdr->count);
      of += (uint16_t)hdr->count * sizeof(int32_t);
      break;
    case ELEMENT_FLOAT:
      store.log(hdr->pid, (float*)(m_data + of), hdr->count);
      of += (uint16_t)hdr->count * sizeof(float);
      break;
    case ELEMENT_FLOAT_D1:
      store.log(hdr->pid, (float*)(m_data + of), hdr->count, "%.1f");
      of += (uint16_t)hdr->count * sizeof(float);
      break;
    case ELEMENT_FLOAT_D2:
      store.log(hdr->pid, (float*)(m_data + of), hdr->count, "%.2f");
      of += (uint16_t)hdr->count * sizeof(float);
      break;
    default:
      return;
    }
  }
}

void CBufferManager::init()
{
  total = BUFFER_SLOTS;
  slots = (CBuffer**)malloc(BUFFER_SLOTS * sizeof(void*));
  for (int n = 0; n < BUFFER_SLOTS; n++) {
    void* mem = malloc(BUFFER_LENGTH);
    if (!mem) {
      Serial.println("OUT OF RAM");
      total = n;
      break;
    }
    slots[n] = new CBuffer((uint8_t*)mem);
  }
  assert(total > 0);
}

void CBufferManager::purge()
{
  for (int n = 0; n < total; n++) slots[n]->purge();
}

CBuffer* CBufferManager::getFree()
{
  if (last) {
    CBuffer* slot = last;
    last = 0;
    if (slot->state == BUFFER_STATE_EMPTY) return slot;
  }
  uint32_t ts = 0xffffffff;
  int m = 0;
  for (int n = 0; n < total; n++) {
    if (slots[n]->state == BUFFER_STATE_EMPTY) {
      return slots[n];
    } else if (slots[n]->state == BUFFER_STATE_FILLED && slots[n]->timestamp < ts) {
        m = n;
        ts = slots[n]->timestamp;
    }
  }
  while (slots[m]->state == BUFFER_STATE_LOCKED) delay(1);
  slots[m]->purge();
  return slots[m];
}

CBuffer* CBufferManager::getOldest()
{
  uint32_t ts = 0xffffffff;
  int m = -1;
  for (int n = 0; n < total; n++) {
    if (slots[n]->state == BUFFER_STATE_FILLED && slots[n]->timestamp < ts) {
        m = n;
        ts = slots[n]->timestamp;
    }
  }
  if (m >= 0) {
    slots[m]->state = BUFFER_STATE_LOCKED;
    return slots[m];
  }
  return 0;
}

CBuffer* CBufferManager::getNewest()
{
  uint32_t ts = 0;
  int m = -1;
  for (int n = 0; n < total; n++) {
    if (slots[n]->state == BUFFER_STATE_FILLED && slots[n]->timestamp > ts) {
      m = n;
      ts = slots[n]->timestamp;
    }
  }
  if (m >= 0) {
    slots[m]->state = BUFFER_STATE_LOCKED;
    return slots[m];
  }
  return 0;
}

void CBufferManager::free(CBuffer* slot)
{
  slot->purge();
  last = slot;
}

void CBufferManager::printStats()
{
  int bytes = 0;
  int count = 0;
  int samples = 0;
  for (int n = 0; n < total; n++) {
    if (slots[n]->state != BUFFER_STATE_FILLED) continue;
    bytes += slots[n]->offset;
    samples += slots[n]->total;
    count++;
  }
  if (slots) {
    Serial.print("[BUF] ");
    Serial.print(samples);
    Serial.print(" samples | ");
    Serial.print(bytes);
    Serial.print(" bytes | ");
    Serial.print(count);
    Serial.print('/');
    Serial.println(total);
  }
}

bool CellUDPTime::networkTime(uint32_t& utcSec)
{
  // +CCLK: "yy/MM/dd,hh:mm:ss+zz" — zz is the local zone's offset from GMT in
  // quarter-hours, so subtracting it yields true UTC. A modem that never saw
  // NITZ answers with its 1980 power-on default; timeAnchorSet() floors that.
  if (!sendCommand("AT+CCLK?\r")) return false;
  char* p = strstr(getBuffer(), "+CCLK:");
  if (!p) return false;
  p = strchr(p, '"');
  if (!p) return false;

  int yy, mo, dd, hh, mi, ss, tz = 0;
  char sign = '+';
  if (sscanf(p + 1, "%d/%d/%d,%d:%d:%d%c%d", &yy, &mo, &dd, &hh, &mi, &ss, &sign, &tz) < 6)
    return false;

  int32_t offset = (int32_t)tz * 15 * 60;
  if (sign == '-') offset = -offset;
  utcSec = (uint32_t)((int32_t)timeCivilToEpoch(2000 + yy, mo, dd, hh, mi, ss) - offset);
  return true;
}

bool TeleClientUDP::verifyChecksum(char* data)
{
  uint8_t sum = 0;
  char *s = strrchr(data, '*');
  if (!s) return false;
  for (char *p = data; p < s; p++) sum += *p;
  if (hex2uint8(s + 1) == sum) {
    *s = 0;
    return true;
  }
  return false;
}

bool TeleClientUDP::notify(byte event, const char* payload)
{
  char buf[48];
  char cache[128];
  CStorageRAM netbuf;
  netbuf.init(cache, 128);
  netbuf.header(devid);
  netbuf.dispatch(buf, sprintf(buf, "EV=%X", (unsigned int)event));
  netbuf.dispatch(buf, sprintf(buf, "TS=%lu", millis()));
  netbuf.dispatch(buf, sprintf(buf, "ID=%s", devid));
  if (rssi) {
    netbuf.dispatch(buf, sprintf(buf, "SSI=%d", (int)rssi));
  }
  if (vin[0]) {
    netbuf.dispatch(buf, sprintf(buf, "VIN=%s", vin));
  }
  if (payload) {
    netbuf.dispatch(payload, strlen(payload));
  }
  netbuf.tailer();
  for (byte attempts = 0; attempts < 3; attempts++) {
    if (!cell.send(netbuf.buffer(), netbuf.length())) break;
    if (event == EVENT_ACK) return true;
    int bytesRecv = 0;
    char *data = cell.receive(&bytesRecv);
    if (!data || bytesRecv == 0) {
      Serial.println("[UDP] Timeout");
      continue;
    }
    rxBytes += bytesRecv;
    if (!verifyChecksum(data)) {
      Serial.print("[UDP] Checksum mismatch:");
      Serial.println(data);
      continue;
    }
    char pattern[16];
    sprintf(pattern, "EV=%u", event);
    if (!strstr(data, pattern)) {
      Serial.print("[UDP] Invalid reply: ");
      Serial.println(data);
      continue;
    }
    if (event == EVENT_LOGIN) {
      char *p = strstr(data, "TM=");
      if (p) {
        // The server's own clock, one round-trip old. Coarser than GNSS but it
        // arrives the moment the link comes up, which on a cold start is well
        // before the first fix.
        timeAnchorSet(TIME_SRC_NET, (uint32_t)atol(p + 3), timeTicks());
      }
      p = strstr(data, "SN=");
      if (p) {
        char *q = strchr(p, ',');
        if (q) *q = 0;
      }
      feedid = hex2uint16(data);
      login = true;
    } else if (event == EVENT_LOGOUT) {
      login = false;
    }
    return true;
  }
  return false;
}

bool TeleClientUDP::connect(bool quick)
{
  byte event = login ? EVENT_RECONNECT : EVENT_LOGIN;
  bool success = false;

  cell.close();
  if (quick) {
    return cell.open(0, 0);
  }

  packets = 0;

  for (byte attempts = 0; attempts < 3; attempts++) {
    Serial.print(event == EVENT_LOGIN ? "LOGIN(" : "RECONNECT(");
    Serial.print(SERVER_HOST);
    Serial.print(':');
    Serial.print(SERVER_PORT);
    Serial.println(")...");
    if (!cell.open(SERVER_HOST, SERVER_PORT)) {
      if (!cell.check()) break;
      Serial.println("[NET] Unable to connect");
      delay(3000);
      continue;
    }
    if (!notify(event)) {
      if (!cell.check()) break;
      cell.close();
      Serial.println("[NET] Server timeout");
      continue;
    }
    success = true;
    break;
  }
  if (event == EVENT_LOGIN) startTime = millis();
  if (success) {
    lastSyncTime = millis();
  }
  return success;
}

bool TeleClientUDP::ping()
{
  bool success = false;
  for (byte n = 0; n < 3 && !success; n++) {
    success = cell.open(SERVER_HOST, SERVER_PORT);
    if (success) {
      if ((success = notify(EVENT_PING))) break;
      cell.close();
      delay(1000);
    }
  }
  if (success) lastSyncTime = millis();
  return success;
}

bool TeleClientUDP::transmit(const char* packetBuffer, unsigned int packetSize)
{
  if (++packets >= 64) {
    cell.close();
    cell.open(0, 0);
    packets = 0;
  }
  Serial.print("[CELL] ");
  Serial.print(packetSize);
  Serial.println(" bytes being sent");
  if (cell.send(packetBuffer, packetSize)) {
    txBytes += packetSize;
    txCount++;
    return true;
  }
  return false;
}

void TeleClientUDP::inbound()
{
  do {
    int len = 0;
    char *data = cell.receive(&len, 50);
    if (!data || len == 0) break;
    data[len] = 0;
    Serial.print("[UDP] ");
    Serial.println(data);
    rxBytes += len;
    if (!verifyChecksum(data)) {
      Serial.print("[UDP] Checksum mismatch:");
      Serial.println(data);
      break;
    }
    char *p = strstr(data, "EV=");
    if (!p) break;
    int eventID = atoi(p + 3);
    switch (eventID) {
    case EVENT_SYNC:
        feedid = hex2uint16(data);
        Serial.print("[UDP] FEED ID:");
        Serial.println(feedid);
        break;
    }
    lastSyncTime = millis();
  } while(0);
}

void TeleClientUDP::shutdown()
{
  if (login) {
    notify(EVENT_LOGOUT);
    login = false;
    Serial.println("[NET] Logout");
  }
  cell.end();
  Serial.println("[CELL] Deactivated");
}
