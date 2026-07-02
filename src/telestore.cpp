#include <FreematicsPlus.h>
#include "config.h"
#include "telestore.h"
#include "teleclient.h"

void CStorage::log(uint16_t pid, uint8_t values[], uint8_t count)
{
    char buf[256];
    byte n = snprintf(buf, sizeof(buf), "%X%c%u", pid, m_delimiter, (unsigned int)values[0]);
    for (byte m = 1; m < count; m++) {
        n += snprintf(buf + n, sizeof(buf) - n, ";%u", (unsigned int)values[m]);
    }
    dispatch(buf, n);
}

void CStorage::log(uint16_t pid, uint16_t values[], uint8_t count)
{
    char buf[256];
    byte n = snprintf(buf, sizeof(buf), "%X%c%u", pid, m_delimiter, (unsigned int)values[0]);
    for (byte m = 1; m < count; m++) {
        n += snprintf(buf + n, sizeof(buf) - n, ";%u", (unsigned int)values[m]);
    }
    dispatch(buf, n);
}

void CStorage::log(uint16_t pid, uint32_t values[], uint8_t count)
{
    char buf[256];
    byte n = snprintf(buf, sizeof(buf), "%X%c%u", pid, m_delimiter, values[0]);
    for (byte m = 1; m < count; m++) {
        n += snprintf(buf + n, sizeof(buf) - n, ";%u", values[m]);
    }
    dispatch(buf, n);
}

void CStorage::log(uint16_t pid, int32_t values[], uint8_t count)
{
    char buf[256];
    byte n = snprintf(buf, sizeof(buf), "%X%c%d", pid, m_delimiter, values[0]);
    for (byte m = 1; m < count; m++) {
        n += snprintf(buf + n, sizeof(buf) - n, ";%d", values[m]);
    }
    dispatch(buf, n);
}

void CStorage::log(uint16_t pid, float values[], uint8_t count, const char* fmt)
{
    char buf[256];
    char *p = buf + snprintf(buf, sizeof(buf), "%X%c", pid, m_delimiter);
    for (byte m = 0; m < count && (p - buf) < sizeof(buf) - 3; m++) {
        if (m > 0) *(p++) = ';';
        int l = snprintf(p, sizeof(buf) - (p - buf), fmt, values[m]);
        char *q = strchr(p, '.');
        if (q && atoi(q + 1) == 0) {
            *q = 0;
            if (*p == '-' && *(p + 1) == '0') {
                *p = '0';
                *(++p) = 0;
            } else {
                p = q;
            }
        } else {
            p += l;
        }
    }
    dispatch(buf, (int)(p - buf));
}

void CStorage::timestamp(uint32_t ts)
{
    log(PID_TIMESTAMP, &ts, 1);
}

void CStorage::dispatch(const char* buf, byte len)
{
    // output data via serial
    Serial.write((uint8_t*)buf, len);
    Serial.write(' ');
    m_samples++;
}

byte CStorage::checksum(const char* data, int len)
{
    byte sum = 0;
    for (int i = 0; i < len; i++) sum += data[i];
    return sum;
}

void CStorageRAM::dispatch(const char* buf, byte len)
{
    // reserve some space for checksum
    int remain = m_cacheSize - m_cacheBytes - len - 3;
    if (remain < 0) {
        // m_cache full
        return;
    }
    // store data in m_cache
    memcpy(m_cache + m_cacheBytes, buf, len);
    m_cacheBytes += len;
    m_cache[m_cacheBytes++] = ',';
    m_samples++;
}

void CStorageRAM::header(const char* devid)
{
    m_cacheBytes = sprintf(m_cache, "%s#", devid);
}

void CStorageRAM::tailer()
{
    if (m_cache[m_cacheBytes - 1] == ',') m_cacheBytes--;
    m_cacheBytes += sprintf(m_cache + m_cacheBytes, "*%X", (unsigned int)checksum(m_cache, m_cacheBytes));
}

void CStorageRAM::untailer()
{
    char *p = strrchr(m_cache, '*');
    if (p) {
        *p = ',';
        m_cacheBytes = p + 1 - m_cache;
    }
}

/*******************************************************************************
  RecordSpool — per-outage SD spool of unsent UDP packets.
*******************************************************************************/

// Use a monotonic sequence number rather than a wall-clock timestamp for the
// filename, so the scheme is robust before time(NULL) is set (LOGIN sets it).
// Oldest-first drain = lowest sequence number first; PID_ABS_TIME riding in
// the packet payload carries the absolute capture time.
static inline void spoolPath(char* out, size_t cap, uint32_t seq)
{
    snprintf(out, cap, "/SPOOL/%u.PKT", (unsigned int)seq);
}

bool RecordSpool::init()
{
    SPI.begin();
    if (!SD.begin(PIN_SD_CS, SPI, SPI_FREQ)) {
        Serial.println("NO SD CARD");
        return false;
    }
    unsigned int total = SD.totalBytes() >> 20;
    unsigned int used  = SD.usedBytes()  >> 20;
    Serial.print("SD:");
    Serial.print(total);
    Serial.print(" MB total, ");
    Serial.print(used);
    Serial.println(" MB used");

    if (!SD.exists("/SPOOL")) SD.mkdir("/SPOOL");

    // Scan existing /SPOOL/*.PKT to choose the next sequence number. Pre-existing
    // files are orphans from a previous outage that did not get drained — leave
    // them in place so drainOneRecord() picks them up later.
    File root = SD.open("/SPOOL");
    uint32_t maxSeq = 0;
    int orphans = 0;
    if (root) {
        File f;
        while ((f = root.openNextFile())) {
            const char* name = strrchr(f.name(), '/');
            name = name ? name + 1 : f.name();
            unsigned int n = atoi(name);
            if (n > maxSeq) maxSeq = n;
            if (n > 0) orphans++;
            f.close();
        }
        root.close();
    }
    m_nextSeq = maxSeq + 1;
    if (orphans) {
        Serial.print("[SPOOL] ");
        Serial.print(orphans);
        Serial.println(" orphan file(s) pending drain");
    }
    return true;
}

bool RecordSpool::openOutageFile()
{
    if (m_outage) return true;
    char path[32];
    spoolPath(path, sizeof(path), m_nextSeq);
    m_outage = SD.open(path, FILE_WRITE);
    if (!m_outage) {
        Serial.print("[SPOOL] open fail: ");
        Serial.println(path);
        return false;
    }
    m_outageSeq = m_nextSeq++;
    m_appendCount = 0;
    Serial.print("[SPOOL] outage start ");
    Serial.println(path);
    return true;
}

bool RecordSpool::append(const char* data, uint16_t len)
{
    if (len == 0) return true;
    if (!m_outage && !openOutageFile()) return false;
    uint8_t hdr[2] = { (uint8_t)(len >> 8), (uint8_t)(len & 0xFF) };
    if (m_outage.write(hdr, 2) != 2) return false;
    if (m_outage.write((const uint8_t*)data, len) != (int)len) return false;
    m_appendCount++;
    // Periodic sync without close — bounds power-cut loss to ~30 records while
    // keeping the FAT directory entry honest for boot-time recovery.
    if ((m_appendCount % 30) == 0) m_outage.flush();
    return true;
}

void RecordSpool::endOutage()
{
    if (!m_outage) return;
    Serial.print("[SPOOL] outage end ");
    Serial.print(m_appendCount);
    Serial.println(" record(s)");
    m_outage.flush();
    m_outage.close();
    m_outageSeq = 0;
    m_appendCount = 0;
}

void RecordSpool::closeForStandby()
{
    if (m_outage) {
        m_outage.flush();
        m_outage.close();
        m_outageSeq = 0;
    }
    if (m_drain) {
        m_drain.close();
        m_drainSeq = 0;
    }
}

int RecordSpool::findOldestSeq(uint32_t excludeSeq)
{
    File root = SD.open("/SPOOL");
    if (!root) return -1;
    int oldest = -1;
    File f;
    while ((f = root.openNextFile())) {
        const char* name = strrchr(f.name(), '/');
        name = name ? name + 1 : f.name();
        unsigned int n = atoi(name);
        f.close();
        if (n == 0) continue;
        if (n == excludeSeq) continue;
        if (oldest < 0 || (int)n < oldest) oldest = (int)n;
    }
    root.close();
    return oldest;
}

bool RecordSpool::hasFilesToDrain()
{
    if (m_drain) return true;
    return findOldestSeq(m_outageSeq) >= 0;
}

bool RecordSpool::openNextDrainFile()
{
    if (m_drain) return true;
    int seq = findOldestSeq(m_outageSeq);
    if (seq < 0) return false;
    char path[32];
    spoolPath(path, sizeof(path), (uint32_t)seq);
    m_drain = SD.open(path, FILE_READ);
    if (!m_drain) {
        Serial.print("[SPOOL] drain open fail: ");
        Serial.println(path);
        return false;
    }
    m_drainSeq = (uint32_t)seq;
    Serial.print("[SPOOL] draining ");
    Serial.print(path);
    Serial.print(" (");
    Serial.print((unsigned int)m_drain.size());
    Serial.println(" bytes)");
    return true;
}

void RecordSpool::dropDrainFile(const char* reason)
{
    char path[32];
    spoolPath(path, sizeof(path), m_drainSeq);
    if (m_drain) m_drain.close();
    SD.remove(path);
    Serial.print("[SPOOL] ");
    Serial.print(reason);
    Serial.print(": ");
    Serial.println(path);
    m_drainSeq = 0;
}

bool RecordSpool::drainOneRecord(TeleClientUDP& tc)
{
    if (!m_drain && !openNextDrainFile()) return false;

    // Reached end of file → unlink, look for the next file on the next call.
    if (m_drain.position() >= m_drain.size()) {
        dropDrainFile("drained");
        return true;
    }

    uint32_t recordStart = m_drain.position();
    uint8_t hdr[2];
    if (m_drain.read(hdr, 2) != 2) {
        dropDrainFile("short header");
        return true;
    }
    uint16_t len = ((uint16_t)hdr[0] << 8) | hdr[1];
    if (len == 0 || len > SERIALIZE_BUFFER_SIZE) {
        dropDrainFile("bad length");
        return true;
    }

    char buf[SERIALIZE_BUFFER_SIZE];
    if ((uint16_t)m_drain.read((uint8_t*)buf, len) != len) {
        dropDrainFile("short payload");
        return true;
    }

    if (tc.transmit(buf, len)) {
        return true;
    }

    // Transmit failed — leave the record in place so the next drain pass
    // replays from here. Close the read handle; the cell is in a bad way and
    // a new outage spool file will open the moment live transmits start
    // failing too. Seek back so the position is preserved if we keep the
    // handle open later (defensive — we close right after).
    m_drain.seek(recordStart);
    m_drain.close();
    m_drainSeq = 0;
    return false;
}
