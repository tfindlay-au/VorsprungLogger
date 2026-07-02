#ifndef TELESTORE_H_INCLUDED
#define TELESTORE_H_INCLUDED

#include <SPI.h>
#include <FS.h>
#include <SD.h>

class CStorage {
public:
    virtual bool init() { return true; }
    virtual void uninit() {}
    virtual void log(uint16_t pid, uint8_t values[], uint8_t count);
    virtual void log(uint16_t pid, uint16_t values[], uint8_t count);
    virtual void log(uint16_t pid, uint32_t values[], uint8_t count);
    virtual void log(uint16_t pid, int32_t values[], uint8_t count);
    virtual void log(uint16_t pid, float values[], uint8_t count, const char* fmt = "%f");
    virtual void timestamp(uint32_t ts);
    virtual void purge() { m_samples = 0; }
    virtual uint16_t samples() { return m_samples; }
    virtual void dispatch(const char* buf, byte len);
protected:
    byte checksum(const char* data, int len);
    virtual void header(const char* devid) {}
    virtual void tailer() {}
    int m_samples = 0;
    char m_delimiter = ':';
};

class CStorageRAM: public CStorage {
public:
    void init(char* cache, unsigned int cacheSize)
    {
        m_cacheSize = cacheSize;
        m_cache = cache;
    }
    void uninit()
    {
        if (m_cache) {
            delete m_cache;
            m_cache = 0;
            m_cacheSize = 0;
        }
    }
    void purge() { m_cacheBytes = 0; m_samples = 0; }
    unsigned int length() { return m_cacheBytes; }
    char* buffer() { return m_cache; }
    void dispatch(const char* buf, byte len);
    void header(const char* devid);
    void tailer();
    void untailer();
protected:
    unsigned int m_cacheSize = 0;
    unsigned int m_cacheBytes = 0;
    char* m_cache = 0;
};

// Forward declaration to avoid pulling teleclient.h into the header.
class TeleClientUDP;

// Per-outage spool of unsent UDP packets on the SD card.
//
// On a failed transmit the telemetry task hands the encoded packet to
// append(): records are length-prefixed binary blobs (2-byte big-endian
// length, then the exact bytes that would have gone on the wire) and live
// in /SPOOL/<seq>.PKT, one file per outage. endOutage() closes the active
// file on the first successful transmit afterwards; drainOneRecord() then
// walks the oldest file forward, re-sending records throttled by the
// caller until the file is empty and unlinks it.
//
// Files-in-/SPOOL/ are the only state — no NVS, no high-water mark. A
// crashed mid-drain leaves the file in place so the next attempt replays
// it from byte zero. Some records arrive at the server twice; Traccar
// accepts the duplicates (no unique constraint on (deviceid, devicetime)).
class RecordSpool {
public:
    bool init();
    bool append(const char* data, uint16_t len);
    void endOutage();          // close active outage file (drain picks it up)
    void closeForStandby();    // flush+close everything before reboot
    bool hasFilesToDrain();    // any spool files awaiting drain
    bool drainOneRecord(TeleClientUDP& tc);

private:
    bool openOutageFile();
    bool openNextDrainFile();
    int  findOldestSeq(uint32_t excludeSeq);
    void dropDrainFile(const char* reason);

    File m_outage;
    File m_drain;
    uint32_t m_outageSeq = 0;
    uint32_t m_drainSeq = 0;
    uint32_t m_nextSeq = 1;
    uint32_t m_appendCount = 0;
};

#endif // TELESTORE_H_INCLUDED
