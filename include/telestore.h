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
    // Adopt an already-encoded packet so it can be re-opened with untailer(),
    // extended and re-checksummed. Used by the spool drain to back-stamp a
    // record that was written before a time anchor existed.
    void load(const char* data, unsigned int len);
protected:
    unsigned int m_cacheSize = 0;
    unsigned int m_cacheBytes = 0;
    char* m_cache = 0;
};

// Forward declaration to avoid pulling teleclient.h into the header.
class TeleClientUDP;

// Marker byte introducing a record that still needs back-stamping. A stamped
// record starts with the device ID, which is always alphanumeric, so old
// spool files written by earlier firmware read back unambiguously.
#define SPOOL_MARK_UNSTAMPED 0x00
// Marker + boot ID (4) + capture tick (8), all big-endian.
#define SPOOL_STAMP_PREFIX 13

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
// Records built before this session had a UTC anchor carry no timestamp of
// their own, so they go in via appendUnstamped() with their raw capture tick
// and the boot ID that tick belongs to; drainOneRecord() resolves the tick
// against the anchor and splices the time PIDs in on the way out.
//
// Files-in-/SPOOL/ are the only state — no NVS, no high-water mark. A
// crashed mid-drain leaves the file in place so the next attempt replays
// it from byte zero. Some records arrive at the server twice; Traccar
// accepts the duplicates (no unique constraint on (deviceid, devicetime)).
class RecordSpool {
public:
    bool init();
    bool append(const char* data, uint16_t len);
    bool appendUnstamped(const char* data, uint16_t len, uint64_t tickUs);
    void endOutage();          // close active outage file (drain picks it up)
    void closeForStandby();    // flush+close everything before reboot
    bool hasFilesToDrain();    // any spool files awaiting drain
    // `store` is scratch space for back-stamping; it is left purged.
    bool drainOneRecord(TeleClientUDP& tc, CStorageRAM& store);

private:
    bool writeRecord(const uint8_t* prefix, uint8_t prefixLen, const char* data, uint16_t len);
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
