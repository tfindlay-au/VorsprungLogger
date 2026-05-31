/*************************************************************************
* UDS (ISO 14229) ReadDataByIdentifier client for Freematics ONE+ Model B.
* Built on top of COBD's raw CAN access (sendCANMessage / setHeader*).
*
* Scope (Phase 4 + Phase 6 of the UDS project — see PROJECT_CONTEXT.md):
*   - Single-frame ISO-TP only (responses arrive in one frame; DLC up to 07)
*   - Default session only (no DiagnosticSessionControl, no TesterPresent)
*   - 11-bit CAN IDs only; multiple ECUs addressed per request:
*       engine ECU    request 0x7E0 / response 0x7E8
*       SCR/reductant request 0x7EA / response 0x794
*     A wide receive filter (0xFFFFF800 / 0x7E0) catches both responses;
*     the request ID is selected per DID by the caller.
*   - Read-only (service 0x22 ReadDataByIdentifier)
*
* The co-processor handles ISO-TP framing on outbound single frames: we pass
* only the raw UDS service+data bytes ({0x22, HI, LO}) and it prepends the
* single-frame length byte itself. Responses come back as ASCII hex text
* (e.g. "62 F4 05 64 \r\r"); the parser pulls them into bytes.
*
* These were validated live on a 2017 Audi A6 Allroad (ECU 4G2 907 311 B)
* in the Phase 3 uds_active_listener.ino sketch.
*************************************************************************/

#ifndef FREEMATICS_UDS_H
#define FREEMATICS_UDS_H

#include <stdint.h>
#include "FreematicsBase.h"
#include "FreematicsOBD.h"

#define UDS_REQ_ENGINE 0x7E0            // engine ECU request ID (default)
// Wide receive filter: with this mask every 11-bit response ID matches, so
// both the engine (0x7E8) and SCR module (0x794) replies are accepted.
#define UDS_RX_WIDE_FILTER 0x7E0
#define UDS_RX_WIDE_MASK   0xFFFFF800
#define UDS_MAX_PAYLOAD 16

class CUDS
{
public:
	void begin(COBD* obd);

	// Send a ReadDataByIdentifier request to ECU `reqId` and parse the
	// response. On success returns true; payload points at the data bytes
	// that follow the {0x62, HI, LO} echo, and payloadLen is their count.
	// On any failure (timeout, negative response, parse mismatch) returns
	// false and sets lastNRC() to the NRC byte if it was a negative
	// response, or 0 otherwise.
	bool readDID(uint16_t did, uint8_t* payload, uint8_t& payloadLen, uint16_t reqId = UDS_REQ_ENGINE);

	uint8_t lastNRC() const { return m_lastNRC; }

	// Re-apply the receive filter + a default request header to the
	// co-processor. Call this after obd.init() resets the ELM327, since
	// init() clears the header and filter state.
	void applyHeaderConfig();

private:
	COBD* m_obd = nullptr;
	uint8_t m_lastNRC = 0;
};

// Extract UDS response bytes from the co-processor's ASCII-hex response
// buffer. Returns the number of bytes copied into `out`. The first byte
// (out[0]) is the UDS service byte: 0x62 for positive, 0x7F for negative.
// Returns 0 if no recognisable service byte is present.
int udsParseAsciiHex(const char* buf, int buflen, uint8_t* out, int outsize);

#endif
