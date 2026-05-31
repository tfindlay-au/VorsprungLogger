/*************************************************************************
* UDS (ISO 14229) ReadDataByIdentifier client — see FreematicsUDS.h.
*************************************************************************/

#include <Arduino.h>
#include <ctype.h>
#include <stdlib.h>
#include "FreematicsUDS.h"

void CUDS::begin(COBD* obd)
{
	m_obd = obd;
	applyHeaderConfig();
}

void CUDS::applyHeaderConfig()
{
	if (!m_obd) return;
	m_obd->setHeaderMask(UDS_RX_WIDE_MASK);
	m_obd->setHeaderFilter(UDS_RX_WIDE_FILTER);
	m_obd->setCANID(UDS_REQ_ENGINE);
}

bool CUDS::readDID(uint16_t did, uint8_t* payload, uint8_t& payloadLen, uint16_t reqId)
{
	payloadLen = 0;
	m_lastNRC = 0;
	if (!m_obd) return false;

	// Re-apply per request: OBD's readPID flow uses its own header (ATSH)
	// settings, so the co-processor's source header and receive filter may
	// have drifted between UDS reads. The request header is set per DID so
	// the right ECU (engine 0x7E0 / SCR 0x7EA) is addressed.
	m_obd->setHeaderMask(UDS_RX_WIDE_MASK);
	m_obd->setHeaderFilter(UDS_RX_WIDE_FILTER);
	m_obd->setCANID(reqId);

	byte msg[3];
	msg[0] = 0x22;
	msg[1] = (byte)(did >> 8);
	msg[2] = (byte)(did & 0xFF);

	char buf[128];
	memset(buf, 0, sizeof(buf));
	int n = m_obd->sendCANMessage(msg, sizeof(msg), buf, sizeof(buf));
	if (n <= 0) return false;

	uint8_t uds[32];
	int udsLen = udsParseAsciiHex(buf, n, uds, sizeof(uds));
	if (udsLen <= 0) return false;

	if (uds[0] == 0x7F) {
		// Negative response: 7F 22 NRC
		if (udsLen >= 3) m_lastNRC = uds[2];
		return false;
	}

	if (uds[0] != 0x62 || udsLen < 3) return false;

	uint16_t didEcho = ((uint16_t)uds[1] << 8) | uds[2];
	if (didEcho != did) return false;

	int dataLen = udsLen - 3;
	if (dataLen <= 0) return false;
	if (dataLen > UDS_MAX_PAYLOAD) dataLen = UDS_MAX_PAYLOAD;
	memcpy(payload, &uds[3], dataLen);
	payloadLen = (uint8_t)dataLen;
	return true;
}

int udsParseAsciiHex(const char* buf, int buflen, uint8_t* out, int outsize)
{
	uint8_t collected[32];
	int ncol = 0;
	int i = 0;

	while (i < buflen && ncol < (int)sizeof(collected)) {
		while (i < buflen && !isxdigit((unsigned char)buf[i])) i++;
		if (i >= buflen) break;
		int start = i;
		while (i < buflen && isxdigit((unsigned char)buf[i])) i++;
		int runlen = i - start;
		// Only 2-digit tokens are data bytes. 3+ digits are the CAN ID
		// prefix (e.g. "7E8"); skip them.
		if (runlen == 2) {
			char hex[3] = { buf[start], buf[start + 1], 0 };
			collected[ncol++] = (uint8_t)strtol(hex, NULL, 16);
		}
	}

	int svcIdx = -1;
	for (int k = 0; k < ncol; k++) {
		if (collected[k] == 0x62 || collected[k] == 0x7F) {
			svcIdx = k;
			break;
		}
	}
	if (svcIdx < 0) return 0;

	int n = 0;
	for (int k = svcIdx; k < ncol && n < outsize; k++) {
		out[n++] = collected[k];
	}
	return n;
}
