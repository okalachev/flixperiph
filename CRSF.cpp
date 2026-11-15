#include "CRSF.h"

#ifdef ESP32 // no support for other platforms yet

void CRSF::begin(int rxPin, int txPin, bool inv, int baud) {
	this->rxPin = rxPin;
	this->txPin = txPin;
	this->baud = baud;
	uart->begin(baud, SERIAL_8N1, this->rxPin, this->txPin, inv);
	uart->flush();
	resetParser();
}

bool CRSF::read() {
	hasNewChannelsFlag = false;
	while (uart->available()) {
		const uint8_t b = static_cast<uint8_t>(uart->read());
		if (parseByte(b)) {
			hasNewChannelsFlag = true;
		}
	}
	return hasNewChannelsFlag;
}

void CRSF::getChannels(uint16_t (&ch)[16]) const {
	for (uint8_t i = 0; i < CRSFData::numCh; ++i) {
		ch[i] = crsfData.channels[i];
	}
}

uint32_t CRSF::channelsAgeMs() const {
	if (!crsfData.channelsValid) return 0xFFFFFFFFu;
	return millis() - crsfData.channelsUpdatedMs;
}

bool CRSF::linkUp(uint32_t timeoutMs) const {
	if (!crsfData.channelsValid) return false;
	return channelsAgeMs() <= timeoutMs;
}

bool CRSF::parseByte(uint8_t b) {
	const uint32_t nowUs = micros();
	if (lastByteUs != 0 && (nowUs - lastByteUs) > 3000) {
		resetParser();
	}
	lastByteUs = nowUs;

	switch (state) {
		case 0: {
			frame[0] = b;
			state = 1;
			return false;
		}
		case 1: {
			if (b < minLen || b > maxLen) {
				++parseErrorCount;
				resetParser();
				return false;
			}
			expectedLen = b;
			frame[1] = b;
			frameIndex = 2;
			state = 2;
			return false;
		}
		case 2: {
			if (frameIndex >= maxFrameLen) {
				++parseErrorCount;
				resetParser();
				return false;
			}
			frame[frameIndex++] = b;
			if (frameIndex >= static_cast<uint8_t>(headerLen + expectedLen)) {
				const bool parsed = parseFrame();
				resetParser();
				return parsed;
			}
			return false;
		}
		default:
			resetParser();
			return false;
	}
}

bool CRSF::parseFrame() {
	/* frame: [addr][len][type][payload...][crc] */
	const uint8_t len = frame[1];
	if (len < minLen) {
		++parseErrorCount;
		return false;
	}

	const uint8_t type = frame[2];
	const uint8_t payloadLen = static_cast<uint8_t>(len - 2); /* type + payload + crc */
	const uint8_t *payload = &frame[3];
	const uint8_t crcRx = frame[headerLen + len - 1];
	const uint8_t crcCalc = crc8DvbS2(&frame[2], static_cast<uint8_t>(len - 1));

	if (crcRx != crcCalc) {
		++crcErrorCount;
		return false;
	}

	++totalFramesCount;
	if (type == typeRcChannelsPacked) {
		return parseRcChannels(payload, payloadLen);
	}
	if (type == typeLinkStats) {
		return parseLinkStats(payload, payloadLen);
	}
	return false;
}

bool CRSF::parseRcChannels(const uint8_t *payload, uint8_t payloadLen) {
	if (payloadLen != 22) {
		++parseErrorCount;
		return false;
	}

	uint32_t bitPos = 0;
	for (uint8_t ch = 0; ch < CRSFData::numCh; ++ch) {
		const uint32_t bytePos = bitPos >> 3;
		const uint8_t shift = static_cast<uint8_t>(bitPos & 0x07);
		uint32_t value = payload[bytePos];
		value |= static_cast<uint32_t>(payload[bytePos + 1]) << 8;
		value |= static_cast<uint32_t>(payload[bytePos + 2]) << 16;
		crsfData.channels[ch] = static_cast<uint16_t>((value >> shift) & 0x07FFu);
		bitPos += 11;
	}

	crsfData.channelsValid = true;
	crsfData.channelsUpdatedMs = millis();
	if (crsfData.failsafeCounter > 0) {
		crsfData.failsafeCounter--;
	}
	return true;
}

bool CRSF::parseLinkStats(const uint8_t *payload, uint8_t payloadLen) {
	if (payloadLen < 10) {
		++parseErrorCount;
		return false;
	}

	crsfData.link.valid = true;
	crsfData.link.uplinkRssi1 = payload[0];
	crsfData.link.uplinkRssi2 = payload[1];
	crsfData.link.uplinkLinkQuality = payload[2];
	crsfData.link.uplinkSnr = static_cast<int8_t>(payload[3]);
	crsfData.link.activeAntenna = payload[4];
	crsfData.link.rfMode = payload[5];
	crsfData.link.uplinkTxPower = payload[6];
	crsfData.link.downlinkRssi = payload[7];
	crsfData.link.downlinkLinkQuality = payload[8];
	crsfData.link.downlinkSnr = static_cast<int8_t>(payload[9]);

	if (crsfData.link.uplinkLinkQuality == 0) {
		if (crsfData.failsafeCounter < 0xFFFFu) {
			crsfData.failsafeCounter++;
		}
	}
	return false;
}

uint8_t CRSF::crc8DvbS2(const uint8_t *data, uint8_t len) {
	uint8_t crc = 0;
	for (uint8_t i = 0; i < len; ++i) {
		crc ^= data[i];
		for (uint8_t b = 0; b < 8; ++b) {
			if (crc & 0x80) {
				crc = static_cast<uint8_t>((crc << 1) ^ crcPoly);
			} else {
				crc = static_cast<uint8_t>(crc << 1);
			}
		}
	}
	return crc;
}

void CRSF::resetParser() {
	state = 0;
	expectedLen = 0;
	frameIndex = 0;
}

#endif
