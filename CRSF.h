/*
 * CRSF (Crossfire) receiver parser for Arduino / ESP32.
 *
 * The class reads incoming CRSF frames from a HardwareSerial instance,
 * decodes RC channels, and tracks basic link status.
 */

#pragma once

#include <Arduino.h>

struct CRSFLinkStats {
	bool valid = false;
	uint8_t uplinkRssi1 = 0;
	uint8_t uplinkRssi2 = 0;
	uint8_t uplinkLinkQuality = 0;
	int8_t uplinkSnr = 0;
	uint8_t activeAntenna = 0;
	uint8_t rfMode = 0;
	uint8_t uplinkTxPower = 0;
	uint8_t downlinkRssi = 0;
	uint8_t downlinkLinkQuality = 0;
	int8_t downlinkSnr = 0;
};

struct CRSFData {
	static constexpr uint8_t numCh = 16;
	uint16_t channels[numCh] = {0};
	bool channelsValid = false;
	uint32_t channelsUpdatedMs = 0;
	uint16_t failsafeCounter = 0;
	CRSFLinkStats link;
};

class CRSF {
 public:
	CRSF(HardwareSerial& bus, const int rxpin, const int txpin)
		: uart(&bus), rxPin(rxpin), txPin(txpin) {}
	explicit CRSF(HardwareSerial& bus) : uart(&bus) {}

	void begin(int rxpin = -1, int txpin = -1, bool inv = true, int baud = 400000);

	/* Reads all pending bytes from UART and parses frames. */
	bool read();

	inline CRSFData data() const { return crsfData; }
	inline CRSFLinkStats linkStats() const { return crsfData.link; }
	inline bool hasNewChannels() const { return hasNewChannelsFlag; }

	inline uint16_t channel(uint8_t index) const {
		return (index < CRSFData::numCh) ? crsfData.channels[index] : 0;
	}

	/* Copies all 16 channels to user buffer (1000-2000 us typical). */
	void getChannels(uint16_t (&ch)[16]) const;

	/* Last successfully parsed RC frame age. */
	uint32_t channelsAgeMs() const;

	/* Returns true when channel data is considered fresh. */
	bool linkUp(uint32_t timeoutMs = 200) const;

	inline uint32_t totalFrames() const { return totalFramesCount; }
	inline uint32_t crcErrors() const { return crcErrorCount; }
	inline uint32_t parseErrors() const { return parseErrorCount; }

 private:
	HardwareSerial *uart = nullptr;
	int8_t rxPin = -1;
	int8_t txPin = -1;
	int32_t baud = 420000;

	static constexpr uint8_t maxFrameLen = 64;
	static constexpr uint8_t headerLen = 2;
	static constexpr uint8_t crcPoly = 0xD5;

	static constexpr uint8_t typeLinkStats = 0x14;
	static constexpr uint8_t typeRcChannelsPacked = 0x16;

	/* CRSF length byte = type + payload + crc, min = 2, max = 62. */
	static constexpr uint8_t minLen = 2;
	static constexpr uint8_t maxLen = 62;

	uint8_t frame[maxFrameLen] = {0};
	uint8_t state = 0;
	uint8_t expectedLen = 0;
	uint8_t frameIndex = 0;
	uint32_t lastByteUs = 0;

	uint32_t totalFramesCount = 0;
	uint32_t crcErrorCount = 0;
	uint32_t parseErrorCount = 0;

	bool hasNewChannelsFlag = false;
	CRSFData crsfData;

	bool parseByte(uint8_t b);
	bool parseFrame();
	bool parseRcChannels(const uint8_t *payload, uint8_t payloadLen);
	bool parseLinkStats(const uint8_t *payload, uint8_t payloadLen);

	static uint8_t crc8DvbS2(const uint8_t *data, uint8_t len);
	void resetParser();
};

