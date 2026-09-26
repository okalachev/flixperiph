/*
 * WS2812 LED driver for Arduino using ESP32 RMT peripheral.
 * Repository: https://github.com/okalachev/flixperiph
 */

#pragma once

#ifdef ESP32 // no support on other platforms yet

#include <Arduino.h>
#include "esp32-hal-rmt.h"

struct RGB {
	uint8_t r, g, b;
};

class WS2812 {
public:
	~WS2812() {
		end();
	}

	bool begin(int pin, size_t count) {
		if (initialized_) end(); // initialize again

		pin_ = pin;
		count_ = count;
		symbolCount_ = count_ * 24 + 1;

		symbols_ = static_cast<rmt_data_t*>(malloc(symbolCount_ * sizeof(rmt_data_t)));

		last_ = static_cast<RGB*>(malloc(count_ * sizeof(RGB)));

		if (!symbols_ || !last_) {
			free(symbols_);
			free(last_);

			symbols_ = nullptr;
			last_ = nullptr;

			return false;
		}

		// 10 MHz: 1 tick = 100 ns
		if (!rmtInit(pin_, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 10'000'000)) {
			free(symbols_);
			free(last_);
			symbols_ = nullptr;
			last_ = nullptr;
			return false;
		}

		initialized_ = true;
		return true;
	}

	void end() {
		if (initialized_) {
			rmtDeinit(pin_);
			initialized_ = false;
		}

		free(symbols_);
		free(last_);

		symbols_ = nullptr;
		last_ = nullptr;

		count_ = 0;
		symbolCount_ = 0;

		transmitting_ = false;
		lastValid_ = false;
	}


	bool write(const RGB* colors) {
		if (!initialized_ || !colors || !count_) return false;

		if (lastValid_ && memcmp(colors, last_, count_ * sizeof(RGB)) == 0) { // nothing changed
			return true;
		}

		if (transmitting_) {
			if (!rmtTransmitCompleted(pin_))
				return false;

			transmitting_ = false;
		}

		encode(colors);

		if (!rmtWriteAsync(pin_, symbols_, symbolCount_))
			return false;

		memcpy(last_, colors, count_ * sizeof(RGB));

		lastValid_ = true;
		transmitting_ = true;

		return true;
	}

	void setBrightness(float brightness) {
		brightness_ = brightness;
	}

private:
	static void encodeByte(rmt_data_t*& out, uint8_t value) {
		const rmt_data_t bit0 = { 4, 1, 8, 0 };
		const rmt_data_t bit1 = { 8, 1, 4, 0 };

		for (uint8_t mask = 0x80; mask; mask >>= 1)
			*out++ = (value & mask) ? bit1 : bit0;
	}

	void encode(const RGB* colors) {
		rmt_data_t* out = symbols_;

		for (size_t i = 0; i < count_; ++i) {
			// WS2812 — GRB
			encodeByte(out, colors[i].g * brightness_);
			encodeByte(out, colors[i].r * brightness_);
			encodeByte(out, colors[i].b * brightness_);
		}

		// Reset/latch: 300 us LOW
		*out = { 3000, 0, 1, 0 };
	}

	int pin_;
	size_t count_;
	float brightness_ = 1;

	rmt_data_t* symbols_ = nullptr;
	RGB* last_ = nullptr;

	size_t symbolCount_ = 0;

	bool initialized_ = false;
	bool transmitting_ = false;
	bool lastValid_ = false;
};

#endif
