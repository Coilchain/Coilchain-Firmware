/**
 * @copyright Copyright (c) 2021 Coilchain Development Team
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions
 * and limitations under the License.
 *
 * @file VescDriverUart.hpp
 * @brief Driver to communicate over UART with VESC motor contollers
 * @details More about VESC BLDC (brushless direct current) electric motor controllers by Benjamin Vedder on https://vesc-project.com/
 * @author Matthias Grob <maetugr@gmail.com>
 */

#pragma once

#include <stdint.h>
#include <stdio.h>
#include "VescProtocol.h"

class VescDriver {
public:
	VescDriver(FILE *device) : _device(device) {};
	~VescDriver() { fclose(_device); };

	void commandCurrent(float current);
	void commandBrakeCurrent(float current);

	void requestFirmwareVersion();
	void requestValues();
	float getRpm() { return _vesc_values.rpm; };
	float getInputVoltage() { return _vesc_values.input_voltage; };
	int32_t getTachometer() { return _vesc_values.tachometer; }
	float getInputCurrent() { return _vesc_values.input_current; }

	void parseInputByte(uint8_t byte); ///< call when data is ready to read

private:
	// de-/serialize packets
	size_t sendPacket(const uint8_t *payload, const uint16_t payload_length);
	void parsePayload(const uint8_t *payload, const uint16_t payload_length);
	uint16_t crc16(const uint8_t *buffer, const uint16_t length);

	// big-endian helpers
	void insertInt32(uint8_t *buffer, uint16_t &index, int32_t value);
	int16_t extractInt16(const uint8_t *buffer, uint16_t &index);
	float extractFloat16(const uint8_t *buffer, uint16_t &index);
	int32_t extractInt32(const uint8_t *buffer, uint16_t &index);
	float extractFloat32(const uint8_t *buffer, uint16_t &index);

	// byte stream access through _device
	size_t write(const uint8_t *buffer, const uint16_t length);

	FILE *_device;

	// input packet parsing
	size_t _input_byte_index{0}; ///< keeps track of the input packets parsing state
	uint8_t _input_start_byte{0};
	uint16_t _input_payload_length{0};
	uint8_t _input_payload[MAXIMUM_PAYLOAD_LENGTH];
	uint16_t _input_payload_crc{0};

	// information storage for getters
	VescVersion _vesc_version{};
	VescValues _vesc_values{};
};
