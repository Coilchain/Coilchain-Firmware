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
 * @file VescDriverUart.cpp
 * @author Matthias Grob <maetugr@gmail.com>
 */

#include "VescDriver.hpp"
#include <memory.h>

void VescDriver::commandCurrent(float current)
{
	uint8_t command[5]{VescCommand::SET_CURRENT};
	uint16_t index{1u};
	insertInt32(command, index, static_cast<int32_t>(current * 1000.f));
	sendPacket(command, 5u);
}

void VescDriver::commandBrakeCurrent(float current)
{
	uint8_t command[5]{VescCommand::SET_CURRENT_BRAKE};
	uint16_t index{1u};
	insertInt32(command, index, static_cast<int32_t>(current * 1000.f));
	sendPacket(command, 5u);
}

void VescDriver::requestFirmwareVersion()
{
	uint8_t command{VescCommand::FW_VERSION};
	sendPacket(&command, 1u);
}

void VescDriver::requestValues()
{
	uint8_t command{VescCommand::GET_VALUES};
	sendPacket(&command, 1u);
}

void VescDriver::requestRpm()
{
	uint8_t command[5]{VescCommand::GET_VALUES_SELECTIVE};
	uint16_t index{1};
	insertUInt32(command, index, 0x80);
	sendPacket(command, 5u);
}

size_t VescDriver::sendPacket(const uint8_t *payload, const uint16_t payload_length)
{
	if (payload_length == 0 || payload_length > MAXIMUM_PAYLOAD_LENGTH) {
		return 0;
	}

	uint8_t packet_buffer[payload_length + PACKET_OVERHEAD_LENGTH];
	uint16_t index{0};

	// Start byte and payload size
	if (payload_length <= 256) {
		packet_buffer[index++] = 2;
		packet_buffer[index++] = payload_length;
	} else {
		packet_buffer[index++] = 3;
		packet_buffer[index++] = static_cast<uint8_t>(payload_length >> 8);
		packet_buffer[index++] = static_cast<uint8_t>(payload_length & 0xFF);
	}

	// Payload
	memcpy(&packet_buffer[index], payload, payload_length);
	index += payload_length;

	// CRC
	const uint16_t crc = crc16(payload, payload_length);
	packet_buffer[index++] = static_cast<uint8_t>(crc >> 8);
	packet_buffer[index++] = static_cast<uint8_t>(crc & 0xFF);

	// Stop byte
	packet_buffer[index++] = 3;

	return write(packet_buffer, index);
}

void VescDriver::parseInputByte(uint8_t byte)
{
	if (_input_byte_index == 0) {
		// Start byte
		if (byte == 2 /*|| byte == 3*/) {
			_input_start_byte = byte;
			_input_byte_index++;
		}

	} else if (_input_byte_index == 1) {
		// Payload size
		_input_byte_index++;

		if (_input_start_byte == 2) {
			// Short packet
			_input_payload_length = byte;

			if (_input_payload_length < 1) {
				_input_byte_index = 0;
			}
		} else {
			// Long packet high byte
			_input_payload_length = byte << 8;
		}

	} else if (_input_byte_index == 2 && _input_start_byte == 3) {
		// Payload size long packet low byte
		_input_payload_length |= byte;
		_input_byte_index++;

		if (_input_payload_length < 255 || _input_payload_length > MAXIMUM_PAYLOAD_LENGTH) {
			_input_byte_index = 0;
		}

	} else if (_input_byte_index < _input_start_byte + _input_payload_length) {
		// Payload
		_input_payload[_input_byte_index - _input_start_byte] = byte;
		_input_byte_index++;

	} else if (_input_byte_index == _input_start_byte + _input_payload_length) {
		// CRC high byte
		_input_payload_crc = byte << 8;
		_input_byte_index++;

	} else if (_input_byte_index == _input_start_byte + _input_payload_length + 1u) {
		// CRC low byte
		_input_payload_crc |= byte;
		_input_byte_index++;

		if (_input_payload_crc != crc16(_input_payload, _input_payload_length)) {
			_input_byte_index = 0;
		}

	} else if (_input_byte_index == _input_start_byte + _input_payload_length + 2u) {
		// Stop byte
		_input_byte_index = 0;

		if (byte == 3) {
			parsePayload(_input_payload, _input_payload_length);
		}
	}
}

void VescDriver::parsePayload(const uint8_t *payload, const uint16_t payload_length)
{
	uint16_t index{1u};
	switch (payload[0]) {
	case VescCommand::FW_VERSION:
		if (payload_length >= 9u) {
			_vesc_version.version_major = payload[index++];
			_vesc_version.version_minor = payload[index++];
			// strcpy(_vesc_version.hardware_name, reinterpret_cast<const char *>(&payload[index]));
			// index += strlen(_vesc_version.hardware_name) + 1u;
			// memcpy(_vesc_version.stm32_uuid_8, &payload[index], sizeof(_vesc_version.stm32_uuid_8));
			// index += 12;
			// _vesc_version.pairing_done = payload[index++];
			// _vesc_version.test_version_number = payload[index++];
			// _vesc_version.hardware_type = payload[index++];
			// _vesc_version.custom_configuration = payload[index++];
		}
		break;
	case VescCommand::GET_VALUES:
	case VescCommand::GET_VALUES_SELECTIVE:
		uint32_t mask = 0xFFFFFFFF; // all values for GET_VALUES
		uint8_t mask_index{0u};
		if (payload[0] == VescCommand::GET_VALUES_SELECTIVE) {
			mask = extractUInt32(payload, index); // selected values for GET_VALUES_SELECTIVE
		}

		if (mask & static_cast<uint32_t>(1 << mask_index++)) _vesc_values.fet_temperature = extractFloat16(payload, index) / 10.f;
		if (mask & static_cast<uint32_t>(1 << mask_index++)) _vesc_values.motor_temperature = extractFloat16(payload, index) / 10.f;
		if (mask & static_cast<uint32_t>(1 << mask_index++)) _vesc_values.motor_current = extractFloat32(payload, index) / 100.f;
		if (mask & static_cast<uint32_t>(1 << mask_index++)) _vesc_values.input_current = extractFloat32(payload, index) / 100.f;
		if (mask & static_cast<uint32_t>(1 << mask_index++)) _vesc_values.reset_average_id = extractFloat32(payload, index) / 100.f;
		if (mask & static_cast<uint32_t>(1 << mask_index++)) _vesc_values.reset_average_iq = extractFloat32(payload, index) / 100.f;
		if (mask & static_cast<uint32_t>(1 << mask_index++)) _vesc_values.duty_cycle = extractFloat16(payload, index) / 1000.f;
		if (mask & static_cast<uint32_t>(1 << mask_index++)) _vesc_values.rpm = extractInt32(payload, index);
		if (mask & static_cast<uint32_t>(1 << mask_index++)) _vesc_values.input_voltage = extractFloat16(payload, index) / 10.f;
		if (mask & static_cast<uint32_t>(1 << mask_index++)) _vesc_values.used_charge_Ah = extractFloat32(payload, index) / 1e4f;
		if (mask & static_cast<uint32_t>(1 << mask_index++)) _vesc_values.charged_charge_Ah = extractFloat32(payload, index) / 1e4f;
		if (mask & static_cast<uint32_t>(1 << mask_index++)) _vesc_values.used_energy_Wh = extractFloat32(payload, index) / 1e4f;
		if (mask & static_cast<uint32_t>(1 << mask_index++)) _vesc_values.charged_energy_wh = extractFloat32(payload, index) / 10.f;
		if (mask & static_cast<uint32_t>(1 << mask_index++)) _vesc_values.tachometer = extractInt32(payload, index);
		if (mask & static_cast<uint32_t>(1 << mask_index++)) _vesc_values.tachometer_absolute = extractInt32(payload, index);
		if (mask & static_cast<uint32_t>(1 << mask_index++)) _vesc_values.fault = payload[index++];
		if (mask & static_cast<uint32_t>(1 << mask_index++)) _vesc_values.position_pid = extractFloat32(payload, index) / 1e6f;
		if (mask & static_cast<uint32_t>(1 << mask_index++)) _vesc_values.controller_id = payload[index++];
		if (mask & static_cast<uint32_t>(1 << mask_index++)) _vesc_values.ntc_temperature_mos1 = extractFloat16(payload, index) / 10.f;
		if (mask & static_cast<uint32_t>(1 << mask_index++)) _vesc_values.ntc_temperature_mos2 = extractFloat16(payload, index) / 10.f;
		if (mask & static_cast<uint32_t>(1 << mask_index++)) _vesc_values.ntc_temperature_mos3 = extractFloat16(payload, index) / 10.f;
		if (mask & static_cast<uint32_t>(1 << mask_index++)) _vesc_values.read_reset_average_vd = extractFloat32(payload, index) / 1000.f;
		if (mask & static_cast<uint32_t>(1 << mask_index++)) _vesc_values.read_reset_average_vq = extractFloat32(payload, index) / 1000.f;
		break;
	}
	_new_data_available = true;
}

uint16_t VescDriver::crc16(const uint8_t *buffer, const uint16_t length)
{
	uint16_t checksum{0};

	for (size_t i = 0; i < length; i++) {
		uint8_t table_index = (((checksum >> 8) ^ buffer[i]) & 0xFF);
		checksum = CRC_TABLE[table_index] ^ (checksum << 8);
	}

	return checksum;
}

void VescDriver::insertInt32(uint8_t *buffer, uint16_t &index, int32_t value)
{
	buffer[index++] = value >> 24;
	buffer[index++] = value >> 16;
	buffer[index++] = value >> 8;
	buffer[index++] = value;
}

void VescDriver::insertUInt32(uint8_t *buffer, uint16_t &index, uint32_t value)
{
	buffer[index++] = value >> 24;
	buffer[index++] = value >> 16;
	buffer[index++] = value >> 8;
	buffer[index++] = value;
}

int16_t VescDriver::extractInt16(const uint8_t *buffer, uint16_t &index)
{
	index += 2;
	return static_cast<int16_t>(buffer[index - 2] << 8 | buffer[index - 1]);
}

float VescDriver::extractFloat16(const uint8_t *buffer, uint16_t &index)
{
	return static_cast<float>(extractInt16(buffer, index));
}

int32_t VescDriver::extractInt32(const uint8_t *buffer, uint16_t &index)
{
	index += 4;
	return static_cast<int32_t>(buffer[index - 4] << 24 | buffer[index - 3] << 16 | buffer[index - 2] << 8 | buffer[index - 1]);
}

float VescDriver::extractFloat32(const uint8_t *buffer, uint16_t &index)
{
	return static_cast<float>(extractInt32(buffer, index));
}

uint32_t VescDriver::extractUInt32(const uint8_t *buffer, uint16_t &index)
{
	index += 4;
	return static_cast<uint32_t>(buffer[index - 4] << 24 | buffer[index - 3] << 16 | buffer[index - 2] << 8 | buffer[index - 1]);
}

size_t VescDriver::write(const uint8_t *buffer, const uint16_t length) {
	return _serial.write(buffer, length);
}
