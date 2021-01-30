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
 * @file main.cpp
 * @brief Main Coilchain application
 * @author Matthias Grob <maetugr@gmail.com>
 */

#include <mbed.h>

#include "VescDriver.hpp"

static constexpr PinName VESC1_TX_PIN{PA_11};
static constexpr PinName VESC1_RX_PIN{PA_12};
static constexpr PinName VESC2_TX_PIN{PB_6};
static constexpr PinName VESC2_RX_PIN{PB_7};
static constexpr PinName PEDAL_INTERRUPT_PIN{PC_6};

static constexpr int UPDATE_FREQUENCY_HZ{10};
volatile bool is_pedal_interrupt_to_handle{false};

void pedal_interrup_callback()
{
	is_pedal_interrupt_to_handle = true;
}

int main() {
	// Hardware initalization
	DigitalOut led(LED1);

	BufferedSerial computer(USBTX, USBRX, 9600);

	BufferedSerial vesc1_uart(VESC1_TX_PIN, VESC1_RX_PIN, 115200);
	VescDriver vesc_generator(fdopen(&vesc1_uart, "r+b"));

	BufferedSerial vesc2_uart(VESC2_TX_PIN, VESC2_RX_PIN, 115200);
	VescDriver vesc_motor(fdopen(&vesc2_uart, "r+b"));

	InterruptIn pedal_interrupt(PEDAL_INTERRUPT_PIN);
	pedal_interrupt.fall(&pedal_interrup_callback);

	// Application start
	printf("Coilchain v0.1\n");

	float current{0.f};

	while(true) {
		// Handle computer input
		if (computer.readable()) {
			char letter{};
			computer.read(&letter, 1);

			if (letter == 'q' && current < 5.f) {
				current += .1f;
			}

			if (letter == 'a' && current > .0f) {
				current -= .1f;
			}

			if (letter == 'v') {
				vesc_generator.requestFirmwareVersion();
			}

			if (letter == 't') {
				vesc_generator.requestValues();
			}

			printf("Voltage: %.3f\n", vesc_generator.getInputVoltage());
			printf("Current: %.3f\n", current);
		}

		vesc_generator.commandCurrent(current);

		while (vesc1_uart.readable()) {
			vesc_generator.processInput();
		}

		// Handle pedal interrupt
		if (is_pedal_interrupt_to_handle) {
			printf("Pedal interrupt\n");
			is_pedal_interrupt_to_handle = false;
		}

		// Toggle LED and wait
		led = !led;
		wait_us(static_cast<int>(1e6) / UPDATE_FREQUENCY_HZ);
	}
}
