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

static constexpr int UPDATE_FREQUENCY_HZ{10};
static constexpr PinName VESC1_TX_PIN{PA_11};
static constexpr PinName VESC1_RX_PIN{PA_12};
static constexpr PinName VESC2_TX_PIN{PB_6};
static constexpr PinName VESC2_RX_PIN{PB_7};
static constexpr PinName PEDAL_INTERRUPT_PIN{PC_6};

// Hardware initalization
static DigitalOut led(LED1);
static BufferedSerial computer(USBTX, USBRX, 9600);
static BufferedSerial vesc1_uart(VESC1_TX_PIN, VESC1_RX_PIN, 115200);
static BufferedSerial vesc2_uart(VESC2_TX_PIN, VESC2_RX_PIN, 115200);

volatile bool is_pedal_interrupt_to_handle{false};

void pedal_interrupt_callback()
{
	is_pedal_interrupt_to_handle = true;
}

int main()
{
	vesc1_uart.set_blocking(false);
	VescDriver vesc_generator(fdopen(&vesc1_uart, "r+b"));
	VescDriver vesc_motor(fdopen(&vesc2_uart, "r+b"));

	InterruptIn pedal_interrupt(PEDAL_INTERRUPT_PIN);
	pedal_interrupt.fall(&pedal_interrupt_callback);

	// Application start
	printf("Coilchain v0.1\n");

	float current{0.f};

	while(true) {
		// Handle computer input
		if (computer.readable()) {
			char letter{};
			computer.read(&letter, 1);

			switch (letter) {
			case 'q':
				current += .1f;
				current = current > 5.f ? 5.f : current;
				break;
			case 'a':
				current -= .1f;
				current = current < 0.f ? 0.f : current;
				break;
			case 's':
				current = 0.f;
				break;
			case 'v':
				vesc_generator.requestFirmwareVersion();
				break;
			case 't':
				vesc_generator.requestValues();
				printf("Voltage: %.3f\n", vesc_generator.getInputVoltage());
				break;
			case 'r':
				vesc_generator.requestRpm();
				printf("RPM: %.3f\n", vesc_generator.getRpm());
				break;
			}

			printf("Current: %.3f\n", current);
		}

		vesc_generator.commandCurrent(current);

		while (vesc1_uart.readable()) {
			uint8_t byte;
			if (vesc1_uart.read(&byte, 1)) {
				vesc_generator.parseInputByte(byte);
			}
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
