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

static constexpr int UPDATE_FREQUENCY_HZ{1000};
static constexpr PinName VESC1_TX_PIN{PA_11};
static constexpr PinName VESC1_RX_PIN{PA_12};
static constexpr PinName VESC2_TX_PIN{PB_6};
static constexpr PinName VESC2_RX_PIN{PB_7};
static constexpr PinName PEDAL_INTERRUPT_PIN{PC_6};

// Hardware initalization
static DigitalOut led(LED1);
static BufferedSerial computer(USBTX, USBRX, 9600);
static BufferedSerial vesc1_uart(VESC1_TX_PIN, VESC1_RX_PIN, 921600);
static BufferedSerial vesc2_uart(VESC2_TX_PIN, VESC2_RX_PIN, 115200);

volatile bool is_pedal_interrupt_to_handle{false};

void pedal_interrupt_callback()
{
	is_pedal_interrupt_to_handle = true;
}

void generator_control(VescDriver &vesc_generator, float p_gain, float rpm_setpoint) {
	static constexpr float RPM_MOTOR_SCALE = 1 / (14.583f * 7.f);
	const float rpm = vesc_generator.getRpm() * RPM_MOTOR_SCALE;
	float brake_current = p_gain * (rpm - rpm_setpoint);
	brake_current = brake_current > 0.f ? brake_current : 0.f;

	vesc_generator.commandBrakeCurrent(brake_current);
	vesc_generator.requestRpm();
}

int main()
{
	vesc1_uart.set_blocking(false);
	VescDriver vesc_generator(vesc1_uart);
	//VescDriver vesc_motor(fdopen(&vesc2_uart, "r+b"));

	InterruptIn pedal_interrupt(PEDAL_INTERRUPT_PIN);
	pedal_interrupt.fall(&pedal_interrupt_callback);

	Timer t;
	t.start();

	// Application start
	printf("Coilchain v0.1\n");

	float rpm_setpoint{70.f};
	float p_gain{2.f};

	while(true) {
		// Handle computer input
		if (computer.readable()) {
			char letter{};
			computer.read(&letter, 1);

			switch (letter) {
			case 'q':
				rpm_setpoint += 1.f;
				printf("rpm_setpoint: %.3f\n", rpm_setpoint);
				break;
			case 'a':
				rpm_setpoint -= 1.f;
				printf("rpm_setpoint: %.3f\n", rpm_setpoint);
				break;

			case 'w':
				p_gain += .1f;
				printf("p_gain: %.3f\n", p_gain);
				break;
			case 's':
				p_gain -= .1f;
				printf("p_gain: %.3f\n", p_gain);
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
		}

		while (vesc1_uart.readable()) {
			uint8_t byte;
			if (vesc1_uart.read(&byte, 1)) {
				vesc_generator.parseInputByte(byte);
			}

			if (vesc_generator._new_data_available) {
				generator_control(vesc_generator, p_gain, rpm_setpoint);
				vesc_generator._new_data_available = false;
				t.reset();
			}
		}

		// Handle pedal interrupt
		if (is_pedal_interrupt_to_handle) {
			is_pedal_interrupt_to_handle = false;
		}

		if (t.read() > 1.f) {
			printf("%.3f\n", t.read());
			t.reset();
			printf("%.3f\n", t.read());
			vesc_generator.requestRpm();
		}

		// Toggle LED and wait
		led = !led;
		wait_us(static_cast<int>(1e6) / UPDATE_FREQUENCY_HZ);
	}
}
