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
#include "MovingAverage.hpp"

#define TWO_PI 6.283185307179586476925286766559

static constexpr int UPDATE_FREQUENCY_HZ{50};
static constexpr PinName VESC1_TX_PIN{PA_11};
static constexpr PinName VESC1_RX_PIN{PA_12};
static constexpr PinName VESC2_TX_PIN{PB_6};
static constexpr PinName VESC2_RX_PIN{PB_7};
static constexpr PinName PEDAL_INTERRUPT_PIN{PC_6};

static constexpr float FULL_TACHOMETER_ROT = 600.f;

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

	float brake_current_integral{0.f};
	float current{0.f};
	MovingAverage<float> rpm_filter;
	float pedal_position{0.f};
	int32_t zero_tachometer{0};
	float sinus_phase{1.3f};

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

			case 't':
				brake_current_integral += .1f;
				brake_current_integral = brake_current_integral > 20.f ? 20.f : brake_current_integral;
				break;
			case 'g':
				brake_current_integral -= .1f;
				brake_current_integral = brake_current_integral < 0.f ? 0.f : brake_current_integral;
				break;
			case 'h':
				brake_current_integral = 0.f;
				break;

			case 'v':
				vesc_generator.requestFirmwareVersion();
				vesc_motor.requestFirmwareVersion();
				break;
			case 'b':
				vesc_generator.requestValues();
				vesc_motor.requestValues();
				break;

			case 'o':
				sinus_phase += .1f;
				sinus_phase = sinus_phase > TWO_PI ? TWO_PI : sinus_phase;
				break;
			case 'l':
				sinus_phase -= .1f;
				sinus_phase = sinus_phase < 0.f ? 0.f : sinus_phase;
				break;
			}

			printf("Voltage: %.3f\n", vesc_generator.getInputVoltage());
			printf("Current: %.3f\n", current);
		}

		vesc_generator.requestValues();

		while (vesc1_uart.readable()) {
			uint8_t byte;
			if (vesc1_uart.read(&byte, 1)) {
				vesc_generator.parseInputByte(byte);
			}
		}

		while (vesc2_uart.readable()) {
			uint8_t byte;
			if (vesc2_uart.read(&byte, 1)) {
				vesc_motor.parseInputByte(byte);
			}
		}

		// Handle pedal interrupt
		if (is_pedal_interrupt_to_handle) {
			printf("Pedal interrupt\n");
			zero_tachometer = vesc_generator.getTachometer();
			is_pedal_interrupt_to_handle = false;
		}

		// Filter rpm
		static constexpr float RPM_MOTOR_SCALE = 1 / (14.583f * 7.f);
		const float rpm = rpm_filter.update(vesc_generator.getRpm() * RPM_MOTOR_SCALE);
		// printf("rpm: %.3f %.3f ", vesc_generator.getRpm(), rpm);

		// rpm control
		static constexpr float RPM_SETPOINT = 75.f;
		static constexpr float P_GAIN = .2f;
		static constexpr float I_GAIN = .003f;
		const float rpm_error = rpm - RPM_SETPOINT;
		brake_current_integral += I_GAIN * rpm_error;
		brake_current_integral = brake_current_integral > 0.f ? brake_current_integral : 0.f;
		float brake_current = P_GAIN * rpm_error;// + brake_current_integral;
		brake_current = brake_current > 0.f ? brake_current : 0.f;

		// pedal position tracking
		pedal_position = (static_cast<float>(vesc_generator.getTachometer()) - zero_tachometer) / FULL_TACHOMETER_ROT * TWO_PI;
		printf("pedal: %.3f ", pedal_position);

		static constexpr float SIN_INTENS = 4; // 2 is highest, infinity is lowest

		float sinus_braking = (cos(2*(pedal_position + sinus_phase)) + SIN_INTENS - 1) / SIN_INTENS;
		// printf("sinus: %.3f ", pedal_position);
		
		vesc_generator.commandBrakeCurrent(brake_current * sinus_braking);
		printf("brake: %.3f %.3f ", brake_current, brake_current * sinus_braking);
		printf("input: %.3f ", vesc_generator.getInputCurrent());
		// printf("phase: %.3f ", sinus_phase);

		float output_current = 10.f * fabsf(vesc_generator.getInputCurrent());
		output_current = output_current < 30.f ? output_current : 30.f;
		output_current = output_current > 0.f ? output_current : 0.f;

		vesc_motor.commandCurrent(output_current);

		printf("\n");

		// printf("%.3f,%.3f\n", sinus_braking, sinus_phase);
		// printf("%.3f\n", pedal_position);

		// Toggle LED and wait
		led = !led;
		wait_us(static_cast<int>(1e6) / UPDATE_FREQUENCY_HZ);
	}
}
