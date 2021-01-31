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
 * @file MovingAverage.hpp
 * @brief Simple moving average filter
 * @author Matthias Grob <maetugr@gmail.com>
 */

#pragma once

template <typename T>
class MovingAverage
{
public:
	explicit MovingAverage() = default;
	~MovingAverage() = default;

	const T update(const T &sample)
	{
		// wrap overflow
		if (current_index >= _sample_count) {
			current_index = 0u;
		}

		// add sample to ringbuffer
		_samples[current_index] = sample;
		current_index++;

		// calculate average
		T sum{};
		for (unsigned i = 0; i < _sample_count; i++) {
			sum += _samples[i];
		}

		return sum / _sample_count;
	}

protected:
	static constexpr unsigned _sample_count{50};
	T _samples[_sample_count];
	unsigned current_index{0u};
};
