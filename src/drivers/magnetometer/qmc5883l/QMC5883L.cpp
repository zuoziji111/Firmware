/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "QMC5883L.hpp"

using namespace time_literals;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

QMC5883L::QMC5883L(int bus, uint8_t address, enum Rotation rotation) :
	I2C(MODULE_NAME, nullptr, bus, address, I2C_SPEED),
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(get_device_id())),
	_px4_mag(get_device_id(), external() ? ORB_PRIO_VERY_HIGH : ORB_PRIO_HIGH, rotation)
{
	set_device_type(DRV_MAG_DEVTYPE_QMC5883L);

	_px4_mag.set_device_type(DRV_MAG_DEVTYPE_QMC5883L);
	_px4_mag.set_external(external());
}

QMC5883L::~QMC5883L()
{
	Stop();

	perf_free(_transfer_perf);
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
}

bool QMC5883L::Init()
{
	if (I2C::init() != PX4_OK) {
		PX4_ERR("I2C::init failed");
		return false;
	}

	return Reset();
}

void QMC5883L::Stop()
{
	// wait until stopped
	while (_state.load() != STATE::STOPPED) {
		_state.store(STATE::REQUEST_STOP);
		ScheduleNow();
		px4_usleep(10);
	}
}

bool QMC5883L::Reset()
{
	_state.store(STATE::RESET);
	ScheduleClear();
	ScheduleNow();
	return true;
}

void QMC5883L::PrintInfo()
{
	perf_print_counter(_transfer_perf);
	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);

	_px4_mag.print_status();
}

int QMC5883L::probe()
{
	// no identifier, read X LSB register once
	const uint8_t cmd = static_cast<uint8_t>(Register::X_LSB);
	uint8_t buffer{};
	return transfer(&cmd, 1, &buffer, 1);
}

void QMC5883L::Run()
{
	switch (_state.load()) {
	case STATE::RESET:
		// CNTL2: Software Reset
		RegisterSetAndClearBits(Register::CNTL2, CNTL2_BIT::SOFT_RST, 0);
		_reset_timestamp = hrt_absolute_time();
		_state.store(STATE::WAIT_FOR_RESET);
		ScheduleDelayed(1_ms); // POR Completion Time
		break;

	case STATE::WAIT_FOR_RESET:

		// SOFT_RST: This bit is automatically reset to zero after POR routine
		if ((RegisterRead(Register::CNTL2) & CNTL2_BIT::SOFT_RST) == 0) {
			// if reset succeeded then configure
			_state.store(STATE::CONFIGURE);
			ScheduleNow();

		} else {
			// RESET not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 100_ms) {
				PX4_ERR("Reset failed, retrying");
				_state.store(STATE::RESET);
				ScheduleNow();

			} else {
				PX4_DEBUG("Reset not complete, check again in 10 ms");
				ScheduleDelayed(10_ms);
			}
		}

		break;

	case STATE::CONFIGURE:
		if (Configure()) {
			// if configure succeeded then start reading every 20 ms (50 Hz)
			_state.store(STATE::READ);
			ScheduleOnInterval(20_ms, 20_ms);

		} else {
			PX4_DEBUG("Configure failed, retrying");
			// try again in 50 ms
			ScheduleDelayed(50_ms);
		}

		break;

	case STATE::READ: {
			struct TransferBuffer {
				uint8_t X_LSB;
				uint8_t X_MSB;
				uint8_t Y_LSB;
				uint8_t Y_MSB;
				uint8_t Z_LSB;
				uint8_t Z_MSB;
				uint8_t STATUS;
			} buffer{};

			perf_begin(_transfer_perf);

			bool failure = false;
			const hrt_abstime timestamp_sample = hrt_absolute_time();

			const uint8_t cmd = static_cast<uint8_t>(Register::X_LSB);

			if (transfer(&cmd, 1, (uint8_t *)&buffer, sizeof(buffer)) != PX4_OK) {
				perf_count(_bad_transfer_perf);
				failure = true;
			}

			perf_end(_transfer_perf);

			// process data if successful transfer, data ready, no overflow
			if (!failure && (buffer.STATUS && STATUS_BIT::DRDY) && (buffer.STATUS && STATUS_BIT::OVL == 0)) {
				float x = combine(buffer.X_MSB, buffer.X_LSB);
				float y = combine(buffer.Y_MSB, buffer.Y_LSB);
				float z = combine(buffer.Z_MSB, buffer.Z_LSB);

				// sensor's frame is +x forward, +y left, +z up
				y = (y == INT16_MIN) ? INT16_MAX : -y; // -y
				z = (z == INT16_MIN) ? INT16_MAX : -z; // -z

				_px4_mag.update(timestamp_sample, x, y, z);
			}

			if (failure || hrt_elapsed_time(&_last_config_check_timestamp) > 100_ms) {
				// check registers incrementally
				if (RegisterCheck(_register_cfg[_checked_register], true)) {
					_last_config_check_timestamp = timestamp_sample;
					_checked_register = (_checked_register + 1) % size_register_cfg;

				} else {
					// register check failed, force reconfigure
					PX4_DEBUG("Health check failed, reconfiguring");
					_state.store(STATE::CONFIGURE);
					ScheduleNow();
					return;
				}
			}

			// limit temperature updates to 1 Hz
			if (hrt_elapsed_time(&_temperature_update_timestamp) > 1_s) {
				_temperature_update_timestamp = timestamp_sample;

				const uint8_t cmd_temperature = static_cast<uint8_t>(Register::TEMP_LSB);

				struct TransferBufferTemperature {
					uint8_t TOUT_LSB;
					uint8_t TOUT_MSB;
				} buffer_temperature{};

				if (transfer(&cmd_temperature, 1, (uint8_t *)&buffer_temperature, sizeof(buffer_temperature)) == PX4_OK) {
					const int16_t temperature_raw = combine(buffer_temperature.TOUT_MSB, buffer_temperature.TOUT_LSB);

					// The temperature coefficient is about 100 LSB/°C
					const float temperature_C = temperature_raw / 100.f;
					_px4_mag.set_temperature(temperature_C);
				}
			}
		}

		break;

	case STATE::REQUEST_STOP:
		ScheduleClear();
		_state.store(STATE::STOPPED);
		break;

	case STATE::STOPPED:
		// DO NOTHING
		break;
	}
}

bool QMC5883L::Configure()
{
	bool success = true;

	for (const auto &reg : _register_cfg) {
		if (!RegisterCheck(reg)) {
			success = false;
		}
	}

	_px4_mag.set_scale(1.f / 12000.f); // 12000 LSB/Gauss (Field Range = ±2G)

	return success;
}

bool QMC5883L::RegisterCheck(const register_config_t &reg_cfg, bool notify)
{
	bool success = true;

	const uint8_t reg_value = RegisterRead(reg_cfg.reg);

	if (reg_cfg.set_bits && !(reg_value & reg_cfg.set_bits)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not set)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.set_bits);
		success = false;
	}

	if (reg_cfg.clear_bits && (reg_value & reg_cfg.clear_bits)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not cleared)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.clear_bits);
		success = false;
	}

	if (!success) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);

		if (notify) {
			perf_count(_bad_register_perf);
			_px4_mag.increase_error_count();
		}
	}

	return success;
}

uint8_t QMC5883L::RegisterRead(Register reg)
{
	const uint8_t cmd = static_cast<uint8_t>(reg);
	uint8_t buffer{};
	transfer(&cmd, 1, &buffer, 1);
	return buffer;
}

void QMC5883L::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t buffer[2] { (uint8_t)reg, value };
	transfer(buffer, sizeof(buffer), nullptr, 0);
}

void QMC5883L::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);
	uint8_t val = orig_val;

	if (setbits) {
		val |= setbits;
	}

	if (clearbits) {
		val &= ~clearbits;
	}

	RegisterWrite(reg, val);
}
