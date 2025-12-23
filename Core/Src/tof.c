#include "tof.h"

#include "FreeRTOS.h" // IWYU pragma: keep
#include "task.h"
#include "log.h"
#include "uart_command.h"
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define TOF_I2C_ADDRESS (0x52U)
#define TOF_SIGNAL_MIN_Mcps (2.0f)
#define TOF_SNR_MIN_RATIO (6.0f)
#define TOF_SIGMA_MAX_MM (15.0f)

static VL53L0X_Dev_t s_vl53l0x_dev = {0};
static VL53L0X_RangingMeasurementData_t s_ranging_data = {0};
static uint16_t s_distance_mm = 0;
static bool s_is_initialized = false;
static bool s_is_measurement_started = false;

static void tof_reset_state(void)
{
	memset(&s_ranging_data, 0, sizeof(s_ranging_data));
	s_distance_mm = 0;
	s_is_initialized = false;
	s_is_measurement_started = false;
}

static void tof_log_error(const char *stage, VL53L0X_Error status)
{
	LOG_ERROR("VL53L0X %s failed with status: %d", stage, (int)status);
}

static bool tof_start_measurement(void)
{
	VL53L0X_Error status = VL53L0X_SetDeviceMode(&s_vl53l0x_dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
	if (status != VL53L0X_ERROR_NONE) {
		tof_log_error("SetDeviceMode", status);
		return false;
	}

	status = VL53L0X_StartMeasurement(&s_vl53l0x_dev);
	if (status != VL53L0X_ERROR_NONE) {
		tof_log_error("StartMeasurement", status);
		return false;
	}

	s_is_measurement_started = true;
	return true;
}

static bool tof_perform_initialization(void)
{
	VL53L0X_Error status = VL53L0X_ERROR_NONE;
	uint32_t ref_spad_count = 0U;
	uint8_t is_aperture_spads = 0U;
	uint8_t vhv_settings = 0U;
	uint8_t phase_cal = 0U;

	memset(&s_vl53l0x_dev, 0, sizeof(s_vl53l0x_dev));
	s_vl53l0x_dev.I2cDevAddr = TOF_I2C_ADDRESS;

	LOG_INFO("Initializing VL53L0X...");

	status = VL53L0X_DataInit(&s_vl53l0x_dev);
	if (status != VL53L0X_ERROR_NONE) {
		tof_log_error("DataInit", status);
		return false;
	}

	status = VL53L0X_StaticInit(&s_vl53l0x_dev);
	if (status != VL53L0X_ERROR_NONE) {
		tof_log_error("StaticInit", status);
		return false;
	}

	status = VL53L0X_PerformRefCalibration(&s_vl53l0x_dev, &vhv_settings, &phase_cal);
	if (status != VL53L0X_ERROR_NONE) {
		tof_log_error("PerformRefCalibration", status);
		return false;
	}

	status = VL53L0X_PerformRefSpadManagement(&s_vl53l0x_dev, &ref_spad_count, &is_aperture_spads);
	if (status != VL53L0X_ERROR_NONE) {
		tof_log_error("PerformRefSpadManagement", status);
		return false;
	}

	status = VL53L0X_SetRefCalibration(&s_vl53l0x_dev, vhv_settings, phase_cal);
	if (status != VL53L0X_ERROR_NONE) {
		tof_log_error("SetRefCalibration", status);
		return false;
	}

	status = VL53L0X_SetLimitCheckValue(&s_vl53l0x_dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
																			(FixPoint1616_t)(TOF_SIGNAL_MIN_Mcps * 65536.0f));
	if (status != VL53L0X_ERROR_NONE) {
		tof_log_error("SetLimitCheckValue(Signal)", status);
		return false;
	}

	status = VL53L0X_SetLimitCheckValue(&s_vl53l0x_dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
																			(FixPoint1616_t)(TOF_SIGMA_MAX_MM * 65536.0f));
	if (status != VL53L0X_ERROR_NONE) {
		tof_log_error("SetLimitCheckValue(Sigma)", status);
		return false;
	}

	status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&s_vl53l0x_dev, 70000U);
	if (status != VL53L0X_ERROR_NONE) {
		tof_log_error("SetMeasurementTimingBudget", status);
		return false;
	}

	status = VL53L0X_SetGpioConfig(&s_vl53l0x_dev, 0, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,
									VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY,
									VL53L0X_INTERRUPTPOLARITY_LOW);
	if (status != VL53L0X_ERROR_NONE) {
		tof_log_error("SetGpioConfig", status);
		return false;
	}

	if (!tof_start_measurement()) {
		return false;
	}

	LOG_INFO("VL53L0X initialized successfully");
	return true;
}

void TofInit(void)
{
	if (s_is_initialized) {
		return;
	}

	const TickType_t init_delay = pdMS_TO_TICKS(500U);
	vTaskDelay(init_delay);

	if (tof_perform_initialization()) {
		s_is_initialized = true;
	} else {
		LOG_ERROR("VL53L0X initialization failed. Check connections and power.");
		tof_reset_state();
    }
}

void TofHandler(void)
{
	const TickType_t recovery_delay = pdMS_TO_TICKS(1000U);

	if (!s_is_initialized) {
		TofInit();
		if (!s_is_initialized) {
			vTaskDelay(recovery_delay);
			return;
		}
	}

	if (!s_is_measurement_started) {
		if (!tof_start_measurement()) {
			tof_reset_state();
			vTaskDelay(recovery_delay);
			return;
		}
	}

	if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) == 0U) {
		return;
	}

	VL53L0X_Error status = VL53L0X_GetRangingMeasurementData(&s_vl53l0x_dev, &s_ranging_data);
	if (status == VL53L0X_ERROR_NONE) {
		float signal = (float)s_ranging_data.SignalRateRtnMegaCps / 65536.0f;
		float ambient = (float)s_ranging_data.AmbientRateRtnMegaCps / 65536.0f;
		float snr = (ambient > 0.0001f) ? (signal / ambient) : signal;

		if (s_ranging_data.RangeStatus == 0U &&
				snr >= TOF_SNR_MIN_RATIO &&
				signal >= TOF_SIGNAL_MIN_Mcps &&
				s_ranging_data.EffectiveSpadRtnCount > 0U) {
			s_distance_mm = (uint16_t)s_ranging_data.RangeMilliMeter;
		} else {
			s_distance_mm = 999;
		}
	} else {
		tof_log_error("GetRangingMeasurementData", status);
	}

	status = VL53L0X_ClearInterruptMask(&s_vl53l0x_dev, 0);
	if (status != VL53L0X_ERROR_NONE) {
		tof_log_error("ClearInterruptMask", status);
		if (!tof_start_measurement()) {
			tof_reset_state();
		}
    }
}

uint16_t TofGetDistance(void)
{
	return s_distance_mm;
}
