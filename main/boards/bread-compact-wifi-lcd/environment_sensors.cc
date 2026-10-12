#include "environment_sensors.h"

#include <esp_err.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_adc/adc_cali_scheme.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cmath>
#include <cstdio>

#define TAG "EnvironmentSensors"

namespace {
constexpr uint8_t kAhtAddress = 0x38;
constexpr uint8_t kBmpAddress = 0x77;
constexpr uint8_t kAgsAddress = 0x1A;
constexpr int64_t kAgsMinimumIntervalUs = 2000000;
}  // namespace

EnvironmentSensors::EnvironmentSensors() {
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = SENSOR_I2C_SDA_PIN,
        .scl_io_num = SENSOR_I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {.enable_internal_pullup = 1},
    };

    esp_err_t err = i2c_new_master_bus(&bus_config, &i2c_bus_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Create I2C bus failed: %s", esp_err_to_name(err));
        return;
    }

    aht_ok_ = AddI2cDevice(kAhtAddress, &aht_device_) && InitializeAht20();
    bmp_ok_ = AddI2cDevice(kBmpAddress, &bmp_device_) && InitializeBmp280();
    ags_ok_ = AddI2cDevice(kAgsAddress, &ags_device_);
    uv_ok_ = InitializeUvAdc();

    if (ags_ok_) {
        ags_status_ = "waiting_for_first_read";
    }

    ESP_LOGI(TAG, "Init status: AHT20=%d BMP280=%d AGS10=%d UV_ADC=%d",
             aht_ok_, bmp_ok_, ags_ok_, uv_ok_);
}

EnvironmentSensors::~EnvironmentSensors() {
    if (adc_cali_handle_ != nullptr) {
        adc_cali_delete_scheme_curve_fitting(adc_cali_handle_);
    }
    if (adc_handle_ != nullptr) {
        adc_oneshot_del_unit(adc_handle_);
    }

    if (aht_device_ != nullptr) i2c_master_bus_rm_device(aht_device_);
    if (bmp_device_ != nullptr) i2c_master_bus_rm_device(bmp_device_);
    if (ags_device_ != nullptr) i2c_master_bus_rm_device(ags_device_);
    if (i2c_bus_ != nullptr) i2c_del_master_bus(i2c_bus_);
}

bool EnvironmentSensors::AddI2cDevice(
    uint8_t address, i2c_master_dev_handle_t* handle) {
    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = SENSOR_I2C_CLOCK_HZ,
        .scl_wait_us = 0,
        .flags = {.disable_ack_check = 0},
    };

    esp_err_t err = i2c_master_bus_add_device(i2c_bus_, &device_config, handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Add I2C device 0x%02X failed: %s",
                 address, esp_err_to_name(err));
        return false;
    }

    err = i2c_master_probe(i2c_bus_, address, 100);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "I2C device 0x%02X did not ACK: %s",
                 address, esp_err_to_name(err));
        i2c_master_bus_rm_device(*handle);
        *handle = nullptr;
        return false;
    }
    return true;
}

bool EnvironmentSensors::ReadRegister(
    i2c_master_dev_handle_t device, uint8_t reg,
    uint8_t* data, size_t length) {
    return device != nullptr &&
           i2c_master_transmit_receive(device, &reg, 1, data, length, 500) == ESP_OK;
}

bool EnvironmentSensors::WriteBytes(
    i2c_master_dev_handle_t device, const uint8_t* data, size_t length) {
    return device != nullptr &&
           i2c_master_transmit(device, data, length, 500) == ESP_OK;
}

bool EnvironmentSensors::InitializeAht20() {
    vTaskDelay(pdMS_TO_TICKS(40));

    uint8_t status = 0;
    if (ReadRegister(aht_device_, 0x71, &status, 1) && (status & 0x08)) {
        return true;
    }

    const uint8_t calibrate[] = {0xBE, 0x08, 0x00};
    if (!WriteBytes(aht_device_, calibrate, sizeof(calibrate))) {
        ESP_LOGW(TAG, "AHT20 calibration command failed");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    return true;
}

bool EnvironmentSensors::ReadAht20() {
    const uint8_t trigger[] = {0xAC, 0x33, 0x00};
    uint8_t data[6] = {};

    if (!WriteBytes(aht_device_, trigger, sizeof(trigger))) return false;
    vTaskDelay(pdMS_TO_TICKS(85));

    if (i2c_master_receive(aht_device_, data, sizeof(data), 500) != ESP_OK) {
        return false;
    }
    if (data[0] & 0x80) return false;

    const uint32_t raw_humidity =
        (static_cast<uint32_t>(data[1]) << 12) |
        (static_cast<uint32_t>(data[2]) << 4) |
        (static_cast<uint32_t>(data[3]) >> 4);
    const uint32_t raw_temperature =
        (static_cast<uint32_t>(data[3] & 0x0F) << 16) |
        (static_cast<uint32_t>(data[4]) << 8) |
        data[5];

    aht_humidity_percent_ = raw_humidity * 100.0f / 1048576.0f;
    aht_temperature_c_ = raw_temperature * 200.0f / 1048576.0f - 50.0f;
    return true;
}

uint16_t EnvironmentSensors::ReadU16Le(const uint8_t* data) {
    return static_cast<uint16_t>(
        data[0] | (static_cast<uint16_t>(data[1]) << 8));
}

int16_t EnvironmentSensors::ReadS16Le(const uint8_t* data) {
    return static_cast<int16_t>(ReadU16Le(data));
}

bool EnvironmentSensors::InitializeBmp280() {
    uint8_t chip_id = 0;
    if (!ReadRegister(bmp_device_, 0xD0, &chip_id, 1) || chip_id != 0x58) {
        ESP_LOGW(TAG, "BMP280 ID invalid: 0x%02X", chip_id);
        return false;
    }

    uint8_t raw[24] = {};
    if (!ReadRegister(bmp_device_, 0x88, raw, sizeof(raw))) return false;

    bmp_calibration_.t1 = ReadU16Le(&raw[0]);
    bmp_calibration_.t2 = ReadS16Le(&raw[2]);
    bmp_calibration_.t3 = ReadS16Le(&raw[4]);
    bmp_calibration_.p1 = ReadU16Le(&raw[6]);
    bmp_calibration_.p2 = ReadS16Le(&raw[8]);
    bmp_calibration_.p3 = ReadS16Le(&raw[10]);
    bmp_calibration_.p4 = ReadS16Le(&raw[12]);
    bmp_calibration_.p5 = ReadS16Le(&raw[14]);
    bmp_calibration_.p6 = ReadS16Le(&raw[16]);
    bmp_calibration_.p7 = ReadS16Le(&raw[18]);
    bmp_calibration_.p8 = ReadS16Le(&raw[20]);
    bmp_calibration_.p9 = ReadS16Le(&raw[22]);
    if (bmp_calibration_.p1 == 0) return false;

    const uint8_t config[] = {0xF5, 0xA0};
    const uint8_t control[] = {0xF4, 0x57};
    return WriteBytes(bmp_device_, config, sizeof(config)) &&
           WriteBytes(bmp_device_, control, sizeof(control));
}

bool EnvironmentSensors::ReadBmp280() {
    uint8_t raw[6] = {};
    if (!ReadRegister(bmp_device_, 0xF7, raw, sizeof(raw))) return false;

    const int32_t adc_p = (static_cast<int32_t>(raw[0]) << 12) |
                          (static_cast<int32_t>(raw[1]) << 4) |
                          (raw[2] >> 4);
    const int32_t adc_t = (static_cast<int32_t>(raw[3]) << 12) |
                          (static_cast<int32_t>(raw[4]) << 4) |
                          (raw[5] >> 4);
    if (adc_t == 0x80000 || adc_p == 0x80000) return false;

    const double var1_t =
        (static_cast<double>(adc_t) / 16384.0 -
         bmp_calibration_.t1 / 1024.0) * bmp_calibration_.t2;
    const double temp_delta =
        static_cast<double>(adc_t) / 131072.0 -
        bmp_calibration_.t1 / 8192.0;
    const double var2_t =
        temp_delta * temp_delta * bmp_calibration_.t3;
    const double t_fine = var1_t + var2_t;
    bmp_temperature_c_ = static_cast<float>(t_fine / 5120.0);

    double var1 = t_fine / 2.0 - 64000.0;
    double var2 = var1 * var1 * bmp_calibration_.p6 / 32768.0;
    var2 += var1 * bmp_calibration_.p5 * 2.0;
    var2 = var2 / 4.0 + bmp_calibration_.p4 * 65536.0;
    var1 = (bmp_calibration_.p3 * var1 * var1 / 524288.0 +
            bmp_calibration_.p2 * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * bmp_calibration_.p1;
    if (var1 == 0.0) return false;

    double pressure = 1048576.0 - adc_p;
    pressure = (pressure - var2 / 4096.0) * 6250.0 / var1;
    var1 = bmp_calibration_.p9 * pressure * pressure / 2147483648.0;
    var2 = pressure * bmp_calibration_.p8 / 32768.0;
    pressure += (var1 + var2 + bmp_calibration_.p7) / 16.0;
    if (!std::isfinite(pressure) || pressure <= 0) return false;

    pressure_hpa_ = static_cast<float>(pressure / 100.0);
    altitude_m_ = 44330.0f *
        (1.0f - powf(pressure_hpa_ / 1013.25f, 0.1903f));
    return true;
}

uint8_t EnvironmentSensors::Ags10Crc8(
    const uint8_t* data, size_t length) {
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80)
                ? static_cast<uint8_t>((crc << 1) ^ 0x31)
                : static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}

bool EnvironmentSensors::ReadAgs10() {
    uint8_t data[5] = {};
    if (i2c_master_receive(ags_device_, data, sizeof(data), 500) != ESP_OK) {
        ags_status_ = "i2c_read_error";
        return false;
    }
    if (Ags10Crc8(data, 4) != data[4]) {
        ags_status_ = "crc_error";
        return false;
    }
    if (data[0] & 0x01) {
        ags_status_ = "not_ready";
        return false;
    }

    tvoc_ppb_ = (static_cast<uint32_t>(data[1]) << 16) |
                (static_cast<uint32_t>(data[2]) << 8) | data[3];
    ags_status_ = "ready";
    return true;
}

bool EnvironmentSensors::InitializeUvAdc() {
    adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    if (adc_oneshot_new_unit(&unit_config, &adc_handle_) != ESP_OK) {
        return false;
    }

    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_oneshot_config_channel(
            adc_handle_, ADC_CHANNEL_8, &channel_config) != ESP_OK) {
        return false;
    }

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t calibration_config = {
        .unit_id = ADC_UNIT_1,
        .chan = ADC_CHANNEL_8,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(
            &calibration_config, &adc_cali_handle_) != ESP_OK) {
        adc_cali_handle_ = nullptr;
        ESP_LOGW(TAG, "ADC calibration unavailable; voltage will be approximate");
    }
#endif
    return true;
}

bool EnvironmentSensors::ReadUv() {
    if (adc_handle_ == nullptr ||
        adc_oneshot_read(adc_handle_, ADC_CHANNEL_8, &uv_raw_) != ESP_OK) {
        return false;
    }

    if (adc_cali_handle_ != nullptr) {
        if (adc_cali_raw_to_voltage(
                adc_cali_handle_, uv_raw_, &uv_millivolts_) != ESP_OK) {
            return false;
        }
    } else {
        uv_millivolts_ = uv_raw_ * 3300 / 4095;
    }
    return true;
}

std::string EnvironmentSensors::ReadJson() {
    aht_valid_ = aht_ok_ && ReadAht20();
    bmp_valid_ = bmp_ok_ && ReadBmp280();
    uv_valid_ = uv_ok_ && ReadUv();

    const int64_t now = esp_timer_get_time();
    if (ags_ok_ &&
        (last_ags_read_us_ == 0 ||
         now - last_ags_read_us_ >= kAgsMinimumIntervalUs)) {
        ags_valid_ = ReadAgs10();
        last_ags_read_us_ = now;
    }

    char json[512];
    snprintf(json, sizeof(json),
             "{\"aht20\":{\"valid\":%s,\"temperature_c\":%.2f,"
             "\"humidity_percent\":%.2f},"
             "\"bmp280\":{\"valid\":%s,\"temperature_c\":%.2f,"
             "\"pressure_hpa\":%.2f,\"altitude_m\":%.2f},"
             "\"ags10\":{\"valid\":%s,\"status\":\"%s\",\"tvoc_ppb\":%lu},"
             "\"guva_s12sd\":{\"valid\":%s,\"adc_raw\":%d,"
             "\"voltage_mv_approx\":%d,\"uv_index_approx\":%.2f,"
             "\"uv_index_is_calibrated\":false}}",
             aht_valid_ ? "true" : "false",
             aht_temperature_c_, aht_humidity_percent_,
             bmp_valid_ ? "true" : "false",
             bmp_temperature_c_, pressure_hpa_, altitude_m_,
             ags_valid_ ? "true" : "false", ags_status_,
             static_cast<unsigned long>(tvoc_ppb_),
             uv_valid_ ? "true" : "false",
             uv_raw_, uv_millivolts_, uv_millivolts_ / 100.0f);
    return json;
}
