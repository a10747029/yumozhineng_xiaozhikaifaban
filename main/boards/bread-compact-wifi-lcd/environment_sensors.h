#pragma once

#include "config.h"

#include <driver/i2c_master.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>

#include <cstddef>
#include <cstdint>
#include <string>

class EnvironmentSensors {
public:
    EnvironmentSensors();
    ~EnvironmentSensors();

    EnvironmentSensors(const EnvironmentSensors&) = delete;
    EnvironmentSensors& operator=(const EnvironmentSensors&) = delete;

    // Reads AHT20, BMP280 and UV on demand. AGS10 reads are rate-limited
    // to one read every two seconds and return the most recent cached sample.
    std::string ReadJson();

private:
    struct BmpCalibration {
        uint16_t t1 = 0;
        int16_t t2 = 0;
        int16_t t3 = 0;
        uint16_t p1 = 0;
        int16_t p2 = 0;
        int16_t p3 = 0;
        int16_t p4 = 0;
        int16_t p5 = 0;
        int16_t p6 = 0;
        int16_t p7 = 0;
        int16_t p8 = 0;
        int16_t p9 = 0;
    };

    bool AddI2cDevice(uint8_t address, i2c_master_dev_handle_t* handle);
    bool ReadRegister(i2c_master_dev_handle_t device, uint8_t reg,
                      uint8_t* data, size_t length);
    bool WriteBytes(i2c_master_dev_handle_t device, const uint8_t* data,
                    size_t length);

    bool InitializeAht20();
    bool ReadAht20();
    bool InitializeBmp280();
    bool ReadBmp280();
    bool ReadAgs10();
    bool InitializeUvAdc();
    bool ReadUv();

    static uint8_t Ags10Crc8(const uint8_t* data, size_t length);
    static uint16_t ReadU16Le(const uint8_t* data);
    static int16_t ReadS16Le(const uint8_t* data);

    i2c_master_bus_handle_t i2c_bus_ = nullptr;
    i2c_master_dev_handle_t aht_device_ = nullptr;
    i2c_master_dev_handle_t bmp_device_ = nullptr;
    i2c_master_dev_handle_t ags_device_ = nullptr;

    adc_oneshot_unit_handle_t adc_handle_ = nullptr;
    adc_cali_handle_t adc_cali_handle_ = nullptr;

    BmpCalibration bmp_calibration_;

    bool aht_ok_ = false;
    bool bmp_ok_ = false;
    bool ags_ok_ = false;
    bool uv_ok_ = false;

    bool aht_valid_ = false;
    bool bmp_valid_ = false;
    bool ags_valid_ = false;
    bool uv_valid_ = false;

    float aht_temperature_c_ = 0;
    float aht_humidity_percent_ = 0;
    float bmp_temperature_c_ = 0;
    float pressure_hpa_ = 0;
    float altitude_m_ = 0;
    uint32_t tvoc_ppb_ = 0;
    int uv_raw_ = 0;
    int uv_millivolts_ = 0;

    const char* ags_status_ = "not_initialized";
    int64_t last_ags_read_us_ = 0;
};
