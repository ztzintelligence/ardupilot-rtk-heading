#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/SPIDevice.h>

#include "AP_Airspeed_Backend.h"

class AP_Airspeed_RSC : public AP_Airspeed_Backend
{
public:
    AP_Airspeed_RSC(AP_Airspeed &_frontend, uint8_t _instance);
    ~AP_Airspeed_RSC(void) {}

    // probe and initialise the sensor
    bool init() override;

    // return the current differential pressureijn Pascal
    bool get_differential_pressure(float &pressure) override;

    // return the current temperature in degress C
    bool get_temperature(float &temperature) override;

private:
    /*
    * Enum for the datarates supported by the sensor
    * N/F - Normal/Fast - 256 KHz/512 KHz
    * DR - Data Rate
    * SPS - Samples per second
    */
    typedef enum {
        N_DR_20_SPS = 0,
        N_DR_45_SPS,
        N_DR_90_SPS,
        N_DR_175_SPS,
        N_DR_330_SPS,
        N_DR_600_SPS,
        N_DR_1000_SPS,
        N_DR_NA,
        F_DR_40_SPS,
        F_DR_90_SPS,
        F_DR_180_SPS,
        F_DR_350_SPS,
        F_DR_660_SPS,
        F_DR_1200_SPS,
        F_DR_2000_SPS,
        F_DR_NA } RSC_DATA_RATE;

    // Enum for modes supported by the RSC sensor
    typedef enum {
        NORMAL_MODE = 0,
        NA_MODE,
        FAST_MODE } RSC_MODE;

    // Enum for pressure/temperature reading
    typedef enum {
        PRESSURE = 0,
        TEMPERATURE } READING_T;

    void _measure();
    void _collect();
    void _timer();

    bool _data_ready();

    void _eeprom_read(uint16_t address, uint8_t num_bytes, uint8_t *data);
    void _adc_convert(READING_T type);
    void _adc_read(uint8_t *data);
    float _calculate_pressure();

    void _get_pressure_range();
    void _get_pressure_minimum();
    void _get_initial_adc_values();
    void _get_coefficients();

    void _adc_configure();

    void _set_data_rate(RSC_DATA_RATE dr);

    float _pressure_range;
    float _pressure_minimum;
    uint8_t _adc_init_values[4];

    // calculate compensated pressure
    float _coeff_matrix[3][4];

    RSC_DATA_RATE _data_rate;
    RSC_MODE _mode;

    READING_T _type;

    uint8_t _state;
    int16_t _t_raw;
    int32_t _p_raw;

    float _temperature;
    float _pressure;

    float _pressure_sum;
    uint8_t _press_count;

    uint32_t _last_sample_time_ms;

    AP_HAL::DigitalSource *_drdy_pin;
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _eeprom_dev;
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _adc_dev;
};

