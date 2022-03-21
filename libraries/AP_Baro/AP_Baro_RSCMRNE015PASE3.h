#pragma once

#include "AP_Baro_Backend.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Device.h>

class AP_Baro_RSCMRNE015PASE3 : public AP_Baro_Backend
{
public:
    void update() override;

    enum RSCMRNE_TYPE {
        BARO_015PASE3 = 0,
        BARO_015XXXXX = 1
    };

    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev, enum RSCMRNE_TYPE rscmrne_type = BARO_015PASE3);

private:
    /*
     * Update @accum and @count with the new sample in @val, taking into
     * account a maximum number of samples given by @max_count; in case
     * maximum number is reached, @accum and @count are updated appropriately
     */
    static void _update_and_wrap_accumulator(uint32_t *accum, uint32_t val,
                                             uint8_t *count, uint8_t max_count);

    AP_Baro_RSCMRNE015PASE3(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev, enum RSCMRNE_TYPE rscmrne_type);

    bool _init();

    bool _read_prom_word(uint16_t addr, uint8_t * pResBuf, uint32_t len);
    uint8_t _read_prom_byte(uint16_t addr);
 
    uint32_t _configure_adc();
    uint32_t _read_adc();
    bool _init_adc();
    bool _adc_cmd_no_res(uint8_t addr, uint8_t cmd, uint8_t len, uint8_t * pValue);
    bool _adc_cmd_byte_no_res(uint8_t cmd);
    bool _adc_cmd_set_press_temp(uint8_t cmd, uint8_t value);
    bool _read_sensor_bytes(uint8_t * pDataBuff, uint8_t len);
    bool _read_sensor_bytes2(uint8_t * pDataBuff, uint8_t len);

    void _timer();

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    /* Shared values between thread sampling the HW and main thread */
    struct {
        uint32_t s_D1;
        uint32_t s_D2;
        uint8_t d1_count;
        uint8_t d2_count;
    } _accum;

    uint8_t _state;
    uint8_t _instance;

    /* Last compensated values from accumulated sample */
    float _D1, _D2;

    // Internal calibration registers
    struct {
        uint16_t c1, c2, c3, c4, c5, c6;
    } _cal_reg;

    bool _discard_next;

    enum RSCMRNE_TYPE _rscmrne_type;

    uint8_t _eepromCfg[4];
};
