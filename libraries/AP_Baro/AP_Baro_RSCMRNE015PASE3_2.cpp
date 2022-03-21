/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "AP_Baro_RSCMRNE015PASE3_2.h"

#include <utility>
#include <stdio.h>

#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>

#include "Pressure_Comp.h"
#include "crc.h"

#include "AP_Baro_RSCMRNE_Eeprom.h"

EepromAttribute eepromAttribute;
CompStatus_Enum RSCMRNE015PASE3InitStatus = CRC_FAILURE;

extern const AP_HAL::HAL &hal;

/* PROM READ CMD */
#define CMD_READ_EEPROM 0x03

#define ADDR_EEPROM_SENSOR_NAME 0
#define ADDR_EEPROM_ADC_CFG_61  61
#define ADDR_EEPROM_ADC_CFG_63  63
#define ADDR_EEPROM_ADC_CFG_65  65
#define ADDR_EEPROM_ADC_CFG_67  67

#define ADDR_EEPROM_PRESSURE_RANGE  27
#define ADDR_EEPROM_PRESSURE_MINIMUM 31

#define ADDR_EEPROM_PRESSURE_OFFSET_C0 130
#define ADDR_EEPROM_PRESSURE_OFFSET_C1 134
#define ADDR_EEPROM_PRESSURE_OFFSET_C2 138
#define ADDR_EEPROM_PRESSURE_OFFSET_C3 142

#define ADDR_EEPROM_PRESSURE_ADCCFG0 61 // 1
#define ADDR_EEPROM_PRESSURE_ADCCFG1 63 // 1
#define ADDR_EEPROM_PRESSURE_ADCCFG2 65 // 1
#define ADDR_EEPROM_PRESSURE_ADCCFG3 67 // 1

#define ADDR_EEPROM_PRESSURE_SPANC0 210
#define ADDR_EEPROM_PRESSURE_SPANC1 214
#define ADDR_EEPROM_PRESSURE_SPANC2 218
#define ADDR_EEPROM_PRESSURE_SPANC3 222

#define ADDR_EEPROM_PRESSURE_SHAPEC0 290
#define ADDR_EEPROM_PRESSURE_SHAPEC1 294
#define ADDR_EEPROM_PRESSURE_SHAPEC2 298
#define ADDR_EEPROM_PRESSURE_SHAPEC3 302

#define ADDR_EEPROM_PRESSURE_UNIT 35    // 5

/* Write ADC Reg */
#define CMD_WRITE_ADC       0x40
#define CMD_WRITE_ADC_ALL   0x43

#define ADC_REG_ADDR0 0b0000
#define ADC_REG_ADDR1 0b0100
#define ADC_REG_ADDR2 0b1000
#define ADC_REG_ADDR3 0b1100

#define ADC_REG_VALUE_LEN1 0b00
#define ADC_REG_VALUE_LEN2 0b01
#define ADC_REG_VALUE_LEN3 0b10
#define ADC_REG_VALUE_LEN4 0b11

#define CMD_ADC_RESET   0x06
#define CMD_ADC_PRESS_TEMP  0x44
#define CMD_ADC_SET_P_VAL   0b01000100
#define CMD_ADC_SET_T_VAL   0b01000110
#define CMD_ADC_START_CONV  0x08

/* write to one of these addresses to start pressure conversion */
#define ADDR_CMD_CONVERT_D1_OSR256  0x40
#define ADDR_CMD_CONVERT_D1_OSR512  0x42
#define ADDR_CMD_CONVERT_D1_OSR1024 0x44
#define ADDR_CMD_CONVERT_D1_OSR2048 0x46
#define ADDR_CMD_CONVERT_D1_OSR4096 0x48

/* write to one of these addresses to start temperature conversion */
#define ADDR_CMD_CONVERT_D2_OSR256  0x50
#define ADDR_CMD_CONVERT_D2_OSR512  0x52
#define ADDR_CMD_CONVERT_D2_OSR1024 0x54
#define ADDR_CMD_CONVERT_D2_OSR2048 0x56
#define ADDR_CMD_CONVERT_D2_OSR4096 0x58

/*
  use an OSR of 1024 to reduce the self-heating effect of the
  sensor. Information from MS tells us that some individual sensors
  are quite sensitive to this effect and that reducing the OSR can
  make a big difference
 */
static const uint8_t ADDR_CMD_CONVERT_PRESSURE = ADDR_CMD_CONVERT_D1_OSR1024;
static const uint8_t ADDR_CMD_CONVERT_TEMPERATURE = ADDR_CMD_CONVERT_D2_OSR1024;

/*
  constructor
 */
AP_Baro_RSCMRNE015PASE3_2::AP_Baro_RSCMRNE015PASE3_2(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev, enum RSCMRNE_TYPE rscmrne_type)
    : AP_Baro_Backend(baro)
    , _dev(std::move(dev))
    , _rscmrne_type(rscmrne_type)
{
}

AP_Baro_Backend *AP_Baro_RSCMRNE015PASE3_2::probe(AP_Baro &baro,
                                       AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                       enum RSCMRNE_TYPE rscmrne_type)
{
    if (!dev) {
        return nullptr;
    }
    AP_Baro_RSCMRNE015PASE3_2 *sensor = new AP_Baro_RSCMRNE015PASE3_2(baro, std::move(dev), rscmrne_type);
    if (!sensor || !sensor->_init()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

// static uint32_t pressureCnt = 0;
// static float pressureAcum = 0;

bool AP_Baro_RSCMRNE015PASE3_2::_init()
{
    if (!_dev) {
        return false;
    }

    if (!_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        AP_HAL::panic("PANIC: AP_Baro_RSCMRNE015PASE3_2: failed to take serial semaphore for init");
    }

    // high retries for init
    _dev->set_retries(10);

    printf("Start init AP_Baro_RSCMRNE015PASE3_2 ==================\n");

    uint8_t initCnt = 0;
    // wait eeprom read and comp finished.
    while(RSCMRNE015PASE3InitStatus != COMPINIT_OK)
    {
        hal.scheduler->delay(100);
        initCnt++;

        if(initCnt > 5)
        {
            _dev->get_semaphore()->give();
            return false;
        }
    }

    printf("2: Compensate_Pressure_Init =============== \n");

    // _dev->switchSpiMode();

    // reset adc
    if(!_adc_cmd_byte_no_res(CMD_ADC_RESET))
    {
        printf("CMD_ADC_RESET false ============\n");
        _dev->get_semaphore()->give();
        return false;
    }

    // delay ms
    hal.scheduler->delay(20);

        // reset adc
    if(!_adc_cmd_byte_no_res(CMD_ADC_RESET))
    {
        printf("CMD_ADC_RESET false ============\n");
        _dev->get_semaphore()->give();
        return false;
    }

    // delay ms
    hal.scheduler->delay(20);


    // if(!_adc_cmd_no_res(ADC_REG_ADDR0, CMD_WRITE_ADC, ADC_REG_VALUE_LEN4, eepromAttribute.adcCfg))
    if(!_adc_cmd_reset(eepromAttribute.adcCfg))
    {
        printf("Config adc: 4 value. Failed ==============\n");
        _dev->get_semaphore()->give();
        return false;
    }

    hal.scheduler->delay(20);

    if(!_adc_cmd_set_press_temp(CMD_ADC_PRESS_TEMP, CMD_ADC_SET_T_VAL))
    {
        printf("set temp failed !!!!!!!!!!!!!!!!!!!!\n");
        _dev->get_semaphore()->give();
        return false;
    }

    uint8_t sensorBuf[4] = {0};

    // raw temperature data
    uint32_t temp_raw = 0;
    // raw pressure data
    uint32_t pressure_raw = 0;

    uint16_t tryTimes = 3;
    while(tryTimes--)
    {
        hal.scheduler->delay(100);

        if(_read_sensor_bytes(sensorBuf, 3))
        {
            // tempValue = ((sensorBuf[0] << 6) + ((sensorBuf[1] & 0xFC) >> 2)) * 0.03125f;
            temp_raw = ((sensorBuf[0] << 16) + (sensorBuf[1] << 8)) >> 10;

            printf("read sensor temperature: ++++++++++++++++  %d %d %d \n", sensorBuf[0], sensorBuf[1], sensorBuf[2]);
        }
        else 
        {
            printf("-------------ee--------\n");
        }
    }

    // bool autozero = false;

    CompReturn_Struct compStatus;

    if(!_adc_cmd_set_press_temp(CMD_ADC_PRESS_TEMP, CMD_ADC_SET_P_VAL))
    {
        printf("set press failed!!!!!!!!!!!!!!!!!!!!!\n");

        _dev->get_semaphore()->give();
        return false;
    }

    tryTimes = 3;
    while(tryTimes--)
    {
        hal.scheduler->delay(20);
        // get pressure
        if(_read_sensor_bytes(sensorBuf, 3))
        {
            pressure_raw = (sensorBuf[0] << 16) + (sensorBuf[1] << 8) + sensorBuf[2];
            printf("pressure raw ===========> %d, %d, %d, %d -------- %d.\n", sensorBuf[0], sensorBuf[1], sensorBuf[2], pressure_raw, temp_raw);

            compStatus = Compensate_Pressure(pressure_raw, temp_raw);
            if (compStatus.CompStatus == PRESSURE_VALID)
            {
                // printf("Filtered pressure is %f ------------------------ \n", compStatus.f32PressureOutput*68.94757f);
                printf("compStatus ok, pressure is %f, %f\n", compStatus.f32PressureOutput, compStatus.f32PressureOutput * 68.94757f);
                // pressureCnt++;
                // pressureAcum += compStatus.f32PressureOutput * 68.94757f;

                // if(pressureCnt == 10)
                // {
                //     printf("Filtered pressure is %f +++++++++++++++++++++++\n", pressureAcum/10.0f);
                //     pressureCnt = 0;
                //     pressureAcum = 0;
                // }
            }
            else
            {
                printf("compStatus not ok, pressure is %f\n", compStatus.f32PressureOutput * 68.94757f);
            }
        }
    }

    ////////////////////////
    memset(&_accum, 0, sizeof(_accum));

    if(!_adc_cmd_set_press_temp(CMD_ADC_PRESS_TEMP, CMD_ADC_SET_T_VAL))
    {
        printf("init failed: set press temp failed. ----------------\n");
        _dev->get_semaphore()->give();
        return false;
    }
    _state = 0;

    _instance = _frontend.register_sensor();

    // if (_rscmrne_type == BARO_015PASE3_2) {
    //     _frontend.set_type(_instance, AP_Baro::BARO_TYPE_WATER);
    // }

    // lower retries for run
    _dev->set_retries(3);
    
    _dev->get_semaphore()->give();

    /* Request 150Hz update */
    _dev->register_periodic_callback(15 * AP_USEC_PER_MSEC,
                                     FUNCTOR_BIND_MEMBER(&AP_Baro_RSCMRNE015PASE3_2::_timer, void));
    return true;
}

bool AP_Baro_RSCMRNE015PASE3_2::_init_adc(void)
{
    // reset adc
    if(!_adc_cmd_byte_no_res(CMD_ADC_RESET))
    {
        printf("CMD_ADC_RESET false ============\n");
        return false;
    }

    // delay ms
    hal.scheduler->delay(20);

        // reset adc
    if(!_adc_cmd_byte_no_res(CMD_ADC_RESET))
    {
        printf("CMD_ADC_RESET false ============\n");
        return false;
    }

    // delay ms
    hal.scheduler->delay(20);

    // if(!_adc_cmd_no_res(ADC_REG_ADDR0, CMD_WRITE_ADC, ADC_REG_VALUE_LEN4, eepromAttribute.adcCfg))
    if(!_adc_cmd_reset(eepromAttribute.adcCfg))
    {
        printf("Config adc: 4 value. Failed ==============\n");
        return false;
    }
    hal.scheduler->delay(20);

    return true;
}

//////////////////// Read EEPROM Data /////////////////
bool AP_Baro_RSCMRNE015PASE3_2::_read_prom_word(uint16_t addr, uint8_t * pResBuf, uint32_t len)
{
    uint8_t reg[2];
    // uint8_t val[2];

    reg[0] = CMD_READ_EEPROM | ((addr & 0x100) >> 5);

    // printf("reg[0] is %d ====================\n", reg[0]);

    reg[1] = addr & 0xFF;

    if (!_dev->transfer((const uint8_t *)&reg, 2, pResBuf, len)) {
        return false;
    }
    return true;
}

uint8_t AP_Baro_RSCMRNE015PASE3_2::_read_prom_byte(uint16_t addr)
{
    uint8_t reg[2];
    uint8_t val[2] = {0};

    reg[0] = CMD_READ_EEPROM | ((addr & 0x100) >> 5);
    reg[1] = addr & 0xFF;

    if (!_dev->transfer((const uint8_t *)&reg, 2, val, 1)) {
        return 0;
    }
    return val[0];
}

bool AP_Baro_RSCMRNE015PASE3_2::_adc_cmd_reset(uint8_t * pValue)
{
    uint8_t reg[8];

    reg[0] = 0x43;

    reg[1] = pValue[0];
    reg[2] = pValue[1];
    reg[3] = pValue[2];
    reg[4] = pValue[3];

    if (!_dev->transfer((const uint8_t *)&reg, 5, nullptr, 0))
    {
        return false;
    }

    return true;
}

bool AP_Baro_RSCMRNE015PASE3_2::_adc_cmd_no_res(uint8_t addr, uint8_t cmd, uint8_t len, uint8_t * pValue)
{
    if(len > 8)
    {
        return false;
    }

    if(addr > 3)
    {
        return false;
    }

    uint8_t reg[8];

    reg[0] = cmd | addr | len;

    for(int i=0; i<=len; i++)
    {
        reg[i+1] = pValue[i];
    }

    if (!_dev->transfer((const uint8_t *)&reg, 1+len+1, nullptr, 0))
    {
        return false;
    }

    return true;
}

bool AP_Baro_RSCMRNE015PASE3_2::_adc_cmd_set_press_temp(uint8_t cmd, uint8_t value)
{
    uint8_t reg[2];

    reg[0] = cmd;
    reg[1] = value;

    if (!_dev->transfer((const uint8_t *)&reg, 2, nullptr, 0))
    {
        return false;
    }

    return true;
}

bool AP_Baro_RSCMRNE015PASE3_2::_adc_cmd_byte_no_res(uint8_t cmd)
{
    uint8_t reg[2];

    reg[0] = cmd;

    if (!_dev->transfer((const uint8_t *)&reg, 1, nullptr, 0))
    {
        return false;
    }

    return true;
}

bool AP_Baro_RSCMRNE015PASE3_2::_read_sensor_bytes(uint8_t * pDataBuff, uint8_t len)
{
    uint8_t reg[2];

    // reg[0] = CMD_ADC_START_CONV;
    reg[0] = 0;
    reg[1] = 0;

    // bool SPIDevice::transfer_fullduplex(const uint8_t *send, uint8_t *recv, uint32_t len)

    // if (!_dev->transfer((const uint8_t *)&reg, 1, pDataBuff, len)) {
    if (!_dev->transfer((const uint8_t *)&reg, 2, pDataBuff, len)) {
        return false;
    }
    return true;
}

bool AP_Baro_RSCMRNE015PASE3_2::_read_sensor_bytes2(uint8_t * pDataBuff, uint8_t len)
{
    uint8_t reg[2];

    reg[0] = CMD_ADC_START_CONV;

    if (!_dev->transfer((const uint8_t *)&reg, 1, nullptr, 0)) {
        return false;
    }

    hal.scheduler->delay_microseconds(50);

    if (!_dev->transfer(nullptr, 0, pDataBuff, len)) {
        return false;
    }

    return true;

    /////////////////////////////////////////////////////////////////
    // uint8_t aReg;

    // aReg = ADIS16488_REG_REG(regnum);

    // uint8_t req[2] = {aReg, 0};
    // uint8_t reply[2] {};
    // dev->transfer(req, sizeof(req), nullptr, 0);
    // hal.scheduler->delay_microseconds(T_STALL_US);
    // dev->transfer(nullptr, 0, reply, sizeof(reply));
    // uint16_t ret = (reply[0]<<8U) | reply[1];
    // return ret;
}

uint32_t AP_Baro_RSCMRNE015PASE3_2::_read_adc()
{
    // uint8_t val[3];
    // if (!_dev->transfer(&CMD_MS56XX_READ_ADC, 1, val, sizeof(val))) {
    //     return 0;
    // }
    // return (val[0] << 16) | (val[1] << 8) | val[2];

    return 0;
}

/*
 * Read the sensor with a state machine
 * We read one time temperature (state=0) and then 4 times pressure (states 1-4)
 *
 * Temperature is used to calculate the compensated pressure and doesn't vary
 * as fast as pressure. Hence we reuse the same temperature for 4 samples of
 * pressure.
*/
void AP_Baro_RSCMRNE015PASE3_2::_timer(void)
{
    static uint8_t next_state = 0;
    static uint32_t resetTimesCnt = 0;
    static uint32_t runCnt = 0;
    uint32_t adc_val = 0;

    uint8_t sensorBuf[4] = {0};

    // runCnt++;

    // if((runCnt%300) == 1)
    // {
    //     printf("in _timer: resetTimesCnt is %d <======================== \n", resetTimesCnt);
    // }


    if(_read_sensor_bytes(sensorBuf, 3))
    {
        if(0 == _state)
        {
            adc_val = ((sensorBuf[0] << 16) + (sensorBuf[1] << 8)) >> 10;
        }
        else
        {
            adc_val = (sensorBuf[0] << 16) + (sensorBuf[1] << 8) + sensorBuf[2];

            if(sensorBuf[0] > 0xFB)
            {
                _init_adc();
                resetTimesCnt++;

                printf("in _timer: force reset and init adc, cnt = %d <==========================================\n", resetTimesCnt);

                _adc_cmd_set_press_temp(CMD_ADC_PRESS_TEMP, CMD_ADC_SET_P_VAL);
                _state = 1;

                hal.scheduler->delay(20);

                return;
            }
        }
    }
    else
    {
        printf("in timer loop1 ------- failed.\n");
        return;
    }

    if(_state)
    {
        if((runCnt++%100) == 1)
        {
            printf("in _timer: resetTimesCnt is %d, sensorBuf[0] is %d <======================== \n", resetTimesCnt, sensorBuf[0]);
        }
    }

    {
        WITH_SEMAPHORE(_sem);

        if (_state == 0) { // temp;
            _update_and_wrap_accumulator(&_accum.s_D2, adc_val,
                                        &_accum.d2_count, 32);
        }
        else    // pressure;
        {
            _update_and_wrap_accumulator(&_accum.s_D1, adc_val,
                                        &_accum.d1_count, 128);
        }
    }

    next_state = (next_state + 1) % 10;

    if(0 == next_state)
    {
        _adc_cmd_set_press_temp(CMD_ADC_PRESS_TEMP, CMD_ADC_SET_T_VAL);
        _state = 0;
    }
    
    if(1 == next_state)
    {
        _adc_cmd_set_press_temp(CMD_ADC_PRESS_TEMP, CMD_ADC_SET_P_VAL);
        _state = 1;
    }
}

void AP_Baro_RSCMRNE015PASE3_2::_update_and_wrap_accumulator(uint32_t *accum, uint32_t val,
                                                  uint8_t *count, uint8_t max_count)
{
    *accum += val;
    *count += 1;
    if (*count == max_count) {
        *count = max_count / 2;
        *accum = *accum / 2;
    }
}

void AP_Baro_RSCMRNE015PASE3_2::update()
{
    uint32_t sD1, sD2;
    uint8_t d1count, d2count;

    {
        WITH_SEMAPHORE(_sem);

        if (_accum.d1_count == 0) {
            return;
        }

        sD1 = _accum.s_D1;
        sD2 = _accum.s_D2;
        d1count = _accum.d1_count;
        d2count = _accum.d2_count;
        memset(&_accum, 0, sizeof(_accum));
    }

    if (d1count != 0) {
        _D1 = ((float)sD1) / d1count;
    }
    if (d2count != 0) {
        _D2 = ((float)sD2) / d2count;
    }

    switch (_rscmrne_type) {  // rscmrne_type = BARO_015PASE3
    case BARO_015PASE3:
        break;
    case BARO_015PASE3_2:
         _calculate_rscmrne015();
        break;
    }
}

void AP_Baro_RSCMRNE015PASE3_2::_calculate_rscmrne015(void)
{
    CompReturn_Struct compStatus = Compensate_Pressure(_D1, _D2);

    float pressure_ = compStatus.f32PressureOutput * 68.94757f * 100.0f;

    uint32_t u32Temperature = _D2;
    static int32_t s32Temperature = 0;

    if(u32Temperature < 16384u)
    {
        /*! If the Input Temperature is negative */
        /*! Check if Pressure Input falls in 14bit signed negative data boundary */
        if(u32Temperature > 8191u)
        {
            s32Temperature = (u32Temperature - 16384u);
        }
        else
        {
            s32Temperature = u32Temperature;
        }
    } else {    // input temperature out of range
        // keep last s32Temperature value.
        // do nothing.
    }

    float temperature_ = s32Temperature * 0.03125f;

    if(compStatus.CompStatus == PRESSURE_VALID)
    {
        _copy_to_frontend(_instance, pressure_, temperature_);
    }
}
