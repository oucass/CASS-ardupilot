#include "AC_CASS_03.h"
#include <utility>
#include <stdio.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;

AC_CASS_03::AC_CASS_03() :
    _dev(nullptr),
    _ppb_ozone(0),
    _healthy(false)
{
}

bool AC_CASS_03::init(uint8_t busId, uint8_t i2cAddr)
{

    config = ADS1015_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)
             ADS1015_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)
             ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
             ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
             ADS1015_REG_CONFIG_DR_1600SPS   | // 1600 samples per second (default)
             ADS1015_REG_CONFIG_MODE_SINGLE  | // Single-shot mode (default)
             ADS1015_REG_CONFIG_PGA_6_144V   | // Set PGA/voltage range
             ADS1015_REG_CONFIG_OS_SINGLE;     // Set start single-conversion bit

    // Bus 0 is for Pixhawk 2.1 I2C and Bus 1 is for Pixhawk 1 and PixRacer I2C
    hal.scheduler->delay(200);
    _dev = std::move(hal.i2c_mgr->get_device(busId, i2cAddr));
    
    WITH_SEMAPHORE(_dev->get_semaphore());
    if (!_dev) {
        printf("O3 device is null!");
        return false;
    }

    _dev->set_retries(10);

    if (!_config_read_ozone()) {
        printf("O3 read failed");
        return false;
    }

    hal.scheduler->delay(200);

    _read_adc(adc_ozone);

    // lower retries for run
    _dev->set_retries(3);

    //_dev->get_semaphore()->give();

    /* Request 1Hz update */
    _dev->register_periodic_callback(80000,
                                     FUNCTOR_BIND_MEMBER(&AC_CASS_03::_timer, void));
    return true;
}

void AC_CASS_03::set_i2c_addr(uint8_t addr){
    if (_dev) {
        _dev->set_address(addr);
    }
}

bool AC_CASS_03::_config_read_ozone()
{
    struct PACKED {
            uint8_t reg;
            be16_t val;
        } config_pack;

        config_pack.reg = 0x01;
        config_pack.val = htobe16(config | ADS1015_REG_CONFIG_MUX_SINGLE_0);

        if (!_dev->transfer((uint8_t *)&config_pack, sizeof(config_pack), nullptr, 0)) {
            return false;
        }

        return true;
}

bool AC_CASS_03::_read_adc(float &value)
{
    uint8_t status[2];
    uint8_t data[2];

    if (!_dev->read_registers(0x01, status, sizeof(status))) {
        return false;
    }

    /* check rdy bit */
    if ((status[1] & 0x80) != 0x80 ) {
        return false;
    }

    // An easier way of reading registers
    if (!_dev->read_registers(0x00, data, sizeof(data))) {
        return false;
    }

    value = (float)((data[0] << 8) | data[1]);
    return true;
}

void AC_CASS_03::_timer(void)
{
    // Retreive data from sensor by I2C
    WITH_SEMAPHORE(_sem);       // semaphore for access to shared frontend data
    _healthy = _read_adc(adc_ozone);

    // If data was collected, then calculate ozone in ppm
    if (_healthy) {
        _calculate(adc_ozone);
    }
}

void AC_CASS_03::_calculate(float adc)
{
    float _volt = adc * 0.1875f; //store the true voltage
    //converts to ppm concentration
    _ppb_ozone = _volt*0.03f;
}