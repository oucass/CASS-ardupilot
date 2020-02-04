#include "AC_CASS_HYT271.h"
#include <utility>
#include <stdio.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;

AC_CASS_HYT271::AC_CASS_HYT271() :
    _dev(nullptr),
    _temperature(0),
    _humidity(0),
    _healthy(false)
{
}

bool AC_CASS_HYT271::init(uint8_t busId, uint8_t i2cAddr)
{

    sem = hal.util->new_semaphore();

    // Bus 0 is for Pixhawk 2.1 I2C and Bus 1 is for Pixhawk 1 and PixRacer I2C
    _dev = std::move(hal.i2c_mgr->get_device(busId, i2cAddr));
    if (!_dev) {
        printf("HYT271 device is null!");
        return false;
    }

    if (!_dev->get_semaphore()->take(0)) {
        AP_HAL::panic("PANIC: HYT271: failed to take serial semaphore for init");
    }

    _dev->set_retries(10);

    if (!_measure()) {
        printf("HYT271 read failed");
        _dev->get_semaphore()->give();
        return false;
    }

    // lower retries for run
    _dev->set_retries(2);

    _dev->get_semaphore()->give();

    /* Request 25Hz update */
    // Max conversion time is 12 ms
    _dev->register_periodic_callback(70000,
                                     FUNCTOR_BIND_MEMBER(&AC_CASS_HYT271::_timer, void));
    return true;
}

void AC_CASS_HYT271::set_i2c_addr(uint8_t addr){
    if (_dev) {
        _dev->set_address(addr);
    }
}

bool AC_CASS_HYT271::_measure()
{
    uint8_t cmd = 0x00;
        if (!_dev->transfer(&cmd, 1, nullptr, 0)) {
            return false;
        }
        return true;
}

bool AC_CASS_HYT271::_collect(float &hum, float &temp)
{
    uint8_t data[4];
    int16_t raw;
    // Read sensors
    if (!_dev->transfer(nullptr, 0, data, sizeof(data))) {
        return false;
    }

    // Verify data with the checksum
    if ((data[0] & 0x40) == 0x40){
        return false;
    }

    // Bit shift and convert to floating point number
    raw = (data[0] << 8) | data[1];
    raw = raw & 0x3FFF;

    if(sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)){

        hum = (100.0 / (powf(2,14) - 1)) * (float)raw;

        data[3] = (data[3] >> 2);
        raw = (data[2] << 6) | data[3];
        temp = (165.0 / (powf(2,14) - 1)) * (float)raw + 233.15f;

        sem->give(); 
    }

    return true; 
}

void AC_CASS_HYT271::_timer(void)
{
        _healthy = _collect(_humidity, _temperature);   // Retreive data from the sensor
        _measure();                                     // Request a new measurement to the sensor
}