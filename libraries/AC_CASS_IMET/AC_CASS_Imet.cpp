#include "AC_CASS_Imet.h"
#include <utility>
#include <stdio.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;

AC_CASS_Imet::AC_CASS_Imet() :
    _dev(nullptr),
    _temperature(0),
    _resist(0),
    _healthy(false)
{
}

bool AC_CASS_Imet::init(uint8_t busId, uint8_t i2cAddr)
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
    _dev = std::move(hal.i2c_mgr->get_device(busId, i2cAddr));

    if (!_dev) {
        printf("IMET device is null!");
        return false;
    }
    WITH_SEMAPHORE(_sem);

    _dev->set_retries(10);

    if (!_config_read_thermistor()) {
        printf("IMET read failed");
        return false;
    }

    hal.scheduler->delay(5);

    if (!_config_read_source()) {
        printf("IMET read failed");
        return false;
    }

    hal.scheduler->delay(200);

    flag = false;
    adc_thermistor = 0;
    adc_source = 0;
    runs = 0;

    for(uint8_t i=0; i<3; i++){
        memset(coeff,1.0,sizeof(coeff));
    }

    _read_adc(adc_source);
    _config_read_thermistor();

    // lower retries for run
    _dev->set_retries(3);

    //_dev->get_semaphore()->give();

    /* Request 25Hz update */
    // Max conversion time is 12 ms
    _dev->register_periodic_callback(40000,
                                     FUNCTOR_BIND_MEMBER(&AC_CASS_Imet::_timer, void));
    return true;
}

void AC_CASS_Imet::set_i2c_addr(uint8_t addr){
    if (_dev) {
        _dev->set_address(addr);
    }
}

void AC_CASS_Imet::set_sensor_coeff(float *k){
    for(uint8_t i=0; i<3; i++){
        coeff[i] = k[i];
    }
}

bool AC_CASS_Imet::_config_read_thermistor()
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

bool AC_CASS_Imet::_config_read_source()
{
    struct PACKED {
            uint8_t reg;
            be16_t val;
        } config_pack;

        config_pack.reg = 0x01;
        config_pack.val = htobe16(config | ADS1015_REG_CONFIG_MUX_SINGLE_1);

        if (!_dev->transfer((uint8_t *)&config_pack, sizeof(config_pack), nullptr, 0)) {
            return false;
        }

        return true;
}

bool AC_CASS_Imet::_read_adc(float &value)
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

void AC_CASS_Imet::_timer(void)
{
    // Retreive data from sensor by I2C
    WITH_SEMAPHORE(_sem);       // semaphore for access to shared frontend data
    float temp = adc_source;
    if(flag == false){
        _healthy = _read_adc(adc_thermistor);
        runs += 1;
    }
    else{
        _healthy = _read_adc(temp);
        adc_source = (adc_source + temp)/2;
    }   

    // If data was collected, then calculate temperature and resistance
    if (_healthy) {
        _calculate(adc_source, adc_thermistor);
    }

    // After 20 samples, re-measure voltage source and update it.
    if(runs == 20) {
        _config_read_source();
        flag = true;
        runs = 0;
    }
    else{
        _config_read_thermistor();
        flag = false; 
    }
}

void AC_CASS_Imet::_calculate(float source, float thermistor)
{
    _resist = 64900.0f * (source / thermistor - 1.0f);
    //_volt = thermistor * 0.1875f; //store the true thermistor voltage
    //converts to temperature (kelvin)
    _temperature = 1.0f / (coeff[0] + coeff[1] * logf(_resist) + coeff[2] * powf(logf(_resist), 3));
}