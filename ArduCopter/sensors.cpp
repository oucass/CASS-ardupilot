#include "Copter.h"

void Copter::init_CASS_imet(){

    float coeff[4][3];

    // //CS2 BILLSONDE SENSORS
    // //IMET temp number 56236:
    // coeff[0][0] = 1.00993256e-03f;
    // coeff[0][1] = 2.62155349e-04f;
    // coeff[0][2] = 1.48648209e-07f;

    // //IMET temp number 45364:
    // coeff[1][0] = 1.01307391e-03f;
    // coeff[1][1] = 2.61020114e-04f;
    // coeff[1][2] = 1.52660155e-07f;

    // //IMET temp number 56220:
    // coeff[2][0] = 1.01704202e-03f;
    // coeff[2][1] = 2.61212821e-04f;
    // coeff[2][2] = 1.51670179e-07f;

    // //IMET temp number none:
    // coeff[3][0] = 1.0;
    // coeff[3][1] = 1.0;
    // coeff[3][2] = 1.0;
    
    //CS2.5 TONYSONDE SENSORS
    //IMET temp number 57562:
    coeff[0][0] = 1.02777010e-03f;
    coeff[0][1] = 2.59349232e-04f;
    coeff[0][2] = 1.56043078e-07f;

    //IMET temp number 57563:
    coeff[1][0] = 9.91077399e-04f;
    coeff[1][1] = 2.64646362e-04f;
    coeff[1][2] = 1.43596294e-07f;

    //IMET temp number 58821:
    coeff[2][0] = 1.00786813e-03f;
    coeff[2][1] = 2.61722397e-04f;
    coeff[2][2] = 1.48476183e-07f;

    //IMET temp number none:
    coeff[3][0] = 1.0;
    coeff[3][1] = 1.0;
    coeff[3][2] = 1.0;

    // //CS2.5 CHRISSONDE SENSORS
    // //IMET temp number 57549:
    // coeff[0][0] = 1.01991700e-03f;
    // coeff[0][1] = 2.60780405e-04f;
    // coeff[0][2] = 1.52712244e-07f;

    // //IMET temp number 58814:
    // coeff[1][0] = 1.01248342e-03f;
    // coeff[1][1] = 2.61214144e-04f;
    // coeff[1][2] = 1.50257257e-07f;

    // //IMET temp number 58822:
    // coeff[2][0] = 1.00761568e-03f;
    // coeff[2][1] = 2.61899525e-04f;
    // coeff[2][2] = 1.49075281e-07f;

    // //IMET temp number none:
    // coeff[3][0] = 1.0;
    // coeff[3][1] = 1.0;
    // coeff[3][2] = 1.0;

    // //CS2.5 DEREKSONDE SENSORS
    // //IMET temp number 57555:
    // coeff[0][0] = 1.02009822e-03f;
    // coeff[0][1] = 2.60475129e-04f;
    // coeff[0][2] = 1.54057907e-07f;

    // //IMET temp number 57564:
    // coeff[1][0] = 1.01554852e-03f;
    // coeff[1][1] = 2.61642038e-04f;
    // coeff[1][2] = 1.50140249e-07f;

    // //IMET temp number 57565:
    // coeff[2][0] = 1.02812678e-03f;
    // coeff[2][1] = 2.59630119e-04f;
    // coeff[2][2] = 1.55078895e-07f;

    // //IMET temp number none:
    // coeff[3][0] = 1.0;
    // coeff[3][1] = 1.0;
    // coeff[3][2] = 1.0;

    // //RED COPTERSONDE SENSORS
    // //IMET temp number 48623:
    // coeff[0][0] = 1.00733068f * (float)pow(10, -3);
    // coeff[0][1] = 2.62299300f * (float)pow(10, -4);
    // coeff[0][2] = 1.48361439f * (float)pow(10, -7);

    // //IMET temp number 48627:
    // coeff[1][0] = 1.00097308f * (float)pow(10, -3);
    // coeff[1][1] = 2.62806129f * (float)pow(10, -4);
    // coeff[1][2] = 1.46350112f * (float)pow(10, -7);

    // //IMET temp number 45363:
    // coeff[2][0] = 9.93118592f * (float)pow(10, -4);
    // coeff[2][1] = 2.63743049f * (float)pow(10, -4);
    // coeff[2][2] = 1.47415476f * (float)pow(10, -7);

    // //IMET temp number 48622:
    // coeff[3][0] = 1.00880279f * (float)pow(10, -3);
    // coeff[3][1] = 2.61500024f * (float)pow(10, -4);
    // coeff[3][2] = 1.49421629f * (float)pow(10, -7);

    // Initialize and set I2C addresses
    uint8_t deafult_i2cAddr = 0x48;
    uint8_t busId = 0;
    for(uint8_t i=0; i<4; i++){
        CASS_Imet[i].init(busId,deafult_i2cAddr + i);
    }
    // Set sensor coefficients
    CASS_Imet[0].set_sensor_coeff(coeff[0]);
    CASS_Imet[1].set_sensor_coeff(coeff[1]);
    CASS_Imet[2].set_sensor_coeff(coeff[2]);
    CASS_Imet[3].set_sensor_coeff(coeff[3]);
}

void Copter::init_CASS_hyt271(){
    // Initialize and set I2C addresses
    uint8_t deafult_i2cAddr = 0x10;
    uint8_t busId = 0;
    for(uint8_t i=0; i<4; i++){
        CASS_HYT271[i].init(busId,deafult_i2cAddr + i);
    }
}

// return barometric altitude in centimeters
void Copter::read_barometer(void)
{
    barometer.update();

    baro_alt = barometer.get_altitude() * 100.0f;

    motors->set_air_density_ratio(barometer.get_air_density_ratio());
}

void Copter::init_rangefinder(void)
{
#if RANGEFINDER_ENABLED == ENABLED
   rangefinder.set_log_rfnd_bit(MASK_LOG_CTUN);
   rangefinder.init(ROTATION_PITCH_270);
   rangefinder_state.alt_cm_filt.set_cutoff_frequency(RANGEFINDER_WPNAV_FILT_HZ);
   rangefinder_state.enabled = rangefinder.has_orientation(ROTATION_PITCH_270);

   // upward facing range finder
   rangefinder_up_state.alt_cm_filt.set_cutoff_frequency(RANGEFINDER_WPNAV_FILT_HZ);
   rangefinder_up_state.enabled = rangefinder.has_orientation(ROTATION_PITCH_90);
#endif
}

// return rangefinder altitude in centimeters
void Copter::read_rangefinder(void)
{
#if RANGEFINDER_ENABLED == ENABLED
    rangefinder.update();

#if RANGEFINDER_TILT_CORRECTION == ENABLED
    const float tilt_correction = MAX(0.707f, ahrs.get_rotation_body_to_ned().c.z);
#else
    const float tilt_correction = 1.0f;
#endif

    // iterate through downward and upward facing lidar
    struct {
        RangeFinderState &state;
        enum Rotation orientation;
    } rngfnd[2] = { {rangefinder_state, ROTATION_PITCH_270}, {rangefinder_up_state, ROTATION_PITCH_90}};

    for (uint8_t i=0; i < ARRAY_SIZE(rngfnd); i++) {
        // local variables to make accessing simpler
        RangeFinderState &rf_state = rngfnd[i].state;
        enum Rotation rf_orient = rngfnd[i].orientation;

        // update health
        rf_state.alt_healthy = ((rangefinder.status_orient(rf_orient) == RangeFinder::Status::Good) &&
                                (rangefinder.range_valid_count_orient(rf_orient) >= RANGEFINDER_HEALTH_MAX));

        // tilt corrected but unfiltered, not glitch protected alt
        rf_state.alt_cm = tilt_correction * rangefinder.distance_cm_orient(rf_orient);

        // glitch handling.  rangefinder readings more than RANGEFINDER_GLITCH_ALT_CM from the last good reading
        // are considered a glitch and glitch_count becomes non-zero
        // glitches clear after RANGEFINDER_GLITCH_NUM_SAMPLES samples in a row.
        // glitch_cleared_ms is set so surface tracking (or other consumers) can trigger a target reset
        const int32_t glitch_cm = rf_state.alt_cm - rf_state.alt_cm_glitch_protected;
        if (glitch_cm >= RANGEFINDER_GLITCH_ALT_CM) {
            rf_state.glitch_count = MAX(rf_state.glitch_count+1, 1);
        } else if (glitch_cm <= -RANGEFINDER_GLITCH_ALT_CM) {
            rf_state.glitch_count = MIN(rf_state.glitch_count-1, -1);
        } else {
            rf_state.glitch_count = 0;
            rf_state.alt_cm_glitch_protected = rf_state.alt_cm;
        }
        if (abs(rf_state.glitch_count) >= RANGEFINDER_GLITCH_NUM_SAMPLES) {
            // clear glitch and record time so consumers (i.e. surface tracking) can reset their target altitudes
            rf_state.glitch_count = 0;
            rf_state.alt_cm_glitch_protected = rf_state.alt_cm;
            rf_state.glitch_cleared_ms = AP_HAL::millis();
        }

        // filter rangefinder altitude
        uint32_t now = AP_HAL::millis();
        const bool timed_out = now - rf_state.last_healthy_ms > RANGEFINDER_TIMEOUT_MS;
        if (rf_state.alt_healthy) {
            if (timed_out) {
                // reset filter if we haven't used it within the last second
                rf_state.alt_cm_filt.reset(rf_state.alt_cm);
            } else {
                rf_state.alt_cm_filt.apply(rf_state.alt_cm, 0.05f);
            }
            rf_state.last_healthy_ms = now;
        }

        // send downward facing lidar altitude and health to waypoint navigation library
        if (rf_orient == ROTATION_PITCH_270) {
            if (rangefinder_state.alt_healthy || timed_out) {
                wp_nav->set_rangefinder_alt(rangefinder_state.enabled, rangefinder_state.alt_healthy, rangefinder_state.alt_cm_filt.get());
            }
        }
    }

#else
    // downward facing rangefinder
    rangefinder_state.enabled = false;
    rangefinder_state.alt_healthy = false;
    rangefinder_state.alt_cm = 0;

    // upward facing rangefinder
    rangefinder_up_state.enabled = false;
    rangefinder_up_state.alt_healthy = false;
    rangefinder_up_state.alt_cm = 0;
#endif
}

// return true if rangefinder_alt can be used
bool Copter::rangefinder_alt_ok()
{
    return (rangefinder_state.enabled && rangefinder_state.alt_healthy);
}

// return true if rangefinder_alt can be used
bool Copter::rangefinder_up_ok()
{
    return (rangefinder_up_state.enabled && rangefinder_up_state.alt_healthy);
}

/*
  update RPM sensors
 */
void Copter::rpm_update(void)
{
#if RPM_ENABLED == ENABLED
    rpm_sensor.update();
    if (rpm_sensor.enabled(0) || rpm_sensor.enabled(1)) {
        if (should_log(MASK_LOG_RCIN)) {
            logger.Write_RPM(rpm_sensor);
        }
    }
#endif
}

// initialise optical flow sensor
void Copter::init_optflow()
{
#if OPTFLOW == ENABLED
    // initialise optical flow sensor
    optflow.init(MASK_LOG_OPTFLOW);
#endif      // OPTFLOW == ENABLED
}

void Copter::compass_cal_update()
{
    compass.cal_update();

    if (hal.util->get_soft_armed()) {
        return;
    }

    static uint32_t compass_cal_stick_gesture_begin = 0;

    if (compass.is_calibrating()) {
        if (channel_yaw->get_control_in() < -4000 && channel_throttle->get_control_in() > 900) {
            compass.cancel_calibration_all();
        }
    } else {
        bool stick_gesture_detected = compass_cal_stick_gesture_begin != 0 && !motors->armed() && channel_yaw->get_control_in() > 4000 && channel_throttle->get_control_in() > 900;
        uint32_t tnow = millis();

        if (!stick_gesture_detected) {
            compass_cal_stick_gesture_begin = tnow;
        } else if (tnow-compass_cal_stick_gesture_begin > 1000*COMPASS_CAL_STICK_GESTURE_TIME) {
#ifdef CAL_ALWAYS_REBOOT
            compass.start_calibration_all(true,true,COMPASS_CAL_STICK_DELAY,true);
#else
            compass.start_calibration_all(true,true,COMPASS_CAL_STICK_DELAY,false);
#endif
        }
    }
}

void Copter::accel_cal_update()
{
    if (hal.util->get_soft_armed()) {
        return;
    }
    ins.acal_update();
    // check if new trim values, and set them
    float trim_roll, trim_pitch;
    if(ins.get_new_trim(trim_roll, trim_pitch)) {
        ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
    }

#ifdef CAL_ALWAYS_REBOOT
    if (ins.accel_cal_requires_reboot()) {
        hal.scheduler->delay(1000);
        hal.scheduler->reboot(false);
    }
#endif
}

// initialise proximity sensor
void Copter::init_proximity(void)
{
#if PROXIMITY_ENABLED == ENABLED
    g2.proximity.init();
#endif
}

// init visual odometry sensor
void Copter::init_visual_odom()
{
#if VISUAL_ODOMETRY_ENABLED == ENABLED
    g2.visual_odom.init();
#endif
}

// winch and wheel encoder initialisation
void Copter::winch_init()
{
#if WINCH_ENABLED == ENABLED
    g2.wheel_encoder.init();
    g2.winch.init(&g2.wheel_encoder);
#endif
}

// winch and wheel encoder update
void Copter::winch_update()
{
#if WINCH_ENABLED == ENABLED
    g2.wheel_encoder.update();
    g2.winch.update();
#endif
}
