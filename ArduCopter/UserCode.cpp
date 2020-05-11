#include "Copter.h"
#include <utility>
#include <SRV_Channel/SRV_Channel.h>
#include <Filter/LowPassFilter2p.h>

//Humidity sensor Params
const int N_RH = 4;     //supports up to 4

//IMET sensor Params
const int N_imet = 4;   //supports up to 4

//Fan control params
//It will run the fan at SERVO_MAX, which can be set in the params list
uint16_t fan_pwm_on = 100;
uint16_t fan_pwm_off = 0;
bool _fan_status;

//Wind estimator Params
float _wind_speed, _wind_dir;
float R13, R23, R33;
float last_yrate;
bool high_wind_flag;
//g.wind_vane_wsA -> Coefficient A of the linear wind speed equation, from calibration
//g.wind_vane_wsB -> Coefficient B of the linear wind speed equation, from calibration
//g.wind_vane_min_roll -> Minimum roll angle that the wind vane will correct (too low and the copter will oscilate)
//g.wind_vane_fine_rate -> Maximum yaw angle rate at which the Copter will rotate
//g.wind_vane_fine_gain -> Wind vane gain: higher values will increase the resposivness

//Declare digital LPF
LowPassFilter2pFloat filt_thrvec_x;
LowPassFilter2pFloat filt_thrvec_y;
LowPassFilter2pFloat filt_thrvec_z;

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up

    //Initialize Wind estimator
    _wind_dir = 0.0f;  _wind_speed = 0.0f;
    R13 = 0.0f; R23 = 0.0f;
    last_yrate = 0;
    high_wind_flag = false;

    //Wind filter initialization
    if(g.wind_vane_cutoff < 0.06){
        //Min Fc = 0.06 for stable yaw
        filt_thrvec_x.set_cutoff_frequency(20,0.06);
        filt_thrvec_y.set_cutoff_frequency(20,0.06);
        filt_thrvec_z.set_cutoff_frequency(20,0.06);
    }
    else{
        //Initialize Butterworth filter
        filt_thrvec_x.set_cutoff_frequency(20,g.wind_vane_cutoff);
        filt_thrvec_y.set_cutoff_frequency(20,g.wind_vane_cutoff);
        filt_thrvec_z.set_cutoff_frequency(20,g.wind_vane_cutoff);
    }

    // Initialize Fan Control
    SRV_Channels::set_output_scaled(SRV_Channel::k_egg_drop, fan_pwm_off);
    _fan_status = false;
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    #if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    // Read Temperature, Resistance and Health. Write sensors packet into the SD card
    // Temperature Data Logger ///////////////////////////////////////////////////////////////////////////////////////////
    struct log_IMET pkt_temp = {
        LOG_PACKET_HEADER_INIT(LOG_IMET_MSG),
        time_stamp             : AP_HAL::micros64(),                //Store time in microseconds
        fan_status             : _fan_status,                       //Store Fan on/off status
        healthy1               : copter.CASS_Imet[0].healthy(),     //Store sensor health
        healthy2               : copter.CASS_Imet[1].healthy(),
        healthy3               : copter.CASS_Imet[2].healthy(),
        healthy4               : copter.CASS_Imet[3].healthy(),
        temperature1           : copter.CASS_Imet[0].temperature(), //Store iMet Temperature
        temperature2           : copter.CASS_Imet[1].temperature(),
        temperature3           : copter.CASS_Imet[2].temperature(),
        temperature4           : copter.CASS_Imet[3].temperature(),
        resist1                : copter.CASS_Imet[0].resistance(),  //Store iMet bead resistence
        resist2                : copter.CASS_Imet[1].resistance(),
        resist3                : copter.CASS_Imet[2].resistance(),
        resist4                : copter.CASS_Imet[3].resistance()
    }; 
    #else
        float alt; float simT;
        copter.ahrs.get_relative_position_D_home(alt);
        alt = -1*alt;  
        if(alt < 900){
            simT = 2.99e-11f*powf(alt,4) - 3.70454e-8*powf(alt,3) - 3.86806e-6*powf(alt,2) + 1.388511e-2*alt + 287.66;
        }
        else{
            simT = 289.64f;
        }
        uint32_t m = AP_HAL::millis();
        // Write simulated sensors packet into the SD card
        struct log_IMET pkt_temp = {
            LOG_PACKET_HEADER_INIT(LOG_IMET_MSG),
            time_stamp             : AP_HAL::micros64(),
            fan_status             : _fan_status,
            healthy1               : copter.CASS_Imet[0].healthy(),
            healthy2               : copter.CASS_Imet[1].healthy(),
            healthy3               : copter.CASS_Imet[2].healthy(),
            healthy4               : copter.CASS_Imet[3].healthy(),
            temperature1           : simT + sinf(0.001f*float(m)) * 0.002f,
            temperature2           : simT + sinf(0.0007f*float(m)) * 0.003f,
            temperature3           : simT + sinf(0.0003f*float(m)) * 0.0025f,
            temperature4           : simT + sinf(0.0005f*float(m)) * 0.0028f,
            resist1                : -356.9892f*simT + 110935.3763f + sinf(0.001f*float(m)) * 0.002f,
            resist2                : -356.9892f*simT + 110935.3763f + sinf(0.0007f*float(m)) * 0.003f,
            resist3                : -356.9892f*simT + 110935.3763f + sinf(0.0003f*float(m)) * 0.0025f,
            resist4                : -356.9892f*simT + 110935.3763f + sinf(0.0005f*float(m)) * 0.0028f
        };
    #endif
    copter.DataFlash.WriteBlock(&pkt_temp, sizeof(pkt_temp));   //Send package to SD card
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
     #if CONFIG_HAL_BOARD != HAL_BOARD_SITL
        // Read Rel. Humidity, Temperature and Health. Write sensors packet into the SD card
        // Relative Humidity Data Logger ///////////////////////////////////////////////////////////////////////////////////////////
        struct log_RH pkt_RH = {
            LOG_PACKET_HEADER_INIT(LOG_RH_MSG),
            time_stamp             : AP_HAL::micros64(),                        //Store time in microseconds
            healthy1               : copter.CASS_HYT271[0].healthy(),           //Store senors health
            healthy2               : copter.CASS_HYT271[1].healthy(),
            healthy3               : copter.CASS_HYT271[2].healthy(),
            healthy4               : copter.CASS_HYT271[3].healthy(),
            humidity1              : copter.CASS_HYT271[0].relative_humidity(), //Store Rel. humidity
            humidity2              : copter.CASS_HYT271[1].relative_humidity(),
            humidity3              : copter.CASS_HYT271[2].relative_humidity(),
            humidity4              : copter.CASS_HYT271[3].relative_humidity(),
            RHtemp1                : copter.CASS_HYT271[0].temperature(),       //Store temperature
            RHtemp2                : copter.CASS_HYT271[1].temperature(),
            RHtemp3                : copter.CASS_HYT271[2].temperature(),
            RHtemp4                : copter.CASS_HYT271[3].temperature()
        };
    #else
        float alt; float simH;
        copter.ahrs.get_relative_position_D_home(alt);
        alt = -1*alt; 
        if(alt < 900){
            simH = -2.44407e-10f*powf(alt,4) + 3.88881064e-7*powf(alt,3) - 1.41943e-4*powf(alt,2) - 2.81895e-2*alt + 51.63;
        }
        else{
            simH = 34.44f;
        }
        uint32_t m = AP_HAL::millis();
        // Write sensors packet into the SD card
        struct log_RH pkt_RH = {
            LOG_PACKET_HEADER_INIT(LOG_RH_MSG),
            time_stamp             : AP_HAL::micros64(),
            healthy1               : copter.CASS_HYT271[0].healthy(),
            healthy2               : copter.CASS_HYT271[1].healthy(),
            healthy3               : copter.CASS_HYT271[2].healthy(),
            healthy4               : copter.CASS_HYT271[3].healthy(),
            humidity1              : simH + sinf(0.1f*float(m)) * 0.02f,
            humidity2              : simH + sinf(0.3f*float(m)) * 0.03f,
            humidity3              : simH + sinf(0.5f*float(m)) * 0.025f,
            humidity4              : simH + sinf(0.7f*float(m)) * 0.035f,
            RHtemp1                : 298.15f + sinf(0.0003f*m) * 2.0f,
            RHtemp2                : 298.15f + sinf(0.0004f*m) * 2.0f,
            RHtemp3                : 298.15f + sinf(0.0005f*m) * 2.0f,
            RHtemp4                : 298.15f + sinf(0.0006f*m) * 2.0f
        };
    #endif
    copter.DataFlash.WriteBlock(&pkt_RH, sizeof(pkt_RH));   //Send package to SD card
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    //Run algo after Copter takes off
    if(!ap.land_complete && copter.position_ok()){ // !arming.is_armed(), !ap.land_complete, motors->armed()

        //Fan Control ////////////////////////////////////////////////////////////////////////////////////////

        //Get AGL altitude in cm
        float alt;
        copter.ahrs.get_relative_position_D_home(alt);
        alt = -100.0f*alt;
   
        //Smart fan on/off logic
        if(alt > 185.0f && SRV_Channels::get_output_scaled(SRV_Channel::k_egg_drop) < 50){
            SRV_Channels::set_output_scaled(SRV_Channel::k_egg_drop, fan_pwm_on);
            _fan_status = true;
            gcs().send_text(MAV_SEVERITY_INFO, "Scoop Fan activated");
            #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                printf("FAN ON \n");  //printf("PWM: %5.2f \n",var); //for debugging
            #endif
        }
        else{
            if(alt < 140.0f && SRV_Channels::get_output_scaled(SRV_Channel::k_egg_drop) > 50){
                SRV_Channels::set_output_scaled(SRV_Channel::k_egg_drop, fan_pwm_off);
                _fan_status = false;
                #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                    printf("FAN OFF \n");  
                #endif
            }
        }

        //Wind Estimator Algorithm //////////////////////////////////////////////////////////////////////////

        //Get thurst vector elements from the rotation matrix
        R13 = -1*copter.ahrs.get_rotation_body_to_ned().a.z;
        R23 = -1*copter.ahrs.get_rotation_body_to_ned().b.z;
        R33 = -1*copter.ahrs.get_rotation_body_to_ned().c.z;

        //Apply Butterworth LPF on each element
        float thrvec_x, thrvec_y, thrvec_z;
        thrvec_x = filt_thrvec_x.apply(R13);
        thrvec_y = filt_thrvec_y.apply(R23);
        thrvec_z = filt_thrvec_z.apply(R33);

        //Determine wind direction by trigonometry (thrust vector tilt)
        float wind_psi = fmodf(atan2f(thrvec_y,thrvec_x),2*M_PI)*180.0f/M_PI;

        //Get current target roll from the attitude controller
        float troll = copter.wp_nav->get_roll()/100.0f;

        //Define a dead zone around zero roll
        if(fabsf(troll) < g.wind_vane_min_roll){ last_yrate = 0; }

        //Convert roll magnitude into desired yaw rate
        float yrate = constrain_float((troll/5.0f)*g.wind_vane_fine_gain,-g.wind_vane_fine_rate,g.wind_vane_fine_rate);
        last_yrate = 0.98f*last_yrate + 0.02f*yrate; //1st order LPF

        //For large compensation use "wind_psi" estimator, for fine adjusments use "yrate" estimator
        if(fabsf(troll)<g.wind_vane_min_roll){
            //Set WVANE_MIN_ROLL to zero to disable the "yrate" estimator
            //Output "y_rate" estimator
            _wind_dir = copter.cass_wind_direction/100.0f + last_yrate;
            _wind_dir = wrap_360_cd(_wind_dir*100.0f);
        }
        else{ 
            //Output "wind_psi" estimator
            _wind_dir = wrap_360_cd(wind_psi*100.0f);
            last_yrate = 0;
        }

        //Estimate wind speed with filtered parameters
        float thrvec_xy = safe_sqrt(thrvec_x*thrvec_x + thrvec_y*thrvec_y);
        _wind_speed = g.wind_vane_wsA * safe_sqrt(fabsf(thrvec_xy/thrvec_z)) + g.wind_vane_wsB;
        _wind_speed = _wind_speed < 0 ? 0.0f : _wind_speed;

        //Get current velocity and horizontal distance to next waypoint
        Vector3f vel_xyz = copter.inertial_nav.get_velocity();
        float speed_xy = norm(vel_xyz.x,vel_xyz.y); // cm/s
        float dist_to_wp = copter.wp_nav->get_wp_distance_to_destination(); // cm (horizontally)

        //Wind vane is active when flying horizontally steady and wind speed is perceivable
        if(speed_xy < 120.0f && dist_to_wp < 500 && _wind_speed > 0.6f){
            //Min altitude at which the yaw command is sent
            if(alt>400.0f){
                //Send estimated wind direction to the autopilot
                copter.cass_wind_direction = _wind_dir;
                copter.cass_wind_speed = _wind_speed;
            }
            else{
                //Send neutral values
                copter.cass_wind_direction = copter.wp_nav->get_yaw();
                copter.cass_wind_speed = 0.0f;
            }
        }
        else{
            //Reset 1st order filter
            last_yrate = 0.0f;
        }

        //Switch to RTL automatically if wind speed is too high (in m/s)
        //If tolerance is set to zero then function is disabled
        if(!is_zero(g.wind_vane_spd_tol)){
            if(_wind_speed > g.wind_vane_spd_tol && high_wind_flag == false){
                gcs().send_text(MAV_SEVERITY_WARNING, "Switched to RTL due to very high wind");
                copter.set_mode(RTL, MODE_REASON_UNKNOWN);
                high_wind_flag = true;
            }
            else if(_wind_speed < (g.wind_vane_spd_tol - 3.0f) && high_wind_flag == true){
                high_wind_flag = false;
                gcs().send_text(MAV_SEVERITY_INFO, "High wind warning cleared");
            }
        }
        
    }
    else{
        //Reset all global parameters to default values
        copter.cass_wind_direction = (float)copter.initial_armed_bearing;
        copter.cass_wind_speed = 0.0f;
        SRV_Channels::set_output_scaled(SRV_Channel::k_egg_drop, fan_pwm_off);
        last_yrate = 0;
        _fan_status = false;
        filt_thrvec_x.reset();
        filt_thrvec_y.reset();
        filt_thrvec_z.reset();
    }

    //Wind Data Logger ///////////////////////////////////////////////////////////////////////////////////////////

    // Write wind direction packet into the SD card
    struct log_WIND pkt_wind_est = {
        LOG_PACKET_HEADER_INIT(LOG_WIND_MSG),
        time_stamp             : AP_HAL::micros64(),
        _wind_dir              : _wind_dir/100,
        _wind_speed            : _wind_speed,
        _R13                   : R13,
        _R23                   : R23,
        _R33                   : R33
    };
    copter.DataFlash.WriteBlock(&pkt_wind_est, sizeof(pkt_wind_est));

    // float data[5] = {0};
    // data[0] = _wind_dir/100.0f;
    // data[1] = _wind_speed;
    // data[2] = Vel_xy;
    // data[3] = var_wind_dir;
    // data[4] = alt;
    // copter.send_cass_data(3, data, 5);
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(uint8_t ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(uint8_t ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(uint8_t ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif
