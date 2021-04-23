#include "UserParameters.h"

// "USR" + 13 chars remaining for param name 
const AP_Param::GroupInfo UserParameters::var_info[] = {
    
    // Put your parameters definition here
    // Note the maximum length of parameter name is 13 chars
    AP_GROUPINFO("_INT8", 0, UserParameters, _int8, 0),
    AP_GROUPINFO("_INT16", 1, UserParameters, _int16, 0),
    AP_GROUPINFO("_FLOAT", 2, UserParameters, _float, 0),

    //CASS custom parameters publish
    // Weather sensors
    AP_GROUPINFO("_SENSOR1", 3, UserParameters, _sensor1, 0),
    AP_GROUPINFO("_SENSOR2", 4, UserParameters, _sensor2, 0),
    AP_GROUPINFO("_SENSOR3", 5, UserParameters, _sensor3, 0),
    AP_GROUPINFO("_SENSOR4", 6, UserParameters, _sensor4, 0),
    AP_GROUPINFO("_SENSOR5", 7, UserParameters, _sensor5, 0),
    AP_GROUPINFO("_SENSOR6", 8, UserParameters, _sensor6, 0),
    AP_GROUPINFO("_SENSOR7", 9, UserParameters, _sensor7, 0),
    AP_GROUPINFO("_SENSOR8", 10, UserParameters, _sensor8, 0),
    // Wind Vane
    AP_GROUPINFO("_WV_MINRLL", 11, UserParameters, wind_vane_min_roll, 0),
    AP_GROUPINFO("_WV_FRATE", 12, UserParameters, wind_vane_fine_rate, 1),
    AP_GROUPINFO("_WV_FGAIN", 13, UserParameters, wind_vane_fine_gain, 1),
    AP_GROUPINFO("_WV_CUTOFF", 14, UserParameters, wind_vane_cutoff, 0.06f),
    AP_GROUPINFO("_WV_WSA", 15, UserParameters, wind_vane_wsA, 30.0f),
    AP_GROUPINFO("_WV_WSB", 16, UserParameters, wind_vane_wsB, 9.0f),
    AP_GROUPINFO("_WV_SPDTOL", 17, UserParameters, wind_vane_spd_tol, 19.0f),
    AP_GROUPINFO("_WV_RTLEN", 18, UserParameters, wind_vane_enabled, 1.0f),
    AP_GROUPINFO("_WV_FS", 19, UserParameters, wind_vane_fs, 10.0f),
    // VPBatt
    AP_GROUPINFO("_VPBATT_EN", 20, UserParameters, vpbatt_enabled, 1.0f),
    AP_GROUPINFO("_VPBATT_RES", 21, UserParameters, vpbatt_reserve, 30.0f),
    AP_GROUPINFO("_VPBATT_WH", 22, UserParameters, vpbatt_wh, 89.0f),
    // Mission Auto-generator
    AP_GROUPINFO("_AUTOVP_ALT", 23, UserParameters, autovp_max_altitude, 120.0f),

    AP_GROUPEND
};
