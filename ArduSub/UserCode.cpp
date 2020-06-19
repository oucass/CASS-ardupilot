#include "Sub.h"

#ifdef USERHOOK_INIT
void Sub::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

#ifdef USERHOOK_FASTLOOP
void Sub::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USER_VPBATT_LOOP
void Sub::user_vpbatt_monitor()
{
    // put your 50Hz code here
}
#endif

#ifdef USER_TEMPERATURE_LOOP
void Sub::user_temperature_logger()
{
    // put your 10Hz code here
}
#endif

#ifdef USER_HUMIDITY_LOOP
void Sub::user_humidity_logger()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USER_WVANE_LOOP
void Sub::user_wvane_logger()
{
    // put your 1Hz code here
}
#endif
