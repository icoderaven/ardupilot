/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifdef USERHOOK_INIT
void userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

#ifdef USERHOOK_FASTLOOP
void userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void userhook_50Hz()
{
    // put your 50Hz code here
    //Initiate disarming of motors if channel 7 is kicked to High
    if(g.rc_7.radio_in > 1600)
    {
        ap.rc_override_active = false;
        init_disarm_motors();
    }
    //Also keep the  throttle higher than 0 to avoid the disarm on mode change issue
    if(is_RPY){
        g.rc_3.control_in = 100;
    }
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void userhook_SlowLoop()
{
    // put your 3.3Hz code here
    if( millis() - last_heartbeat_ms > 1000) //i.e. if we haven't heard in 1 seconds
    {
       hal.rcin->clear_overrides(); //Stop listening to the GCS
    }
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif
