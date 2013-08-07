/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifdef USERHOOK_INIT
void userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
     g.failsafe_gcs = FS_GCS_DISABLED;
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
        g.rc_3.control_in = 540;
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
    /**if( millis() - last_heartbeat_ms > 1000) //i.e. if we haven't heard in 1 seconds
    {
       hal.rcin->clear_overrides(); //Stop listening to the GCS
       //Also set is_RPY to false
       is_RPY = false;
       set_mode(STABILIZE);
    }*/
     if( millis() - last_cmd_time > 400 && is_RPY) //i.e. if we haven't heard in 1 seconds
    {
       //Clear the commanded roll pitch and yaw
       cmd_roll = 0;
       cmd_pitch = 0;
       cmd_yaw = 0;
       gcs_send_text_fmt( PSTR("Resetting command rpy, lct = %d, ct = %d"), last_cmd_time, millis());
       is_RPY = false;
    }
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void userhook_SuperSlowLoop()
{
    // put your 1Hz code here
    //gcs_send_text_fmt( PSTR("Throttle Control in %d :::"), g.rc_3.control_in);
}
#endif
