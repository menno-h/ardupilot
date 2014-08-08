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
    
    //////////////////////////////////////////////////////////////////////////////////////////
    // (01/03/2014-Menno) // Decoding of pilot climb rate input with channel 3 in CRUISE flight mode     
    //////////////////////////////////////////////////////////////////////////////////////////
    int32_t CRUISE_ALTITUDE_MIN = -10;
    int32_t CRUISE_ALTITUDE_MAX = 100;
    
    if (control_mode==CRUISE) {
      
    int32_t old_altitude = control_cruise_altitude;  
    int16_t desired_climb_rate = get_cruise_climb_rate(g.rc_3.control_in);
        
    if (desired_climb_rate < 0 && old_altitude <= CRUISE_ALTITUDE_MIN) {
      control_cruise_climb_rate = 0;
      control_cruise_altitude = CRUISE_ALTITUDE_MIN;}
    else if (desired_climb_rate > 0 && old_altitude>= CRUISE_ALTITUDE_MAX) {
      control_cruise_climb_rate = 0;
      control_cruise_altitude = CRUISE_ALTITUDE_MAX;}
    else {
      control_cruise_climb_rate = desired_climb_rate;
      if ((desired_climb_rate<0 && !motors.limit.throttle_lower) || (desired_climb_rate>0 && !motors.limit.throttle_upper)){
         control_cruise_altitude += control_cruise_climb_rate*0.01;}      
      }
    }
    
    
    /////////
    //
    /////////
    
          
}
#endif

#ifdef USERHOOK_50HZLOOP
void userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_2HZLOOP // (04/08/2014-Menno)
void userhook_2Hz()
{
    airspeed_ratio_update();
}
#endif

#ifdef USERHOOK_20HZLOOP // (04/08/2014-Menno)
void userhook_20Hz()
{
    read_airspeed();
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
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif
