#ifndef __A37342_002_CONFIG_H
#define __A37342_002_CONFIG_H



// FAULT CONFIGURATION

#define LINAC_2_5

#define _P1500_MAGNETX

#ifdef LINAC_2_5
// Configure the trip points for the 2.5 System

// Thermistor Fault Setup
#define COOLANT_TRIP_THERMISTOR_VOLTAGE          1544 //3936  // or 1544? This is the voltage (in millivolts) that indicates a temerperature of 36 Deg C.  NTC voltage less than this is over temp
#define CABINET_TRIP_THERMISTOR_VOLTAGE          1184 //3040  // or 1184? This is the voltage (in millivolts) that indicates a temerperature of 46 Deg C.  NTC voltage less than this is over temp
#define LINAC_TRIP_THERMISTOR_VOLTAGE            0     // This is the voltage (in millivolts) that indicates a temerperature of > 105 Deg C.  NTC voltage less than this is over temp
#define ANALOG_TEMPERATURE_TRIP_TIME             1000  // 10 Seconds

// Temperature Switch Fault Setup
#define TEMPERATURE_SWITCH_FILTER_TIME           200   // 2 seconds

#ifdef _P1500_MAGNETX
// Flow Meter Fault Setup
#define MINIMUM_FLOW_MAGNETRON                   1000
#define MINIMUM_FLOW_LINAC                       1000
#define MINIMUM_FLOW_HV_TANK                     1000 // HVPS and HV TANK
#define MINIMUM_FLOW_HVPS                        1000 // HEAT EXCHANGER
#define MINIMUM_FLOW_CIRCULATOR                  1000 // CIRCULATOR
#define MINIMUM_FLOW_SPARE                       0
#else
// Flow Meter Fault Setup
#define MINIMUM_FLOW_MAGNETRON                   3800
#define MINIMUM_FLOW_LINAC                       6500
#define MINIMUM_FLOW_HV_TANK                     7800
#define MINIMUM_FLOW_HVPS                        0
#define MINIMUM_FLOW_CIRCULATOR                  0
#define MINIMUM_FLOW_SPARE                       0
#endif


// SF6 Management Configuration
#define MINIMUM_COOLANT_TEMP_FOR_SF6_MANAGEMENT  292   // 292 Deg K / 20 Deg C
#define MINIMUM_PRESSURE_FOR_SF6_MANAGEMENT      3000  // 30 PSI   
#define SF6_MINIMUM_TARGET_PRESSURE              4000  // 40 PSI
#define SF6_MAXIMUM_TARGET_PRESSURE              4200  // 42 PSI
#define SF6_TIME_CHARGING                        500   // 5 Seconds - Time that the SF6 bottle is connected to waveguide
#define SF6_TIME_DELAY                           500   // 5 Seconds - Time after charging to let SF6 pressure sensor stabilize
#define SF6_UNDER_PRESSURE_TRIP                  3700  // 37 PSI 
#define SF6_PRESSURE_TRIP_TIME                   1000  // 10 seconds


#endif


#ifdef LINAC_6_4
// Configure the trip points for the 2.5 System

#endif




















#endif
