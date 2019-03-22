#ifndef __A37342_002_CONFIG_H
#define __A37342_002_CONFIG_H



// FAULT CONFIGURATION

#define __LINAC_2_5
//#define __LINAC_MAGNETX




#ifdef __LINAC_2_5
// Configure the trip points for the 2.5 System

// Thermistor Fault Setup
#define COOLANT_TRIP_THERMISTOR_VOLTAGE_OVER_TEMP_UNDER_VOLTAGE  1544 // 37 Deg C - This is NTC Voltage (decreases as temperature increases)
#define COOLANT_TRIP_THERMISTOR_VOLTAGE_UNDER_TEMP_OVER_VOLTAGE  2280 // 20 Deg C
#define CABINET_TRIP_THERMISTOR_VOLTAGE                          1184 // 47 Deg C 
#define LINAC_TRIP_THERMISTOR_VOLTAGE                               0 // 105 Deg C 
#define ANALOG_TEMPERATURE_TRIP_TIME                             1000 // 10 Seconds

// Temperature Switch Fault Setup
#define TEMPERATURE_SWITCH_FILTER_TIME                            200 // 2 seconds

// Flow Meter Fault Setup
#define MINIMUM_FLOW_MAGNETRON                                   3800
#define MINIMUM_FLOW_LINAC                                       6500
#define MINIMUM_FLOW_HV_TANK                                     7800
#define MINIMUM_FLOW_HVPS                                           0
#define MINIMUM_FLOW_CIRCULATOR                                     0
#define MINIMUM_FLOW_SPARE                                          0

// SF6 Management Configuration
#define MINIMUM_COOLANT_TEMP_FOR_SF6_MANAGEMENT                  2920  // 292 Deg K / 20 Deg C
#define MINIMUM_PRESSURE_FOR_SF6_MANAGEMENT                      3000  // 30 PSI  - THIS IS NO LONGER USED   
#define SF6_MINIMUM_TARGET_PRESSURE                              3200  // 32 PSI
#define SF6_MAXIMUM_TARGET_PRESSURE                              3400  // 34 PSI
#define SF6_TIME_CHARGING                                         500  // 5 Seconds - Time that the SF6 bottle is connected to waveguide
#define SF6_TIME_DELAY                                            500  // 5 Seconds - Time after charging to let SF6 pressure sensor stabilize
#define SF6_UNDER_PRESSURE_TRIP                                  3100  // 31 PSI 
#define SF6_PRESSURE_TRIP_TIME                                   1000  // 10 seconds

#define SF6_PULSES_AVAILABLE_AT_POWER_UP                          25  // 25 Pulses  
#define SF6_HOUR_COUNTER                                      360000  // One more pulse every hour

#define SOFTWARE_DASH_NUMBER                                     002

#endif


#ifdef __LINAC_MAGNETX
// Configure the trip points for the MAGNETX SYSTEM

// Thermistor Fault Setup
#define COOLANT_TRIP_THERMISTOR_VOLTAGE_OVER_TEMP_UNDER_VOLTAGE  1544 // 37 Deg C - This is NTC Voltage (decreases as temperature increases)
#define COOLANT_TRIP_THERMISTOR_VOLTAGE_UNDER_TEMP_OVER_VOLTAGE  2280 // 20 Deg C
#define CABINET_TRIP_THERMISTOR_VOLTAGE                          1184 // 47 Deg C 
#define LINAC_TRIP_THERMISTOR_VOLTAGE                               0 // 105 Deg C 
#define ANALOG_TEMPERATURE_TRIP_TIME                             1000 // 10 Seconds

// Temperature Switch Fault Setup
#define TEMPERATURE_SWITCH_FILTER_TIME                            200 // 2 seconds

// Flow Meter Fault Setup
#define MINIMUM_FLOW_MAGNETRON                                   1000
#define MINIMUM_FLOW_LINAC                                       1000
#define MINIMUM_FLOW_HV_TANK                                     1000 // HVPS and HV TANK
#define MINIMUM_FLOW_HVPS                                        1000 // HEAT EXCHANGER
#define MINIMUM_FLOW_CIRCULATOR                                  1000 // CIRCULATOR
#define MINIMUM_FLOW_SPARE                                          0

// SF6 Management Configuration
#define MINIMUM_COOLANT_TEMP_FOR_SF6_MANAGEMENT                  2920  // 292 Deg K / 20 Deg C
#define MINIMUM_PRESSURE_FOR_SF6_MANAGEMENT                      3000  // 30 PSI  - THIS IS NO LONGER USED   
#define SF6_MINIMUM_TARGET_PRESSURE                              3200  // 32 PSI
#define SF6_MAXIMUM_TARGET_PRESSURE                              3400  // 34 PSI
#define SF6_TIME_CHARGING                                         500  // 5 Seconds - Time that the SF6 bottle is connected to waveguide
#define SF6_TIME_DELAY                                            500  // 5 Seconds - Time after charging to let SF6 pressure sensor stabilize
#define SF6_UNDER_PRESSURE_TRIP                                  3100  // 31 PSI 
#define SF6_PRESSURE_TRIP_TIME                                   1000  // 10 seconds

#define SF6_PULSES_AVAILABLE_AT_POWER_UP                          50  // 25 Pulses  
#define SF6_HOUR_COUNTER                                      180000  // One more pulse every hour

#define SOFTWARE_DASH_NUMBER                                     102

#endif



















#endif
