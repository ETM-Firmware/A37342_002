#ifndef __A36926_000_H
#define __A36926_000_H

// A36926-002 This is the Can interface board configured as Cooling Interface board

#include <xc.h>
#include <adc12.h>
#include <timer.h>
#include <libpic30.h>
#include <incap.h>
#include "ETM.h"
#include "P1395_CAN_SLAVE.h"
#include "FIRMWARE_VERSION.h"



#define FCY_CLK     10000000


/*
  Hardware Module Resource Usage

  CAN1   - Used/Configured by ETM CAN 
  Timer4 - Used/Configured by ETM CAN - Used to Time sending of messages (status update / logging data and such) 
  Timer5 - Used/Configured by ETM CAN - Used for detecting error on can bus

  SPI1   - Used/Configured by LTC265X Module
  I2C    - Used/Configured by EEPROM Module


  Timer1 - Used for 10msTicToc
  Timer2 - Used for Input Capture module

  ADC Module - See Below For Specifics

*/



// ---------- BASE A36926 I/O CONFIGURATION ----------------- //
 
#define PIC_DIG_IN_1      _RD8
#define PIC_DIG_IN_2      _RD9
#define PIC_DIG_IN_3      _RD10
#define PIC_DIG_IN_4      _RD11
#define PIC_DIG_IN_5      _RD12
#define PIC_DIG_IN_6      _RD13
#define PIC_DIG_IN_7      _RD14
#define PIC_DIG_IN_8      _RD15

#define AUTO_INHIBIT_DETECT          _RA14   // This is INT3
#define RESET_DETECT                 _RG14



#define PIC_RELAY_OUT                _LATD3
#define PIC_OUTPUT_LAMBDA_SELECT     _LATD2
#define PIC_DIGITAL_OUT_2_NOT        _LATD1
#define PIC_DIGITAL_OUT_1_NOT        _LATD0
#define PIC_15V_SUPPLY_ENABLE        _LATA6

#define TEST_POINT_A                 _LATF6
#define TEST_POINT_B                 _LATF7
#define TEST_POINT_C                 _LATF8
#define TEST_POINT_D                 _LATF2
#define TEST_POINT_E                 _LATF3
#define TEST_POINT_F                 _LATB14


#define LED_OPERATIONAL              _LATA7
#define LED_A_RED                    _LATG12
#define LED_B_GREEN                  _LATG13



/*
  BASE ANALOG CONFIGURATION
  PIC_ADC_AN1  is on AN2
  PIC_ADC_AN2  is on AN3
  PIC_ADC_AN3  is on AN4
  PIC_ADC_AN4  is on AN5
  
  PIC_ADC_+15V_MON is on AN6
  PIC_ADC_-15V_MON is on AN7
  PIC_ADC_5V_MON   is on AN8
  PIC_ADC_TEST_DAC is on AN9
*/


// Pins that must be configured as outputs
/*
  A6,A7
  B14
  C
  D0,D1,D2,D3
  F2,F3,F6,F7,F8
  G12,G13
*/

#define A36926_TRISA_VALUE 0b1111111110111111
#define A36926_TRISB_VALUE 0b1011111111111111
#define A36926_TRISC_VALUE 0b1111111111111111
#define A36926_TRISD_VALUE 0b1111111111110000
#define A36926_TRISF_VALUE 0b1111111100110011
#define A36926_TRISG_VALUE 0b1100111111111111



// ----------------------- CONFIGURE PINS FOR A36926-002 Code ----------------- //


// -------- Digital Input Pins ----------//
#define PIN_DIGITAL_INPUT_1_CABINET_TEMP_SWITCH  PIC_DIG_IN_7
#define PIN_DIGITAL_INPUT_2_COOLANT_TEMP_SWITCH  PIC_DIG_IN_8
#define ILL_TEMP_SWITCH_OVER_TEMP                0

#define PIN_RELAY_OUT_SF6_SOLENOID               PIC_RELAY_OUT
#define OLL_CLOSE_SOLENOID                       1

#define PIN_TEMP_SWITCH_1_PULL_UP                PIC_DIGITAL_OUT_1_NOT
#define PIN_TEMP_SWITCH_2_PULL_UP                PIC_DIGITAL_OUT_2_NOT
#define OLL_TEMP_SWITCH_PULL_UP                  0

#define PIN_LED_OPERATIONAL_GREEN                LED_OPERATIONAL
#define PIN_LED_A_RED                            LED_A_RED
#define PIN_LED_B_GREEN                          LED_B_GREEN
#define OLL_LED_ON                               0

#define PIN_OUT_TP_A                             TEST_POINT_A
#define PIN_OUT_TP_B                             TEST_POINT_B
#define PIN_OUT_TP_C                             TEST_POINT_C
#define PIN_OUT_TP_D                             TEST_POINT_D
#define PIN_OUT_TP_E                             TEST_POINT_E
#define PIN_OUT_TP_F                             TEST_POINT_F


#define PIN_RESET_DETECT                         RESET_DETECT






// ------------------------ CONFIGURE ADC MODULE ------------------- //

// ----------------- ANALOG INPUT PINS ---------------- //
/* 
   AN2 - Thermistor 1
   AN3 - Thermistor 2
   AN4 - Thermistor 3
   AN5 - 4-20mA SF6 Pressure
   AN6 - +15V Mon
   AN7 - -15V Mon   
   AN8 - 5V Mon
   AN9 - ADC Test Input
   

   
*/

/*
  This sets up the ADC to work as following
  AUTO Sampeling
  External Vref+/Vref-
  With 10MHz System Clock, ADC Clock is 450ns (4.5 clocks per ADC clock), Sample Time is 6 ADC Clock so total sample time is 9.0uS
  Conversion rate of 111KHz (13.888 Khz per Channel), 138 Samples per 10mS interrupt
  8 Samples per Interrupt, use alternating buffers
  Scan Through Selected Inputs
*/

#define ADCON1_SETTING  (ADC_MODULE_OFF & ADC_IDLE_STOP & ADC_FORMAT_INTG & ADC_CLK_AUTO & ADC_AUTO_SAMPLING_ON)
#define ADCON2_SETTING  (ADC_VREF_EXT_EXT & ADC_SCAN_ON & ADC_SAMPLES_PER_INT_8 & ADC_ALT_BUF_ON & ADC_ALT_INPUT_OFF)
#define ADCHS_SETTING   (ADC_CH0_POS_SAMPLEA_AN2 & ADC_CH0_NEG_SAMPLEA_VREFN & ADC_CH0_POS_SAMPLEB_AN2 & ADC_CH0_NEG_SAMPLEB_VREFN)
#define ADPCFG_SETTING  (ENABLE_AN2_ANA & ENABLE_AN3_ANA & ENABLE_AN4_ANA & ENABLE_AN5_ANA & ENABLE_AN6_ANA & ENABLE_AN7_ANA & ENABLE_AN8_ANA & ENABLE_AN9_ANA)
#define ADCSSL_SETTING  (SKIP_SCAN_AN0 & SKIP_SCAN_AN1 & SKIP_SCAN_AN10 & SKIP_SCAN_AN11 & SKIP_SCAN_AN12 & SKIP_SCAN_AN13 & SKIP_SCAN_AN14 & SKIP_SCAN_AN15)
#define ADCON3_SETTING  (ADC_SAMPLE_TIME_4 & ADC_CONV_CLK_SYSTEM & ADC_CONV_CLK_9Tcy2)




/* 
   TMR1 Configuration
   Timer1 - Used for 10msTicToc
   Period should be set to 10mS
   With 10Mhz Clock, x8 multiplier will yield max period of 52.4mS, 800ns per tick
*/

#define T1CON_VALUE                    (T1_ON & T1_IDLE_CON & T1_GATE_OFF & T1_PS_1_8 & T1_SOURCE_INT)
#define PR1_VALUE_10_MILLISECONDS      12500



/* 
   TMR2 Configuration
   Timer2 - PWM frequency Capture
   Period should be set to 10mS
   Maximum Period = 2^16 * 256 / Fcy = 1.6777216  seconds with 10Mhz Clock
*/

#define T2CON_VALUE                    (T2_ON & T2_IDLE_CON & T2_GATE_OFF & T2_PS_1_256 & T2_SOURCE_INT)
#define PR2_VALUE_MAX                  0xFFFF






// -------------------- A36746 STATUS BIT CONFIGURATION ------------------------ //
#define _STATUS_SF6_SOLENOID_RELAY_CLOSED               _LOGGED_STATUS_0
#define _STATUS_SF6_COOLANT_TOO_LOW                     _LOGGED_STATUS_1
#define _STATUS_SF6_PRESSURE_TO_LOW_TO_MANAGE           _LOGGED_STATUS_2
#define _STATUS_SF6_FILL_REQUIRED                       _LOGGED_STATUS_3
#define _STATUS_SF6_NO_PULSES_AVAILABLE                 _LOGGED_STATUS_4
#define _STATUS_SF6_FILLING                             _LOGGED_STATUS_5



// -------------------- A36746 FAULTS/WARNINGS CONFIGURATION-------------------- //
#define _FAULT_CAN_COMMUNICATION_LATCHED                _FAULT_0
#define _FAULT_FLOW_SENSOR_1                            _FAULT_1 // Magnetron Coolant Flow
#define _FAULT_FLOW_SENSOR_2                            _FAULT_2 // unused
#define _FAULT_FLOW_SENSOR_3                            _FAULT_3 // HV Tank Coolant Flow
#define _FAULT_FLOW_SENSOR_4                            _FAULT_4 // unused
#define _FAULT_FLOW_SENSOR_5                            _FAULT_5 // Circulator Coolant Flow
#define _FAULT_FLOW_SENSOR_6                            _FAULT_6  // unused
#define _FAULT_CABINET_TEMP_SWITCH                      _FAULT_7
#define _FAULT_COOLANT_TEMP_SWITCH                      _FAULT_8
#define _FAULT_CABINET_OVER_TEMP                        _FAULT_9
#define _FAULT_COOLANT_OVER_TEMP                        _FAULT_A
#define _FAULT_LINAC_OVER_TEMP                          _FAULT_B  // unused
#define _FAULT_SF6_UNDER_PRESSURE                       _FAULT_C
#define _FAULT_SF6_PRESSURE_SWITCH                      _FAULT_D  // unused

typedef struct {
  unsigned int period_array[256];
  unsigned int current_index;
  unsigned int previous_timer_reading;
  unsigned int counter_10ms_loop;
  
} PWMReading;


typedef struct {
  unsigned int  flow_reading;           // This is the flow reading in mL per minute
  unsigned int  frequency;              // This is frequency in HZ.  It is caluclated from the period readings in the period array
  // The period array contains a history of the time between input capture events
  // Input capture events occur on the rising and falling edge so period is actually 2 of these times
  // An average period is calculated by adding these all together and diving by 8
  unsigned int  period_array[16];
  unsigned int  array_index;            // This is the index where the next period reading should go
  unsigned int  long_pulse;             // This is used to signal that the period is greater than one timer period
  unsigned int  previous_timer_reading; // The period is current timer reading - the previous timer reading
  unsigned int* ICXBUF_ptr;             // This is pointer to the input capture associate with this PWM input
  unsigned int* TMRX_ptr;               // This is pointer to the timer associated with the input capture module
  unsigned int minimum_flow;            // This is the minimum flow for this particular flow meter
  TYPE_DIGITAL_INPUT digital_fault;     // This is the digital structure used to filter the fault data
} TYPE_FLOW_READING;



typedef struct {
  AnalogInput analog_input_thermistor_1;              // 1mV per LSB - This is the voltage reading of the thermistor
  AnalogInput analog_input_thermistor_2;              // 1mV per LSB - This is the voltage reading of the thermistor
  AnalogInput analog_input_thermistor_3;              // 1mV per LSB - This is the voltage reading of the thermistor
  AnalogInput analog_input_SF6_pressure;              // .01 PSI per LSB
  AnalogInput analog_input_5v_mon;                    // 1mV per LSB
  AnalogInput analog_input_15v_mon;                   // 1mV per LSB
  AnalogInput analog_input_neg_15v_mon;               // 1mV per LSB
  unsigned int adc_test;

  unsigned int coolant_temperature_kelvin;            // 1 Deg K per LSB - This is temperature calculated from the voltage reading
  unsigned int cabinet_temperature_kelvin;            // 1 Deg K per LSB - This is temperature calculated from the voltage reading
  unsigned int linac_temperature_kelvin;              // 1 Deg K per LSB - This is temperature calculated from the voltage reading

  unsigned int accumulator_counter;                   // Used to count the ADC readings used in averaging

  unsigned int control_state;
  unsigned int startup_counter;
  unsigned int test_timer;
  unsigned int fault_active;

  TYPE_DIGITAL_INPUT digital_input_7_cabinet_temp_switch;
  TYPE_DIGITAL_INPUT digital_input_8_coolant_temp_switch;

  TYPE_FLOW_READING flow_meter_1_magnetron;
  TYPE_FLOW_READING flow_meter_2_linac;
  TYPE_FLOW_READING flow_meter_3_hv_tank;
  TYPE_FLOW_READING flow_meter_4_hvps;       // unused
  TYPE_FLOW_READING flow_meter_5_circulator; // unused
  TYPE_FLOW_READING flow_meter_6_alternate;  // unused

  //unsigned int flow_magnetron;
  //unsigned int flow_linac;
  //unsigned int flow_hv_tank;
  //unsigned int flow_hvps;       // unused for now
  //unsigned int flow_circulator; // unused for now
  //unsigned int flow_spare;      // unused for now

  unsigned int SF6_state;
  unsigned int SF6_fill_threshold;
  unsigned int SF6_low_pressure_override_counter;
  unsigned int SF6_state_charging_counter;
  unsigned int SF6_state_delay_counter;

  // THESE TWO VARIABLES MUST BE ADJACENT TO EACH OTHER IN MEMORY BECAUSE THEY ARE ACCESSED AS A PAIR WHEN READING/WRITING THE EEPROM
  unsigned int SF6_pulses_available;           // This is how many pulses can be sent out without ECB intervention
  unsigned int SF6_bottle_pulses_remaining;    // This is how many pulses are remainging in the bottle
  

} TYPE_COOLING_GLOBALS;


// State Definitions
#define STATE_STARTUP                10
#define STATE_NOT_READY              20
#define STATE_TESTING                30
#define STATE_READY                  40

#define ETM_EEPROM_PAGE_COOLING_INTERFACE      0x70





















#endif
