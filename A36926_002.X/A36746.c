#include "A36926_002.h"

// A36926-002 This is the Can interface board configured as Cooling Interface board

_FOSC(ECIO & CSW_FSCM_OFF); 
_FWDT(WDT_ON & WDTPSA_512 & WDTPSB_8);  // 8 Second watchdog timer 
_FBORPOR(PWRT_OFF & BORV_45 & PBOR_OFF & MCLR_EN);
_FBS(WR_PROTECT_BOOT_OFF & NO_BOOT_CODE & NO_BOOT_EEPROM & NO_BOOT_RAM);
_FSS(WR_PROT_SEC_OFF & NO_SEC_CODE & NO_SEC_EEPROM & NO_SEC_RAM);
_FGS(CODE_PROT_OFF);
_FICD(PGD);


#define THERMISTOR_LOOK_UP_TABLE_VALUES 472,472,472,472,472,472,472,472,472,472,472,472,472,472,472,472,472,472,472,472,472,472,472,472,472,472,472,472,472,472,472,472,472,472,472,377,376,375,374,373,372,371,370,370,369,368,367,366,366,365,364,363,363,362,361,361,360,360,359,358,358,357,357,356,355,355,354,354,353,353,352,352,351,351,350,350,350,349,349,348,348,347,347,346,346,346,345,345,344,344,344,343,343,343,342,342,341,341,341,340,340,340,339,339,339,338,338,338,337,337,337,336,336,336,336,335,335,335,334,334,334,333,333,333,333,332,332,332,332,331,331,331,330,330,330,330,329,329,329,329,328,328,328,328,327,327,327,327,326,326,326,326,326,325,325,325,325,324,324,324,324,323,323,323,323,323,322,322,322,322,322,321,321,321,321,320,320,320,320,320,319,319,319,319,319,318,318,318,318,318,318,317,317,317,317,317,316,316,316,316,316,315,315,315,315,315,315,314,314,314,314,314,313,313,313,313,313,313,312,312,312,312,312,311,311,311,311,311,311,310,310,310,310,310,310,309,309,309,309,309,309,308,308,308,308,308,308,307,307,307,307,307,307,307,306,306,306,306,306,306,305,305,305,305,305,305,304,304,304,304,304,304,304,303,303,303,303,303,303,302,302,302,302,302,302,302,301,301,301,301,301,301,301,300,300,300,300,300,300,299,299,299,299,299,299,299,298,298,298,298,298,298,298,297,297,297,297,297,297,297,296,296,296,296,296,296,296,295,295,295,295,295,295,295,294,294,294,294,294,294,294,293,293,293,293,293,293,293,292,292,292,292,292,292,292,291,291,291,291,291,291,291,290,290,290,290,290,290,290,289,289,289,289,289,289,288,288,288,288,288,288,288,287,287,287,287,287,287,287,286,286,286,286,286,286,286,285,285,285,285,285,285,285,284,284,284,284,284,284,284,283,283,283,283,283,283,282,282,282,282,282,282,282,281,281,281,281,281,281,281,280,280,280,280,280,280,279,279,279,279,279,279,279,278,278,278,278,278,278,277,277,277,277,277,277,276,276,276,276,276,276,275,275,275,275,275,275,274,274,274,274,274,274,273,273,273,273,273,273,272,272,272,272,272,272,271,271,271,271,271,270,270,270,270,270,270,269,269,269,269,269,268,268,268,268,268,267,267,267,267,267,266,266,266,266,266,265,265,265,265,265,264,264,264,264,263,263,263,263,263,262,262,262,262,261,261,261,261,260,260,260,260,259,259,259,259,258,258,258,258,257,257,257,257,256,256,256,255,255,255,255,254,254,254,253,253,253,252,252,252,251,251,251,250,250,250,249,249,248,248,248,247,247,246,246,246,245,245,244,244,243,243,242,242,241,241,240,240,239,238,238,237,236,236,235,234,233,232,232,232,232,232,232,232,232,232,232,232,232,232,232,232,232,232,232,172,172,172,172,172




const unsigned int ThermistorLookupTable[630] = {THERMISTOR_LOOK_UP_TABLE_VALUES};

TYPE_FLOW_READING flow_meter_1;
TYPE_FLOW_READING flow_meter_2;
TYPE_FLOW_READING flow_meter_3;
TYPE_FLOW_READING flow_meter_4;
TYPE_FLOW_READING flow_meter_5;
TYPE_FLOW_READING flow_meter_6;





CoolingGlobalStruct global_data_A36746;

void DoSF6Management(void);

#define TEMPERATURE_SWITCH_FILTER_TIME            200        // 2 seconds

#define COOLING_INTERFACE_BOARD_TEST_TIME         100        // 1 second


#define MINIMUM_FLOW_MAGNETRON                    4000
#define MINIMUM_FLOW_LINAC                        6700
#define MINIMUM_FLOW_HV_TANK                      8000
#define MINIMUM_FLOW


#define STARTUP_LED_FLASH_TIME       400       // 4 Seconds


#define FLOW_METER_ML_PER_HZ      81
#define FLOW_METER_CONSTANT       841

#define PERIOD_MAX_FREQUENCY      70   // 558 Hz
#define FLOW_METER_MIN_FREQUENCY  15
#define PWM_MAX_PERIOD            1954 // This is equiv to 10Hz 


#define SF6_STATE_TEST       10
#define SF6_STATE_CHARGING   20
#define SF6_STATE_DELAY      30

#define MINIMUM_COOLANT_TEMP_FOR_SF6_MANAGEMENT  292   // 292 Deg K / 20 Deg C
#define MINIMUM_PRESSURE_FOR_SF6_MANAGEMENT      3000  // 30 PSI   

#define SF6_MINIMUM_TARGET_PRESSURE              4000  // 40 PSI
#define SF6_MAXIMUM_TARGET_PRESSURE              4200  // 42 PSI

#define SF6_TIME_CHARGING                        500 // 5 Seconds
#define SF6_TIME_DELAY                           500 // 5 Seconds



//#define COOLANT_TRIP_TEMPERATURE        308 // This is in Deg K units
//#define CABINET_TRIP_TEMPERATURE        318 // This is in Deg K units

#define COOLANT_TRIP_THERMISTOR_VOLTAGE       3936  // This is the voltage (in millivolts) that indicates a temerperature of 36 Deg C.  If the thermistor voltage is less than this it should trip
#define CABINET_TRIP_THERMISTOR_VOLTAGE       3040  // This is the voltage (in millivolts) that indicates a temerperature of 46 Deg C.  If the thermistor voltage is less than this it should trip

#define TEMPERATURE_SENSOR_FIXED_SCALE         .15625

#define ANALOG_TEMPERATURE_TRIP_TIME    1000  // 10 Seconds



#define SF6_SENSOR_FIXED_SCALE                 .19629   // DPARKER NEED TO TEST
#define SF6_SENSOR_FIXED_OFFSET                -12736    // Calculated 4mA Offset
#define SF6_UNDER_PRESSURE_TRIP                3700     // 37 PSI DPARKER NEED TO TEST
#define SF6_PRESSURE_TRIP_TIME                 1000     // 10 seconds



void InitializeA36746(void);
void DoStateMachine(void);
void DoA36746(void);
void FlashLEDs(void);



int main(void) {
  global_data_A36746.control_state = STATE_STARTUP;
  while (1) {
    DoStateMachine();
  }
}



void DoStateMachine(void) {
  switch (global_data_A36746.control_state) {
    
  case STATE_STARTUP:
    InitializeA36746();
    _CONTROL_NOT_CONFIGURED = 0;
    _CONTROL_NOT_READY = 1;
    global_data_A36746.startup_counter = 0;
    while (global_data_A36746.control_state == STATE_STARTUP) {
      DoA36746();
      FlashLEDs();
      if (global_data_A36746.startup_counter >= STARTUP_LED_FLASH_TIME) {
	global_data_A36746.control_state = STATE_NOT_READY;	
      }
    }
    break;

    
  case STATE_NOT_READY:
    PIN_LED_OPERATIONAL_GREEN = OLL_LED_ON;
    _CONTROL_NOT_READY = 1;
    while (global_data_A36746.control_state == STATE_NOT_READY) {
      DoA36746();
      if (_SYNC_CONTROL_RESET_ENABLE) {
	global_data_A36746.control_state = STATE_TESTING;
      }
    }
    break;


  case STATE_TESTING:
    _CONTROL_NOT_READY = 1;
    global_data_A36746.test_timer = 0;
    while (global_data_A36746.control_state == STATE_TESTING) {
      DoA36746();
      if (global_data_A36746.test_timer >= COOLING_INTERFACE_BOARD_TEST_TIME) {
	global_data_A36746.control_state = STATE_READY;
      }
      if (global_data_A36746.fault_active) {
	global_data_A36746.control_state = STATE_NOT_READY;
      }
    }
    break;


  case STATE_READY:
    _FAULT_REGISTER = 0;
    _CONTROL_NOT_READY = 0;
    while (global_data_A36746.control_state == STATE_READY) {
      DoA36746();
      if (global_data_A36746.fault_active) {
	global_data_A36746.control_state = STATE_NOT_READY;
      }

    }
    break;

  default:
    global_data_A36746.control_state = STATE_STARTUP;
    break;
  }
}



#define PWM_MAX_PERIOD 19531 // This is a period of one half second (since we trigger and rising and falling edge a period is actually two of these - one second)


/*
  Example Case
  IC register reads  - 0xFFFF followed by 0x0001
  That could be a time of 2 timer tics or 2^16 +2 timer tics or n*2^16 +2 timer tics
  One is very high frequency and the other is a very low frequency.  How do we determine?
*/


void InitializeFlowMeter(TYPE_FLOW_READING* flow_ptr, unsigned int min_flow, unsigned int* ptr_icxbuf, unsigned int* ptr_tmrx) {
  flow_ptr->minimum_flow = min_flow;
  flow_ptr->ICXBUF_ptr = ptr_icxbuf;
  flow_ptr->TMRX_ptr = ptr_tmrx;
  ETMDigitalInitializeInput(flow_ptr->digital_fault, 0, 100); // Initialize to not faulted with 1 second delay.
}


void UpdateFlowMeterInputCapture(TYPE_FLOW_READING* flow_ptr) {
  unsigned int previous_period;
  unsigned int current_period;
  unsigned int latest_capture;
  unsigned int latest_tmr;
  latest_tmr     = *flow_ptr->TMRX_ptr;
  latest_tmr    -= flow_ptr->previous_timer_reading;
  if (latest_tmr >= PWM_MAX_PERIOD) { 
    flow_ptr->long_pulse = 1;
    flow_ptr->array_index++;
    flow_ptr->array_index &= 0x000F;
    flow_ptr->period_array[flow_ptr->current_index] = PWM_MAX_PERIOD;
  }
  
  while((*(flow_ptr->ICXBUF_ptr + 2)) & 0x0008) {
    // ICBNE bit is set
    latest_capture = *flow_ptr->ICXBUF_ptr;
    current_period = latest_capture - flow_ptr->previous_timer_reading;
    flow_ptr->previous_timer_reading = latest_capture;
    if (flow_ptr->long_pulse) {
      flow_ptr->long_pulse = 0;
      current_period = PWM_MAX_PERIOD;
    }
    flow_ptr->array_index++;
    flow_ptr->array_index &= 0x000F;
    flow_ptr->period_array[flow_ptr->current_index] = current_period;
  }
}


unsigned int CheckFlowMeterFault(TYPE_FLOW_READING* flow_ptr) {
  unsigned long period_lng;
  unsigned int period
  period_lng  = flow_ptr->period_array[0];
  period_lng += flow_ptr->period_array[1];
  period_lng += flow_ptr->period_array[2];
  period_lng += flow_ptr->period_array[3];

  period_lng += flow_ptr->period_array[4];
  period_lng += flow_ptr->period_array[5];
  period_lng += flow_ptr->period_array[6];
  period_lng += flow_ptr->period_array[7];

  period_lng += flow_ptr->period_array[8];
  period_lng += flow_ptr->period_array[9];
  period_lng += flow_ptr->period_array[10];
  period_lng += flow_ptr->period_array[11];

  period_lng += flow_ptr->period_array[12];
  period_lng += flow_ptr->period_array[13];
  period_lng += flow_ptr->period_array[14];
  period_lng += flow_ptr->period_array[15];

  period_lng >>= 3;
  period = period_lng;

  if (period <= PERIOD_MAX_FREQUENCY) {
    period = PERIOD_MAX_FREQUENCY;
  }
  flow_ptr->frequency = 39062 / period;
  
  if (flow_ptr->frequency < FLOW_METER_MIN_FREQUENCY) {
    flow_ptr->flow_reading = 0;
  } else {
    flow_ptr->flow_reading = FLOW_METER_ML_PER_HZ*flow_ptr->frequency + FLOW_METER_CONSTANT;
  }

  if (flow_ptr->min_flow == 0) {
    return 0;
  }

  if (flow_ptr->flow_reading < flow_ptr->minimum_flow) {
    ETMDigitalUpdateInput(flow_ptr->digital_fault, 1);
  } else {
    ETMDigitalUpdateInput(flow_ptr->digital_fault, 0);
  }
  
  return = ETMDigitalFilteredOutput(flow_ptr->digital_fault);
}


unsigned int ConvertThermistorVoltageToKelvin(unsigned int thermistor_voltage) {
  unsigned int thermistor_index;
  thermistor_index = (global_data_A36746.analog_input_coolant_temperature.reading_scaled_and_calibrated >>4);
  if (thermistor_index > 625) {
    thermistor_index = 625;
  }
  return ThermistorLookupTable[thermistor_index];
  
}

void DoA36746(void) {
  unsigned int timer_reading;
  unsigned int frequency;
  unsigned int thermistor_index;

  ETMCanSlaveDoCan();

  // This needs to happen every time through the control loop to capture high frequency PWM
  UpdateFlowMeterInputCapture(&flow_meter_1);
  UpdateFlowMeterInputCapture(&flow_meter_2);
  UpdateFlowMeterInputCapture(&flow_meter_3);
  UpdateFlowMeterInputCapture(&flow_meter_4);
  UpdateFlowMeterInputCapture(&flow_meter_5);
  UpdateFlowMeterInputCapture(&flow_meter_6);
  
  if (_T1IF) {
    // Timer has expired so execute the scheduled code (should be once every 10ms)
    _T1IF = 0;

    DoSF6Management();

    if (global_data_A36746.control_state == STATE_TESTING) {
      global_data_A36746.test_timer++;
    }
    
    if (global_data_A36746.control_state == STATE_STARTUP) {
      global_data_A36746.startup_counter++;
    }
  

    // Do Math on ADC inputs
    // Scale the ADC readings to engineering units
    ETMAnalogScaleCalibrateADCReading(&global_data_A36746.analog_input_coolant_temperature);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36746.analog_input_cabinet_temperature);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36746.analog_input_linac_temperature);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36746.analog_input_SF6_pressure);

    // Convert the temperature reading voltage into temperature
    global_data_A36746.coolant_temperature_kelvin = ConvertThermistorVoltageToKelvin(global_data_A36746.analog_input_coolant_temperature.reading_scaled_and_calibrated);
    global_data_A36746.cabinet_temperature_kelvin = ConvertThermistorVoltageToKelvin(global_data_A36746.analog_input_cabinet_temperature.reading_scaled_and_calibrated);
    global_data_A36746.linac_temperature_kelvin   = ConvertThermistorVoltageToKelvin(global_data_A36746.analog_input_linac_temperature.reading_scaled_and_calibrated);

    // Converters the PWM readings into flow and stores in the PWM structure
    global_data_A36746.flow_magnetron  = CalculateFlowMeterFlow(&flow_meter_1);
    global_data_A36746.flow_linac      = CalculateFlowMeterFlow(&flow_meter_2);
    global_data_A36746.flow_hv_tank    = CalculateFlowMeterFlow(&flow_meter_3);
    global_data_A36746.flow_hvps       = CalculateFlowMeterFlow(&flow_meter_4);
    global_data_A36746.flow_circulator = CalculateFlowMeterFlow(&flow_meter_5);
    global_data_A36746.flow_spare      = CalculateFlowMeterFlow(&flow_meter_6);


    // Update Temperature Switch Inputs
    ETMDigitalUpdateInput(&digital_input_7_cabinet_temp_switch, PIN_DIGITAL_INPUT_1_CABINET_TEMP_SWITCH);
    ETMDigitalUpdateInput(&digital_input_8_coolant_temp_switch, PIN_DIGITAL_INPUT_2_COOLANT_TEMP_SWITCH);

    // ------------ CHECK FOR FAULTS ----------- //
    UpdateFaults();



    /*
    // If the system is faulted or inhibited set the red LED
    if (_CONTROL_NOT_READY) {
      PIN_LED_A_RED = OLL_LED_ON;
    } else {
      PIN_LED_A_RED = !OLL_LED_ON;
    }
    */


    /*
    local_debug_data.debug_3 = PWM1_IC1_magnetron_flow.period_array[PWM1_IC1_magnetron_flow.current_index] + PWM1_IC1_magnetron_flow.period_array[(PWM1_IC1_magnetron_flow.current_index - 1) & 0xFF];
    local_debug_data.debug_4 = PWM2_IC2_linac_flow.period_array[PWM2_IC2_linac_flow.current_index] + PWM2_IC2_linac_flow.period_array[(PWM2_IC2_linac_flow.current_index - 1) & 0xFF];
    local_debug_data.debug_5 = PWM3_IC3_hv_tank_flow.period_array[PWM3_IC3_hv_tank_flow.current_index] + PWM3_IC3_hv_tank_flow.period_array[(PWM3_IC3_hv_tank_flow.current_index - 1) & 0xFF];

    local_debug_data.debug_6 = global_data_A36746.control_state;
    local_debug_data.debug_7 = global_data_A36746.analog_input_coolant_temperature.reading_scaled_and_calibrated;
    local_debug_data.debug_8 = global_data_A36746.cabinet_temperature_kelvin;
    local_debug_data.debug_9 = global_data_A36746.flow_magnetron;
    local_debug_data.debug_A = global_data_A36746.flow_hv_tank;
    local_debug_data.debug_B = global_data_A36746.analog_input_SF6_pressure.reading_scaled_and_calibrated;
    local_debug_data.debug_C = global_data_A36746.SF6_state;
    local_debug_data.debug_D = global_data_A36746.SF6_pulses_available;
    local_debug_data.debug_E = global_data_A36746.SF6_bottle_pulses_remaining;
    local_debug_data.debug_F = global_data_A36746.SF6_low_pressure_override_counter;
    
    */




    
  }
}

void UpdateFaults(void) {

    if (_CONTROL_CAN_COM_LOSS) {
      _FAULT_CAN_COMMUNICATION_LATCHED = 1;
      global_data_A36746.fault_active = 1;
    } else {
      if (_SYNC_CONTROL_RESET_ENABLE) {
	_FAULT_CAN_COMMUNICATION_LATCHED = 0;
      }
    }
    
    if (CheckFlowMeterFault(flow_meter_1)) {
      _FAULT_FLOW_SENSOR_1 = 1;
    } else {
      if (_SYNC_CONTROL_RESET_ENABLE) {
	_FAULT_FLOW_SENSOR_1 = 0;
      }    
    }

    if (CheckFlowMeterFault(flow_meter_2)) {
      _FAULT_FLOW_SENSOR_2 = 1;
    } else {
      if (_SYNC_CONTROL_RESET_ENABLE) {
	_FAULT_FLOW_SENSOR_2 = 0;
      }    
    }

    if (CheckFlowMeterFault(flow_meter_3)) {
      _FAULT_FLOW_SENSOR_3 = 1;
    } else {
      if (_SYNC_CONTROL_RESET_ENABLE) {
	_FAULT_FLOW_SENSOR_3 = 0;
      }    
    }

    if (CheckFlowMeterFault(flow_meter_4)) {
      _FAULT_FLOW_SENSOR_4 = 1;
    } else {
      if (_SYNC_CONTROL_RESET_ENABLE) {
	_FAULT_FLOW_SENSOR_4 = 0;
      }    
    }

    if (CheckFlowMeterFault(flow_meter_5)) {
      _FAULT_FLOW_SENSOR_5 = 1;
    } else {
      if (_SYNC_CONTROL_RESET_ENABLE) {
	_FAULT_FLOW_SENSOR_5 = 0;
      }    
    }

    if (CheckFlowMeterFault(flow_meter_6)) {
      _FAULT_FLOW_SENSOR_6 = 1;
    } else {
      if (_SYNC_CONTROL_RESET_ENABLE) {
	_FAULT_FLOW_SENSOR_6 = 0;
      }    
    }

    
    if (ETMDigitalFilteredOutput(&digital_input_7_cabinet_temp_switch) == ILL_TEMP_SWITCH_OVER_TEMP) {
      _FAULT_CABINET_TEMP_SWITCH = 1;
    } else {
      if (_SYNC_CONTROL_RESET_ENABLE) {
	_FAULT_CABINET_TEMP_SWITCH = 0;
      }
    }

#ifdef LINAC_6_4
    if (ETMDigitalFilteredOutput(&digital_input_8_coolant_temp_switch) == ILL_TEMP_SWITCH_OVER_TEMP) {
      _FAULT_COOLANT_TEMP_SWITCH = 1;
    } else {
      if (_SYNC_CONTROL_RESET_ENABLE) {
	_FAULT_COOLANT_TEMP_SWITCH = 0;
      }
    }
#endif
    
    // Check for over temperature on Thermistor 1
    // We are using the under absolute function because as temperature goes up, resistance (and voltage) go down 
    if (ETMAnalogCheckUnderAbsolute(&global_data_A36746.analog_input_thermistor_1)) {
      _FAULT_CABINET_OVER_TEMP = 1;
    } else {
      if (_SYNC_CONTROL_RESET_ENABLE) {
	_FAULT_CABINET_OVER_TEMP = 0;
      }
    }

#ifdef LINAC_6_4    
    // Check for over temperature on Thermistor 2
    if (ETMAnalogCheckUnderAbsolute(&global_data_A36746.analog_input_thermistor_2)) {
      // We are using the under absolute function because as temperature goes up, resistance (and voltage) go down 
      _FAULT_COOLANT_OVER_TEMP = 1;
    } else {
      if (_SYNC_CONTROL_RESET_ENABLE) {
	_FAULT_COOLANT_OVER_TEMP = 0;
      }
    }
      
    // Check for over temperature on Thermistor 3
    if (ETMAnalogCheckUnderAbsolute(&global_data_A36746.analog_input_thermistor_3)) {
      _FAULT_CABINET_OVER_TEMP = 1;
    } else {
      if (_SYNC_CONTROL_RESET_ENABLE) {
	_FAULT_CABINET_OVER_TEMP = 0;
      }
    }
#endif    

    // Check for under pressure on SF6
    if (ETMAnalogCheckUnderAbsolute(&global_data_A36746.analog_input_SF6_pressure)) {
      _FAULT_SF6_UNDER_PRESSURE = 1;
      global_data_A36746.fault_active = 1;
    } else if (_SYNC_CONTROL_RESET_ENABLE) {
      _FAULT_SF6_UNDER_PRESSURE = 0;
    }
}


void InitializeA36746(void) {


  // Initialize the status register and load the inhibit and fault masks
  _CONTROL_REGISTER = 0;
  _FAULT_REGISTER = 0;
  _WARNING_REGISTER = 0;
  _NOT_LOGGED_REGISTER = 0;

  
  // Initialize all I/O Registers
  TRISA = A36926_TRISA_VALUE;
  TRISB = A36926_TRISB_VALUE;
  TRISC = A36926_TRISC_VALUE;
  TRISD = A36926_TRISD_VALUE;
  TRISF = A36926_TRISF_VALUE;
  TRISG = A36926_TRISG_VALUE;


  // Initialize TMR2
  PR2 = PR2_VALUE_MAX;
  TMR2 = 0;
  T2CON = T2CON_VALUE;


  // Initialize TMR1
  PR1   = PR1_VALUE_10_MILLISECONDS;
  TMR1  = 0;
  _T1IF = 0;
  T1CON = T1CON_VALUE;





  //Initialize the internal ADC for Startup Power Checks
  // ---- Configure the dsPIC ADC Module ------------ //
  ADCON1 = ADCON1_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADCON2 = ADCON2_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADPCFG = ADPCFG_SETTING;             // Set which pins are analog and which are digital I/O
  ADCHS  = ADCHS_SETTING;              // Configure the high speed ADC module based on H file parameters

  ADCON3 = ADCON3_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADCSSL = ADCSSL_SETTING;

  _ADIF = 0;
  _ADIP = 6; // This needs to be higher priority than the CAN interrupt (Which defaults to 4)
  _ADIE = 1;
  _ADON = 1;


  // Initialize the External EEprom
  ETMEEPromConfigureExternalDevice(EEPROM_SIZE_8K_BYTES, FCY_CLK, 400000, EEPROM_I2C_ADDRESS_0, 1);

  // Initialize the Can module
#define AGILE_REV 77
#define SERIAL_NUMBER 100

  // Initialize the Can module
  ETMCanSlaveInitialize(CAN_PORT_1, FCY_CLK, ETM_CAN_ADDR_HV_LAMBDA_BOARD, _PIN_RG13, 4);
  ETMCanSlaveLoadConfiguration(36926, 0, AGILE_REV, FIRMWARE_AGILE_REV, FIRMWARE_BRANCH, FIRMWARE_BRANCH_REV, SERIAL_NUMBER);


  // Initialize the Analog Input & Output Scaling


  ETMAnalogInitializeInput(&global_data_A36746.analog_input_coolant_temperature,
			   MACRO_DEC_TO_SCALE_FACTOR_16(TEMPERATURE_SENSOR_FIXED_SCALE),
			   OFFSET_ZERO,
			   ANALOG_INPUT_1,
			   NO_OVER_TRIP,
			   COOLANT_TRIP_THERMISTOR_VOLTAGE,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_RELATIVE_COUNTER,
			   ANALOG_TEMPERATURE_TRIP);
  
  ETMAnalogInitializeInput(&global_data_A36746.analog_input_cabinet_temperature,
			   MACRO_DEC_TO_SCALE_FACTOR_16(TEMPERATURE_SENSOR_FIXED_SCALE),
			   OFFSET_ZERO,
			   ANALOG_INPUT_4,
			   NO_OVER_TRIP,
			   CABINET_TRIP_THERMISTOR_VOLTAGE,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_RELATIVE_COUNTER,
			   ANALOG_TEMPERATURE_TRIP_TIME);

  ETMAnalogInitializeInput(&global_data_A36746.analog_input_linac_temperature,
			   MACRO_DEC_TO_SCALE_FACTOR_16(TEMPERATURE_SENSOR_FIXED_SCALE),
			   OFFSET_ZERO,
			   ANALOG_INPUT_4,
			   NO_OVER_TRIP,
			   CABINET_TRIP_THERMISTOR_VOLTAGE,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_RELATIVE_COUNTER,
			   ANALOG_TEMPERATURE_TRIP_TIME);


  ETMAnalogInitializeInput(&global_data_A36746.analog_input_SF6_pressure,
			   MACRO_DEC_TO_SCALE_FACTOR_16(SF6_SENSOR_FIXED_SCALE),
			   SF6_SENSOR_FIXED_OFFSET,
			   ANALOG_INPUT_6,
			   NO_OVER_TRIP,
			   SF6_UNDER_PRESSURE_TRIP,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_RELATIVE_COUNTER,
			   SF6_PRESSURE_TRIP_TIME);

  ETMAnalogInitializeInput(&global_data_A36926.analog_input_5v_mon,
			   MACRO_DEC_TO_SCALE_FACTOR_16(.12500),
			   OFFSET_ZERO,
			   ANALOG_INPUT_5,
			   PWR_5V_OVER_FLT,
			   PWR_5V_UNDER_FLT,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_RELATIVE_COUNTER,
			   NO_ABSOLUTE_COUNTER);
  
  ETMAnalogInitializeInput(&global_data_A36926.analog_input_15v_mon,
			   MACRO_DEC_TO_SCALE_FACTOR_16(.25063),
			   OFFSET_ZERO,
			   ANALOG_INPUT_6,
			   PWR_15V_OVER_FLT,
			   PWR_15V_UNDER_FLT,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_RELATIVE_COUNTER,
			   NO_ABSOLUTE_COUNTER);
  
  ETMAnalogInitializeInput(&global_data_A36926.analog_input_neg_15v_mon,
			   MACRO_DEC_TO_SCALE_FACTOR_16(.06250),
			   OFFSET_ZERO,
			   ANALOG_INPUT_7,
			   PWR_NEG_15V_OVER_FLT,
			   PWR_NEG_15V_UNDER_FLT,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_RELATIVE_COUNTER,
			   NO_ABSOLUTE_COUNTER);
  
  ETMAnalogInitializeInput(&global_data_A36926.analog_input_pic_adc_test_dac,
			   MACRO_DEC_TO_SCALE_FACTOR_16(1),
			   OFFSET_ZERO,
			   ANALOG_INPUT_8,
			   ADC_DAC_TEST_OVER_FLT,
			   ADC_DAC_TEST_UNDER_FLT,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_RELATIVE_COUNTER,
			   NO_ABSOLUTE_COUNTER);
  

#define TEMPERATURE_SWITCH_FILTER_TIME      100 // 1 Second

  ETMDigitalInitializeInput(&global_data_A36746.digital_input_7_cabinet_temp_switch, !ILL_TEMP_SWITCH_OVER_TEMP, TEMPERATURE_SWITCH_FILTER_TIME);
  ETMDigitalInitializeInput(&global_data_A36746.digital_input_8_coolant_temp_switch, !ILL_TEMP_SWITCH_OVER_TEMP, TEMPERATURE_SWITCH_FILTER_TIME);


  global_data_A36746.SF6_state = SF6_STATE_DELAY;
  global_data_A36746.SF6_fill_threshold = SF6_MINIMUM_TARGET_PRESSURE;
  global_data_A36746.SF6_low_pressure_override_counter = 0;
  global_data_A36746.SF6_state_delay_counter = 0;
  
  ETMEEPromReadPage(ETM_EEPROM_PAGE_COOLING_INTERFACE, 2, &global_data_A36746.SF6_pulses_available);
  // This reads SF6_pulses_available and SF6_bottle_pulses_remaining from the external EEPROM with a single I2C command
  if (global_data_A36746.SF6_pulses_available > 25) {
    global_data_A36746.SF6_pulses_available = 25;
  }
  if (global_data_A36746.SF6_bottle_pulses_remaining > 700) {
    global_data_A36746.SF6_bottle_pulses_remaining = 700;
  }

#define PWM1_INITITAL_PERIOD 195 // 200 Hz
#define PWM2_INITITAL_PERIOD 391 // 100 Hz
#define PWM3_INITITAL_PERIOD 1302 // 30 Hz
#define PWM_INITIAL_PERIOD   1302 // 30 Hz

  PWM1_IC1_magnetron_flow.period_array[PWM1_IC1_magnetron_flow.current_index] = PWM1_INITITAL_PERIOD;
  PWM2_IC2_linac_flow.period_array[PWM2_IC2_linac_flow.current_index] = PWM2_INITITAL_PERIOD;
  PWM3_IC3_hv_tank_flow.period_array[PWM3_IC3_hv_tank_flow.current_index] = PWM3_INITITAL_PERIOD;
  PWM4.period_array[PWM4.current_index] = PWM_INITIAL_PERIOD;
  PWM5.period_array[PWM5.current_index] = PWM_INITIAL_PERIOD;
  PWM6.period_array[PWM6.current_index] = PWM_INITIAL_PERIOD;

  
#define ICXCON_VALUE  (IC_TIMER2_SRC & IC_INT_1CAPTURE & IC_EVERY_EDGE)

  IC1CON = ICXCON_VALUE;
  IC2CON = ICXCON_VALUE;
  IC3CON = ICXCON_VALUE;
  IC4CON = ICXCON_VALUE;
  IC5CON = ICXCON_VALUE;
  IC6CON = ICXCON_VALUE;
}



void FlashLEDs(void) {
  switch (((global_data_A36746.startup_counter >> 4) & 0b11)) {
    
  case 0:
    PIN_LED_OPERATIONAL_GREEN = !OLL_LED_ON;
    PIN_LED_A_RED = !OLL_LED_ON;
    PIN_LED_B_GREEN = !OLL_LED_ON;
    break;
    
  case 1:
    PIN_LED_OPERATIONAL_GREEN = OLL_LED_ON;
    PIN_LED_A_RED = !OLL_LED_ON;
    PIN_LED_B_GREEN = !OLL_LED_ON;
    break;
    
  case 2:
    PIN_LED_OPERATIONAL_GREEN = OLL_LED_ON;
    PIN_LED_A_RED = OLL_LED_ON;
    PIN_LED_B_GREEN = !OLL_LED_ON;
    break;
    
  case 3:
    PIN_LED_OPERATIONAL_GREEN = OLL_LED_ON;
    PIN_LED_A_RED = OLL_LED_ON;
    PIN_LED_B_GREEN = OLL_LED_ON;
    break;
  }
}










void DoSF6Management(void) {
  switch (global_data_A36746.SF6_state) {
    
  case SF6_STATE_TEST:
    _STATUS_SF6_FILLING = 0;

    if (!_SYNC_CONTROL_RESET_ENABLE) {
      break;
    }

    if (global_data_A36746.coolant_temperature_kelvin < MINIMUM_COOLANT_TEMP_FOR_SF6_MANAGEMENT) {
      _STATUS_SF6_COOLANT_TOO_LOW = 1;
      break;
    }
    _STATUS_SF6_COOLANT_TOO_LOW = 0;

    if ((global_data_A36746.analog_input_SF6_pressure.reading_scaled_and_calibrated < MINIMUM_PRESSURE_FOR_SF6_MANAGEMENT) && (global_data_A36746.SF6_low_pressure_override_counter == 0)) {
      _STATUS_SF6_PRESSURE_TO_LOW_TO_MANAGE = 1;
      break;
    }
    _STATUS_SF6_PRESSURE_TO_LOW_TO_MANAGE = 0;

    if (global_data_A36746.analog_input_SF6_pressure.reading_scaled_and_calibrated > global_data_A36746.SF6_fill_threshold) {
      _STATUS_SF6_FILL_REQUIRED = 0;
      break;
    }
    _STATUS_SF6_FILL_REQUIRED = 1;

    if (!global_data_A36746.SF6_pulses_available) {
      _STATUS_SF6_NO_PULSES_AVAILABLE = 1;
      break;
    }
    _STATUS_SF6_NO_PULSES_AVAILABLE = 0;

    

    if (global_data_A36746.SF6_pulses_available) {
      global_data_A36746.SF6_pulses_available--;
    }

    if (global_data_A36746.SF6_low_pressure_override_counter) {
      global_data_A36746.SF6_low_pressure_override_counter--;
    }

    if (global_data_A36746.SF6_bottle_pulses_remaining) {
      global_data_A36746.SF6_bottle_pulses_remaining--;
    }
    

    ETMEEPromWritePage(ETM_EEPROM_PAGE_COOLING_INTERFACE, 2, &global_data_A36746.SF6_pulses_available);
    // This writes SF6_pulses_available and SF6_bottle_pulses_remaining to the external EEPROM with a single I2C command (and single FLASH write cycle)

    _STATUS_SF6_FILL_REQUIRED = 1;
    global_data_A36746.SF6_fill_threshold = SF6_MAXIMUM_TARGET_PRESSURE;
    global_data_A36746.SF6_state_charging_counter = 0;
    global_data_A36746.SF6_state = SF6_STATE_CHARGING;

    break;


  case SF6_STATE_CHARGING:
    global_data_A36746.SF6_state_charging_counter++;
    PIN_RELAY_OUT_SF6_SOLENOID = OLL_CLOSE_SOLENOID;
    _STATUS_SF6_SOLENOID_RELAY_CLOSED = 1;
    _STATUS_SF6_FILLING = 1;
    if (global_data_A36746.SF6_state_charging_counter >= SF6_TIME_CHARGING) {
      global_data_A36746.SF6_state_delay_counter = 0;
      global_data_A36746.SF6_state = SF6_STATE_DELAY;
    }
    break;

  case SF6_STATE_DELAY:
    PIN_RELAY_OUT_SF6_SOLENOID = !OLL_CLOSE_SOLENOID;
    _STATUS_SF6_SOLENOID_RELAY_CLOSED = 0;
    _STATUS_SF6_FILLING = 1;
    global_data_A36746.SF6_state_delay_counter++;
    if (global_data_A36746.SF6_state_delay_counter >= SF6_TIME_DELAY) {
      _STATUS_SF6_FILLING = 0;
      global_data_A36746.SF6_state = SF6_STATE_TEST;

    }
    break;

  default:
    global_data_A36746.SF6_state = SF6_STATE_DELAY;
    break;
  }
}




void __attribute__((interrupt, no_auto_psv)) _ADCInterrupt(void) {
  _ADIF = 0;
  
  // Copy Data From Buffer to RAM
  if (_BUFS) {
    // read ADCBUF 0-7
    global_data_A36746.analog_input_coolant_temperature.adc_accumulator += ADCBUF0;
    global_data_A36746.analog_input_cabinet_temperature.adc_accumulator += ADCBUF1;
    global_data_A36746.analog_input_linac_temperature.adc_accumulator   += ADCBUF2;
    global_data_A36746.analog_input_SF6_pressure.adc_accumulator        += ADCBUF3;

    global_data_A36926.analog_input_15v_mon.adc_accumulator             += ADCBUF4;
    global_data_A36926.analog_input_neg_15v_mon.adc_accumulator         += ADCBUF5;
    global_data_A36926.analog_input_5v_mon.adc_accumulator              += ADCBUF6;
    global_data_A36926.analog_input_pic_adc_test_dac.adc_accumulator    += ADCBUF7;
  } else {
    // read ADCBUF 8-15
    global_data_A36746.analog_input_coolant_temperature.adc_accumulator += ADCBUF8;
    global_data_A36746.analog_input_cabinet_temperature.adc_accumulator += ADCBUF9;
    global_data_A36746.analog_input_linac_temperature.adc_accumulator   += ADCBUFA;
    global_data_A36746.analog_input_SF6_pressure.adc_accumulator        += ADCBUFB;

    global_data_A36926.analog_input_15v_mon.adc_accumulator             += ADCBUFC;
    global_data_A36926.analog_input_neg_15v_mon.adc_accumulator         += ADCBUFD;
    global_data_A36926.analog_input_5v_mon.adc_accumulator              += ADCBUFE;
    global_data_A36926.analog_input_pic_adc_test_dac.adc_accumulator    += ADCBUFF;
  }
  
  global_data_A36746.accumulator_counter++;
  
  if (global_data_A36746.accumulator_counter >= 128) {
    global_data_A36746.accumulator_counter = 0;    
    
    global_data_A36746.analog_input_coolant_temperature.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples 
    global_data_A36746.analog_input_coolant_temperature.filtered_adc_reading = global_data_A36746.analog_input_coolant_temperature.adc_accumulator;
    global_data_A36746.analog_input_coolant_temperature.adc_accumulator = 0;

    global_data_A36746.analog_input_cabinet_temperature.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples 
    global_data_A36746.analog_input_cabinet_temperature.filtered_adc_reading = global_data_A36746.analog_input_cabinet_temperature.adc_accumulator;
    global_data_A36746.analog_input_cabinet_temperature.adc_accumulator = 0;

    global_data_A36746.analog_input_linac_temperature.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples 
    global_data_A36746.analog_input_linac_temperature.filtered_adc_reading = global_data_A36746.analog_input_cabinet_temperature.adc_accumulator;
    global_data_A36746.analog_input_linac_temperature.adc_accumulator = 0;

    global_data_A36746.analog_input_SF6_pressure.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples 
    global_data_A36746.analog_input_SF6_pressure.filtered_adc_reading = global_data_A36746.analog_input_SF6_pressure.adc_accumulator;
    global_data_A36746.analog_input_SF6_pressure.adc_accumulator = 0;
    
    global_data_A36926.analog_input_15v_mon.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples 
    global_data_A36926.analog_input_15v_mon.filtered_adc_reading = global_data_A36926.analog_input_15v_mon.adc_accumulator;
    global_data_A36926.analog_input_15v_mon.adc_accumulator = 0;
    
    global_data_A36926.analog_input_neg_15v_mon.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples 
    global_data_A36926.analog_input_neg_15v_mon.filtered_adc_reading = global_data_A36926.analog_input_neg_15v_mon.adc_accumulator;
    global_data_A36926.analog_input_neg_15v_mon.adc_accumulator = 0;            
    
    global_data_A36926.analog_input_5v_mon.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples 
    global_data_A36926.analog_input_5v_mon.filtered_adc_reading = global_data_A36926.analog_input_5v_mon.adc_accumulator;
    global_data_A36926.analog_input_5v_mon.adc_accumulator = 0;
    
    global_data_A36926.analog_input_pic_adc_test_dac.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples 
    global_data_A36926.analog_input_pic_adc_test_dac.filtered_adc_reading = global_data_A36926.analog_input_pic_adc_test_dac.adc_accumulator;
    global_data_A36926.analog_input_pic_adc_test_dac.adc_accumulator = 0;
    
  }
}




void __attribute__((interrupt, no_auto_psv)) _DefaultInterrupt(void) {
  // Clearly should not get here without a major problem occuring
  // DPARKER do something to save the state into a RAM location that is not re-initialized and then reset
  Nop();
  Nop();
  __asm__ ("Reset");
}
