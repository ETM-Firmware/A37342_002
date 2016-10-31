#include "A37342_002.h"

//#define __BENCH_TOP_MODE

void InitializeFlowMeter(TYPE_FLOW_READING* flow_ptr, unsigned int min_flow, unsigned int* ptr_icxbuf, unsigned int* ptr_tmrx);

void CaptureInput(TYPE_FLOW_READING* flow_ptr);

void UpdateFaults(void);


// A37342-002 This is the Can interface board configured as Cooling Interface board

_FOSC(ECIO & CSW_FSCM_OFF); 
_FWDT(WDT_ON & WDTPSA_512 & WDTPSB_8);  // 8 Second watchdog timer 
_FBORPOR(PWRT_OFF & BORV_45 & PBOR_OFF & MCLR_EN);
_FBS(WR_PROTECT_BOOT_OFF & NO_BOOT_CODE & NO_BOOT_EEPROM & NO_BOOT_RAM);
_FSS(WR_PROT_SEC_OFF & NO_SEC_CODE & NO_SEC_EEPROM & NO_SEC_RAM);
_FGS(CODE_PROT_OFF);
_FICD(PGD);


const unsigned int ThermistorLookupTable[1024] = {THERMISTOR_LOOK_UP_TABLE_VALUES};


TYPE_COOLING_GLOBALS global_data_A37342;

void DoSF6Management(void);




void InitializeA36746(void);
void DoStateMachine(void);
void DoA36746(void);
void FlashLEDs(void);



int main(void) {
  global_data_A37342.control_state = STATE_STARTUP;
  while (1) {
    DoStateMachine();
  }
}



void DoStateMachine(void) {
  switch (global_data_A37342.control_state) {
    
  case STATE_STARTUP:
    InitializeA36746();
    _CONTROL_NOT_CONFIGURED = 0;
    _CONTROL_NOT_READY = 1;
    PIN_RELAY_OUT_SF6_SOLENOID = !OLL_CLOSE_SOLENOID;
    global_data_A37342.startup_counter = 0;
    while (global_data_A37342.control_state == STATE_STARTUP) {
      DoA36746();
      FlashLEDs();
      if (global_data_A37342.startup_counter >= STARTUP_LED_FLASH_TIME) {
	global_data_A37342.control_state = STATE_NOT_READY;	
      }
    }
    break;

    
  case STATE_NOT_READY:
    PIN_LED_OPERATIONAL_GREEN = OLL_LED_ON;
    PIN_RELAY_OUT_SF6_SOLENOID = !OLL_CLOSE_SOLENOID;
    _CONTROL_NOT_READY = 1;
    while (global_data_A37342.control_state == STATE_NOT_READY) {
      DoA36746();
      if (ETMCanSlaveGetSyncMsgResetEnable()) {
	global_data_A37342.control_state = STATE_TESTING;
      }
    }
    break;


  case STATE_TESTING:
    _CONTROL_NOT_READY = 1;
    global_data_A37342.test_timer = 0;
    PIN_RELAY_OUT_SF6_SOLENOID = !OLL_CLOSE_SOLENOID;
    while (global_data_A37342.control_state == STATE_TESTING) {
      DoA36746();
      if (global_data_A37342.test_timer >= COOLING_INTERFACE_BOARD_TEST_TIME) {
	global_data_A37342.control_state = STATE_READY;
      }

      if (_FAULT_REGISTER) {
	global_data_A37342.control_state = STATE_NOT_READY;
      }
    }
    break;


  case STATE_READY:
    _FAULT_REGISTER = 0;
    _CONTROL_NOT_READY = 0;
    PIN_RELAY_OUT_SF6_SOLENOID = OLL_CLOSE_SOLENOID;
    while (global_data_A37342.control_state == STATE_READY) {
      DoA36746();
      if (_FAULT_REGISTER) {
	global_data_A37342.control_state = STATE_NOT_READY;
      }
    }
    break;

  default:
    global_data_A37342.control_state = STATE_STARTUP;
    break;
  }
}



/*
  Example Case
  IC register reads  - 0xFFFF followed by 0x0001
  That could be a time of 2 timer tics or 2^16 +2 timer tics or n*2^16 +2 timer tics
  One is very high frequency and the other is a very low frequency.  How do we determine?
*/


void InitializeFlowMeter(TYPE_FLOW_READING* flow_ptr, unsigned int min_flow, unsigned int* ptr_icxbuf, unsigned int* ptr_tmrx) {
  flow_ptr->period_array[0] = PWM_MAX_PERIOD;
  flow_ptr->period_array[1] = PWM_MAX_PERIOD;
  flow_ptr->period_array[2] = PWM_MAX_PERIOD;
  flow_ptr->period_array[3] = PWM_MAX_PERIOD;

  flow_ptr->period_array[4] = PWM_MAX_PERIOD;
  flow_ptr->period_array[5] = PWM_MAX_PERIOD;
  flow_ptr->period_array[6] = PWM_MAX_PERIOD;
  flow_ptr->period_array[7] = PWM_MAX_PERIOD;

  flow_ptr->period_array[8] = PWM_MAX_PERIOD;
  flow_ptr->period_array[9] = PWM_MAX_PERIOD;
  flow_ptr->period_array[10] = PWM_MAX_PERIOD;
  flow_ptr->period_array[11] = PWM_MAX_PERIOD;

  flow_ptr->period_array[12] = PWM_MAX_PERIOD;
  flow_ptr->period_array[13] = PWM_MAX_PERIOD;
  flow_ptr->period_array[14] = PWM_MAX_PERIOD;
  flow_ptr->period_array[15] = PWM_MAX_PERIOD;
    
  flow_ptr->minimum_flow = min_flow;
  flow_ptr->ICXBUF_ptr = ptr_icxbuf;
  flow_ptr->TMRX_ptr = ptr_tmrx;
  ETMDigitalInitializeInput(&flow_ptr->digital_fault, 0, 100); // Initialize to not faulted with 1 second delay.
}


void UpdateFlowMeterInputCapture(TYPE_FLOW_READING* flow_ptr) {
  unsigned int latest_tmr;

  // Check to see if there is an overflow
  latest_tmr     = *flow_ptr->TMRX_ptr;
  latest_tmr    -= flow_ptr->previous_timer_reading;
  if (latest_tmr >= PWM_MAX_PERIOD) { 
    flow_ptr->long_pulse = 1;
    flow_ptr->array_index++;
    flow_ptr->array_index &= 0x000F;
    flow_ptr->period_array[flow_ptr->array_index] = PWM_MAX_PERIOD;
  }

  // Figure out if the interupt flag is active and if so update that flow meter
  if (flow_ptr->ICXBUF_ptr == &IC1BUF) {
    if (_IC1IF) {
      _IC1IF = 0;
      CaptureInput(flow_ptr);
    }
  }
  
  if (flow_ptr->ICXBUF_ptr == &IC2BUF) {
    if (_IC2IF) {
      _IC2IF = 0;
      CaptureInput(flow_ptr);
    }
  }
  
  if (flow_ptr->ICXBUF_ptr == &IC3BUF) {
    if (_IC3IF) {
      _IC3IF = 0;
      CaptureInput(flow_ptr);
    }
  } 
  
  if (flow_ptr->ICXBUF_ptr == &IC4BUF) {
    if (_IC4IF) {
      _IC4IF = 0;
      CaptureInput(flow_ptr);
    }
  } 
  
  if (flow_ptr->ICXBUF_ptr == &IC5BUF) {
    if (_IC5IF) {
      _IC5IF = 0;
      CaptureInput(flow_ptr);
    }
  } 

  if (flow_ptr->ICXBUF_ptr == &IC6BUF) {
    if (_IC6IF) {
      _IC6IF = 0;
      CaptureInput(flow_ptr);
    }
  }
}

void CaptureInput(TYPE_FLOW_READING* flow_ptr) {
  unsigned int latest_capture;
  unsigned int current_period;

  while((*((flow_ptr->ICXBUF_ptr) + 1)) & 0x0008) {
    //while(IC1CON & 0x0008) {
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
    flow_ptr->period_array[flow_ptr->array_index] = current_period;
  }
}

unsigned int CheckFlowMeterFault(TYPE_FLOW_READING* flow_ptr) {
  unsigned long period_lng;
  unsigned int period;
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
  //period_lng = 390620;
  //period_lng /= period;
  //flow_ptr->frequency = period_lng;  // This is now in deci
  //flow_ptr->frequency = RCFilterNTau(flow_ptr->frequency, period_lng, 12);
  flow_ptr->frequency = 39062 / period;
  
  if (flow_ptr->frequency < FLOW_METER_MIN_FREQUENCY) {
    flow_ptr->flow_reading = 0;
  } else {
    flow_ptr->flow_reading = FLOW_METER_ML_PER_HZ*flow_ptr->frequency + FLOW_METER_CONSTANT;
    //flow_ptr->flow_reading = RCFilterNTau(flow_ptr->flow_reading, (FLOW_METER_ML_PER_HZ*flow_ptr->frequency + FLOW_METER_CONSTANT), RC_FILTER_256_TAU);
  }

  if (flow_ptr->minimum_flow == 0) {
    return 0;
  }

  if (flow_ptr->flow_reading < flow_ptr->minimum_flow) {
    ETMDigitalUpdateInput(&flow_ptr->digital_fault, 1);
  } else {
    ETMDigitalUpdateInput(&flow_ptr->digital_fault, 0);
  }
  
  return ETMDigitalFilteredOutput(&flow_ptr->digital_fault);
}

unsigned int ConvertThermistorVoltageToKelvin(unsigned int thermistor_voltage) {
  unsigned int thermistor_index;
  thermistor_index = (thermistor_voltage >> 2);
  if (thermistor_index > 1023) {
    thermistor_index = 1023;
  }
  return ThermistorLookupTable[thermistor_index];
  
}

void DoA36746(void) {

  ETMCanSlaveDoCan();

  // DPARKER - Need to figure out how to update flow meters only if the _IF is active

  // This needs to happen every time through the control loop to capture high frequency PWM
  UpdateFlowMeterInputCapture(&global_data_A37342.flow_meter_1_magnetron);
  UpdateFlowMeterInputCapture(&global_data_A37342.flow_meter_2_linac);
  UpdateFlowMeterInputCapture(&global_data_A37342.flow_meter_3_hv_tank);
  UpdateFlowMeterInputCapture(&global_data_A37342.flow_meter_4_hvps);
  UpdateFlowMeterInputCapture(&global_data_A37342.flow_meter_5_circulator);
  UpdateFlowMeterInputCapture(&global_data_A37342.flow_meter_6_alternate);
  
  if (_T1IF) {
    // Timer has expired so execute the scheduled code (should be once every 10ms)
    _T1IF = 0;

    DoSF6Management();

    global_data_A37342.test_timer++;
    if (global_data_A37342.test_timer > COOLING_INTERFACE_BOARD_TEST_TIME) {
      global_data_A37342.test_timer = COOLING_INTERFACE_BOARD_TEST_TIME;
    }
    
    
    global_data_A37342.startup_counter++;
    if (global_data_A37342.startup_counter > STARTUP_LED_FLASH_TIME) {
      global_data_A37342.startup_counter = STARTUP_LED_FLASH_TIME;
    }
  

    // Do Math on ADC inputs
    // Scale the ADC readings to engineering units
    ETMAnalogScaleCalibrateADCReading(&global_data_A37342.analog_input_thermistor_1);
    ETMAnalogScaleCalibrateADCReading(&global_data_A37342.analog_input_thermistor_2);
    ETMAnalogScaleCalibrateADCReading(&global_data_A37342.analog_input_thermistor_3);
    ETMAnalogScaleCalibrateADCReading(&global_data_A37342.analog_input_SF6_pressure);

    // Convert the temperature reading voltage into temperature
    global_data_A37342.coolant_temperature_kelvin = ConvertThermistorVoltageToKelvin(global_data_A37342.analog_input_thermistor_1.reading_scaled_and_calibrated);
    global_data_A37342.cabinet_temperature_kelvin = ConvertThermistorVoltageToKelvin(global_data_A37342.analog_input_thermistor_2.reading_scaled_and_calibrated);
    global_data_A37342.linac_temperature_kelvin   = ConvertThermistorVoltageToKelvin(global_data_A37342.analog_input_thermistor_3.reading_scaled_and_calibrated);

    ETMCanSlaveSetDebugRegister(0x0, global_data_A37342.flow_meter_1_magnetron.period_array[0]);
    ETMCanSlaveSetDebugRegister(0x1, global_data_A37342.flow_meter_1_magnetron.period_array[1]);
    ETMCanSlaveSetDebugRegister(0x2, global_data_A37342.flow_meter_1_magnetron.array_index);
    ETMCanSlaveSetDebugRegister(0x3, global_data_A37342.flow_meter_1_magnetron.long_pulse);

    ETMCanSlaveSetDebugRegister(0x4, global_data_A37342.flow_meter_1_magnetron.minimum_flow);
    ETMCanSlaveSetDebugRegister(0x5, global_data_A37342.flow_meter_1_magnetron.frequency);
    ETMCanSlaveSetDebugRegister(0x6, global_data_A37342.flow_meter_1_magnetron.flow_reading);
    ETMCanSlaveSetDebugRegister(0x7, global_data_A37342.analog_input_SF6_pressure.filtered_adc_reading);//global_data_A37342.control_state);


    // Update all the logging data
    slave_board_data.log_data[0] = global_data_A37342.flow_meter_1_magnetron.flow_reading;
    slave_board_data.log_data[1] = global_data_A37342.flow_meter_2_linac.flow_reading;
    slave_board_data.log_data[2] = global_data_A37342.flow_meter_3_hv_tank.flow_reading;
    slave_board_data.log_data[3] = global_data_A37342.flow_meter_4_hvps.flow_reading;
    slave_board_data.log_data[4] = global_data_A37342.flow_meter_5_circulator.flow_reading;
    slave_board_data.log_data[5] = global_data_A37342.flow_meter_6_alternate.flow_reading;

    slave_board_data.log_data[8] = global_data_A37342.linac_temperature_kelvin;
    slave_board_data.log_data[9] = global_data_A37342.coolant_temperature_kelvin;
    slave_board_data.log_data[10] = global_data_A37342.cabinet_temperature_kelvin;
    slave_board_data.log_data[11] = global_data_A37342.analog_input_SF6_pressure.reading_scaled_and_calibrated;
    slave_board_data.log_data[12] = global_data_A37342.SF6_pulses_available;
    slave_board_data.log_data[13] = global_data_A37342.SF6_low_pressure_override_counter;
    slave_board_data.log_data[14] = global_data_A37342.SF6_bottle_pulses_remaining;


    

    
    


    // Converters the PWM readings into flow and stores in the PWM structure
    //CalculateFlowMeterFlow(&global_data_A37342.flow_meter_1_magnetron);
    //CalculateFlowMeterFlow(&global_data_A37342.flow_meter_2_linac);
    //CalculateFlowMeterFlow(&global_data_A37342.flow_meter_3_hv_tank);
    //CalculateFlowMeterFlow(&global_data_A37342.flow_meter_4_hvps);
    //CalculateFlowMeterFlow(&global_data_A37342.flow_meter_5_circulator);
    //CalculateFlowMeterFlow(&global_data_A37342.flow_meter_6_alternate);


    // Update Temperature Switch Inputs
    ETMDigitalUpdateInput(&global_data_A37342.digital_input_7_cabinet_temp_switch, PIN_DIGITAL_INPUT_1_CABINET_TEMP_SWITCH);
    ETMDigitalUpdateInput(&global_data_A37342.digital_input_8_coolant_temp_switch, PIN_DIGITAL_INPUT_2_COOLANT_TEMP_SWITCH);

    // ------------ CHECK FOR FAULTS ----------- //
    UpdateFaults();

  }
}

void UpdateFaults(void) {

  _STATUS_FLOW_OK = 1;

  if (ETMCanSlaveGetComFaultStatus()) {
    _FAULT_CAN_COMMUNICATION_LATCHED = 1;
  } else {
    if (ETMCanSlaveGetSyncMsgResetEnable()) {
      _FAULT_CAN_COMMUNICATION_LATCHED = 0;
    }
  }
  
  if (CheckFlowMeterFault(&global_data_A37342.flow_meter_1_magnetron)) {
    _FAULT_FLOW_SENSOR_1 = 1;
    _STATUS_FLOW_OK = 0;
  } else {
    if (ETMCanSlaveGetSyncMsgResetEnable()) {
      _FAULT_FLOW_SENSOR_1 = 0;
    }    
  }
  
  if (CheckFlowMeterFault(&global_data_A37342.flow_meter_2_linac)) {
#ifndef __BENCH_TOP_MODE
    _FAULT_FLOW_SENSOR_2 = 1;
    _STATUS_FLOW_OK = 0;
#endif
  } else {
    if (ETMCanSlaveGetSyncMsgResetEnable()) {
      _FAULT_FLOW_SENSOR_2 = 0;
    }    
  }

  if (CheckFlowMeterFault(&global_data_A37342.flow_meter_3_hv_tank)) {
#ifndef __BENCH_TOP_MODE
    _FAULT_FLOW_SENSOR_3 = 1;
    _STATUS_FLOW_OK = 0;
#endif
  } else {
    if (ETMCanSlaveGetSyncMsgResetEnable()) {
      _FAULT_FLOW_SENSOR_3 = 0;
    }    
  }

  if (CheckFlowMeterFault(&global_data_A37342.flow_meter_4_hvps)) {
#ifndef __BENCH_TOP_MODE
    _FAULT_FLOW_SENSOR_4 = 1;
    _STATUS_FLOW_OK = 0;
#endif
  } else {
    if (ETMCanSlaveGetSyncMsgResetEnable()) {
      _FAULT_FLOW_SENSOR_4 = 0;
    }    
  }

  if (CheckFlowMeterFault(&global_data_A37342.flow_meter_5_circulator)) {
#ifndef __BENCH_TOP_MODE
    _FAULT_FLOW_SENSOR_5 = 1;
    _STATUS_FLOW_OK = 0;
#endif
  } else {
    if (ETMCanSlaveGetSyncMsgResetEnable()) {
      _FAULT_FLOW_SENSOR_5 = 0;
    }    
  }

  if (CheckFlowMeterFault(&global_data_A37342.flow_meter_6_alternate)) {
#ifndef __BENCH_TOP_MODE
    _FAULT_FLOW_SENSOR_6 = 1;
    _STATUS_FLOW_OK = 0;
#endif
  } else {
    if (ETMCanSlaveGetSyncMsgResetEnable()) {
      _FAULT_FLOW_SENSOR_6 = 0;
    }    
  }

    
  if (ETMDigitalFilteredOutput(&global_data_A37342.digital_input_7_cabinet_temp_switch) == ILL_TEMP_SWITCH_OVER_TEMP) {
#ifndef __BENCH_TOP_MODE
    _FAULT_CABINET_TEMP_SWITCH = 1;
#endif
  } else {
    if (ETMCanSlaveGetSyncMsgResetEnable()) {
      _FAULT_CABINET_TEMP_SWITCH = 0;
    }
  }

  if (ETMDigitalFilteredOutput(&global_data_A37342.digital_input_8_coolant_temp_switch) == ILL_TEMP_SWITCH_OVER_TEMP) {
#ifndef __BENCH_TOP_MODE
    //_FAULT_COOLANT_TEMP_SWITCH = 1;
#endif
  } else {
    if (ETMCanSlaveGetSyncMsgResetEnable()) {
      _FAULT_COOLANT_TEMP_SWITCH = 0;
    }
  }

    
  // Check for over temperature on Thermistor 1
  // We are using the under absolute function because as temperature goes up, resistance (and voltage) go down 
  if (ETMAnalogCheckUnderAbsolute(&global_data_A37342.analog_input_thermistor_1)) {
#ifndef __BENCH_TOP_MODE
    _FAULT_COOLANT_OVER_TEMP = 1;
#endif
  } else {
    if (ETMCanSlaveGetSyncMsgResetEnable()) {
      _FAULT_COOLANT_OVER_TEMP = 0;
    }
  }

  // Check for over temperature on Thermistor 2
  if (ETMAnalogCheckUnderAbsolute(&global_data_A37342.analog_input_thermistor_2)) {
    // We are using the under absolute function because as temperature goes up, resistance (and voltage) go down 
#ifndef __BENCH_TOP_MODE
    _FAULT_CABINET_OVER_TEMP = 1;
#endif
  } else {
    if (ETMCanSlaveGetSyncMsgResetEnable()) {
      _FAULT_CABINET_OVER_TEMP = 0;
    }
  }
      
  // Check for over temperature on Thermistor 3
//  if (ETMAnalogCheckUnderAbsolute(&global_data_A37342.analog_input_thermistor_3)) {
//    _FAULT_CABINET_OVER_TEMP = 1;
//  } else {
//    if (ETMCanSlaveGetSyncMsgResetEnable()) {
//      _FAULT_CABINET_OVER_TEMP = 0;
//    }
//  }


  // Check for under pressure on SF6
  if (ETMAnalogCheckUnderAbsolute(&global_data_A37342.analog_input_SF6_pressure)) {
#ifndef __BENCH_TOP_MODE
    _FAULT_SF6_UNDER_PRESSURE = 1;
#endif
  } else if (ETMCanSlaveGetSyncMsgResetEnable()) {
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
  TRISA = A37342_TRISA_VALUE;
  TRISB = A37342_TRISB_VALUE;
  TRISC = A37342_TRISC_VALUE;
  TRISD = A37342_TRISD_VALUE;
  TRISF = A37342_TRISF_VALUE;
  TRISG = A37342_TRISG_VALUE;


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
  ETMEEPromUseExternal();
  ETMEEPromConfigureExternalDevice(EEPROM_SIZE_8K_BYTES, FCY_CLK, 400000, EEPROM_I2C_ADDRESS_0, 1);

  // Initialize the Can module
  ETMCanSlaveInitialize(CAN_PORT_1, FCY_CLK, ETM_CAN_ADDR_COOLING_INTERFACE_BOARD, _PIN_RG13, 4, _PIN_RA7, _PIN_RG12);
  ETMCanSlaveLoadConfiguration(37342, 002, FIRMWARE_AGILE_REV, FIRMWARE_BRANCH, FIRMWARE_BRANCH_REV);


  // Initialize the Analog Input & Output Scaling


  ETMAnalogInitializeInput(&global_data_A37342.analog_input_thermistor_1,
			   MACRO_DEC_TO_SCALE_FACTOR_16(TEMPERATURE_SENSOR_FIXED_SCALE),
			   OFFSET_ZERO,
			   ANALOG_INPUT_1,
			   NO_OVER_TRIP,
			   COOLANT_TRIP_THERMISTOR_VOLTAGE,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_RELATIVE_COUNTER,
			   ANALOG_TEMPERATURE_TRIP_TIME);
  
  ETMAnalogInitializeInput(&global_data_A37342.analog_input_thermistor_2,
			   MACRO_DEC_TO_SCALE_FACTOR_16(TEMPERATURE_SENSOR_FIXED_SCALE),
			   OFFSET_ZERO,
			   ANALOG_INPUT_2,
			   NO_OVER_TRIP,
			   CABINET_TRIP_THERMISTOR_VOLTAGE,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_RELATIVE_COUNTER,
			   ANALOG_TEMPERATURE_TRIP_TIME);

  ETMAnalogInitializeInput(&global_data_A37342.analog_input_thermistor_3,
			   MACRO_DEC_TO_SCALE_FACTOR_16(TEMPERATURE_SENSOR_FIXED_SCALE),
			   OFFSET_ZERO,
			   ANALOG_INPUT_3,
			   NO_OVER_TRIP,
			   LINAC_TRIP_THERMISTOR_VOLTAGE,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_RELATIVE_COUNTER,
			   ANALOG_TEMPERATURE_TRIP_TIME);


  ETMAnalogInitializeInput(&global_data_A37342.analog_input_SF6_pressure,
			   MACRO_DEC_TO_SCALE_FACTOR_16(SF6_SENSOR_FIXED_SCALE),
			   SF6_SENSOR_FIXED_OFFSET,
			   ANALOG_INPUT_4,
			   NO_OVER_TRIP,
			   SF6_UNDER_PRESSURE_TRIP,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_RELATIVE_COUNTER,
			   SF6_PRESSURE_TRIP_TIME);

  ETMAnalogInitializeInput(&global_data_A37342.analog_input_5v_mon,
			   MACRO_DEC_TO_SCALE_FACTOR_16(.12500),
			   OFFSET_ZERO,
			   ANALOG_INPUT_5,
			   PWR_5V_OVER_FLT,
			   PWR_5V_UNDER_FLT,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_RELATIVE_COUNTER,
			   NO_ABSOLUTE_COUNTER);
  
  ETMAnalogInitializeInput(&global_data_A37342.analog_input_15v_mon,
			   MACRO_DEC_TO_SCALE_FACTOR_16(.25063),
			   OFFSET_ZERO,
			   ANALOG_INPUT_6,
			   PWR_15V_OVER_FLT,
			   PWR_15V_UNDER_FLT,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_RELATIVE_COUNTER,
			   NO_ABSOLUTE_COUNTER);
  
  ETMAnalogInitializeInput(&global_data_A37342.analog_input_neg_15v_mon,
			   MACRO_DEC_TO_SCALE_FACTOR_16(.06250),
			   OFFSET_ZERO,
			   ANALOG_INPUT_7,
			   PWR_NEG_15V_OVER_FLT,
			   PWR_NEG_15V_UNDER_FLT,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_RELATIVE_COUNTER,
			   NO_ABSOLUTE_COUNTER);
  

  ETMDigitalInitializeInput(&global_data_A37342.digital_input_7_cabinet_temp_switch, !ILL_TEMP_SWITCH_OVER_TEMP, TEMPERATURE_SWITCH_FILTER_TIME);
  ETMDigitalInitializeInput(&global_data_A37342.digital_input_8_coolant_temp_switch, !ILL_TEMP_SWITCH_OVER_TEMP, TEMPERATURE_SWITCH_FILTER_TIME);


  global_data_A37342.SF6_state = SF6_STATE_DELAY;
  global_data_A37342.SF6_fill_threshold = SF6_MINIMUM_TARGET_PRESSURE;
  global_data_A37342.SF6_low_pressure_override_counter = 0;
  global_data_A37342.SF6_state_delay_counter = 0;
  
  ETMEEPromReadPage(ETM_EEPROM_PAGE_COOLING_INTERFACE, 2, &global_data_A37342.SF6_pulses_available);
  // This reads SF6_pulses_available and SF6_bottle_pulses_remaining from the external EEPROM with a single I2C command
  if (global_data_A37342.SF6_pulses_available > 25) {
    global_data_A37342.SF6_pulses_available = 25;
  }
  if (global_data_A37342.SF6_bottle_pulses_remaining > 30000) {
    global_data_A37342.SF6_bottle_pulses_remaining = 0;
  }


  InitializeFlowMeter(&global_data_A37342.flow_meter_1_magnetron,  MINIMUM_FLOW_MAGNETRON,  (unsigned int*)&IC1BUF, (unsigned int*)&TMR2);
  InitializeFlowMeter(&global_data_A37342.flow_meter_2_linac,      MINIMUM_FLOW_LINAC,      (unsigned int*)&IC2BUF, (unsigned int*)&TMR2);
  InitializeFlowMeter(&global_data_A37342.flow_meter_3_hv_tank,    MINIMUM_FLOW_HV_TANK,    (unsigned int*)&IC3BUF, (unsigned int*)&TMR2);
  InitializeFlowMeter(&global_data_A37342.flow_meter_4_hvps,       MINIMUM_FLOW_HVPS,       (unsigned int*)&IC4BUF, (unsigned int*)&TMR2);
  InitializeFlowMeter(&global_data_A37342.flow_meter_5_circulator, MINIMUM_FLOW_CIRCULATOR, (unsigned int*)&IC5BUF, (unsigned int*)&TMR2);
  InitializeFlowMeter(&global_data_A37342.flow_meter_6_alternate,  MINIMUM_FLOW_SPARE,      (unsigned int*)&IC6BUF, (unsigned int*)&TMR2);

  // DPARKER consider moving ICXCON to Initialize Flow Meter
#define ICXCON_VALUE  (IC_TIMER2_SRC & IC_INT_1CAPTURE & IC_EVERY_EDGE)
  
  IC1CON = ICXCON_VALUE;
  IC2CON = ICXCON_VALUE;
  IC3CON = ICXCON_VALUE;
  IC4CON = ICXCON_VALUE;
  IC5CON = ICXCON_VALUE;
  IC6CON = ICXCON_VALUE;

}


void FlashLEDs(void) {
  switch (((global_data_A37342.startup_counter >> 4) & 0b11)) {
    
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
  switch (global_data_A37342.SF6_state) {
    
  case SF6_STATE_TEST:
    _STATUS_SF6_FILLING = 0;

    if (!ETMCanSlaveGetSyncMsgResetEnable()) {
      break;
      // DPARKER do we need to move to a know state here?? For example close the SF6 contactor??
    }

    if (global_data_A37342.coolant_temperature_kelvin < MINIMUM_COOLANT_TEMP_FOR_SF6_MANAGEMENT) {
      _STATUS_SF6_COOLANT_TOO_LOW = 1;
      break;
    }
    _STATUS_SF6_COOLANT_TOO_LOW = 0;

    if ((global_data_A37342.analog_input_SF6_pressure.reading_scaled_and_calibrated < MINIMUM_PRESSURE_FOR_SF6_MANAGEMENT) && (global_data_A37342.SF6_low_pressure_override_counter == 0)) {
      _STATUS_SF6_PRESSURE_TO_LOW_TO_MANAGE = 1;
      break;
    }
    _STATUS_SF6_PRESSURE_TO_LOW_TO_MANAGE = 0;

    if (global_data_A37342.analog_input_SF6_pressure.reading_scaled_and_calibrated > global_data_A37342.SF6_fill_threshold) {
      _STATUS_SF6_FILL_REQUIRED = 0;
      break;
    }
    _STATUS_SF6_FILL_REQUIRED = 1;

    if (!global_data_A37342.SF6_pulses_available) {
      _STATUS_SF6_NO_PULSES_AVAILABLE = 1;
      break;
    }
    _STATUS_SF6_NO_PULSES_AVAILABLE = 0;

    

    if (global_data_A37342.SF6_pulses_available) {
      global_data_A37342.SF6_pulses_available--;
    }

    if (global_data_A37342.SF6_low_pressure_override_counter) {
      global_data_A37342.SF6_low_pressure_override_counter--;
    }

    if (global_data_A37342.SF6_bottle_pulses_remaining) {
      global_data_A37342.SF6_bottle_pulses_remaining--;
    }
    
    // This writes SF6_pulses_available and SF6_bottle_pulses_remaining to the external EEPROM with a single I2C command (and single FLASH write cycle)
    ETMEEPromWritePage(ETM_EEPROM_PAGE_COOLING_INTERFACE, 2, &global_data_A37342.SF6_pulses_available);

    
    _STATUS_SF6_FILL_REQUIRED = 1;
    global_data_A37342.SF6_fill_threshold = SF6_MAXIMUM_TARGET_PRESSURE;
    global_data_A37342.SF6_state_charging_counter = 0;
    global_data_A37342.SF6_state = SF6_STATE_CHARGING;

    break;


  case SF6_STATE_CHARGING:
    global_data_A37342.SF6_state_charging_counter++;
    //    PIN_RELAY_OUT_SF6_SOLENOID = OLL_CLOSE_SOLENOID;
    _STATUS_SF6_SOLENOID_RELAY_CLOSED = 1;
    _STATUS_SF6_FILLING = 1;
    if (global_data_A37342.SF6_state_charging_counter >= SF6_TIME_CHARGING) {
      global_data_A37342.SF6_state_delay_counter = 0;
      global_data_A37342.SF6_state = SF6_STATE_DELAY;
    }
    break;

  case SF6_STATE_DELAY:
    //PIN_RELAY_OUT_SF6_SOLENOID = !OLL_CLOSE_SOLENOID;
    _STATUS_SF6_SOLENOID_RELAY_CLOSED = 0;
    _STATUS_SF6_FILLING = 1;
    global_data_A37342.SF6_state_delay_counter++;
    if (global_data_A37342.SF6_state_delay_counter >= SF6_TIME_DELAY) {
      _STATUS_SF6_FILLING = 0;
      global_data_A37342.SF6_state = SF6_STATE_TEST;

    }
    break;

  default:
    global_data_A37342.SF6_state = SF6_STATE_DELAY;
    break;
  }
}




void __attribute__((interrupt, no_auto_psv)) _ADCInterrupt(void) {
  _ADIF = 0;
  
  // Copy Data From Buffer to RAM
  if (_BUFS) {
    // read ADCBUF 0-7
    global_data_A37342.analog_input_thermistor_1.adc_accumulator        += ADCBUF0;
    global_data_A37342.analog_input_thermistor_2.adc_accumulator        += ADCBUF1;
    global_data_A37342.analog_input_thermistor_3.adc_accumulator        += ADCBUF2;
    global_data_A37342.analog_input_SF6_pressure.adc_accumulator        += ADCBUF3;

    global_data_A37342.analog_input_15v_mon.adc_accumulator             += ADCBUF4;
    global_data_A37342.analog_input_neg_15v_mon.adc_accumulator         += ADCBUF5;
    global_data_A37342.analog_input_5v_mon.adc_accumulator              += ADCBUF6;
    global_data_A37342.adc_test                                          = (ADCBUF7 << 4);
  } else {
    // read ADCBUF 8-15
    global_data_A37342.analog_input_thermistor_1.adc_accumulator        += ADCBUF8;
    global_data_A37342.analog_input_thermistor_2.adc_accumulator        += ADCBUF9;
    global_data_A37342.analog_input_thermistor_3.adc_accumulator        += ADCBUFA;
    global_data_A37342.analog_input_SF6_pressure.adc_accumulator        += ADCBUFB;

    global_data_A37342.analog_input_15v_mon.adc_accumulator             += ADCBUFC;
    global_data_A37342.analog_input_neg_15v_mon.adc_accumulator         += ADCBUFD;
    global_data_A37342.analog_input_5v_mon.adc_accumulator              += ADCBUFE;
    global_data_A37342.adc_test                                          = (ADCBUFF << 4);
  }
  
  global_data_A37342.accumulator_counter++;
  
  if (global_data_A37342.accumulator_counter >= 128) {
    global_data_A37342.accumulator_counter = 0;    
    
    global_data_A37342.analog_input_thermistor_1.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples 
    global_data_A37342.analog_input_thermistor_1.filtered_adc_reading = global_data_A37342.analog_input_thermistor_1.adc_accumulator;
    global_data_A37342.analog_input_thermistor_1.adc_accumulator = 0;

    global_data_A37342.analog_input_thermistor_2.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples 
    global_data_A37342.analog_input_thermistor_2.filtered_adc_reading = global_data_A37342.analog_input_thermistor_2.adc_accumulator;
    global_data_A37342.analog_input_thermistor_2.adc_accumulator = 0;

    global_data_A37342.analog_input_thermistor_3.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples 
    global_data_A37342.analog_input_thermistor_3.filtered_adc_reading = global_data_A37342.analog_input_thermistor_3.adc_accumulator;
    global_data_A37342.analog_input_thermistor_3.adc_accumulator = 0;

    global_data_A37342.analog_input_SF6_pressure.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples 
    global_data_A37342.analog_input_SF6_pressure.filtered_adc_reading = global_data_A37342.analog_input_SF6_pressure.adc_accumulator;
    global_data_A37342.analog_input_SF6_pressure.adc_accumulator = 0;
    
    global_data_A37342.analog_input_15v_mon.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples 
    global_data_A37342.analog_input_15v_mon.filtered_adc_reading = global_data_A37342.analog_input_15v_mon.adc_accumulator;
    global_data_A37342.analog_input_15v_mon.adc_accumulator = 0;
    
    global_data_A37342.analog_input_neg_15v_mon.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples 
    global_data_A37342.analog_input_neg_15v_mon.filtered_adc_reading = global_data_A37342.analog_input_neg_15v_mon.adc_accumulator;
    global_data_A37342.analog_input_neg_15v_mon.adc_accumulator = 0;            
    
    global_data_A37342.analog_input_5v_mon.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples 
    global_data_A37342.analog_input_5v_mon.filtered_adc_reading = global_data_A37342.analog_input_5v_mon.adc_accumulator;
    global_data_A37342.analog_input_5v_mon.adc_accumulator = 0;
    
  }
}



void ETMCanSlaveExecuteCMDBoardSpecific(ETMCanMessage* message_ptr) {
  unsigned int index_word;

  index_word = message_ptr->word3;
  switch (index_word) 
    {
      /*
	Place all board specific commands here
      */

    case ETM_CAN_REGISTER_COOLING_CMD_SF6_PULSE_LIMIT_OVERRIDE:
      global_data_A37342.SF6_pulses_available = 25;
      ETMEEPromWritePage(ETM_EEPROM_PAGE_COOLING_INTERFACE, 2, &global_data_A37342.SF6_pulses_available);
      break;
      
    case ETM_CAN_REGISTER_COOLING_CMD_RESET_BOTTLE_COUNT:
      global_data_A37342.SF6_bottle_pulses_remaining = message_ptr->word0;
      ETMEEPromWritePage(ETM_EEPROM_PAGE_COOLING_INTERFACE, 2, &global_data_A37342.SF6_pulses_available);
      break;
      
    case ETM_CAN_REGISTER_COOLING_CMD_SF6_LEAK_LIMIT_OVERRIDE:
      global_data_A37342.SF6_low_pressure_override_counter = message_ptr->word0;
      break;

    default:
      //local_can_errors.invalid_index++;
      break;
    }
}


void __attribute__((interrupt, no_auto_psv)) _DefaultInterrupt(void) {
  // Clearly should not get here without a major problem occuring
  // DPARKER do something to save the state into a RAM location that is not re-initialized and then reset
  Nop();
  Nop();
  __asm__ ("Reset");
}
