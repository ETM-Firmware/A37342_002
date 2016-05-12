#ifndef __A36926_002_H
#define __A36926_002_H

// A36926-002 This is the Can interface board configured as Cooling Interface board

#include <xc.h>
#include <adc12.h>
#include <timer.h>
#include <libpic30.h>
#include <incap.h>
#include "ETM.h"
#include "P1395_CAN_SLAVE.h"
#include "FIRMWARE_VERSION.h"
#include "A36926_002_CONFIG.h"


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

#define A36926_TRISA_VALUE 0b1111111111111111
#define A36926_TRISB_VALUE 0b1011111111111111
#define A36926_TRISC_VALUE 0b1111111111111111
#define A36926_TRISD_VALUE 0b1111111111110000
#define A36926_TRISF_VALUE 0b1111111000110011
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
#define _STATUS_FLOW_OK                                 _LOGGED_STATUS_6


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


#define PWR_5V_OVER_FLT                  5500
#define PWR_5V_UNDER_FLT                 4500
#define PWR_15V_OVER_FLT                 16000
#define PWR_15V_UNDER_FLT                14000
#define PWR_NEG_15V_OVER_FLT             16000
#define PWR_NEG_15V_UNDER_FLT            14000


#define TEMPERATURE_SENSOR_FIXED_SCALE         .15625

#define SF6_SENSOR_FIXED_SCALE                 .97656 //.19629   
#define SF6_SENSOR_FIXED_OFFSET                -2560 //-12736   // Calculated 4mA Offset


#define SF6_STATE_TEST       10
#define SF6_STATE_CHARGING   20
#define SF6_STATE_DELAY      30

#define STARTUP_LED_FLASH_TIME    400       // 4 Seconds
#define COOLING_INTERFACE_BOARD_TEST_TIME     100   // 1 second

#define FLOW_METER_ML_PER_HZ      81
#define FLOW_METER_CONSTANT       841

#define PERIOD_MAX_FREQUENCY      70   // 558 Hz
#define FLOW_METER_MIN_FREQUENCY  15
#define PWM_MAX_PERIOD            1954 // This is equiv to 10Hz 




#define THERMISTOR_LOOK_UP_TABLE_VALUES 4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,4720,3770,3764,3757,3751,3745,3739,3733,3727,3722,3716,3710,3705,3700,3695,3689,3684,3679,3674,3670,3665,3660,3656,3651,3646,3642,3638,3633,3629,3625,3621,3617,3613,3609,3605,3601,3597,3593,3589,3586,3582,3578,3575,3571,3568,3564,3560,3557,3554,3550,3547,3544,3541,3537,3534,3531,3528,3525,3522,3519,3516,3513,3510,3507,3504,3501,3498,3496,3493,3490,3487,3485,3482,3479,3476,3474,3471,3469,3466,3463,3461,3458,3456,3453,3451,3448,3446,3444,3441,3439,3436,3434,3432,3429,3427,3425,3423,3420,3418,3416,3414,3411,3409,3407,3405,3403,3401,3398,3396,3394,3392,3390,3388,3386,3384,3382,3380,3378,3376,3374,3372,3370,3368,3366,3364,3362,3360,3359,3357,3355,3353,3351,3349,3348,3346,3344,3342,3340,3339,3337,3335,3333,3331,3330,3328,3326,3325,3323,3321,3319,3318,3316,3314,3313,3311,3309,3308,3306,3304,3303,3301,3300,3298,3296,3295,3293,3292,3290,3289,3287,3285,3284,3282,3281,3279,3278,3276,3275,3273,3272,3270,3269,3267,3266,3264,3263,3261,3260,3259,3257,3256,3254,3253,3251,3250,3249,3247,3246,3244,3243,3242,3240,3239,3237,3236,3235,3233,3232,3231,3229,3228,3227,3225,3224,3223,3221,3220,3219,3217,3216,3215,3213,3212,3211,3210,3208,3207,3206,3204,3203,3202,3201,3199,3198,3197,3196,3194,3193,3192,3191,3189,3188,3187,3186,3185,3183,3182,3181,3180,3179,3177,3176,3175,3174,3173,3171,3170,3169,3168,3167,3166,3164,3163,3162,3161,3160,3159,3157,3156,3155,3154,3153,3152,3151,3149,3148,3147,3146,3145,3144,3143,3142,3140,3139,3138,3137,3136,3135,3134,3133,3132,3131,3130,3128,3127,3126,3125,3124,3123,3122,3121,3120,3119,3118,3117,3116,3115,3113,3112,3111,3110,3109,3108,3107,3106,3105,3104,3103,3102,3101,3100,3099,3098,3097,3096,3095,3094,3093,3092,3091,3090,3089,3088,3087,3086,3085,3084,3083,3082,3080,3080,3079,3078,3077,3076,3075,3074,3073,3072,3071,3070,3069,3068,3067,3066,3065,3064,3063,3062,3061,3060,3059,3058,3057,3056,3055,3054,3053,3052,3051,3050,3049,3048,3047,3046,3045,3044,3043,3042,3042,3041,3040,3039,3038,3037,3036,3035,3034,3033,3032,3031,3030,3029,3028,3027,3027,3026,3025,3024,3023,3022,3021,3020,3019,3018,3017,3016,3015,3014,3014,3013,3012,3011,3010,3009,3008,3007,3006,3005,3004,3003,3003,3002,3001,3000,2999,2998,2997,2996,2995,2994,2993,2993,2992,2991,2990,2989,2988,2987,2986,2985,2984,2984,2983,2982,2981,2980,2979,2978,2977,2976,2975,2975,2974,2973,2972,2971,2970,2969,2968,2967,2967,2966,2965,2964,2963,2962,2961,2960,2959,2959,2958,2957,2956,2955,2954,2953,2952,2951,2951,2950,2949,2948,2947,2946,2945,2944,2944,2943,2942,2941,2940,2939,2938,2937,2937,2936,2935,2934,2933,2932,2931,2930,2929,2929,2928,2927,2926,2925,2924,2923,2923,2922,2921,2920,2919,2918,2917,2916,2916,2915,2914,2913,2912,2911,2910,2909,2909,2908,2907,2906,2905,2904,2903,2902,2901,2901,2900,2899,2898,2897,2896,2895,2895,2894,2893,2892,2891,2890,2889,2888,2887,2887,2886,2885,2884,2883,2882,2881,2880,2880,2879,2878,2877,2876,2875,2874,2873,2873,2872,2871,2870,2869,2868,2867,2866,2865,2865,2864,2863,2862,2861,2860,2859,2858,2857,2857,2856,2855,2854,2853,2852,2851,2850,2849,2849,2848,2847,2846,2845,2844,2843,2842,2841,2840,2840,2839,2838,2837,2836,2835,2834,2833,2832,2831,2831,2830,2829,2828,2827,2826,2825,2824,2823,2822,2821,2820,2820,2819,2818,2817,2816,2815,2814,2813,2812,2811,2810,2809,2809,2808,2807,2806,2805,2804,2803,2802,2801,2800,2799,2798,2797,2796,2796,2795,2794,2793,2792,2791,2790,2789,2788,2787,2786,2785,2784,2783,2782,2781,2780,2779,2778,2777,2776,2776,2775,2774,2773,2772,2771,2770,2769,2768,2767,2766,2765,2764,2763,2762,2761,2760,2759,2758,2757,2756,2755,2754,2753,2752,2751,2750,2749,2748,2747,2746,2745,2744,2743,2742,2741,2740,2739,2738,2737,2736,2735,2733,2732,2731,2730,2729,2728,2727,2726,2725,2724,2723,2722,2721,2720,2719,2718,2717,2716,2714,2713,2712,2711,2710,2709,2708,2707,2706,2705,2703,2702,2701,2700,2699,2698,2697,2696,2695,2693,2692,2691,2690,2689,2688,2687,2685,2684,2683,2682,2681,2680,2678,2677,2676,2675,2674,2672,2671,2670,2669,2668,2666,2665,2664,2663,2662,2660,2659,2658,2657,2655,2654,2653,2652,2650,2649,2648,2647,2645,2644,2643,2641,2640,2639,2637,2636,2635,2633,2632,2631,2629,2628,2627,2625,2624,2623,2621,2620,2618,2617,2616,2614,2613,2611,2610,2609,2607,2606,2604,2603,2601,2600,2598,2597,2595,2594,2592,2591,2589,2588,2586,2585,2583,2581,2580,2578,2577,2575,2573,2572,2570,2569,2567,2565,2564,2562,2560,2558,2557,2555,2553,2551,2550,2548,2546,2544,2543,2541,2539,2537,2535,2533,2531,2529,2528,2526,2524,2522,2520,2518,2516,2514,2512,2509,2507,2505,2503,2501,2499,2497,2494,2492,2490,2488,2485,2483,2481,2478,2476,2474,2471,2469,2466,2464,2461,2458,2456,2453,2450,2448,2445,2442,2439,2437,2434,2431,2428,2425,2422,2418,2415,2412,2409,2406,2402,2399,2395,2392,2388,2384,2380,2377,2373,2369,1720,1720,1720,1720,1720,1720,1720,1720,1720,1720,1720,1720,1720,1720,1720,1720,1720,1720,1720,1720,1720,1720,1720,1720,1720,1720,1720,1720,1720,1720,1720,1720,1720,1720,1720,1720,1720,1720,1720



#endif
