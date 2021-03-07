#pragma once

#define RED_PIN GPIO_Pin_15
#define RED_PORT GPIOC

#define PWM_FREQ (30000)
#define PWM_RES (72000000 / PWM_FREQ)

#define ADC_REF (3.3)     //analog reference voltage
#define ADC_RES (4096.0)  //analog resolution, 12 bit

#define OP_R_INPUT (10000.0)     //opamp input
#define OP_R_FEEDBACK (15000.0)  //opamp feedback
#define OP_R_OUT_LOW (470.0)     //opamp out low
#define OP_R_OUT_HIGH (22.0)     //opamp out high
#define OP_REF (ADC_REF / 2.0)            //opamp reference voltage

#define SHUNT_R (0.002)
#define SHUNT_SERIES_R (470.0)
#define SHUNT_PULLUP_R (15000.0)
#define SHUNT_OP_GAIN (16.0)

#define CURRENT_OFFSET (ADC_REF / (SHUNT_PULLUP_R + SHUNT_SERIES_R) * SHUNT_SERIES_R / SHUNT_R)
#define CURRENT_SCALE (-1.0 / SHUNT_OP_GAIN / SHUNT_R)

#define AIN_SCALE ((560.0 + 1000.0) / 1000.0)
#define DC_SCALE ((22000.0 + 22000.0 + 1500.0) / 1500.0)

#pragma pack(push, 1)
struct adc12_struct_t{
  uint16_t ain0;
  uint16_t ain3;
  uint16_t ain1;
  uint16_t buck1;
  uint16_t ain2;
  uint16_t buck0;
  uint16_t in_volt;
  uint16_t out_volt;
  uint16_t bat_volt;
  uint16_t neg_temp;
  uint16_t pos_temp0;
  uint16_t pos_temp1;
};

struct adc34_struct_t{
  uint16_t bat_cur[6];
};
#pragma pack(pop)

#define IU (1)
#define IV (3)
#define IW (3)

#define IUi (12)
#define IVi (4)
#define IWi (2)

#define A0 (2)
#define A1 (5)
#define A2 (5)
#define A3 (11)
#define A4 (3)

#define SIN (7)
#define COS (6)

#define TEMP (9)
#define DC (8)

#define VREF (18)



#define NTC_TEMP0 (25.0)
#define NTC_TEMP1 (80.0)

#define NTC_R0 (4700.0)
#define NTC_B (3539.0)

#define NTC_PULLUP (1000.0)
