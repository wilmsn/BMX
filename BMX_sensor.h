/*
A generic Arduino library for the Bosch sensor
BMP085 (untested), BMP180, BMP280 and BME280
Autodetect for I2C Adress

I made this library for my nodes. It has only fixed settings.
The settings can be changed in the header  file (this file)
*/
#ifndef __BMX_sensor_h__
#define __BMX_sensor_h__

#include <Wire.h>

#include "Arduino.h"

#define BMX_CHIP_ID_REG         0xD0

#define BMP180_CHIP_ID          0x55
#define BMP280_CHIP_ID          0x58
#define BME280_CHIP_ID          0x60

// Calibration Register for BMP/BME280
#define BMX280_T1               0x88
#define BMX280_T2               0x8A
#define BMX280_T3               0x8C
#define BMX280_P1               0x8E
#define BMX280_P2               0x90
#define BMX280_P3               0x92
#define BMX280_P4               0x94
#define BMX280_P5               0x96
#define BMX280_P6               0x98
#define BMX280_P7               0x9A
#define BMX280_P8               0x9C
#define BMX280_P9               0x9E
#define BMX280_H1               0xA1
#define BMX280_H2               0xE1
#define BMX280_H3               0xE3
#define BMX280_H4               0xE4
#define BMX280_H5               0xE5
#define BMX280_H6               0xE7

// Calibration Register for BMP180
#define BMP180_AC1              0xAA
#define BMP180_AC2              0xAC
#define BMP180_AC3              0xAE
#define BMP180_AC4              0xB0
#define BMP180_AC5              0xB2
#define BMP180_AC6              0xB4
#define BMP180_B1               0xB6
#define BMP180_B2               0xB8
#define BMP180_MB               0xBA
#define BMP180_MC               0xBC
#define BMP180_MD               0xBE
#define BMP180_ADC              0xF6


#define BMX_TEMP_MSB_REG        0xFA
#define BMX_PRESS_MSB_REG       0xF7

#define BMX_RESET_REG           0xE0
#define BME280_CTRL_HUM_REG     0xF2
#define BMX_STATUS_REG          0xF3
#define BMX_CONTROL_REG         0xF4
#define BMX_CONFIG_REG          0xF5
#define BMX_DATA_REG            0xF7

#define BMX_RESET_VAL           0xB6

//Calibration data BMP180
#define CALIB_AC1              cal_var_i16_1
#define CALIB_AC2              cal_var_i16_2
#define CALIB_AC3              cal_var_i16_3
#define CALIB_AC4              cal_var_u16_1
#define CALIB_AC5              cal_var_u16_2
#define CALIB_AC6              cal_var_u16_3
#define CALIB_B1               cal_var_i16_4
//#define CALIB_B2               cal_var_i16_5
#define CALIB_MC               cal_var_i16_6
#define CALIB_MD               cal_var_i16_7

//Calibration data BMx280
#define CALIB_T1               cal_var_u16_1
#define CALIB_T2               cal_var_i16_1
#define CALIB_T3               cal_var_i16_2
#define CALIB_P1               cal_var_u16_2
#define CALIB_P2               cal_var_i16_3
#define CALIB_P3               cal_var_i16_4
#define CALIB_P4               cal_var_i16_5
#define CALIB_P5               cal_var_i16_6
#define CALIB_P6               cal_var_i16_7
#define CALIB_P7               cal_var_i16_8
#define CALIB_P8               cal_var_i16_9
#define CALIB_P9               cal_var_i16_10
#define CALIB_H1               cal_var_u8_1
#define CALIB_H2               cal_var_i16_11
#define CALIB_H3               cal_var_u8_2
#define CALIB_H4               cal_var_i16_12
#define CALIB_H5               cal_var_i16_13
#define CALIB_H6               cal_var_i8_1

// Settings for BMP180
#define BMP180_TEMP             0x2E
//Note: See datasheet for different Oversampling setting
// This is mode = 0
//#define BMP180_MODE            0
//#define BMP180_PRES            0x34
// This is mode = 1
#define BMP180_MODE             1
#define BMP180_PRES             0x74

// Settings for BME280
// See datasheet table 20
// oversampling * 1
#define ORS_H 0x01
// Settings for osrs_t[bit 7..5] osrs_p[bit 4..2] and mode[bit1..0]
// All Values oversampling * 1 and forced mode
#define BMX280_CONTROL_VAL      0b00100101 
// Settings for t_sb[bit7..5] filter[bit4..2] and spi3w_en[bit0]
// standby time 0.5ms(unused); filter off; spi interface off 
#define BMX280_CONFIG_VAL       0b00000000 


class BMX_SENSOR
{
  public:
    BMX_SENSOR();  
    void begin(void);
    void begin(uint8_t);
    void startSingleMeasure(void);
    float getTemperature(void);
    float getPressure(void);
    float getHumidity(void);
    float getPressureAtSealevel(float);
    uint8_t getChipId(void);
    uint8_t getI2Cadr(void);
    bool isBMP180(void);
    bool isBMP280(void);
    bool isBME280(void);
    bool hasTemperature(void);
    bool hasPressure(void);
    bool hasHumidity(void);
    
  private:
    void readRawData(void);
    float calcTemp(void);
    float calcPress(void);
    float calcHumi(void);
    uint8_t read8u(uint8_t);
    uint16_t read16u(uint8_t, bool);
    int8_t read8s(uint8_t);
    int16_t read16s(uint8_t, bool);
    void write8(uint8_t, uint8_t);
    bool statusMeas(void);
    void reset(void);
    void readChipId(void);
    
                             // Calibrierungsvariablen
                             // BMP180   BMx280
    int16_t cal_var_i16_1;   // ac1      T2
    int16_t cal_var_i16_2;   // ac2      T3
    int16_t cal_var_i16_3;   // ac3      P2
    uint16_t cal_var_u16_1;  // ac4      T1
    uint16_t cal_var_u16_2;  // ac5      P1
    uint16_t cal_var_u16_3;  // ac6
    int16_t cal_var_i16_4;   // b1       P3
    int16_t cal_var_i16_5;   //          P4
    int16_t cal_var_i16_6;   // mc       P5
    int16_t cal_var_i16_7;   // md       P6
    int16_t cal_var_i16_8;   //          P7
    int16_t cal_var_i16_9;   //          P8
    int16_t cal_var_i16_10;  //          P9
    int16_t cal_var_i16_11;  //          H2
    int16_t cal_var_i16_12;  //          H4
    int16_t cal_var_i16_13;  //          H5
    int16_t cal_var_u8_1;    //          H1
    int16_t cal_var_u8_2;    //          H3
    int16_t cal_var_i8_1;    //          H6
    
    int32_t adc_T;
    int32_t adc_P;
    int32_t adc_H;
    int64_t t_fine;

    uint8_t _i2cAdd;
    uint8_t chipId;

    float temp;
    float humi;
    float pres;
    
    bool bmp180 = false;
    bool bmp280 = false;
    bool bme280 = false;
    
};

#endif


