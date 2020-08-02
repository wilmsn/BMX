/*
 library for Bosch sensor BMP085(untested), BMP180, BMP280 and BME280
 for I2C connection with autodetect of I2C adress

*/
#include "BMX_sensor.h"

//************************************************************************
//Constructor: Wire Lib öffnen

BMX_SENSOR::BMX_SENSOR()
{
  Wire.begin();
}

//************************************************************************
//begin() ohne Parameter: I2C Adresse auswaehlen

void BMX_SENSOR::begin()
{
  // Sensor suchen, moegliche Adressen: 0x76, 0x77
  _i2cAdd = 0x77;
  readChipId();
  if (chipId) {
    _i2cAdd = 0x76;
    readChipId();
  }    
  begin(_i2cAdd);
}

//************************************************************************
//begin() mit uebergebener I2C Adresse

void BMX_SENSOR::begin(uint8_t i2cAdd)
{
  _i2cAdd = i2cAdd;
  reset();
  delay(250); 

  //Sensortyp ermitteln
  switch (chipId) {
      case BMP180_CHIP_ID:
          bmp180 = true;
          break;
      case BMP280_CHIP_ID:
          bmp280 = true;
          break;
      case BME280_CHIP_ID:
          bme280 = true;
          break;
  }      
  
  //Kalibrierungskoeffizienten einlesen
  if ( bmp280 | bme280 ) {	
  //Wenn der BMX_SENSOR gerade die Koeffizienten ins interne
  //Image-Register liest, dann warte
    while (read8u(BMX_STATUS_REG) & 0x01) delay(1);
    CALIB_T1 = read16u(BMX280_T1, false);
    CALIB_T2 = read16s(BMX280_T2, false);
    CALIB_T3 = read16s(BMX280_T3, false);
    CALIB_P1 = read16u(BMX280_P1, false);
    CALIB_P2 = read16s(BMX280_P2, false);
    CALIB_P3 = read16s(BMX280_P3, false);
    CALIB_P4 = read16s(BMX280_P4, false);
    CALIB_P5 = read16s(BMX280_P5, false);
    CALIB_P6 = read16s(BMX280_P6, false);
    CALIB_P7 = read16s(BMX280_P7, false);
    CALIB_P8 = read16s(BMX280_P8, false);
    CALIB_P9 = read16s(BMX280_P9, false);
    CALIB_H1 = read8u(BMX280_H1);
    CALIB_H2 = read16s(BMX280_H2, false);
    CALIB_H3 = read8u(BMX280_H3);
    CALIB_H4 = ((uint16_t)read8u(BMX280_H4) << 4) | (read8u(BMX280_H5) & 0xF);
    CALIB_H5 = ((uint16_t)read8u(BMX280_H5 + 1) << 4) | (read8u(BMX280_H5) >> 4);
    CALIB_H6 = read8u(BMX280_H6);
  }	   
  if (bmp180) {
    CALIB_AC1=read16s(BMP180_AC1, true);
    CALIB_AC2=read16s(BMP180_AC2, true);
    CALIB_AC3=read16s(BMP180_AC3, true);
    CALIB_AC4=read16u(BMP180_AC4, true);
    CALIB_AC5=read16u(BMP180_AC5, true);
    CALIB_AC6=read16u(BMP180_AC6, true);
    CALIB_B1 =read16s(BMP180_B1, true);
    CALIB_MC =read16s(BMP180_MC, true);
    CALIB_MD =read16s(BMP180_MD, true);
  }
  //Schreiben der gesetzten Parameter in die Register 
  if ( bmp280 | bme280 ) {
    write8(BMX_CONFIG_REG, BMX280_CONFIG_VAL);
    write8(BMX_CONTROL_REG, BMX280_CONTROL_VAL);
  }
}

//*************************************************************************
//Starte Einzelmessung

void BMX_SENSOR::startSingleMeasure()
{
  if ( bmp280 | bme280 ) {
    write8(BME280_CTRL_HUM_REG, ORS_H);
    write8(BMX_CONTROL_REG, BMX280_CONTROL_VAL);
    while (statusMeas()) delay(1); //Warte bis Messzyklus beendet
    readRawData();
  }
  if ( bmp180 ) {
    write8(BMX_CONTROL_REG, BMP180_TEMP);
    delay(10);
    adc_T = read16s(BMP180_ADC, true);
    write8(BMX_CONTROL_REG, BMP180_PRES);
    delay(10);
    adc_P = read16u(BMP180_ADC, true );
    adc_P <<= 8;
    adc_P += read8u(BMP180_ADC + 2);
    adc_P >>= (8 - BMP180_MODE); 
  }
  temp = calcTemp();
  pres = calcPress();
  humi = calcHumi();
}

//*************************************************************************
// Lufttemperatur ausgeben

float BMX_SENSOR::getTemperature()
{
  return temp;
}

//*************************************************************************
// Luftdruck ausgeben

float BMX_SENSOR::getPressure()
{
  return pres;
}

//*************************************************************************
// Luftfeuchtigkeit ausgeben

float BMX_SENSOR::getHumidity()
{
  return humi;
}

//*************************************************************************
// Luftdruck auf Meereshoehe ermitteln

float BMX_SENSOR::getPressureAtSealevel(float hoehe)
{
  //Berechne den Luftdruck auf NN
  return pow(((hoehe*0.0065)/(temp+273.15)+1.0),5.257)*pres;
 }

//*************************************************************************
// Einlesen der Rohdaten ueber die I2C-Schnittstelle

void BMX_SENSOR::readRawData()
{
    uint8_t data_reg = BMX_DATA_REG;
    while (statusMeas()) delay(1); //Warte bis Messzyklus beendet
    Wire.beginTransmission(_i2cAdd);
    Wire.write(data_reg);
    Wire.endTransmission();
    if (bmp280) {
        Wire.requestFrom(_i2cAdd, (uint8_t)6);
    }
    if (bme280) {
        Wire.requestFrom(_i2cAdd, (uint8_t)8);
    }
    adc_P = Wire.read();
    adc_P <<= 8;
    adc_P |= Wire.read();
    adc_P <<= 8;
    adc_P |= Wire.read();
    adc_P >>= 4;
    adc_T = Wire.read();
    adc_T <<= 8;
    adc_T |= Wire.read();
    adc_T <<= 8;
    adc_T |= Wire.read();
    adc_T >>= 4;
    if (bme280) {
        adc_H = Wire.read();
        adc_H <<= 8;
        adc_H |= Wire.read();
    }    
}

//*************************************************************************
// Temperatur ermitteln

float BMX_SENSOR::calcTemp()
{
  int32_t t_var1, t_var2;
  
  if ( bmp280 | bme280 ) {
    t_var1 = (((adc_T >> 3) - (CALIB_T1 << 1)) * CALIB_T2) >> 11;
    t_var2 = (((((adc_T >> 4) - (CALIB_T1)) * ((adc_T >> 4) - CALIB_T1)) >> 12) * CALIB_T3) >> 14;
    t_fine = t_var1 + t_var2;
    return (float)((t_fine * 5 + 128) >> 8) / 100.0;
  }
  if ( bmp180 ) {
    t_var1 = (adc_T - CALIB_AC6) * CALIB_AC5 >> 15;
    t_var2 = ((int32_t)CALIB_MC << 11) / (t_var1 + CALIB_MD);
    t_fine = t_var1 + t_var2;
    return ((float)((t_fine + 8) >> 4))/10.0; 
  }
}

//*************************************************************************
// Luftdruck ermitteln

float BMX_SENSOR::calcPress()
{
  int64_t var1, var2; 
  int32_t t_var1 /* BMP180 => x1 */, t_var2 /* BMP180 => x2 */; 
  int64_t press;
  int32_t x3, b3, b6, p;
  uint32_t b4, b7;

  if ( bmp280 | bme280 ) {  
    var1 = t_fine - 128000;
    var2 = var1 * var1 * (int64_t)CALIB_P6;
    var2 = var2 + ((var1 * (int64_t)CALIB_P5) << 17);
    var2 = var2 + (((int64_t)CALIB_P4) << 35);
    var1 = ((var1 * var1 + (int64_t)CALIB_P3) >> 8) + ((var1 * (int64_t)CALIB_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)CALIB_P1) >> 33;
    if (var1 == 0) return 0;
    press= 1048576 - adc_P;
    press= (((press << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)CALIB_P9) * (press>> 13) * (press>> 13)) >> 25;
    var2 = (((int64_t)CALIB_P8) * press) >> 19;
    press = ((press + var1 + var2) >> 8) + (((int64_t)CALIB_P7) << 4);
    press /= 256;
    return (float) press/ 100.0;
  }
  if ( bmp180 ) {
    b6 = (int32_t)t_fine - 4000;
    t_var1 = (CALIB_B1 * ((b6 * b6) >> 12)) >> 11;
    t_var2 = (CALIB_AC2 * b6) >> 11;
    x3 = t_var1 + t_var2;
    b3 = (((((int32_t)CALIB_AC1) * 4 + x3)  <<  BMP180_MODE ) + 2) >> 2;
    t_var1 = (CALIB_AC3 * b6) >> 13;
    t_var2 = (CALIB_B1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((t_var1 + t_var2) + 2) >> 2;
    b4 = (CALIB_AC4 * (uint32_t)(x3 + 32768)) >> 15;
    b7 = ((uint32_t)(adc_P - b3) * (50000 >> BMP180_MODE));
    if (b7 < 0x80000000) { p = (b7 << 1) / b4; } else { p = (b7 / b4) << 1; }
    t_var1 = (p >> 8) * (p >> 8);
    t_var1 = (t_var1 * 3038) >> 16;
    t_var2 = (-7357 * p) >> 16;
    return (float)(p + ((t_var1 + t_var2 + 3791) >> 4))/100.0;
  }  
}

//*************************************************************************
// Luftfeuchte ermitteln

float BMX_SENSOR::calcHumi()
{
  if (bme280) {  
    int32_t v_x1_u32r;

    v_x1_u32r = (t_fine - ((int32_t)76800));

    v_x1_u32r = (((((adc_H << 14) - (((int32_t)CALIB_H4) << 20) - (((int32_t)CALIB_H5) * 
              v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * 
              ((int32_t)CALIB_H6)) >> 10) * (((v_x1_u32r * ((int32_t)CALIB_H3)) >> 11) +
              ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)CALIB_H2) + 
              8192) >> 14));

    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * 
              ((int32_t)CALIB_H1)) >> 4));

    v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
    v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
    //float h = (v_x1_u32r >> 12);
    return (float)(v_x1_u32r >> 12) / 1024.0;
    //return (float)adc_H;
  } else {
    return -1;
  }
}

//*************************************************************************
// Schreiben von 1 Byte ueber die I2C-Schnittstelle

void BMX_SENSOR::write8(uint8_t add, uint8_t val)
{
    Wire.beginTransmission(_i2cAdd);
    Wire.write(add);
    Wire.write(val);
    Wire.endTransmission();
  delay(5);
}

//*************************************************************************
// Einlesen von 1 Byte (signed) ueber die I2C-Schnittstelle

int8_t BMX_SENSOR::read8s(uint8_t adr)
{
    Wire.beginTransmission(_i2cAdd);
    Wire.write(adr);
    Wire.endTransmission();
    Wire.requestFrom(_i2cAdd, (uint8_t)1);
    return (int8_t)Wire.read();
}

//*************************************************************************
// Einlesen von 1 Byte (unsigned) ueber die I2C-Schnittstelle

uint8_t BMX_SENSOR::read8u(uint8_t adr)
{
    Wire.beginTransmission(_i2cAdd);
    Wire.write(adr);
    Wire.endTransmission();
    Wire.requestFrom(_i2cAdd, (uint8_t)1);
    return (uint8_t)Wire.read();
}

//*************************************************************************
// Einlesen von 2 Bytes (signed) ueber die I2C-Schnittstelle 

int16_t BMX_SENSOR::read16s(uint8_t adr, bool msb_first)
{
    Wire.beginTransmission(_i2cAdd);
    Wire.write(adr);
    Wire.endTransmission();
    Wire.requestFrom(_i2cAdd, (uint8_t)2);
    if (msb_first) {
        return ((int16_t)Wire.read() << 8) | Wire.read();
    } else {
        return (int16_t)Wire.read() | ((int16_t)Wire.read() << 8);
    }
}

//*************************************************************************
// Einlesen von 2 Bytes (unsigned) ueber die I2C-Schnittstelle 

uint16_t BMX_SENSOR::read16u(uint8_t adr, bool msb_first)
{
    Wire.beginTransmission(_i2cAdd);
    Wire.write(adr);
    Wire.endTransmission();
    Wire.requestFrom(_i2cAdd, (uint8_t)2);
    if (msb_first) {
        return (((uint16_t)Wire.read() << 8) | Wire.read());
    } else {
        return ((uint16_t)Wire.read() | (Wire.read() << 8));
    }
}

//*************************************************************************
// Abfrage des Status des Measuring-Bit (nur BMP280/BME280)

bool BMX_SENSOR::statusMeas()
{
  uint8_t reg;
  if ( bmp280 | bme280 ) {	
    reg = read8u(BMX_STATUS_REG);
    return (reg & B00001000) >> 3;
  }
}

//*************************************************************************
// Software-Reset

void BMX_SENSOR::reset()
{
  write8(BMX_RESET_REG, BMX_RESET_VAL);
}

//*************************************************************************
// ChipId aus Sensor lesen

void BMX_SENSOR::readChipId()
{
  chipId = read8u(BMX_CHIP_ID_REG);
}

//*************************************************************************
// Ausgabe der I2C Adresse

uint8_t BMX_SENSOR::getI2Cadr()
{
  return _i2cAdd;    
}

//*************************************************************************
// chipId aus Sensor lesen

uint8_t BMX_SENSOR::getChipId()
{
  return chipId;
}

//*************************************************************************
// true if Sensor is BME280

bool BMX_SENSOR::isBME280(void)
{
  return bme280;
}

//*************************************************************************
// true if Sensor is BMP280

bool BMX_SENSOR::isBMP280(void)
{
  return bmp280;
}

//*************************************************************************
// true if Sensor is BMP180

bool BMX_SENSOR::isBMP180(void)
{
  return bmp180;
}
