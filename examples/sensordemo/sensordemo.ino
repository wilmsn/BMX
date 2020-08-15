/***********************************************************************
*
* Basic functions demo
* Use a 5V type of one of the supported sensor (BMP085, BMP180, BMP280, BME280)
* and connect it like this:
* Arduino     Sensor
*   Vcc ------ Vcc
*   Gnd ------ Gnd
*   Scl ------ Scl
*   Sda ------ Sda  
*
* Sensortype and I2C adress will be printed.
* Sensorvalues will be printed
*
**********************************************************************/ 
#include "BMX_sensor.h"

BMX_SENSOR sensor;

void print_sensor(void) {
  int8_t adr;
  adr = sensor.getI2Cadr();
  switch (sensor.getChipId()) {
    case 0x55:
      Serial.print("BMP180 an Adresse: 0x");
      Serial.println(adr, HEX);
      break;
    case 0x58:
      Serial.print("BMP280 an Adresse: 0x");
      Serial.println(adr, HEX);
      break;
    case 0x60:
      Serial.print("BME280 an Adresse: 0x");
      Serial.println(adr, HEX);
      break;
    default:
      Serial.println("Kein unterstützter Sensor gefunden");
  }
  Serial.println("--------------------------------------");
}

void setup() {
  Serial.begin(9600);
  sensor.begin();
  print_sensor(); 
}

void loop() {
  sensor.startSingleMeasure();
  Serial.print("Temperatur: ");
  if (sensor.hasTemperature()) {
    Serial.println(sensor.getTemperature());
  } else {
    Serial.println("Sensor unterstützt diese Messung nicht");
  }
  Serial.print("Druck: ");
  if (sensor.hasPressure()) {
    Serial.println(sensor.getPressure());
    Serial.print("Druck at Sealevel: ");
    Serial.println(sensor.getPressureAtSealevel(95));
  } else {
    Serial.println("Sensor unterstützt diese Messung nicht");
  }
  Serial.print("Feuchte: ");
  if (sensor.hasHumidity()) {
  Serial.println(sensor.getHumidity());
  } else {
    Serial.println("Sensor unterstützt diese Messung nicht");
  }
  Serial.println("---------------------------------------");
  delay(2000); 
}
