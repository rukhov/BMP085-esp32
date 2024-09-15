/*!
 * @file Adafruit_BMP085.cpp
 *
 * @mainpage Adafruit BMP085 Library
 *
 * @section intro_sec Introduction
 *
 * This is a library for the Adafruit BMP085/BMP180 Barometric Pressure + Temp
 * sensor
 *
 * Designed specifically to work with the Adafruit BMP085 or BMP180 Breakout
 * ----> http://www.adafruit.com/products/391
 * ----> http://www.adafruit.com/products/1603
 *
 * These displays use I2C to communicate, 2 pins are required to
 * interface
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section author Author
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries.
 * Updated by Samy Kamkar for cross-platform support.
 * Ported for FreeRTOS and ESP32 platform by Roman Ukhov
 *
 * @section license License
 *
 * BSD license, all text above must be included in any redistribution
 */

#include "Adafruit_BMP085.h"
#include <driver/i2c_master.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <math.h>

#define BMP085_DEBUG 0  //!< Debug mode

#define BMP085_I2CADDR 0x77  //!< BMP085 I2C address

#define BMP085_CAL_AC1 0xAA  //!< R   Calibration data (16 bits)
#define BMP085_CAL_AC2 0xAC  //!< R   Calibration data (16 bits)
#define BMP085_CAL_AC3 0xAE  //!< R   Calibration data (16 bits)
#define BMP085_CAL_AC4 0xB0  //!< R   Calibration data (16 bits)
#define BMP085_CAL_AC5 0xB2  //!< R   Calibration data (16 bits)
#define BMP085_CAL_AC6 0xB4  //!< R   Calibration data (16 bits)
#define BMP085_CAL_B1  0xB6  //!< R   Calibration data (16 bits)
#define BMP085_CAL_B2  0xB8  //!< R   Calibration data (16 bits)
#define BMP085_CAL_MB  0xBA  //!< R   Calibration data (16 bits)
#define BMP085_CAL_MC  0xBC  //!< R   Calibration data (16 bits)
#define BMP085_CAL_MD  0xBE  //!< R   Calibration data (16 bits)

#define BMP085_CONTROL         0xF4  //!< Control register
#define BMP085_TEMPDATA        0xF6  //!< Temperature data register
#define BMP085_PRESSUREDATA    0xF6  //!< Pressure data register
#define BMP085_READTEMPCMD     0x2E  //!< Read temperature control register value
#define BMP085_READPRESSURECMD 0x34  //!< Read pressure control register value

namespace {

  constexpr auto TAG = "Adafruit_BMP085";

  constexpr unsigned I2C_TIMEOUT_MS = 100;

  void delay(unsigned ms) {
    auto ticks = ms / portTICK_PERIOD_MS;
    if (ticks * portTICK_PERIOD_MS < ms)
      ++ticks;
    vTaskDelay(ticks);
  }
}  // namespace

Adafruit_BMP085::Adafruit_BMP085() : i2c_dev(nullptr) {}

bool Adafruit_BMP085::begin(mode m, i2c_master_bus_handle_t bus) {
  if (m > mode::BMP085_ULTRAHIGHRES)
    m = mode::BMP085_ULTRAHIGHRES;
  oversampling = m;

  if (i2c_dev) {
    i2c_master_bus_rm_device(i2c_dev);  // remove old interface
  }

  i2c_device_config_t cfg = {};

  cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  cfg.device_address = BMP085_I2CADDR;
  cfg.flags.disable_ack_check = false;
  cfg.scl_speed_hz = 100000;
  cfg.scl_wait_us = 0;

  ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &cfg, &i2c_dev));

  if (i2c_master_probe(bus, BMP085_I2CADDR, I2C_TIMEOUT_MS) != ESP_OK || read8(0xD0) != 0x55)
    abort();

  delay(10);

  /* read calibration data */
  ac1 = read16(BMP085_CAL_AC1);
  ac2 = read16(BMP085_CAL_AC2);
  ac3 = read16(BMP085_CAL_AC3);
  ac4 = read16(BMP085_CAL_AC4);
  ac5 = read16(BMP085_CAL_AC5);
  ac6 = read16(BMP085_CAL_AC6);

  b1 = read16(BMP085_CAL_B1);
  b2 = read16(BMP085_CAL_B2);

  mb = read16(BMP085_CAL_MB);
  mc = read16(BMP085_CAL_MC);
  md = read16(BMP085_CAL_MD);
#if (BMP085_DEBUG == 1)
  ESP_LOGI(TAG, "ac1 =  %d", ac1);
  ESP_LOGI(TAG, "ac2 =  %d", ac2);
  ESP_LOGI(TAG, "ac3 =  %d", ac3);
  ESP_LOGI(TAG, "ac4 =  %d", ac4);
  ESP_LOGI(TAG, "ac5 =  %d", ac5);
  ESP_LOGI(TAG, "ac6 =  %d", ac6);

  ESP_LOGI(TAG, "b1 =  %d", b1);
  ESP_LOGI(TAG, "b2 =  %d", b2);

  ESP_LOGI(TAG, "mb =  %d", mb);
  ESP_LOGI(TAG, "mc =  %d", mc);
  ESP_LOGI(TAG, "md =  %d", md);
#endif

  return true;
}

int32_t Adafruit_BMP085::computeB5(int32_t UT) {
  int32_t X1 = (UT - (int32_t)ac6) * ((int32_t)ac5) >> 15;
  int32_t X2 = ((int32_t)mc << 11) / (X1 + (int32_t)md);
  return X1 + X2;
}

uint16_t Adafruit_BMP085::readRawTemperature(void) {
  write8(BMP085_CONTROL, BMP085_READTEMPCMD);
  delay(5);
  auto raw = read16(BMP085_TEMPDATA);
#if BMP085_DEBUG == 1
  ESP_LOGI(TAG, "Raw temp: %d", raw);
#endif
  return raw;
}

uint32_t Adafruit_BMP085::readRawPressure(void) {
  uint32_t raw;

  write8(BMP085_CONTROL, BMP085_READPRESSURECMD + (oversampling << 6));

  if (oversampling == mode::BMP085_ULTRALOWPOWER)
    delay(5);
  else if (oversampling == mode::BMP085_STANDARD)
    delay(8);
  else if (oversampling == mode::BMP085_HIGHRES)
    delay(14);
  else
    delay(26);

  raw = read16(BMP085_PRESSUREDATA);

  raw <<= 8;
  raw |= read8(BMP085_PRESSUREDATA + 2);
  raw >>= (8 - oversampling);

  /* this pull broke stuff, look at it later?
   if (oversampling==0) {
     raw <<= 8;
     raw |= read8(BMP085_PRESSUREDATA+2);
     raw >>= (8 - oversampling);
   }
  */

#if BMP085_DEBUG == 1
  ESP_LOGI(TAG, "Raw pressure: %u", (unsigned int)raw);
#endif
  return raw;
}

int32_t Adafruit_BMP085::readPressure(void) {
  int32_t UT, UP, B3, B5, B6, X1 = 0, X2 = 0, X3, p;
  uint32_t B4, B7;

  UT = readRawTemperature();
  UP = readRawPressure();

#if BMP085_DEBUG == 1
  // use datasheet numbers!
  UT = 27898;
  UP = 23843;
  ac6 = 23153;
  ac5 = 32757;
  mc = -8711;
  md = 2868;
  b1 = 6190;
  b2 = 4;
  ac3 = -14383;
  ac2 = -72;
  ac1 = 408;
  ac4 = 32741;
  oversampling = 0;
#endif

  B5 = computeB5(UT);

#if BMP085_DEBUG == 1
  ESP_LOGI(TAG, "X1 =  %d", (int)X1);
  ESP_LOGI(TAG, "X2 =  %d", (int)X2);
  ESP_LOGI(TAG, "B5 =  %d", (int)B5);
#endif

  // do pressure calcs
  B6 = B5 - 4000;
  X1 = ((int32_t)b2 * ((B6 * B6) >> 12)) >> 11;
  X2 = ((int32_t)ac2 * B6) >> 11;
  X3 = X1 + X2;
  B3 = ((((int32_t)ac1 * 4 + X3) << oversampling) + 2) / 4;

#if BMP085_DEBUG == 1
  ESP_LOGI(TAG, "B6 =  %d", (int)B6);
  ESP_LOGI(TAG, "X1 =  %d", (int)X1);
  ESP_LOGI(TAG, "X2 =  %d", (int)X2);
  ESP_LOGI(TAG, "B3 =  %d", (int)B3);
#endif

  X1 = ((int32_t)ac3 * B6) >> 13;
  X2 = ((int32_t)b1 * ((B6 * B6) >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  B4 = ((uint32_t)ac4 * (uint32_t)(X3 + 32768)) >> 15;
  B7 = ((uint32_t)UP - B3) * (uint32_t)(50000UL >> oversampling);

#if BMP085_DEBUG == 1
  ESP_LOGI(TAG, "X1 =  %d", (int)X1);
  ESP_LOGI(TAG, "X2 =  %d", (int)X2);
  ESP_LOGI(TAG, "B4 =  %d", (unsigned int)B4);
  ESP_LOGI(TAG, "B7 =  %d", (unsigned int)B7);
#endif

  if (B7 < 0x80000000) {
    p = (B7 * 2) / B4;
  } else {
    p = (B7 / B4) * 2;
  }
  X1 = (p >> 8) * (p >> 8);
  X1 = (X1 * 3038) >> 16;
  X2 = (-7357 * p) >> 16;

#if BMP085_DEBUG == 1
  ESP_LOGI(TAG, "p =  %d", (int)p);
  ESP_LOGI(TAG, "X1 =  %d", (int)X1);
  ESP_LOGI(TAG, "X2 =  %d", (int)X2);
#endif

  p = p + ((X1 + X2 + (int32_t)3791) >> 4);
#if BMP085_DEBUG == 1
  ESP_LOGI(TAG, "p =  %d", (int)p);
#endif
  return p;
}

int32_t Adafruit_BMP085::readSealevelPressure(float altitude_meters) {
  float pressure = readPressure();
  return (int32_t)(pressure / pow(1.0 - altitude_meters / 44330, 5.255));
}

float Adafruit_BMP085::readTemperature(void) {
  int32_t UT, B5;  // following ds convention
  float temp;

  UT = readRawTemperature();

#if BMP085_DEBUG == 1
  // use datasheet numbers!
  UT = 27898;
  ac6 = 23153;
  ac5 = 32757;
  mc = -8711;
  md = 2868;
#endif

  B5 = computeB5(UT);
  temp = (B5 + 8) / 160.;
  //temp /= 10;

  return temp;
}

float Adafruit_BMP085::readAltitude(float sealevelPressure) {
  float altitude;

  float pressure = readPressure();

  altitude = 44330 * (1.0 - pow(pressure / sealevelPressure, 0.1903));

  return altitude;
}

/*********************************************************************/

uint8_t Adafruit_BMP085::read8(uint8_t a) {
  uint8_t ret;
  ESP_ERROR_CHECK(i2c_master_transmit_receive(i2c_dev, &a, 1, &ret, 1, I2C_TIMEOUT_MS));
  return ret;
}

uint16_t Adafruit_BMP085::read16(uint8_t a) {
  uint8_t retbuf[2];
  ESP_ERROR_CHECK(i2c_master_transmit_receive(i2c_dev, &a, 1, retbuf, 2, I2C_TIMEOUT_MS));
  return retbuf[1] | (retbuf[0] << 8);
}

void Adafruit_BMP085::write8(uint8_t a, uint8_t d) {
  uint8_t buf[2] = {a, d};
  ESP_ERROR_CHECK(i2c_master_transmit(i2c_dev, buf, 2, I2C_TIMEOUT_MS));
}
