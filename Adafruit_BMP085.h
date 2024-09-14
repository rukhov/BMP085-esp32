/*!
 * @file Adafruit_BMP085.h
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
 * Written by Limor Fried/Ladyada for Adafruit Industries.
 * BSD license, all text above must be included in any redistribution
 */

#pragma once

#include <driver/i2c_master.h>

/*!
 * @brief Main BMP085 class
 */
class Adafruit_BMP085 {
public:
  enum mode : uint8_t {
    BMP085_ULTRALOWPOWER = 0,  //!< Ultra low power mode
    BMP085_STANDARD = 1,       //!< Standard mode
    BMP085_HIGHRES = 2,        //!< High-res mode
    BMP085_ULTRAHIGHRES = 3,   //!< Ultra high-res mode
  };

  Adafruit_BMP085();
  /*!
   * @brief Starts I2C connection
   * @param mode Mode to set, ultra high-res by default
   * @param wire The I2C interface to use, defaults to Wire
   * @return Returns true if successful
   */
  bool begin(mode mode, i2c_master_bus_handle_t bus);
  /*!
   * @brief Gets the temperature over I2C from the BMP085
   * @return Returns the temperature
   */
  float readTemperature(void);
  /*!
   * @brief Gets the pressure over I2C from the BMP085
   * @return Returns the pressure
   */
  int32_t readPressure(void);
  /*!
   * @brief Calculates the pressure at sea level
   * @param altitude_meters Current altitude (in meters)
   * @return Returns the calculated pressure at sea level
   */
  int32_t readSealevelPressure(float altitude_meters = 0);
  /*!
   * @brief Reads the altitude
   * @param sealevelPressure Pressure at sea level, measured in pascals
   * @return Returns the altitude
   */
  float readAltitude(float sealevelPressure = 101325);  // std atmosphere
  /*!
   * @brief Reads the raw temperature
   * @return Returns the raw temperature
   */
  uint16_t readRawTemperature(void);
  /*!
   * @brief Reads the raw pressure
   * @return Returns the raw pressure
   */
  uint32_t readRawPressure(void);

private:
  int32_t computeB5(int32_t UT);
  uint8_t read8(uint8_t addr);
  uint16_t read16(uint8_t addr);
  void write8(uint8_t addr, uint8_t data);

  i2c_master_dev_handle_t i2c_dev;
  uint8_t oversampling;

  int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
  uint16_t ac4, ac5, ac6;
};
