#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#define BMP280_ADDR 0x76
#define CALIBRATION_DATA 0x88
#define TEMP_DATA 0xFA
#define PRESS_DATA 0xF7
#define CTRL_MEAS 0xF4
#define CONFIG 0xF5
#define RESET 0xE0

/*#define T1 27504
#define T2 26435
#define T3 -1000*/

typedef struct {
  uint32_t temp;
  uint32_t press;
  uint8_t reg_address;
} chip_state_t;

bool on_i2c_connect(void *user_data, uint32_t address, bool read) {
  chip_state_t *chip = (chip_state_t *)user_data;
  if (address == BMP280_ADDR) {
    return true;
  }
  return false;
}

uint8_t on_i2c_read(void *user_data) {
  chip_state_t *chip = (chip_state_t *)user_data;
  switch (chip->reg_address) {
    /*case CALIBRATION_DATA: 
    return (T1) & 0xFF;        // LSB of t1
  case CALIBRATION_DATA + 1:
    return (T1 >> 8) & 0xFF;   // MSB of t1
  case CALIBRATION_DATA + 2:
    return (T2) & 0xFF;        // LSB of t2
  case CALIBRATION_DATA + 3:
    return (T2 >> 8) & 0xFF;   // MSB of t2
  case CALIBRATION_DATA + 4:
    return (T3) & 0xFF;        // LSB of t3
  case CALIBRATION_DATA + 5:
    return (T3 >> 8) & 0xFF;   // MSB of t3*/
    case TEMP_DATA:
      return (uint8_t)((uint8_t)(attr_read(chip->temp) >> 16) & 0xFF);
    case TEMP_DATA + 1:
      return (uint8_t)((uint8_t)(attr_read(chip->temp) >> 8) & 0xFF);
    case TEMP_DATA + 2:
      return (uint8_t)((uint8_t)attr_read(chip->temp) & 0xFF);
    case PRESS_DATA:
      return ( attr_read(chip->press) >> 16) & 0xFF;
    case PRESS_DATA + 1:
      return ( attr_read(chip->press) >> 8) & 0xFF;
    case PRESS_DATA + 2:
      return attr_read(chip->press) & 0xFF;
    default:
      return 0xFF;
  }
}

bool on_i2c_write(void *user_data, uint8_t data) {
  chip_state_t *chip = (chip_state_t *)user_data;
  chip->reg_address = data;
  return true;
}

void on_i2c_disconnect(void *user_data) {
}

/*void update_temperature(chip_state_t *chip) {
  // Fetch the temperature value from Wokwi control
  chip->temp = attr_read(temp_attr_id);
}

void update_pressure(chip_state_t *chip) {
  // Fetch the pressure value from Wokwi control
  chip->press = attr_read(press_attr_id);
}*/


void chip_init() {
  static chip_state_t chip;
  
  const i2c_config_t i2c_config = {
    .address = BMP280_ADDR,
    .scl = pin_init("SCL", INPUT_PULLUP),
    .sda = pin_init("SDA", INPUT_PULLUP),
    .connect = on_i2c_connect,
    .read = on_i2c_read,
    .write = on_i2c_write,
    .disconnect = on_i2c_disconnect,
    .user_data = &chip,
  };

  i2c_init(&i2c_config);
  chip.reg_address = 0x00;
  printf("BMP280 initialized!\n");
  chip.temp = attr_init("temp",100);


  chip.press = attr_init("press",20);

  

}

/*void chip_loop() {
  static chip_state_t chip; // Use the global or passed chip state

  printf("Reached chip_loop\n");

  // Update temperature and pressure values
  update_temperature(&chip);
  update_pressure(&chip);

  // Print the updated values
  printf("Updated Temperature: %u\n", chip.temp);
  printf("Updated Pressure: %u\n", chip.press);
}*/

