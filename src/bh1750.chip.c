/*#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define BH1750_ADDR 0x23 
#define LIGHT_DATA 0x23   
#define READ_LIGHT 0x10  

typedef struct {
  uint32_t light;      
  uint8_t reg_address;  
} chip_state_t;

bool on_i2c_connect(void *user_data, uint32_t address, bool read) {
  chip_state_t *chip = (chip_state_t *)user_data;
  
  if (address == BH1750_ADDR) {
    return true;
  }
  return false; 
}

uint8_t on_i2c_read(void *user_data) {
  chip_state_t *chip = (chip_state_t *)user_data;

  switch (chip->reg_address) {
    case LIGHT_DATA:
      return (attr_read(chip->light) >> 8) & 0xFF;  
    case LIGHT_DATA + 1:
      return (attr_read(chip->light) & 0xFF);         
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



void chip_init() {
  static chip_state_t chip; 

  const i2c_config_t i2c_config = {
    .address = BH1750_ADDR,
    .scl = pin_init("SCL", INPUT_PULLUP),  
    .sda = pin_init("SDA", INPUT_PULLUP),  
    .connect = on_i2c_connect,
    .read = on_i2c_read,
    .write = on_i2c_write,
    .disconnect = on_i2c_disconnect,
    .user_data = &chip,
  };

  i2c_init(&i2c_config);
  chip.reg_address = 0x10; 
  printf("BH1750 Light Sensor initialized!\n");

  chip.light=attr_init("Lt",150);
}

*/

#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define BH1750_ADDR 0x23 
#define LIGHT_DATA 0x23   
#define READ_LIGHT 0x10  

typedef enum
{
  ST_EXPECTING_READ_BYTE_1,
  ST_EXPECTING_READ_BYTE_2,
} States;

typedef struct {
  uint32_t light;      
  uint8_t reg_address;  
  States state;
} chip_state_t;

bool on_i2c_connect(void *user_data, uint32_t address, bool read) {
  chip_state_t *chip = (chip_state_t *)user_data;
 chip->state = ST_EXPECTING_READ_BYTE_1;
  if (address == BH1750_ADDR) {
    return true;
  }
  return false; 
}

uint8_t on_i2c_read(void *user_data) {
  chip_state_t *chip = (chip_state_t *)user_data;

  const uint32_t lightlevel = attr_read(chip->light) * 1.2;
  uint8_t level;
  switch (chip->state)
  {
    case ST_EXPECTING_READ_BYTE_1:
      level = lightlevel / 256;
      chip->state = ST_EXPECTING_READ_BYTE_2;
      break;
    case ST_EXPECTING_READ_BYTE_2:
      level = lightlevel % 256;
      chip->state = ST_EXPECTING_READ_BYTE_1;
      break;
  }
  return level; 
}

bool on_i2c_write(void *user_data, uint8_t data) {
  chip_state_t *chip = (chip_state_t *)user_data;

  chip->reg_address = data;
  return true; 
}

void on_i2c_disconnect(void *user_data) {
}


void chip_init() {
  static chip_state_t chip; 
  pin_init("ADDR", INPUT_PULLUP); 


  const i2c_config_t i2c_config = {
    .address = BH1750_ADDR,
    .scl = pin_init("SCL", INPUT_PULLUP),  
    .sda = pin_init("SDA", INPUT_PULLUP),  
    .connect = on_i2c_connect,
    .read = on_i2c_read,
    .write = on_i2c_write,
    .disconnect = on_i2c_disconnect,
    .user_data = &chip,
  };

  i2c_init(&i2c_config);
  chip.reg_address = 0x23; 
  printf("BH1750 Light Sensor initialized!\n");

  chip.light=attr_init("LightLevel",0);
}
