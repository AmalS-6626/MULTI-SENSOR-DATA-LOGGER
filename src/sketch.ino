#include <Wire.h>
#include <BH1750.h>
#include "Arduino.h"
#include <LiquidCrystal_I2C.h>
#include <SdFat.h>

#define SPI_SPEED SD_SCK_MHZ(4)
#define CS_PIN 10

SdFat sd;

SdFile logFile;

#define BMP280_DEFAULT_ADDRESS 0x76

class BMP280
{
public:
  typedef enum {
    eStatusOK,
    eStatusErr,
    eStatusErrDeviceNotDetected,
    eStatusErrParameter
  } eStatus_t;

  typedef struct {
    uint16_t    t1;
    int16_t     t2, t3;
    uint16_t    p1;
    int16_t     p2, p3, p4, p5, p6, p7, p8, p9;
    uint16_t    reserved0;
  } sCalibrateDig_t;

  typedef struct {
    uint8_t   im_update: 1;
    uint8_t   reserved: 2;
    uint8_t   measuring: 1;
  } sRegStatus_t;

  /**
   * @enum eCtrlMeasMode_t
   * @brief Enum control measurement mode (power)
   */
  typedef enum {
    eCtrlMeasModeSleep,
    eCtrlMeasModeForced,
    eCtrlMeasModeNormal = 0x03
  } eCtrlMeasMode_t;

  /**
   * @enum  eSampling_t
   * @brief Enum sampling
   */
  typedef enum {
    eSampling_no,
    eSampling_X1,
    eSampling_X2,
    eSampling_X4,
    eSampling_X8,
    eSampling_X16
  } eSampling_t;

  typedef struct {
    uint8_t   mode: 2;
    uint8_t   osrs_p: 3;
    uint8_t   osrs_t: 3;
  } sRegCtrlMeas_t;

  typedef enum {
    eConfigSpi3w_en_disable,
    eConfigSpi3w_en_enable
  } eConfigSpi3w_en_t;

  /**
   * @enum eConfigFilter_t
   * @brief Enum config filter
   */
  typedef enum {
    eConfigFilter_off,
    eConfigFilter_X2,
    eConfigFilter_X4,
    eConfigFilter_X8,
    eConfigFilter_X16
  } eConfigFilter_t;

  /**
   * @enum eConfigTStandby_t
   * @brief Enum config standby time, unit ms
   */
  typedef enum {
    eeStatus_tConfigTStandby_0_5,    /**< 0.5 ms */
    eConfigTStandby_62_5,
    eConfigTStandby_125,
    eConfigTStandby_250,
    eConfigTStandby_500,
    eConfigTStandby_1000,
    eConfigTStandby_2000,
    eConfigTStandby_4000
  } eConfigTStandby_t;

  typedef struct {
    uint8_t   spi3w_en: 1;
    uint8_t   reserved1: 1;
    uint8_t   filter: 3;
    uint8_t   t_sb: 3;
  } sRegConfig_t;

  typedef struct {
    uint8_t   msb, lsb;
    uint8_t   reserved: 4;
    uint8_t   xlsb: 4;
  } sRegPress_t;

  typedef struct {
    uint8_t   msb, lsb;
    uint8_t   reserved: 4;
    uint8_t   xlsb: 4;
  } sRegTemp_t;

  #define BMP280_REG_START    0x88
  typedef struct {
    sCalibrateDig_t   calib;
    uint8_t   reserved0[(0xd0 - 0xa1 - 1)];
    uint8_t   chip_id;
    #define BMP280_REG_CHIP_ID_DEFAULT    0x58
    uint8_t   reserved1[(0xe0 - 0xd0 - 1)];
    uint8_t   reset;
    uint8_t   reserved2[(0xf3 - 0xe0 - 1)];
    sRegStatus_t    status;
    sRegCtrlMeas_t    ctrlMeas;
    sRegConfig_t      config;
    uint8_t   reserved3;
    sRegPress_t   press;
    sRegTemp_t    temp;
  } sRegs_t;

public:
  explicit BMP280(const uint8_t deviceAddress = BMP280_DEFAULT_ADDRESS);

  /**
   * @fn begin
   * @brief begin Sensor begin
   * @return Enum of eStatus_t
   */
  uint8_t begin();

  /**
   * @fn getTemperature
   * @brief getTemperature Get temperature
   * @return Temprature in Celsius
   */
  float  getTemperature();

  /**
   * @fn getPressure
   * @brief getPressure Get pressure
   * @return Pressure in pa
   */
  uint32_t getPressure();

  /**
   * @fn calAltitude
   * @brief calAltitude Calculate altitude
   * @param seaLevelPressure Sea level pressure
   * @param pressure Pressure in pa
   * @return Altitude in meter
   */
  int16_t calAltitude(uint32_t pressure, float seaLevelPressure = 1013.0);

  /**
   * @fn reset
   * @brief reset Reset sensor
   */
  void reset();

  /**
   * @fn setCtrlMeasMode
   * @brief setCtrlMeasMode Set control measure mode
   * @param eMode One enum of eCtrlMeasMode_t
   */
  void setCtrlMeasMode(eCtrlMeasMode_t eMode);

  /**
   * @fn setCtrlMeasSamplingTemp
   * @brief setCtrlMeasSamplingTemp Set control measure temperature oversampling
   * @param eSampling One enum of eSampling_t
   */
  void setCtrlMeasSamplingTemp(eSampling_t eSampling);

  /**
   * @fn setCtrlMeasSamplingPress
   * @brief setCtrlMeasSamplingPress Set control measure pressure oversampling
   * @param eSampling One enum of eSampling_t
   */
  void setCtrlMeasSamplingPress(eSampling_t eSampling);

  /**
   * @fn setConfigFilter
   * @brief setConfigFilter Set config filter
   * @param eFilter One enum of eConfigFilter_t
   */
  void setConfigFilter(eConfigFilter_t eFilter);

  /**
   * @fn setConfigTStandby
   * @brief setConfigTStandby Set config standby time
   * @param eT One enum of eConfigTStandby_t
   */
  void setConfigTStandby(eConfigTStandby_t eT);
int32_t   getTemperatureRaw();
  int32_t   getPressureRaw();

protected:
  void    getCalibrate();

  

  uint8_t   getReg(uint8_t reg);
  void      writeRegBits(uint8_t reg, uint8_t field, uint8_t val);

  virtual void    writeReg(uint8_t reg, uint8_t *pBuf, uint8_t len);
  virtual void    readReg(uint8_t reg, uint8_t *pBuf, uint8_t len);


public:
  eStatus_t   lastOperateStatus; /**< lastOperateStatus Last operate status*/
  sCalibrateDig_t   _sCalib;

protected:
  int32_t   _t_fine;
  uint8_t   _deviceAddress;
 
};

const BMP280::sRegs_t PROGMEM   _sRegs = BMP280::sRegs_t();
#ifdef __AVR__
typedef uint16_t    platformBitWidth_t;
#else
typedef uint32_t    platformBitWidth_t;
#endif

const platformBitWidth_t    _regsAddr = (platformBitWidth_t) &_sRegs;

#define writeRegBitsHelper(reg, flied, val)   writeRegBits(regOffset(&(reg)), *(uint8_t*) &(flied), *(uint8_t*) &(val))

#define __DBG   0
#if __DBG
# define __DBG_CODE(x)   Serial.print("__DBG_CODE: "); Serial.print(__FUNCTION__); Serial.print(" "); Serial.print(__LINE__); Serial.print(" "); x; Serial.println()
#else
# define __DBG_CODE(x)
#endif

uint8_t regOffset(const void *pReg)
{
  return ((platformBitWidth_t) pReg - _regsAddr + BMP280_REG_START);
}

BMP280::BMP280(const uint8_t deviceAddress)
{
  _deviceAddress = deviceAddress;
}

uint8_t BMP280::begin()
{
  Serial.print("last register addr: "); Serial.print(regOffset(&_sRegs.temp), HEX);
  __DBG_CODE(Serial.print("first register addr: "); Serial.print(regOffset(&_sRegs.calib), HEX));
  __DBG_CODE(Serial.print("status register addr: "); Serial.print(regOffset(&_sRegs.status), HEX));
  __DBG_CODE(Serial.print("id register addr: "); Serial.print(regOffset(&_sRegs.chip_id), HEX));
  __DBG_CODE(Serial.print("res0 register addr: "); Serial.print(regOffset(&_sRegs.reserved0), HEX));

  uint8_t   temp = getReg(regOffset(&_sRegs.chip_id));
  if((temp == BMP280_REG_CHIP_ID_DEFAULT) && (lastOperateStatus == eStatusOK)) {
    reset();
    delay(200);
    getCalibrate();
    setCtrlMeasSamplingPress(eSampling_X8);
    setCtrlMeasSamplingTemp(eSampling_X8);
    setConfigFilter(eConfigFilter_off);
    setConfigTStandby(eConfigTStandby_125);
    setCtrlMeasMode(eCtrlMeasModeNormal);    // set control measurement mode to make these settings effective
  } else
    lastOperateStatus = eStatusErrDeviceNotDetected;
  return lastOperateStatus;
}

float BMP280::getTemperature()
{
  int32_t   raw = getTemperatureRaw();
  float inputMin = 0;
  float inputMax = 65793;
  float outputMin = -100.0;
  float outputMax = 250.0;

    // Map the raw value to the desired output range using linear interpolation
    return (raw / (inputMax - inputMin) * (outputMax - outputMin) + outputMin);
}


uint32_t BMP280::getPressure()
{
  int32_t raw =getPressureRaw();
 float inputMin = 0;
  float inputMax = 65793;
  float outputMin = 0;
  float outputMax = 100000;

  // Map the raw value to the desired output range using linear interpolation
  return (raw / (inputMax - inputMin) * (outputMax - outputMin) + outputMin);
}

int16_t BMP280::calAltitude(uint32_t pressure, float seaLevelPressure)
{
  float z = 44330 * (1.0f - pow(pressure / 100 / seaLevelPressure, 0.1903));
  return round(z);
}

void BMP280::reset()
{
  uint8_t   temp = 0xb6;
  writeReg(regOffset(&_sRegs.reset), (uint8_t*) &temp, sizeof(temp));
  delay(100);
}

void BMP280::setCtrlMeasMode(eCtrlMeasMode_t eMode)
{
  sRegCtrlMeas_t    sRegFlied = {0}, sRegVal = {0};
  sRegFlied.mode = 0b11; //normal mode
  sRegVal.mode = eMode;
  writeRegBitsHelper(_sRegs.ctrlMeas, sRegFlied, sRegVal);
}

void BMP280::setCtrlMeasSamplingTemp(eSampling_t eSampling)
{
  sRegCtrlMeas_t    sRegFlied = {0}, sRegVal = {0};
  sRegFlied.osrs_t = 0b111;
  sRegVal.osrs_t = eSampling;
  writeRegBitsHelper(_sRegs.ctrlMeas, sRegFlied, sRegVal);
}

void BMP280::setCtrlMeasSamplingPress(eSampling_t eSampling)
{
  sRegCtrlMeas_t    sRegFlied = {0}, sRegVal = {0};
  sRegFlied.osrs_p = 0b111;
  sRegVal.osrs_p = eSampling;
  writeRegBitsHelper(_sRegs.ctrlMeas, sRegFlied, sRegVal);
}

void BMP280::setConfigFilter(eConfigFilter_t eFilter)
{
  sRegConfig_t    sRegFlied = {0}, sRegVal = {0};
  sRegFlied.filter = 0b111;
  sRegVal.filter = eFilter;
  writeRegBitsHelper(_sRegs.config, sRegFlied, sRegVal);
}

void BMP280::setConfigTStandby(eConfigTStandby_t eT)
{
  sRegConfig_t    sRegFlied = {0}, sRegVal = {0};
  sRegFlied.t_sb = 0b111;
  sRegVal.t_sb = eT;
  writeRegBitsHelper(_sRegs.config, sRegFlied, sRegVal);
}

void BMP280::getCalibrate()
{
  readReg(regOffset(&_sRegs.calib), (uint8_t*) &_sCalib, sizeof(_sCalib));
}

int32_t BMP280::getTemperatureRaw()
{
  sRegTemp_t    sReg;
  readReg(regOffset(&_sRegs.temp), (uint8_t*) &sReg, sizeof(sReg));
  int32_t raw = (((uint32_t) sReg.msb << 12) | ((uint32_t) sReg.lsb << 4) | ((uint32_t) sReg.xlsb));
  __DBG_CODE(Serial.print("raw: "); Serial.print(raw));
  return raw;
}

int32_t BMP280::getPressureRaw()
{
  sRegPress_t   sReg;
  readReg(regOffset(&_sRegs.press), (uint8_t*) &sReg, sizeof(sReg));
  return (int32_t)((((uint32_t) sReg.msb << 12) | ((uint32_t) sReg.lsb << 4) | ((uint32_t) sReg.xlsb)));
}

uint8_t BMP280::getReg(uint8_t reg)
{
  uint8_t   temp;
  readReg(reg, (uint8_t*) &temp, sizeof(temp));
  return temp;
}

void BMP280::writeRegBits(uint8_t reg, uint8_t field, uint8_t val)
{
  __DBG_CODE(Serial.print("reg: "); Serial.print(reg, HEX); Serial.print(" flied: "); Serial.print(field, HEX); Serial.print(" val: "); Serial.print(val, HEX));

  uint8_t   temp;
  readReg(reg, (uint8_t*) &temp, sizeof(temp));
  temp &= ~field;
  temp |= val;
  writeReg(reg, (uint8_t*) &temp, sizeof(temp));
}

void BMP280::readReg(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
  lastOperateStatus = eStatusErrDeviceNotDetected;
  Wire.beginTransmission(_deviceAddress);
  Wire.write(reg);
  if(Wire.endTransmission() != 0)
    return;
  Wire.requestFrom(_deviceAddress, len);
  for(uint8_t i = 0; i < len; i ++)
    pBuf[i] = Wire.read();
  lastOperateStatus = eStatusOK;
}

void BMP280::writeReg(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
  lastOperateStatus = eStatusErrDeviceNotDetected;
  Wire.beginTransmission(_deviceAddress);
  Wire.write(reg);
  for(uint8_t i = 0; i < len; i ++)
    Wire.write(pBuf[i]);
  if(Wire.endTransmission() != 0)
    return;
  lastOperateStatus = eStatusOK;
}
BMP280 bmp(0x76); // Using BMP280 library
BH1750 lightMeter(0x23);
LiquidCrystal_I2C lcd(0x27,16,2);

void setup() {
  Serial.begin(9600);

  lcd.begin(16,2);
  lcd.backlight();

  Wire.begin();
  bmp._sCalib.t1 = 0;  // Example value for t1 (use the actual calibration value)
bmp._sCalib.t2 = 30000;  // Example value for t2 (use the actual calibration value)
bmp._sCalib.t3 = 30000;  // Example value for t3 (use the actual calibration value)
  // Initialize BMP280
  if (bmp.begin()) {
    Serial.println("BMP280 sensor initialized.");
    // Oversampling and filter settings can be omitted if not supported
  } else {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
  
  // Initialize BH1750
  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE,0x23)) {
    Serial.println("BH1750 sensor initialized.");
  } else {
    Serial.println("Error initializing BH1750 sensor.");
    while (1);
  }

  if (!sd.begin(CS_PIN, SPI_SPEED)) {
    if (sd.card()->errorCode()) {
      Serial.println("SD initialization failed.");
    } else if (sd.vol()->fatType() == 0) {
      Serial.println("Can't find a valid FAT16/FAT32 partition.");
    } else {
      Serial.println("Can't determine error type");
    }
    return;
  }

  Serial.println("Files on card:");
  Serial.println("   Size    Name");

  sd.ls(LS_R | LS_SIZE);
}

int i=0;

void loop() {
  // Read temperature and pressure from BMP280
  float temperature = bmp.getTemperature();  // Use your library's method for temperature
  float pressure = bmp.getPressure(); // Use your library's method for pressure
  
  pressure= pressure*((float)1000/95000);
  // Read light intensity from BH1750
  float lux = lightMeter.readLightLevel();  


  if (logFile.open("LOG.TXT", O_WRONLY | O_CREAT | O_APPEND)) {
    logFile.print("Temp: ");
    logFile.print(temperature);
    logFile.print(" C, Pressure: ");
    logFile.print(pressure);
    logFile.print(" atm, Lux: ");
    logFile.print(lux);

    // Close the file
    logFile.close();
  } else {
    Serial.println("Error opening LOG.TXT!");
  }

  if(i++==1)
  {
  Serial.println("Files on card:");
  Serial.println("   Size    Name");

  sd.ls(LS_R | LS_SIZE);
  }


  Serial.print("Temperature: ");
  Serial.print(bmp.getTemperatureRaw());
  Serial.println(" C");
  
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" C");

  Serial.print("Pressure: ");
  Serial.print(pressure);
  Serial.println(" atm");

  Serial.print("Light Intensity: ");
  Serial.print(lux);
  Serial.println(" lux");

  if(temperature>50 || temperature<0)
  {
    digitalWrite(8,HIGH);
  }
  else
  {
    digitalWrite(8,LOW);
  }

  if(pressure>750||pressure==0)
  {
    digitalWrite(7, HIGH);
  }
  else{
    digitalWrite(7,LOW);
  }

  if(lux>7500)
  {
    digitalWrite(4,HIGH);
  }
  else{
    digitalWrite(4,LOW);
  }

  //lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(" T P L  ");
  lcd.print(temperature);
  lcd.print("C ");
  lcd.setCursor(0,1);
  lcd.print((int)pressure);
  lcd.print("atm ");
  lcd.print((int)lux);
  lcd.print("Lux ");
  delay(1000);  
}
