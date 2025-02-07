#include <Wire.h>
#include <BMP280.h>
#include <BH1750.h>

/*#include <SD.h>
#include <LiquidCrystal_I2C.h>

#define CS_PIN 10

File logFile;  // SD file object*/

#define INT_MAX 32678
#define MPU_ADDR 0x68
#define echo 2            // Example pin for echo
#define trigPin 3         // Example pin for trig
#define inputPin 4        // Example pin for motion sensor
#define soundPin 5        // Pin for sound device (e.g., Piezo Buzzer)

void kalmanFilter(float AcX, float AcY, float GyX, float GyY, float dt);
void soundAlert(int frequency);
void checkWarnings();
void TIME_CALIB_CALIB(float* a,int numsSize);


float thresholdAcX = 500;          // Threshold for accelerometer X-axis
float thresholdAcY = 500;          // Threshold for accelerometer Y-axis
float thresholdAcZ = 500;          // Threshold for accelerometer Z-axis
float thresholdCm = 5;             // Threshold for distance in cm
float thresholdTemperature = 0.5;  // Threshold for temperature in degrees Celsius
float thresholdPressure = 5;       // Threshold for pressure in atm
float thresholdLux = 50;           // Threshold for light intensity in lux
float thresholdAngle = 5;          // Threshold for pitch and roll angles in degrees
float currentTime,previousTime[6]={0},TIME_CALIB[6]={0};
// Previous values for comparison
int16_t previousAcX = 0, previousAcY = 0, previousAcZ = 0;  // Accelerometer previous values
int previousCm = 0;                                         // Previous distance in cm
float previousTemperature = 0.0;                           // Previous temperature
float previousPressure = 0.0;                              // Previous pressure
float previousLux = 0.0;                                   // Previous light intensity
float previousAngleX = 0.0, previousAngleY = 0.0;          // Previous pitch and roll angles
// Increment factor for TIME_CALIB adjustment
float incrementFactor = 50.0;  // Adjust as per requirement
// Value of variable time interval (Time interval-Time calib)
float TIME_INTERVAL=200;
// Value of the time for the loop
float LOOP_TIME=1330.0;

float temperature, pressure, lux;
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
long duration;
int cm;
int motionCount = 0;
unsigned long lastMotionTime = 0;
const unsigned long motionDebounceDelay = 1000;  // 2 seconds debounce time

// Kalman filter variables for pitch and roll (sensor fusion)
float angleX = 0.0, angleY = 0.0;  // Pitch and roll angles
float gyroX = 0.0, gyroY = 0.0;    // Gyroscope data
float accelX = 0.0, accelY = 0.0;  // Accelerometer data

// Kalman filter constants
float Q_angle = 0.001, Q_bias = 0.003, R_measure = 0.03;
float P[2][2] = {{1, 0}, {0, 1}};
float biasX = 0.0, biasY = 0.0;



BMP280 bmp(0x76);
BH1750 lightMeter(0x23);
//LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  // Pin setup
  pinMode(echo, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(inputPin, INPUT);
  pinMode(soundPin, OUTPUT);  // Set sound pin as output

  // Serial communication setup
  Serial.begin(9600);

  if (!bmp.begin()) {
    Serial.println("BMP280 sensor initialized.");
    // Oversampling and filter settings can be omitted if not supported
  } else {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
delay(100);
  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23)) {
    Serial.println("BH1750 sensor initialized.");
  } else {
    Serial.println("Error initializing BH1750 sensor.");
    while (1);
  }
delay(100);
 /* if (!SD.begin(CS_PIN)) {  // Use SD.begin() for SD library
    Serial.println("SD initialization failed!");
    return;
  } else {
    Serial.println("SD initialization succeeded!");
  }*/
delay(100);
  // MPU6050 initialization
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // Wake up MPU
  Wire.write(0);     // Command to wake up
  Wire.endTransmission(true);
  delay(100);
 /* lcd.begin(16, 2);  // Set the LCD size (16 columns, 2 rows)
  lcd.setBacklight(true);  // Turn on the LCD backlight*/
}

void loop() {
  
  currentTime=millis();

  
if(currentTime-previousTime[0]>TIME_INTERVAL-TIME_CALIB[0])
{
  // Read accelerometer and gyroscope data from MPU6050 (14 bytes)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // Starting register for accelerometer data
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14);  // Request 14 bytes of data (accelerometer + gyroscope)
  
  AcX = (Wire.read() << 8) | Wire.read();
  AcY = (Wire.read() << 8) | Wire.read();
  AcZ = (Wire.read() << 8) | Wire.read();
  GyX = (Wire.read() << 8) | Wire.read();
  GyY = (Wire.read() << 8) | Wire.read();
  GyZ = (Wire.read() << 8) | Wire.read();

  // Print accelerometer and gyroscope data to Serial Monitor
  Serial.print("AcX: "); Serial.print(AcX);
  Serial.print(" | AcY: "); Serial.print(AcY);
  Serial.print(" | AcZ: "); Serial.print(AcZ);
  Serial.print(" | GyX: "); Serial.print(GyX);
  Serial.print(" | GyY: "); Serial.print(GyY);
  Serial.print(" | GyZ: "); Serial.println(GyZ);

  previousTime[0]=currentTime;
}
if(currentTime-previousTime[1]>TIME_INTERVAL-TIME_CALIB[1])
{
  // Measure distance with ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echo, HIGH);
  cm = (duration / 2) * 0.0343;

  // Print distance to Serial Monitor
  Serial.print("Distance in cm: ");
  Serial.println(cm);

   
  // Check for motion detection with debounce logic
    if (digitalRead(inputPin) == HIGH) {
    motionCount++;
    previousTime[2]= currentTime;  // Update last motion time
    Serial.print("Motion Detected! Count: ");
    Serial.println(motionCount);
    soundAlert(1000);  // Trigger sound alert when motion is detected
  }
previousTime[1]=currentTime;
}



  
if(currentTime-previousTime[2]>TIME_INTERVAL-TIME_CALIB[2])
{
  temperature = bmp.getTemperature(); 
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" C");
  previousTime[2]=currentTime;
}

  
  

if(currentTime-previousTime[3]>TIME_INTERVAL-TIME_CALIB[3])
{
  pressure = bmp.getPressure(); // Use your library's method for pressure

  pressure = pressure * ((float)1000 / 95000);//pressure conversion 
  Serial.print("Pressure: ");
  Serial.print(pressure);
  Serial.println(" atm");
  previousTime[3]=currentTime;

  float alt= bmp.calAltitude(pressure,1);
  Serial.print("Altitude: ");
  Serial.print(alt);
  Serial.println("m");
}

if(currentTime-previousTime[4]>TIME_INTERVAL-TIME_CALIB[4]){
  lux = lightMeter.readLightLevel();  
   Serial.print("Light Intensity: ");
  Serial.print(lux);
  Serial.println(" lux");
   previousTime[4]=currentTime;
}
  

  if(currentTime-previousTime[5]>TIME_INTERVAL-TIME_CALIB[5])
  {
  float dt = 0.01;  // Time step (assuming loop runs every 1330 ms)
  kalmanFilter(AcX, AcY, GyX, GyY, dt);  // Use AcX, AcY for accelerometer data

  // Output the sensor data and processed results
  Serial.print("Pitch: ");
  Serial.print(angleX);
  Serial.print(" | Roll: ");
  Serial.print(angleY);
  Serial.print(" | Distance: ");
  Serial.println(cm);
   previousTime[5]=currentTime;
  }

//Commented cut out part so that circuit doesnt get overloaded
/*

  // Write data to SD card
  logFile = SD.open("log.csv", FILE_WRITE);  // Open the log file
  if (logFile) {
    logFile.print(temperature); logFile.print(", ");
    logFile.print(pressure); logFile.print(", ");
    logFile.print(lux); logFile.print(", ");
    logFile.print(angleX); logFile.print(", ");
    logFile.print(angleY); logFile.print(", ");
    logFile.println(cm);  // Write data as a CSV row
    logFile.close();  // Close the file
  } else {
    Serial.println("Error opening log file.");
  }

   lcd.setCursor(0,0);
  lcd.print(" T P L  ");
  lcd.print(temperature);
  lcd.print("C ");
  lcd.setCursor(0,1);
  lcd.print((int)pressure);
  lcd.print("atm ");
  lcd.print((int)lux);
  lcd.print("Lux ");*/
// Define individual thresholds for each condition

// Increment TIME_CALIB based on detected changes
if (abs(AcX - previousAcX) > thresholdAcX || abs(AcY - previousAcY) > thresholdAcY || abs(AcZ - previousAcZ) > thresholdAcZ) {
  TIME_CALIB[0] += incrementFactor;
}
if (abs(cm - previousCm) > thresholdCm) {
  TIME_CALIB[1] += incrementFactor;
}
if (abs(temperature - previousTemperature) > thresholdTemperature) {
  TIME_CALIB[2] += incrementFactor;
}
if (abs(pressure - previousPressure) > thresholdPressure) {
  TIME_CALIB[3] += incrementFactor;
}
if (abs(lux - previousLux) > thresholdLux) {
  TIME_CALIB[4] += incrementFactor;
}
if (abs(angleX - previousAngleX) > thresholdAngle || abs(angleY - previousAngleY) > thresholdAngle) {
  TIME_CALIB[5] += incrementFactor;
}

// Update previous values
previousAcX = AcX;
previousAcY = AcY;
previousAcZ = AcZ;
previousCm = cm;
previousTemperature = temperature;
previousPressure = pressure;
previousLux = lux;
previousAngleX = angleX;
previousAngleY = angleY;

  //Function to calculate if all the variables are being processed in same loop and change if needed
  TIME_CALIB_CALIB(TIME_CALIB,6);
  // Warnings based on thresholds for each variable
  checkWarnings();

  
  delay(1000);  // Adjust the loop delay as needed
}

// Kalman Filter for sensor fusion
void kalmanFilter(float AcX, float AcY, float GyX, float GyY, float dt) {
  // Kalman Filter for X-axis (pitch)
  float angleX_predicted = angleX + (GyX - biasX) * dt;
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;
  
  float S = P[0][0] + R_measure;
  float K[2] = {P[0][0] / S, P[1][0] / S};
  
  float y = AcX - angleX_predicted;  // Use AcX for pitch calculation
  angleX += K[0] * y;
  biasX += K[1] * y;
  
  float P00_temp = P[0][0];
  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P[0][1];
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P[0][1];

  // Kalman Filter for Y-axis (roll)
  float angleY_predicted = angleY + (GyY - biasY) * dt;
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;
  
  S = P[0][0] + R_measure;
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;
  
  y = AcY - angleY_predicted;  // Use AcY for roll calculation
  angleY += K[0] * y;
  biasY += K[1] * y;
  
  P00_temp = P[0][0];
  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P[0][1];
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P[0][1];
}

// Function to play a sound alert
void soundAlert(int frequency) {
  tone(soundPin, frequency, 500);  // Play sound for 500ms
}

// Function to check thresholds and trigger warnings
void checkWarnings() {
  if (temperature < 18.0 || temperature > 30.0) {
    soundAlert(1500);  // High or low temperature alert
    Serial.println("Warning: Temperature outside safe range!");
  }
  if (pressure < 950.0 || pressure > 1050.0) {
    soundAlert(1200);  // High or low pressure alert
    Serial.println("Warning: Pressure outside safe range!");
  }
 
  if (lux < 100.0 || lux > 1000.0) {
    soundAlert(1000);  // Light intensity alert
    Serial.println("Warning: Light intensity outside safe range!");
  }
  if (motionCount > 0) {
    soundAlert(2000);  // Motion detected
    Serial.println("Warning: Motion detected!");
  }
  if (angleX > 30.0) {
    soundAlert(1800);  // Pitch alert
    Serial.println("Warning: High pitch angle detected!");
  }
  if (angleY > 30.0) {
    soundAlert(1800);  // Roll alert
    Serial.println("Warning: High roll angle detected!");
  }
}

void TIME_CALIB_CALIB(float* a,int numsSize)
{
  int minN=INT_MAX;
  for(int i=0;i<numsSize;i++)
  {
    if((int)((TIME_INTERVAL-a[i])/LOOP_TIME)<minN)
    {
      minN=(int)((TIME_INTERVAL-a[i])/LOOP_TIME);
    }
  }
  for(int i=0;i<numsSize;i++)
  {
    if((int)((TIME_INTERVAL-a[i])/LOOP_TIME)>minN)
    {
      a[i]=minN*LOOP_TIME + LOOP_TIME/2;
    }
  }

}