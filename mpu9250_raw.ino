/*
This code will give you partially processed values from the mpu6050
gyroscope -- output in deg/sec
accelerometer -- output in g's
temperature -- output in degree celcius
*/

#include <Wire.h>

    #define MPU9250_AD 0x68 // mpu9250 I2C address
    #define MAG_ADDR 0x0C
    #define PWR_MGMT_1_AD 0x6B // power management register

    //magnetometer config register 
    #define USER_CTRL_AD 0x6A
    #define INT_BYPASS_CONFIG_AD 0x37
    #define CNTL1_AD 0x0A   //control 1 register address
    #define ASAX_AD 0x10 
    #define STATUS_1_AD 0x02
    #define HXL_AD 0x03

  #define MAGNE_SENS 6.67f
#define SCALE 0.1499f  // 4912/32760 uT/tick
#define DATA_READY 0x01
#define MAGIC_OVERFLOW 0x8





    
int16_t GyroX, GyroY, GyroZ, AccX, AccY, AccZ,Magx,Magy,Magz, Temp;
float GForceX, GForceY, GForceZ, RotX, RotY, RotZ;
float MagX,MagY,MagZ;
//int SCALE = 1;

//float  asax, asay, asaz;

int8_t Msens_x,Msens_y, Msens_z;

int8_t control_1;
int8_t status_1;
int8_t status_2;
//float Yaw;
float asax, asay, asaz;
void setup()
{
    Serial.begin(9600);
    Wire.begin();
    SetUpMpu6050();
    Serial.println("Apple");
}

void loop()
{
//    SetUpMpu6050();  // This function is meant for setting up the mpu6050 according to our requirements
    RecordMpuData(); // This function is meant to get the latest measurements from the mpu6050
    PrintValues();   // This function is meant for printing the acquired values from the sensor
}

void SetUpMpu6050()
{
    // Power Management of MPU6050 configuration
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);       // This register helps in configuring the power management of MPU6050
    Wire.write(0b00000000); // setting the power management register to 0 to make sure it is not in sleep mode.
    Wire.endTransmission();

    // gyro configuration
    Wire.beginTransmission(0x68);
    Wire.write(0x1B);       // This register helps in configuring the gyro full scale range.
    Wire.write(0b00000000); // setting the gyro full scale to +/-250 deg/sec
    Wire.endTransmission();

    // accelerometer configurations
    Wire.beginTransmission(0x68);
    Wire.write(0x1C);       // This register helps in configuring the accelerometer full scale range
    Wire.write(0b00000000); // setting the accel to +/-2g;
    Wire.endTransmission();


    // magnetometer configuration 

   // Actually we don't need this step cause the reset value of the register 106 is 0x00
   
   Wire.beginTransmission(MAG_AD);
   Wire.write(USER_CTRL_AD);
   Wire.write(0x00);
   Wire.endTransmission();
   
   
   Wire.beginTransmission(MAG_AD);
   Wire.write(INT_BYPASS_CONFIG_AD);
   Wire.write(0x02);   //0000 0010 in binary, turn on the bypass multiplexer
   Wire.endTransmission();



   // setup the Magnetometer to fuse ROM access mode to get the Sensitivity Adjustment values and 16-bit output
   Wire.beginTransmission(MAG_AD);
   Wire.write(CNTL1_AD);
   Wire.write(0x1F);  //0001 1111 in binary
   Wire.endTransmission();
   delay(100);  //wait for the mode changes


   //read the Sensitivit Adjustment values for magnetometer
   Wire.beginTransmission(MAG_AD);
   Wire.write(ASAX_AD);
   Wire.requestFrom(MAG_AD,3);
   while (Wire.available() < 3);
   asax = (Wire.read()-128)*0.5/128+1;
   asay = (Wire.read()-128)*0.5/128+1;
   asaz = (Wire.read()-128)*0.5/128+1;

   //reset the Magnetometer to power down mode
   Wire.beginTransmission(MAG_AD);
   Wire.write(CNTL1_AD);
  Wire. write(0x00);
  Wire.endTransmission();

   //wait for the mode changes
   delay(100);


   //set the Magnetometer to continuous mode 2（100Hz) and 16-bit output
   Wire.beginTransmission(MAG_AD);
   Wire.write(CNTL1_AD);
   Wire.write(0x16);   //0001 0110 in binary
  Wire.endTransmission();
   //wait for the mode changes
   delay(100);



    
    
}

void RecordMpuData()
{
    // Fetching the latest gyro,temp and accel measurement
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);        // starting register
    Wire.endTransmission();
    Wire.requestFrom(0x68, 14); // requesting all 28 bytes from the MPU6050
    while (Wire.available() < 14);
    AccX = Wire.read() << 8 | Wire.read();
    AccY = Wire.read() << 8 | Wire.read();
    AccZ = Wire.read() << 8 | Wire.read();
    Temp = Wire.read() << 8 | Wire.read();
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();

   
//    Wire.write(0x03);        // starting register
//    Wire.endTransmission();



    
//    Wire.requestFrom(0x68, 6); // requesting all 28 bytes from the MPU6050
//    Magx = Wire.read() << 8 | Wire.read();
//    Magy = Wire.read() << 8 | Wire.read();
//    Magz = Wire.read() << 8 | Wire.read();

    Wire.beginTransmission(MAG_ADDR);
    Wire.write(STATUS_1_AD);
    Wire.endTransmission();
    Wire.requestFrom(MAG_ADDR,1);  //pull the DRDY bit
    int8_t DRDY_BIT = Wire.read()<<7;
    if(DRDY_BIT == DATA_READY){
         Wire.beginTransmission(MAG_ADDR);
        Wire.write(HXL_AD);
        Wire.requestFrom(MAG_ADDR,7);
        Wire.endTransmission();
        
    Wire.requestFrom(MAG_ADDR, 6); // requesting all 28 bytes from the MPU6050
    while (Wire.available() < 6);
        byte* buffer = Wire.read();
        if(!(buffer[6] & MAGIC_OVERFLOW)){     //check whether the magnetic sensor is overflown
          Magx = buffer[0] | (buffer[1]<<8);
          Magy = buffer[2] | (buffer[3]<<8);
          Magz = buffer[4] | (buffer[5]<<8);
        }
    }

    
    ProcessAccData();
    ProcessGryroData();
    ProcessTempData();
    ProcessMagData();
}


void ProcessAccData()
{
    /*
    Full Scale Range      LSB Sensitivity
    ±2g                   16384 LSB/g
    ±4g                   8192 LSB/g
    ±8g                   4096 LSB/g
    ±16g                  2048 LSB/g
    */

    // Dividing by 16384 since our full scale range is set to +/- 2g
    GForceX = (float)AccX / 16384.0;
    GForceY = (float)AccY / 16384.0;
    GForceZ = (float)AccZ / 16384.0;
}

void ProcessGryroData()
{
    /*
    Full Scale Range     LSB Sensitivity
    ± 250 °/s             131 LSB/°/s
    ± 500 °/s             65.5 LSB/°/s
    ± 1000 °/s            32.8 LSB/°/s
    ± 2000 °/s            16.4 LSB/°/s
    */

    // Dividing by 131 since our full scale range is set to +/-250 deg/sec
    RotX = (float)GyroX / 131.0;
    RotY = (float)GyroY / 131.0;
    RotZ = (float)GyroZ / 131.0;
}

void ProcessTempData()
{
    // Temperature in degrees C = (TEMP_OUT Register Value as a signed quantity)/340 + 36.53
    Temp = ((float)Temp) / 340 + 36.53;
}


void ProcessMagData(){

MagX = Magx*asax*SCALE;
MagY = Magy*asay*SCALE;
MagZ = Magz*asaz*SCALE;
  
}
void PrintValues()
{
    Serial.print("Gyro(deg)");
    Serial.print(" X=");
    Serial.print(RotX);
    Serial.print(" Y=");
    Serial.print(RotY);
    Serial.print(" Z=");
    Serial.print(RotZ);
    Serial.print("    Temp(cel)= ");
    Serial.print(Temp);
    Serial.print("    Acc(g)");
    Serial.print(" X=");
    Serial.print(GForceX);
    Serial.print(" Y=");
    Serial.print(GForceY);
    Serial.print(" Z=");
    Serial.print(GForceZ);
    Serial.print("    Mag(T)");
    Serial.print(" X=");
    Serial.print(MagX);
    Serial.print(" Y=");
    Serial.print(MagY);
    Serial.print(" Z=");
    Serial.println(MagZ);
    
}







/*########### I2C address finder ############# */

// SPDX-FileCopyrightText: 2023 Carter Nelson for Adafruit Industries
//
// SPDX-License-Identifier: MIT
// --------------------------------------
// i2c_scanner
//
// Modified from https://playground.arduino.cc/Main/I2cScanner/
// --------------------------------------

//#include <Wire.h>
//
//// Set I2C bus to use: Wire, Wire1, etc.
//#define WIRE Wire
//
//void setup() {
//  WIRE.begin();
//
//  Serial.begin(9600);
//  while (!Serial)
//     delay(10);
//  Serial.println("\nI2C Scanner");
//}
//
//
//void loop() {
//  byte error, address;
//  int nDevices;
//
//  Serial.println("Scanning...");
//
//  nDevices = 0;
//  for(address = 1; address < 127; address++ )
//  {
//    // The i2c_scanner uses the return value of
//    // the Write.endTransmisstion to see if
//    // a device did acknowledge to the address.
//    WIRE.beginTransmission(address);
//    error = WIRE.endTransmission();
//
//    if (error == 0)
//    {
//      Serial.print("I2C device found at address 0x");
//      if (address<16)
//        Serial.print("0");
//      Serial.print(address,HEX);
//      Serial.println("  !");
//
//      nDevices++;
//    }
//    else if (error==4)
//    {
//      Serial.print("Unknown error at address 0x");
//      if (address<16)
//        Serial.print("0");
//      Serial.println(address,HEX);
//    }
//  }
//  if (nDevices == 0)
//    Serial.println("No I2C devices found\n");
//  else
//    Serial.println("done\n");
//
//  delay(5000);           // wait 5 seconds for next scan
//}