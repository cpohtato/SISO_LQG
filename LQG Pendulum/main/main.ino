#include <Wire.h>
#include <Servo.h>

//  Constants

const float SAMPLE_FREQ = 250.0;                                    //  IMU sampling frequency [Hz]
const float DT = 1.0/SAMPLE_FREQ;

const int GYRO_FS_SEL = 1;
const int ACC_AFS_SEL = 2;

const float GYRO_TO_DPS = 1/65.5;                                   //  (Degrees per second) per LSB
                                                                    //  GYRO_FS_SEL: 1/131.0, 1/65.5, 1/32.8, 1/16.4
const float ACC_RESOLUTION = 1/4096.0;                              //  ACC_AFS_SEL: 1/16384.0, 1/8192.0, 1/4096.0, 1/2048.0;

const float GYRO_TO_RPS = GYRO_TO_DPS * DEG_TO_RAD;
const float GYRO_TO_DEG = GYRO_TO_DPS * DT;
const float GYRO_TO_RAD = GYRO_TO_DEG * DEG_TO_RAD;

const int HOLDING_TORQUE_OFFSET = 61;
const int MAX_CMD = 180;

const int DELAY_TAPS = 10;

const bool KALMAN = true;

//  MPU6050 global variables
int rawGyroX, rawGyroY, rawGyroZ;                                   //  Gyroscope velocity [gyro bits]
long gyroXCal, gyroYCal, gyroZCal;                                  //  Calibration offset [gyro bits]
long rawAccX, rawAccY, rawAccZ;
int temperature;
long loop_timer;
boolean set_gyro_angles;

float x0 = 0;
float y0 = 0;
float z0 = 0;
float x1 = 0;
float y1 = 0;
float z1 = 0;

float xf0 = 0;
float yf0 = 0;
float zf0 = 0;
float xf1 = 0;
float yf1 = 0;
float zf1 = 0;

//  State variables
float x;                                                              //  Angular position [rad]
float v;                                                              //  Angular velocity [rad/s]
float F;                                                              //  Propeller thrust [N]
int u_d[DELAY_TAPS];                                                  //  Delayed control inputs

//  Kalman model
float Phi_00 = 1;
float Phi_01 = 0.004;
float Phi_11 = 0.984;
float Phi_12 = 0.132;
float Phi_22 = 0.8;
float Phi_23 = 0.00897778;

//  Process covariance
float P_x = 5;
float P_v = 5;
//float Q_x = 1;
//float Q_v = 5;
float Q_x = 0.01*pow(0.01226,2);
float Q_v = 0.01*pow(0.834, 2);

//  Measurement covariance
float R_x = pow(0.24035, 2);
float R_v = pow(0.261678, 2);

//  Kalman gains
float x_K;
float v_K;

//  Gains
float K_x = 199.10018586344037;
float K_v = 24.83877300813682;
float K_F = 15.9578400486208;
float K_u_d[DELAY_TAPS] = {0.1428885578544913, 0.1430272274097408, 
0.14382088899753845, 0.1454433772055937, 0.14811214927796731, 
0.15209919344598022, 0.1577446643852116, 0.1654739275808085, 
0.17581886482892284, 0.18944450615899178};

float debugGyroX = 0;
float debugAccX = 0;

int count = -1;

Servo motor;
const int motorPin = 3;
int motorCmd = 0;

void setup()
{
  pinMode(13, OUTPUT);                                                 //Set output 13 (LED) as output
  digitalWrite(13, HIGH);                                              //LED on for setup
  
  Wire.begin();                                                        //Start I2C as master
  Serial.begin(230400);                                                 //Use only for debugging

  //  Set initial conditions to 0
  x = 0;
  v = 0;
  F = 0;

  for (int i = 0; i < DELAY_TAPS; i++)
  {
    u_d[i] = 0;
  }

  initMPU6050();

  if (KALMAN)
  {
    initKalman();
  }

  motor.attach(motorPin, 1000, 2000);
  motor.write(0);
  delay(3000);
  motor.write(120);
  delay(3000);
  motor.write(0);
  delay(5000);

  digitalWrite(13, LOW);                                               //All done, turn the LED off
  
  loop_timer = micros();                                               //Reset the loop timer
}

void loop()
{
  if (KALMAN)
  {
    estimateStateKalman();
  }
  else
  {
    estimateState();  
  }

  count++;

  if (count == 500)
  {
    count = 0;
  }

  if (count < 250)
  {
    motorCmd = -(int)(K_x * (x + 0.2) + K_v * v + K_F * F);
  }
  else if (count < 500)
  {
    motorCmd = -(int)(K_x * (x - 0.2) + K_v * v + K_F * F);
  }

  //motorCmd = -(int)(K_x * x + K_v * v + K_F * F);

  for (int d; d < DELAY_TAPS; d++)
  {
    motorCmd += -(int)(K_u_d[d] * u_d[d]);
  }

  if (motorCmd + HOLDING_TORQUE_OFFSET > MAX_CMD)
  {
    motorCmd = MAX_CMD - HOLDING_TORQUE_OFFSET;
  }

  if (motorCmd + HOLDING_TORQUE_OFFSET < 0)
  {
    motorCmd = -HOLDING_TORQUE_OFFSET;
  }

  motor.write(motorCmd + HOLDING_TORQUE_OFFSET);

//  Serial.print(x);
//  Serial.print(" ");
//  Serial.println(motorCmd);

  if (count < 250)
  {
    Serial.print(x*100);
    Serial.print(" ");
    Serial.println(-0.2);
  }
  else if (count < 500)
  {
    Serial.print(x*100);
    Serial.print(" ");
    Serial.println(0.2);
  }

  for (int d = 0; d < (DELAY_TAPS - 1); d++)
  {
    u_d[d] = u_d[d+1];
  }
  u_d[DELAY_TAPS - 1] = motorCmd;
  
  //Serial.println(4000 - (micros()-loop_timer));

  while(micros() - loop_timer < 4000);                                 //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
  loop_timer = micros();                                               //Reset the loop timer
}

void initKalman()
{
  float accTotal;
  float accX;
  float accY;
  float accZ;
  
  read_mpu_6050_data();  
  
  accTotal = sqrt((rawAccX * rawAccX) + (rawAccY * rawAccY) + (rawAccZ * rawAccZ));  
  accX = asin((float)rawAccY/accTotal);
  accY = asin((float)rawAccX/accTotal) * -1;

  //  Set initial conditions
  x = accX;
  v = 0;
  F = 0;
  for (int i = 0; i < DELAY_TAPS; i++)
  {
    u_d[i] = 0;
  }
}

void estimateStateKalman()
{
  float v_g_x;
  float v_g_y;

  float accTotal;
  float accX;
  float accY;
  float accZ;
  
  //  Read sensors
  read_mpu_6050_data();   

  //Accelerometer angle calculations
  //Calculate the total accelerometer vector first
  accTotal = sqrt((rawAccX * rawAccX) + (rawAccY * rawAccY) + (rawAccZ * rawAccZ));  
  accX = asin((float)rawAccY/accTotal);
  accY = asin((float)rawAccX/accTotal) * -1;

  v_g_x = (rawGyroX - gyroXCal) * GYRO_TO_DPS * DEG_TO_RAD;
  v_g_x += accY * sin(v_g_x * DT);
  v_g_y = (rawGyroY - gyroYCal) * GYRO_TO_DPS * DEG_TO_RAD;
  v_g_y += accX * sin(v_g_y * DT);
  
  //  Generate prediction
  x = Phi_00 * x + Phi_01 * v;
  v = Phi_11 * v + Phi_12 * F;
  F = Phi_22 * F + Phi_23 * u_d[0];
  
  for (int d = 0; d < (DELAY_TAPS - 1); d++)
  {
    u_d[d] = u_d[d+1];
  }
  u_d[DELAY_TAPS - 1] = motorCmd;

  //  Generate a priori process covariance
  P_x = pow(Phi_00, 2) * P_x + pow(Phi_01, 2) * P_v + Q_x;
  P_v = pow(Phi_11, 2) * P_v + Q_v;

  //  Calculate Kalman gain
  x_K = P_x/(P_x + R_x);
  v_K = P_v/(P_v + R_v);

  //  Corrector
  x = x + x_K * (accX - x);
  v = v + v_K * (v_g_x - v);

  //  A posteriori covariance matrix
  P_x = (1 - x_K) * P_x;
  P_v = (1 - v_K) * P_v;
}

void estimateState()
{
  
  //  Angle traversed over time step
  float gyroDX;
  float gyroDY;
  float gyroDZ;

  //  Integrated gyro output
  float gyroX;
  float gyroY;
  float gyroZ;

  long accTotal;
  float accX;
  float accY;
  float accZ;

  //  Read the raw acc and gyro data from the MPU-6050
  read_mpu_6050_data();                   

  //  Update previous state
  x1 = x0;
  y1 = y0;
  z1 = z0;
  xf1 = xf0;
  yf1 = yf0;
  zf1 = zf0;

  //  Subtract offset calibration value, then integrate velocity over time
  //  DX = x_vel * GYRO_TO_DPS * DT * DEG_TO_RAD
  gyroDX = (rawGyroX - gyroXCal) * GYRO_TO_RAD;
  gyroDY = (rawGyroY - gyroYCal) * GYRO_TO_RAD;
  gyroDZ = (rawGyroZ - gyroZCal) * GYRO_TO_RAD;

  gyroX = xf0 + gyroDX;
  gyroY = yf0 + gyroDY;
  gyroZ = zf0 + gyroDZ;

  //If the IMU has yawed transfer the roll angle to the pitch angle
  //If the IMU has yawed transfer the pitch angle to the roll angle
  gyroX += gyroY * sin(gyroDZ);               
  gyroY -= gyroX * sin(gyroDZ);               
  
  //Accelerometer angle calculations
  //Calculate the total accelerometer vector first
  accTotal = sqrt((rawAccX * rawAccX) + (rawAccY * rawAccY) + (rawAccZ * rawAccZ));  
  accX = asin((float)rawAccY/accTotal);
  accY = asin((float)rawAccX/accTotal) * -1;
  
  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  accX -= 0.0;                                              //Accelerometer calibration value for pitch
  accY -= 0.0;                                               //Accelerometer calibration value for roll

  if (set_gyro_angles)
  {             
    //If the IMU is already started, complementary filter                                   
    x0 = gyroX * 0.9993 + accX * 0.0007;
    y0 = gyroY * 0.9993 + accY * 0.0007;
  }
  else
  { 
    //At first start, set values equal to accelerometer                                                            
    x0 = accX;
    y0 = accY;
    
    //Set the IMU started flag
    set_gyro_angles = true;                                            
  }
  
  //  Low pass filter
  xf0 = xf0 * 0.0 + x0 * 1.0;   
  yf0 = yf0 * 0.0 + y0 * 1.0;

  x = xf0;
  v = (xf0 - xf1) / DT;
  F = (1-0.004/0.01)*F + (8.08/180)*(0.004/0.01)*u_d[0];

  debugGyroX = (rawGyroX - gyroXCal) * GYRO_TO_DPS * DEG_TO_RAD;
  debugAccX = accX;
}

void initMPU6050()
{
  int numSamples = 2000;
  
  //Setup the registers of the MPU-6050 and start the gyro
  setup_mpu_6050_registers();                                          

  //  Take average                                             
  for (int cal_int = 0; cal_int < numSamples ; cal_int++)
  {        
    //  Find gyro offsets     
    read_mpu_6050_data();
    gyroXCal += rawGyroX;
    gyroYCal += rawGyroY;
    gyroZCal += rawGyroZ;
    
    //  Delay 3us to simulate the 250Hz program loop
    delay(3);                                                          
  }

  //  Average over number of samples
  gyroXCal /= numSamples;
  gyroYCal /= numSamples;
  gyroZCal /= numSamples;
}

void read_mpu_6050_data()
{                                             
  //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  rawAccX = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_x variable
  rawAccY = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_y variable
  rawAccZ = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_z variable
  temperature = Wire.read()<<8|Wire.read();                            //Add the low and high byte to the temperature variable
  rawGyroX = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_x variable
  rawGyroY = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_y variable
  rawGyroZ = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_z variable
}

void setup_mpu_6050_registers()
{  
  configPower();
  configAcc();
  configGyro();
}

void configPower()
{
  //Configure Power Management 1, Register 107
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Request register 107, Power Management 1
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
}

void configAcc()
{
  //Configure the accelerometer, Register 28
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Request register 28, Accelerometer Configuration

  switch (ACC_AFS_SEL)
  {
    case 0:
      Wire.write(0x00);                                           //  +/- 2g full scale
      break;
    case 1:
      Wire.write(0x08);                                           //  +/- 4g full scale
      break;
    case 2:
      Wire.write(0x10);                                           //  +/- 8g full scale
      break;
    case 3:
      Wire.write(0x18);                                           //  +/- 16g full scale
      break;
    default:
      break;
  }
  
  Wire.endTransmission();                                              //End the transmission
}

void configGyro()
{
  //Configure the gyro
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Request register 27, Gyroscope Configuration

  switch (GYRO_FS_SEL)
  {
    case 0:
      Wire.write(0x00);                                           //  +/- 250 dps full scale
      break;
    case 1:
      Wire.write(0x08);                                           //  +/- 500 dps full scale
      break;
    case 2:
      Wire.write(0x10);                                           //  +/- 1000 dps full scale
      break;
    case 3:
      Wire.write(0x18);                                           //  +/- 2000 dps full scale
      break;
    default:
      break;
  }
  
  Wire.endTransmission();                                              //End the transmission
}
