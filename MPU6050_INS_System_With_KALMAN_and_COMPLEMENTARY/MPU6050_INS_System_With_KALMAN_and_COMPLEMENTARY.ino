#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter

#include "Lib/MPU6050_Var.h"

#define RESTRICT_PITCH

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;


//==================GLOBAL VARIABLES=========================
MPU_st      imu_st;
//===========================================================


//==================PROTOTYPE FUNCTIONS======================
//Initialization Page
void UART0_INITIALIZE               (void);
void I2C_INITIALIZATION             (void);

//MPU_6050_INS_System
void MPU6050_INITIALIZE                     (void);
void MPU6050_SET_INITIAL_ORIENTATION_VALUE  (void);
void SHOW_INITIAL_ORIENTATION_IN_ASCII      (void);
void FIND_PITCH_ROLL                        (void);
void SHOW_VALUES_IN_ASCII                   (void);
void PID_INPUT_VALUE_SYNC                   (void);
void UPDATE_YPR_AND_PID                     (void);
void SHOW_YAW_PITCH_ROLL_IN_ASCII           (void);
void SHOW_YAW_PITCH_ROLL_FOR_LOG            (void);



/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
double temp_value;

//=====PARAMETERS======
double alpha = 0.93f;
uint8_t test_length = 10000;  //test length should be even number (çift sayı)
double total_number[2];

double rate_timer;
double prevrate_timer = 0;

float pitch_treshold_p =  0.80;
float pitch_treshold_n = -0.80;

float roll_treshold_p =  0.3;
float roll_treshold_n = -0.3;

uint8_t i2cData[14]; // Buffer for I2C data

void setup()
{
  UART0_INITIALIZE();
  I2C_INITIALIZATION();
  MPU6050_INITIALIZE();
  MPU6050_SET_INITIAL_ORIENTATION_VALUE();
}

void loop()
{      
  FIND_PITCH_ROLL();
  SHOW_INITIAL_ORIENTATION_IN_ASCII();
  SHOW_YAW_PITCH_ROLL_IN_ASCII();
}

void UART0_INITIALIZE()
{
  delay(50);
  Serial.begin(115200);
  delay(50);
}

void I2C_INITIALIZATION()
{
  delay(50);
  Wire.begin();
  
  #if ARDUINO >= 157
    Wire.setClock(100000UL); // Set I2C frequency to 100kHz
  #else
    TWBR = ((F_CPU / 100000UL) - 16) / 2; // Set I2C frequency to 100kHz
  #endif
}

void MPU6050_INITIALIZE()
{
  delay(1000);
  
  imu_st.current_ypr_f64[3]     = {0};      //Make current_ypr equal to 0 for previous_ypr
  imu_st.beginning_ypr_f64[3]   = {0};      //Make beginning_ypr equal to 0
  total_number[2]               = {0};      //Make total_number equal to 0 for calibration

  while (1)
  {
    if ( i2cWrite(0x6B, 0x80, false) == 0 ) //MPU6050 reset
    {
      
    }

    else
    {
      Serial.println(F("CHECK_MPU_VCC_CABLE"));
    }
  }       
  delay(5);
  
  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g

  while (1)
  {
    if ( i2cWrite(0x19, i2cData, 4, false) == 0 ) // Write to all four registers at once
      break;

    else
      Serial.println(F("CHECK_MPU_VCC_CABLE"));
  }
  while (i2cWrite(0x1A, 0x00, true));
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode
  while (i2cRead(0x75, i2cData, 1));
  
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.println(F("Error_read_sensor"));
    while (1){};
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
  #ifdef RESTRICT_PITCH // Eq. 25 and 26
    //double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    //double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  #endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  imu_st.timer = micros();

  Serial.println(F("mpu6050_initialization_step_done"));
}

void MPU6050_SET_INITIAL_ORIENTATION_VALUE()
{
  uint8_t i;

  for(i = 0; i < test_length; i++)
  {
    FIND_PITCH_ROLL();

    if ( i > (test_length/2) )
    {
      total_number[0] += imu_st.current_ypr_f64[PITCH];
      total_number[1] += imu_st.current_ypr_f64[ROLL];
    }
  }

  imu_st.beginning_ypr_f64[PITCH]  =  total_number[0] / (test_length/2);
  imu_st.beginning_ypr_f64[ROLL]   =  total_number[1] / (test_length/2);

  Serial.println(F("mpu6050_set_initial_orientation_step_done"));
}

void SHOW_INITIAL_ORIENTATION_IN_ASCII()
{
  Serial.print("Beginning Pitch ");     Serial.print   (imu_st.beginning_ypr_f64[PITCH]);
  Serial.print("    Beginning Roll ");  Serial.println   (imu_st.beginning_ypr_f64[ROLL]);
}

void FIND_PITCH_ROLL()
{
  imu_st.old_ypr_f64[YAW]   = imu_st.current_ypr_f64[YAW];
  imu_st.old_ypr_f64[PITCH] = imu_st.current_ypr_f64[PITCH];
  imu_st.old_ypr_f64[ROLL]  = imu_st.current_ypr_f64[ROLL];
  
  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX    = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY    = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ    = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX   = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY   = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ   = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - imu_st.timer) / 1000000; // Calculate delta time
  imu_st.timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  //double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  //double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);

  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = alpha * (compAngleX + gyroXrate * dt) + (1.0f - alpha) * roll; // Calculate the angle using a Complimentary filter
  compAngleY = alpha * (compAngleY + gyroYrate * dt) + (1.0f - alpha) * pitch;
    
  imu_st.current_ypr_f64[YAW]   = 0;
  imu_st.current_ypr_f64[PITCH] = compAngleY; //Pitch and roll were reverse between them because of MPU6050 position on aircraft.
  imu_st.current_ypr_f64[ROLL]  = compAngleX;

  double rate_timer = micros();
  
  if( ((imu_st.old_ypr_f64[PITCH] - imu_st.current_ypr_f64[PITCH]) > (pitch_treshold_n)) && (((imu_st.old_ypr_f64[PITCH] - imu_st.current_ypr_f64[PITCH])) < pitch_treshold_p) )
    imu_st.current_ypr_f64[PITCH] = imu_st.old_ypr_f64[PITCH];

  if( ((imu_st.old_ypr_f64[ROLL] - imu_st.current_ypr_f64[ROLL]) > (roll_treshold_n)) && (((imu_st.old_ypr_f64[ROLL] - imu_st.current_ypr_f64[ROLL])) < roll_treshold_p) )
    imu_st.current_ypr_f64[ROLL] = imu_st.old_ypr_f64[ROLL];

  prevrate_timer = rate_timer;
  
  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
}

void SHOW_VALUES_IN_ASCII()
{
  /* Print Data */
  #if 0 // Set to 1 to activate
    Serial.print(accX); Serial.print("\t");
    Serial.print(accY); Serial.print("\t");
    Serial.print(accZ); Serial.print("\t");
  
    Serial.print(gyroX); Serial.print("\t");
    Serial.print(gyroY); Serial.print("\t");
    Serial.print(gyroZ); Serial.print("\t");
  
    Serial.print("\t");
  #endif

  Serial.print("Roll ");
  /*Serial.print(roll);*/ Serial.print("\t");
  //Serial.print(gyroXangle); Serial.print("\t");
  Serial.print(compAngleX); Serial.print("\t");
  Serial.print(kalAngleX); Serial.print("\t");

  Serial.print("\t");

  Serial.print("Pitch ");
  /*Serial.print(pitch);*/ Serial.print("\t");
  //Serial.print(gyroYangle); Serial.print("\t");
  Serial.print(compAngleY); Serial.print("\t");
  Serial.print(kalAngleY); Serial.print("\t");

  #if 0 // Set to 1 to print the temperature
    Serial.print("\t");
  
    double temperature = (double)tempRaw / 340.0 + 36.53;
    Serial.print(temperature); Serial.print("\t");
  #endif

  Serial.print("\r\n");
  delay(2);
}

void SHOW_YAW_PITCH_ROLL_IN_ASCII()
{
  Serial.print("YPR\t");
  Serial.print(imu_st.current_ypr_f64[YAW]);
  Serial.print("\t");
  Serial.print(imu_st.current_ypr_f64[PITCH]);
  Serial.print("\t");
  Serial.println(imu_st.current_ypr_f64[ROLL]);
}

void SHOW_YAW_PITCH_ROLL_FOR_LOG()
{
  Serial.print(millis());
  Serial.print(",");
  Serial.print(imu_st.current_ypr_f64[YAW]);
  Serial.print(",");
  Serial.print(imu_st.current_ypr_f64[PITCH]);
  Serial.print(",");
  Serial.print(imu_st.current_ypr_f64[ROLL]);
  Serial.print(",");
  Serial.print(accX);
  Serial.print(",");
  Serial.print(accY);
  Serial.print(",");
  Serial.println(accZ);
}
