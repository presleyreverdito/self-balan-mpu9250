
#include "SparkFunMPU9250-DMP.h"
#include "I2Cdev.h"
#include "Kalman.h"
#include "LMotorController.h"
#include <PID_v1.h> //From https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.h
/*======================Global variable======================*/
// Debug mode
int PID_debug_mode = 1;
int RF_debug_mode = 0;

// MPU9250
MPU9250_DMP imu_9250;
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
double roll, pitch;
float gyroXrate, gyroYrate;
float rad_to_reg = 180 / 3.141592654;
//KALMAN FILTER 

Kalman kalmanX;
Kalman kalmanY;
double gyroXangle, gyroYangle; // Gyroscope angle
double compAngleX, compAngleY; // Complementary filter angle
double kalAngleX, kalAngleY; // Angle after Kalman filter
double corrected_x, corrected_y; // Corrected with offset

// Timer
float now_time;
float pas_time;
float dif_time;
// Motor
#define MIN_ABS_SPEED 75
int IN1 = 2;
int IN2 = 3;
int ENA = 9;
int IN3 = 4;
int IN4 = 5;
int ENB = 10;
double motorSpeedFactorLeft = 1;
double motorSpeedFactorRight = 1;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

// PID                            VALORES DE GANHOR PROXIMOS DO IDEAL
double kp =  13.5;              //                com kalman
                                //        KP =    13.5        |     20    |  34
double ki =   0;                //        KI =    0           |   2       | 0.3
double kd =  0.085;             //         KD =   0.85        | 0.02      | 0.03
double output = 0 ;

// Special angle
float overshoot_angle = 30;
float PID_angle = 8;
double reference_angle = 0;

float throttle = 50;

void kalman() {
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else {
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dif_time); // Calculate the angle using a Kalman filter
  }
  if (abs(kalAngleX) > 90) {
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  }
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dif_time);
  gyroXangle += gyroXrate * dif_time; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dif_time;
  compAngleX = 0.93 * (compAngleX + gyroXrate * dif_time) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dif_time) + 0.07 * pitch;
  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
}





/*======================support function======================*/
void UpdateIMUData(void)
{
  accelX = imu_9250.calcAccel(imu_9250.ax);
  accelY = imu_9250.calcAccel(imu_9250.ay);
  accelZ = imu_9250.calcAccel(imu_9250.az);
  gyroX = imu_9250.calcGyro(imu_9250.gx);
  gyroY = imu_9250.calcGyro(imu_9250.gy);
  gyroZ = imu_9250.calcGyro(imu_9250.gz);

  // Convert to deg/s
  roll = atan(accelY / sqrt(pow(accelX, 2) + pow(accelZ, 2))) * rad_to_reg;
  pitch = atan(-1 * accelX / sqrt(pow(accelY, 2) + pow(accelZ, 2))) * rad_to_reg;
  gyroXrate = gyroX / 131.0;
  gyroYrate = gyroY / 131.0;
}

void printIMUData(double output)
{
  Serial.println( "Angle: " + String(pitch) + "                     Dif_time: " + String(dif_time) + "                        Control signal: " + String(output) /*+ "                       kp_error: " + String(kp_error)*/);
}

         //  SEM KALMAN
PID pid( &pitch, &output, &reference_angle, kp, ki, kd, DIRECT);
       // COM KALMAN
//PID pid( &kalAngleY, &output, &reference_angle, kp, ki, kd, DIRECT);


/*======================setup======================*/
void setup() {
  Serial.begin(9600);

  // MPU-9250
  if (imu_9250.begin() != INV_SUCCESS)
  {
    imu_9250.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);// Enable all sensors:
    imu_9250.setGyroFSR(2000); // Set gyro to 2000 dps
    imu_9250.setAccelFSR(2); // Set accel to +/-2g
    imu_9250.setLPF(5); // Set LPF corner frequency to 5Hz
    imu_9250.setSampleRate(10); // Set sample rate to 10Hz
  }

  //PID setup
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-255, 255);
  // Timer
  pas_time = millis();
}

/*======================main loop======================*/
void loop() {
  //  throttle moving_flag
  int moving_flag = 0;
  // float output = 0;


  // calculate time
  
  dif_time = (now_time - pas_time) / 1000; // in seconds. We work in ms so we haveto divide the value by 1000
  pas_time = now_time;

  // Update IMU data
  if ( imu_9250.dataReady() )
  {
    imu_9250.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
    UpdateIMUData();
    kalman();
  }

  // PID

  pid.Compute();
  motorController.move(output, MIN_ABS_SPEED);



  // print out the debug thing
  if (PID_debug_mode)
    printIMUData(output);

    now_time = millis();
}
