#include "Wire.h"
#include "I2Cdev.h"
#include "MPU9250.h"
#include "BMP180.h"
#include <Filters.h>
#include <MyMadgwickAHRS.h>
#define _USE_MATH_DEFINES
#include<cmath>
#include <Vector.h>

MPU9250 IMU;

int16_t ax, ay, az, gx, gy, gz;
extern volatile float q0, q1, q2, q3;

void setup()
{
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000);
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
    #endif
    Serial.begin(115200);    
    IMU.initialize();
}

long Time = 0;

double LinearVelocity[3] = { 0.0, 0.0, 0.0 };
double LinearPosition[3] = { 0.0, 0.0, 0.0 };
double roll = 0.00;
double pitch = 0.00;

FilterOnePole LinearVelocityX( HIGHPASS, 0.3 );
FilterOnePole LinearVelocityY( HIGHPASS, 0.3 );
FilterOnePole LinearVelocityZ( HIGHPASS, 0.3 );

FilterOnePole LinearPositionX( HIGHPASS, 0.8);
FilterOnePole LinearPositionY( HIGHPASS, 0.8);
FilterOnePole LinearPositionZ( HIGHPASS, 0.8);

int calibration = 0;
volatile double GyroscopeOffset[] = { 0.0, 0.0, 0.0 };

void loop() {
    if (calibration < 100) {
       IMU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
       GyroscopeOffset[0] += IMU.getXGyroOffset();
       GyroscopeOffset[1] += IMU.getYGyroOffset();
       GyroscopeOffset[2] += IMU.getZGyroOffset();
       calibration++;
    }
    if (calibration == 100) {
      GyroscopeOffset[0] /= 100.0;
      GyroscopeOffset[1] /= 100.0;
      GyroscopeOffset[2] /= 100.0;
      calibration++;
    }
    int deltaTime = millis() - Time;
    Time = millis();

    IMU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

 double AccelX = ax / 16384.0;
    double AccelY = ay / 16384.0;
    double AccelZ = az / 16384.0;
    double GyroX;
    double GyroY;
    double GyroZ;
    if (calibration < 100){
      GyroX = 0;
      GyroY = 0;
      GyroZ = 0;
      }
      else {
        GyroX = IMU.getXGyroOffset() - GyroscopeOffset[0];
        GyroY = IMU.getYGyroOffset() - GyroscopeOffset[1];
        GyroZ = IMU.getZGyroOffset() - GyroscopeOffset[2];
        }   
    //double MagnetX = mx;
    //double MagnetY = my;
    //double MagnetZ = mz;

     MyMadgwickAHRSupdate(GyroX, GyroY, GyroZ, AccelX, AccelY, AccelZ, 0,0,0);

double qw, qx, qy, qz =0;
qw = q0;
qx = q1;
qy = q2;
qz = q3;
 double RotationMatrixQuaternion[3][3];
    RotationMatrixQuaternion[0][0] = 1.0f - 2.0f * pow(qy, 2) - 2.0f * pow(qz, 2);
    RotationMatrixQuaternion[0][1] = 2.0f * qx * qy - 2.0f * qz * qw;
    RotationMatrixQuaternion[0][2] = 2.0f * qx * qz + 2.0f * qy * qw;
    RotationMatrixQuaternion[1][0] = 2.0f * qx * qy + 2.0f * qz * qw;
    RotationMatrixQuaternion[1][1] = 1.0f - 2.0f * pow(qx, 2) - 2.0f * pow(qz, 2);
    RotationMatrixQuaternion[1][2] = 2.0f * qy * qz - qw * qx * qw;
    RotationMatrixQuaternion[2][0] = 2.0f * qx * qz - 2.0f * qy * qw;
    RotationMatrixQuaternion[2][1] = 2.0f * qy * qz + 2.0f * qx * qw;
    RotationMatrixQuaternion[2][2] = 1.0f - 2.0f * pow(qx, 2) - 2.0f * pow(qy, 2);


    double AccelMatrix[3][1];
    AccelMatrix[0][0] = AccelX;
    AccelMatrix[1][0] = AccelY;
    AccelMatrix[2][0] = AccelZ;

    double Acc[3][1];

for (int i = 0; i < 3; i++) {
      Acc[i][0] = 0;
      for (int j = 0; j < 3; j++) {
        Acc[i][0] += RotationMatrixQuaternion[i][j] * AccelMatrix[j][0];
      }
    }

    double LinearAcceleration[3] = {
      ((Acc[0][0] - 0.0) * 9.81),
      ((Acc[1][0] - 0.0) * 9.81),
      ((Acc[2][0] - 1.0) * 9.81)
    };

    LinearVelocity[0] = LinearVelocity[0] +  LinearAcceleration[0] * ((double)deltaTime/ 1000.0);
    LinearVelocity[1] = LinearVelocity[1] +  LinearAcceleration[1] * ((double)deltaTime/ 1000.0);
    LinearVelocity[2] = LinearVelocity[2] +  LinearAcceleration[2] * ((double)deltaTime/ 1000.0);

    LinearVelocityX.input(LinearVelocity[0]);
    LinearVelocityY.input(LinearVelocity[1]);
    LinearVelocityZ.input(LinearVelocity[2]);
   
   double velocity[] = { 0.0, 0.0, 0.0 };
    velocity[0] = LinearVelocityX.output();
    velocity[1] = LinearVelocityY.output();
    velocity[2] = LinearVelocityZ.output();
    
     if (abs(LinearAcceleration[0]) * abs(LinearAcceleration[1]) * abs(LinearAcceleration[2]) > 0.1) {
        LinearPosition[0] = LinearPosition[0] + velocity[0] * ((double)deltaTime / 1000.0);
        LinearPosition[1] = LinearPosition[1] + velocity[1] * ((double)deltaTime / 1000.0);
        LinearPosition[2] = LinearPosition[2] + velocity[2] * ((double)deltaTime / 1000.0);
     }
    LinearPositionX.input(LinearPosition[0]);
    LinearPositionY.input(LinearPosition[1]);
    LinearPositionZ.input(LinearPosition[2]);
   //roll = atan2(AccelY, AccelZ) *57.3;
   //pitch = atan2((-AccelX), sqrt(AccelY * AccelY + AccelZ * AccelZ)) * 57.3;
delay(100);
//Serial.print("roll: ");
//Serial.print(roll);
//Serial.print('\t');
//Serial.print("pitch: ");
//Serial.println(pitch);
Serial.print("VelocityX: ");
Serial.print(LinearVelocityX.output());
Serial.print('\t');
Serial.print("VelocityY: ");
Serial.print(LinearVelocityY.output());
Serial.print('\t');
Serial.print("VelocityZ: ");
Serial.println(LinearVelocityZ.output());

Serial.print("LinearPositionX: ");
Serial.print(LinearPositionX.output());
Serial.print('\t');
Serial.print("LinearPositionY: ");
Serial.print(LinearPositionY.output());
Serial.print('\t');
Serial.print("LinearPositionZ: ");
Serial.println(LinearPositionZ.output());
Serial.print('\t');
Serial.print("q0: ");
Serial.print(q0);
Serial.print('\t');
Serial.print("q1: ");
Serial.println(q1);
Serial.print("q2: ");
Serial.print(q2);
Serial.print('\t');
Serial.print("q3: ");
Serial.println(q3);
}
