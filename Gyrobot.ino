//Moteur Gauche = 0, Droit = 1
/*Arduino nano :
  Pwm port: D3,D5,D6,D9,D10,D11
  SDA: A4, SCL: A5
*/


#include <Wire.h>               //I²C Library
#include <math.h>               //Math library
#include "Quaternion.h"

#define MICROSECONDE 1000000.0  //Conversion from 1000000 to second
#define IMURATE 250             //Number of angle mesurement per second
#define CALITE 1000             //Number of mesurement for the calibration
#define MPUADRESS 0x69          //I2C Adress of mpu6050
#define ACCRATE 8192            //Conversion rate from IMU accelormeter to m/s^²
#define GYRORATE 32.8           //Conversion rate from IMU gyroscope to °/s

float alpha = 0.97;             //Complementary filter coeff

double converted_acc_x, converted_acc_y, converted_acc_z;            //Converted to g

float target_angle = 0;

long gyro_y_cal = 0; //Offset values

double acc_y_angle;
double angle_y = 0;
Quaternion current;

double corrected_angle;

long temp = 0, delta = 0;


double averaged_z_acc;

void setup() {
  digitalWrite(13, HIGH);
  Serial.begin(57600);
  current = Quaternion(1, 0, 0);

  init_imu();
  calibrate_gyroscope();
}
template <typename type>
type sign(type value) {
 return type((value>0)-(value<0));
}

void loop() {
  if(micros() > temp + MICROSECONDE / IMURATE) { //Try to keep the loop at a constant rate (We dont want to create an unnecessary load)
    delta = micros() - temp;
    temp = micros();
    update_imu_quat();
    double pid = PID(angle_y, 0);
    Serial.print("a:");
    Serial.print(current.a);
    Serial.print(",b:");
    Serial.print(current.b);
    Serial.print(",c:");
    Serial.print(current.c);
    Serial.print(",d:");
    Serial.print(current.d);
    Serial.print(",angle:");
    Serial.print(corrected_angle);
    Serial.print(",pid:");
    Serial.println(pid);
    if(pid>511)pid=511;
    if(pid<-511)pid=-511;
    if(pid > 0) {
      updateMotor(true, true, pid+512);
      updateMotor(false, false, pid+512);
    } else {
      pid=-pid;
      updateMotor(false, true, pid+512);
      updateMotor(true, false, pid+512);
    }
  }
}
